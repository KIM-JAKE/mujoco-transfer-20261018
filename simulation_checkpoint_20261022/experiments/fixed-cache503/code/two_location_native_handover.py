"""Model-only two-location handover, saved fresh camera+encoders only."""
from pathlib import Path
import json,time,numpy as np
from isaac_kinematics import frame,exp,log
from rgbd_geometry import axis_frame
from loaded_contact_aperture import LoadedContactAperture
from fr3_gpu_ik import propose_ik
from fr3_path_timing import retime_safe_polyline,retime
from grasp_transfer_feasibility import receiver_approach_route,donor_aperture,sequential_transfer_clear

def _public_priors(here):
    path=here/'bin-picking/double-handover-public-priors.json'
    value=json.loads(path.read_text())
    if value.get('schema_version')!=1:raise RuntimeError('Unsupported public handover proposal schema')
    return value

def _model_plan(c, part, measured_G, park_goal, *, parked=False, relocated=False):
    """Snapshot current encoders/image geometry; never mutate robot state."""
    if part!='ring1':raise RuntimeError('Two-location component requires LARGE class')
    already_parked=bool(parked)
    here=Path(__file__).resolve().parent;G=np.asarray(measured_G,float).copy()
    actual=np.asarray(c['art'].get_dof_positions()[0],float).copy();cmd=np.asarray(c['cmd'][0],float).copy()
    ri,li,rf,lf=c['ids'],c['left_ids'],c['fi'],c['left_fi'];rq=cmd[ri];lq=cmd[li]
    left,right=c['left_arm'],c['arm'];rc,lc=c['checkers']['right'],c['checkers']['left']
    wl=float(actual[lf].sum());wr=float(actual[rf].sum())
    binding=c['component_bindings'].get('left')
    if binding is None or not binding.active:raise RuntimeError('Live LEFT measured-aperture binding required')
    binding.check(actual[lf])
    from receiver_pad_contact import contact_state
    contact=contact_state(dict(c,config=dict(c['config'],receiver_axial_only=True)),part,G,wl)
    if not contact['opposed_contact_compatible']:raise RuntimeError('Live image does not confirm temporary cap contact')
    saved=(lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative,getattr(lc,'loaded_contact_aperture',None))
    saved_right=(rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative)
    lc.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@G);lc.loaded_contact_aperture=binding;rc.detach_payload()
    P=left.fk(actual[li])@G;P2=P.copy()
    if not relocated:
        P2[:3,3]=[-.30,-.30,1.35];P2[:3,:3]=np.asarray(c['_model_part_R'])
    target=P2@np.linalg.inv(G);start=left.fk(lq)
    result=dict(state='live_RGBD_encoder_two_location_whole_role_proposal',source='fresh camera class pose plus actual robot encoders',robot_motion_executed=False,uses_free_body_poses=False,full_task_pass=False,issued_certificate=False,all_actual_motion_requires_exact_issue_guard=True,incoming_P=P.tolist(),second_air_P=P2.tolist(),LEFT_measured_G=G.tolist(),source_loaded_contact_record=binding.record,carry=None,receiver_candidates=[],accepted=None,fresh_RGBD_and_replan_required_after_actual_carry=True)
    out=c['vision'].out/'two-location-live-model-last.json'
    def save():out.write_text(json.dumps(result,indent=2))
    def issued(checker,other,other_q,path,own_start,own_target,other_widths):
        # This is a route proposal, not permission to execute. The caller's
        # issue() validates ALL exact retimed rows, actual/commanded other-arm
        # poses and native apertures immediately before each actual movement.
        # Do not repeat future issued certificates after every role change.
        return dict(clear=True,issued_certificate=False,deferred_to_immediate_actual_issue=True,route_proposal_only=True)

    try:
        lc.resolution_rad=.002;rc.resolution_rad=.002
        # Cheap ORIGINAL-FK-admitted endpoint gates precede protected routes.
        left_goals=[actual[li].copy()]if relocated else c['_park_role_cache'].get('left_goals')
        if left_goals is None:
            q=left.solve(target[:3,3],target[:3,:3],c['_left_model_seed']);left_goals=[]if q is None else[q]
            if not left_goals:left_goals=propose_ik(c,left,target[:3,3],target[:3,:3],actual[li],'live_second_air_LEFT_target')[:6]
            c['_park_role_cache']['left_goals']=left_goals
        result['LEFT_endpoint_proposals']=len(left_goals)
        if not left_goals:result['failure']='No current native-model LEFT endpoint';save();return result
        # Release fully while stationary BEFORE rotating out of the ring.
        # This future opening is only a model proposal until the entire
        # composition passes; actual issue() checks every opening row again.
        prepark_opening_required=not already_parked and min(wr,float(cmd[rf].sum()))<.0795
        result['RIGHT_prepark_full_opening_required']=bool(prepark_opening_required)
        if prepark_opening_required:
            own_widths=np.unique(np.linspace(min(wr,float(cmd[rf].sum())),.080,9))
            other_widths=np.unique(np.linspace(min(wl,float(cmd[lf].sum())),max(wl,float(cmd[lf].sum())),5))
            for other_q in [actual[li],cmd[li]]:
                for ow in other_widths:
                    with rc.other_arm_at(lc,other_q,float(ow)):
                        for qq in [actual[ri],cmd[ri]]:
                            for width in own_widths:
                                clear,why=rc.clear(qq,float(width))
                                if not clear:
                                    result['failure']='Stationary full native RIGHT release sweep blocked'
                                    result['RIGHT_prepark_opening_collision']=why;save();return result
            result['RIGHT_prepark_opening_checked_widths_m']=own_widths.tolist()
            # MODEL copy only; no simulator encoder or command is modified.
            cmd=cmd.copy();actual=actual.copy();cmd[rf]=.040;actual[rf]=.040;wr=.080
        park_goals=[actual[ri].copy()]if already_parked else[np.asarray(park_goal,float)]
        result['RIGHT_park_endpoint_proposals']=len(park_goals);parked=None
        with rc.other_arm_at(lc,actual[li],wl):
            for goal in park_goals:
                route=None
                model_hint=[rq.copy(),rq.copy()]if already_parked else c.get('_empty_park_model_proposal')
                if model_hint is not None:
                    hint=[np.asarray(row,float).copy()for row in model_hint];hint[0]=rq.copy();hint[-1]=goal.copy()
                    if rc.path(hint,float(cmd[rf].sum()))[0]:route=hint
                if route is None:route=rc.route(rq,goal,float(cmd[rf].sum()),seconds=.7)
                if route is None:continue
                certificate=issued(rc,lc,actual[li],route,cmd[rf],cmd[rf],np.linspace(min(wl,float(cmd[lf].sum())),max(wl,float(cmd[lf].sum())),5))
                if certificate['clear']:parked=dict(route=np.asarray(route).tolist(),q=np.asarray(goal).tolist(),certificate=certificate);break
        result['RIGHT_empty_park']=parked
        if parked is None:result['failure']='No protected EMPTY RIGHT park route before LEFT carry';save();return result
        # Private MODEL encoders only. No simulator setter or step is called.
        actual=actual.copy();cmd=cmd.copy();rq=np.asarray(parked['q']);actual[ri]=rq;cmd[ri]=rq
        # Endpoint branch is accepted only after the whole future role.
        # A Cartesian continuation must not hide alternative LEFT elbows.
        branch=0;result['LEFT_branch_index']=branch
        if branch>=len(left_goals):
            result['failure']='No cached LEFT endpoint at requested index';save();return result
        goal=np.asarray(left_goals[branch])
        with lc.other_arm_at(rc,actual[ri],max(wr,float(cmd[rf].sum()))):
            legal,why=lc.clear(goal,float(cmd[lf].sum()))
            if not legal:
                result['failure']='Selected LEFT endpoint violates original geometry';result['LEFT_endpoint_collision']=why;save();return result
            candidate=None
            prior_path=[actual[li].copy(),actual[li].copy()]if relocated else c.get('_loaded_route_model_proposal')
            if prior_path is not None:
                proposal=[np.asarray(row,float).copy()for row in prior_path]
                proposal[0]=lq.copy();proposal[-1]=goal.copy()
                # A previously accepted MODEL curve is just a proposal. It
                # must pass current original geometry, then every exact issued
                # row/actual-native-aperture context before overall acceptance.
                model_clear,model_why=lc.path(proposal,float(cmd[lf].sum()))
                result['prior_loaded_model_path_recheck']=dict(clear=model_clear,collision=model_why,model_proposal_only=True,no_recorded_physical_q_replay=True)
                if model_clear:candidate=proposal
            if candidate is None:candidate=lc.route(lq,goal,float(cmd[lf].sum()),seconds=.7)
        if candidate is None:
            result['failure']='No protected LEFT loaded route for this endpoint branch';save();return result
        path=list(candidate);carry_kind='original_LEFT_collision_aware_joint_route_fixed_endpoint_branch'
        result['carry']=dict(path_model_clear=True,clear=None,route=np.asarray(path).tolist(),exact_issued_deferred_until_future_RIGHT_route=True,proposal=carry_kind)
        result['full_RIGHT_endpoint_count']=0
        lq2=path[-1];prior=_public_priors(here)['positive_RIGHT_sensor_observation'];raw=np.linalg.inv(prior['hand_TCP_world'])@frame(prior['result']['pose']['center_m'],axis_frame(prior['result']['pose']['axis']))
        for variant,GR in [('finger_symmetry',np.diag([-1.,-1.,1.,1.])@raw)]:
            clean=dict(c,config=dict(c['config'],receiver_axial_only=False));width=donor_aperture(clean,'ring1',GR);H=P2@np.linalg.inv(GR);row=dict(variant=variant,RIGHT_G=GR.tolist(),target_H=H.tolist(),native_aperture_m=width);result['receiver_candidates'].append(row)
            if width is None or not sequential_transfer_clear(lc,left.fk(lq2),wl,rc,H,width,.080,donor_release_width=.0235):row['failure']='intrinsic_contact_or_pair';save();continue
            target_q=right.solve(H[:3,3],H[:3,:3],c['_known_right_seed'])
            goals=[]if target_q is None else[target_q];row['known_model_endpoint_re_solved']=True;row['GPU_goal_count']=0;row['known_endpoint_count']=len(goals)
            with rc.other_arm_at(lc,lq2,wl):
                row['endpoint_rejections']=[];row['terminal_clear_count']=0
                for goal in goals:
                    clear,why=rc.clear(goal,.080)
                    if not clear:
                        if len(row['endpoint_rejections'])<3:row['endpoint_rejections'].append(why)
                        continue
                    result['full_RIGHT_endpoint_count']+=1
                    row.setdefault('native_clear_endpoint_qs',[]).append(np.asarray(goal).tolist())
                    pre=H.copy();pre[:3,3]-=H[:3,2]*.020;preq=right.solve(pre[:3,3],pre[:3,:3],goal)
                    terminal=None;terminal_failure=None
                    if preq is None:terminal_failure=dict(stage='Cartesian_standoff_IK')
                    else:
                        preclear,prewhy=rc.clear(preq,.080)
                        if not preclear:terminal_failure=dict(stage='Cartesian_standoff_native_geometry',collision=prewhy)
                        else:
                            terminal=receiver_approach_route(c,'native_terminal_only',rc,preq,goal,.080,seconds=.5)
                            if terminal is None:terminal_failure=dict(stage='Cartesian_terminal_IK_or_path')
                    if terminal is not None:row['terminal_clear_count']+=1
                    else:row.setdefault('Cartesian_terminal_failures',[]).append(dict(endpoint_q=np.asarray(goal).tolist(),**terminal_failure))
                    # ONE longer search in the ORIGINAL FCL planner. Its
                    # proposal is never execution permission: all original
                    # native2mrad edge/issued native-width guards still follow.
                    planner=getattr(rc,'fast_planner',None)
                    if planner is None:raise RuntimeError('Original FCL planner required for this bounded model experiment')
                    search_started=time.perf_counter();route=None
                    if c.get('_known_reverse_model_proposal')is not None:
                        hint=[np.asarray(row,float).copy()for row in c['_known_reverse_model_proposal']];hint[0]=rq.copy();hint[-1]=np.asarray(goal).copy()
                        if rc.path(hint,.080)[0]:route=hint
                    if route is None:
                        from fr3_gpu_route import propose_route
                        proposed=propose_route(rc,rq,goal,.080)
                        if proposed is not None and rc.path(np.asarray(proposed),.080)[0]:route=proposed
                    if route is None:route=planner.route(rq,goal,.080,10.)
                    row['original_FCL_search_wall_s']=time.perf_counter()-search_started;row['original_FCL_search_budget_s']=10.
                    route_kind='ORIGINAL_FCL_10s_EMPTY_open_Hand_jointspace_proposal'
                    if route is not None:
                        full_clear,full_why=rc.path(np.asarray(route),.080)
                        row['original_native_model_path_recheck']=dict(clear=full_clear,collision=full_why,resolution_rad=rc.resolution_rad)
                        if not full_clear:route=None
                    if route is None:
                        row.setdefault('route_failures',[]).append(dict(endpoint_q=np.asarray(goal).tolist(),GPU_stats=dict(getattr(rc,'gpu_route_stats',{})),CPU_stats=dict(getattr(getattr(rc,'fast_planner',None),'stats',{}))))
                        continue
                    certificate=issued(rc,lc,lq2,route,cmd[rf],np.full(2,.040),np.linspace(min(wl,float(cmd[lf].sum())),max(wl,float(cmd[lf].sum())),5))
                    if certificate['clear']:
                        carry_cert=issued(lc,rc,actual[ri],path,cmd[lf],cmd[lf],np.linspace(min(wr,float(cmd[rf].sum())),max(wr,float(cmd[rf].sum())),5));result['carry'].update(clear=carry_cert['clear'],certificate=carry_cert)
                        if carry_cert['clear']:result['accepted']=dict(variant=variant,LEFT_carry_route=np.asarray(path).tolist(),LEFT_final_q=lq2.tolist(),RIGHT_route=np.asarray(route).tolist(),RIGHT_G=GR.tolist(),RIGHT_goal_q=np.asarray(goal).tolist(),certificate=certificate,RIGHT_route_kind=route_kind);break
                    row.setdefault('issued_failures',[]).append(certificate)
            save()
            if result['accepted']is not None:break
        save();return result
    finally:
        lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative,lc.loaded_contact_aperture=saved
        rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative=saved_right



def plan(c, part, measured_G, *, parked=False, relocated=False):
    """Original-model seeds and prior curves are proposals, never commands."""
    here=Path(__file__).resolve().parent;root=here.parents[1]/'runs'
    priors=_public_priors(here)
    known=priors['park_model'];left_hint=priors['LEFT_carry_model'];right_hint=priors['RIGHT_model_seed'];prior=priors['positive_RIGHT_sensor_observation']
    GR=np.linalg.inv(prior['hand_TCP_world'])@frame(prior['result']['pose']['center_m'],axis_frame(prior['result']['pose']['axis']))
    local=dict(c,_model_part_R=GR[:3,:3].copy(),_left_model_seed=np.asarray(left_hint['carry']['route'][-1]),_known_right_seed=np.asarray(right_hint),_park_role_cache={},_loaded_route_model_proposal=left_hint['carry']['route'],_empty_park_model_proposal=known['RIGHT_empty_park']['route'])
    arm=c['arm'];actual=np.asarray(c['art'].get_dof_positions()[0]);park=arm.fk(actual[c['ids']]);park[:3,3]=[-.05,-.55,1.25]
    goal=actual[c['ids']].copy()if parked else arm.solve(park[:3,3],park[:3,:3],known['RIGHT_empty_park']['q'])
    if goal is None:raise RuntimeError('Current RIGHT park target has no original-model seed solution')
    # An accepted397 model curve, if present, may accelerate current replan.
    local['_known_reverse_model_proposal']=priors['reverse_RIGHT_model_route']
    try:return _model_plan(local,part,measured_G,goal,parked=parked,relocated=relocated)
    finally:
        c['checkers']['right'].gpu_other_arm_proposal_scale=1.
        c['checkers']['right'].gpu_other_arm_full_scale_sources=()


def relocate(c, part, measured_G, *, issue, observe, payload, event):
    """Actual empty park and loaded holder carry, with fresh image replans."""
    from receiver_pad_contact import contact_state
    from grasp_transfer_feasibility import receiver_aperture
    G=np.asarray(measured_G,float).copy();left=c['left_arm'];li,lf=c['left_ids'],c['left_fi']
    def refresh(previous,label):
        current,pose=observe('left',part,previous,label)
        gap=float(c['art'].get_dof_positions()[0,lf].sum())
        clean=dict(c,config=dict(c['config'],receiver_axial_only=True))
        certificate=contact_state(clean,part,current,gap)
        translation=float(np.linalg.norm(current[:3,3]-previous[:3,3]));angle=float(np.arccos(np.clip(current[:3,2]@previous[:3,2],-1,1)))
        event('two_location_actual_holder_image',label=label,translation_change_m=translation,axis_change_rad=angle,contact=certificate)
        if not certificate['opposed_contact_compatible']or receiver_aperture(clean,part,current)is None or translation>.0015 or angle>.025:
            raise RuntimeError('Two-location held ring lacks fresh unchanged native contact; holder stays closed')
        return current
    first=plan(c,part,G)
    if first.get('accepted')is None:raise RuntimeError('No whole future-role plan before actual empty RIGHT park')
    event('two_location_whole_role_model_routes_proposed',model=first)
    if first.get('RIGHT_prepark_full_opening_required'):
        rq=c['art'].get_dof_positions()[0,c['ids']].copy()
        issue('right',[rq,rq],.080,'double_RIGHT_stationary_full_release',holder='left',Gholder=G)
        window=[];released=False
        for attempt in range(81):
            sample=c['art'].get_dof_positions()[0,c['fi']].copy();window.append(sample);window=window[-5:]
            if len(window)==5 and float(sample.sum())>=.0795 and np.max(np.ptp(window,axis=0))<=.00015 and np.ptp(np.asarray(window).sum(1))<=.00015:
                released=True;break
            for _ in range(int(np.ceil(.025/c['dt']))):c['step']('double_RIGHT_stationary_full_release_encoder_window')
        event('two_location_RIGHT_actual_full_release',confirmed=released,actual_opening_m=float(sample.sum()),stationary_samples=[x.tolist()for x in window])
        if not released:raise RuntimeError('Actual RIGHT did not reach stationary79.5mm full release; no park issued')
        G=refresh(G,'double_LEFT_after_RIGHT_full_release_RGBD')
        first=plan(c,part,G)
        if first.get('accepted')is None:raise RuntimeError('Fresh fully opened RIGHT geometry lacks whole future-role plan')
    issue('right',first['RIGHT_empty_park']['route'],.080,'double_RIGHT_actual_empty_park',holder='left',Gholder=G)
    G=refresh(G,'double_LEFT_after_actual_RIGHT_park_RGBD')
    second=plan(c,part,G,parked=True)
    if second.get('accepted')is None:raise RuntimeError('Fresh post-park geometry has no whole holder carry/regrasp plan')
    event('two_location_fresh_loaded_carry_model_routes_proposed',model=second)
    with payload(c['checkers']['left'],G):
        issue('left',second['accepted']['LEFT_carry_route'],float(c['cmd'][0,lf].sum()),'double_LEFT_actual_second_air_carry',holder='left',Gholder=G)
    G=refresh(G,'double_LEFT_after_actual_second_air_carry_RGBD')
    final=plan(c,part,G,parked=True,relocated=True)
    if final.get('accepted')is None:raise RuntimeError('Fresh actual second-air holder geometry has no native RIGHT approach')
    event('two_location_actual_second_air_fresh_RIGHT_plan',model=final)
    return G,final['accepted']
