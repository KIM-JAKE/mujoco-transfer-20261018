"""Model-only two-location handover, saved fresh camera+encoders only."""
from pathlib import Path
import json,time,numpy as np
from isaac_kinematics import frame,exp,log
from rgbd_geometry import axis_frame
from loaded_contact_aperture import LoadedContactAperture
from fr3_gpu_ik import propose_ik
from fr3_path_timing import retime_safe_polyline,retime
from grasp_transfer_feasibility import receiver_approach_route,donor_aperture,sequential_transfer_clear

def _execute_for_park(c, park_goal=None):
    here=Path(__file__).resolve().parent;r=here.parents[1]/'runs'/c['config']['handover_probe_source_run'];report=json.loads((r/'pipeline-results.json').read_text());obs=json.loads((r/'rgb-observations.json').read_text());o=max((v for v in obs if v['mode']=='held_left'and v.get('part')=='ring1'and v['result'].get('pose')),key=lambda v:v['captured_wall_s']);G=np.linalg.inv(o['hand_TCP_world'])@frame(o['result']['pose']['center_m'],axis_frame(o['result']['pose']['axis']));a=np.load(r/'executed-actuator-commands.npz');actual=a['actual_dof_position'][-1];cmd=a['command'][-1];ri,li,rf,lf=c['ids'],c['left_ids'],c['fi'],c['left_fi'];rq=cmd[ri];lq=cmd[li];left,right=c['left_arm'],c['arm'];rc,lc=c['checkers']['right'],c['checkers']['left'];wl=float(actual[lf].sum());wr=float(actual[rf].sum());saved=(lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative,getattr(lc,'loaded_contact_aperture',None));lc.attach_payload(c['part_collision_templates']['ring1'],frame([0,0,.1034],np.eye(3))@G);rc.detach_payload()
    bound=next(v for v in reversed(report['planning_diagnostics'])if v.get('event')=='loaded_receiver_encoder_geometry_bound')
    contact=bound['fresh_bilateral_CAD_contact'];binding=LoadedContactAperture(side='left',part='ring1',payload_path=lc.payload_path,encoder_samples=bound['measured_finger_samples_m'],sample_times_s=bound['sample_times_s'],contact=contact,measured_grasp=bound['measured_grasp'],control_dt_s=c['dt'],capture_age_physics_s=bound['capture_age_physics_s']);lc.loaded_contact_aperture=binding
    P=left.fk(actual[li])@G;P2=P.copy();P2[:3,3]=[-.30,-.30,1.35]
    if c.get('_model_part_R')is not None:P2[:3,:3]=np.asarray(c['_model_part_R'])
    target=P2@np.linalg.inv(G);start=left.fk(lq);result=dict(state='empty_RIGHT_park_then_two_location_holder_model_preflight',source=r.name,robot_motion_executed=False,uses_free_body_poses=False,full_task_pass=False,incoming_P=P.tolist(),second_air_P=P2.tolist(),LEFT_measured_G=G.tolist(),source_loaded_contact_record=bound,carry=None,receiver_candidates=[],accepted=None,fresh_RGBD_and_replan_required_after_actual_carry=True);out=c['vision'].out/'two-location-handover-preflight.json'
    def save():out.write_text(json.dumps(result,indent=2))
    def issued(checker,other,other_q,path,own_start,own_target,other_widths):
        from dual_aperture_path_check import rows_dual_openings
        qpath=np.asarray(path,float)
        if len(qpath)>2:qs,h,timing=retime_safe_polyline(qpath,c['dt'],own_start,own_target,scale=1.)
        else:qs,h,timing=retime(qpath,.4,c['dt'],own_start,own_target,optimize=bool(c['config'].get('optimized_motion_timing')))
        widths=(own_start+(own_target-own_start)*h[:,None]).sum(1);stats=[]
        # Exact same25 reference contexts as369;351b proved reuse only of
        # geometry independent of the opposite native-finger width.
        contexts=np.linspace(0.,1.,5)if checker is lc else [1.]
        for alpha in contexts:
            native_widths=wl*(1-alpha)+widths[1:]*alpha if checker is lc else widths[1:]
            info={};ok,why=rows_dual_openings(checker,other,other_q,other_widths,qs[1:],native_widths,diagnostics=info);stats.append(info)
            if not ok:return dict(clear=False,collision=why)
        return dict(clear=True,control_rows=len(qs)-1,other_apertures=len(other_widths),own_loaded_width_contexts=len(contexts),timing=timing,exact351b_reuse=stats)

    try:
        lc.resolution_rad=.002;rc.resolution_rad=.002
        # Cheap ORIGINAL-FK-admitted endpoint gates precede protected routes.
        left_goals=c['_park_role_cache'].get('left_goals')
        if left_goals is None:
            candidate=left.solve(target[:3,3],target[:3,:3],c['_left_model_seed'])
            left_goals=[]if candidate is None else[candidate];c['_park_role_cache']['left_goals']=left_goals
        result['LEFT_endpoint_proposals']=len(left_goals)
        if not left_goals:
            endpoint=left.solve(target[:3,3],target[:3,:3],actual[li])
            if endpoint is not None:left_goals=[endpoint]
        if not left_goals:result['failure']='No current-family LEFT endpoint IK at robotwardY second airpoint';save();return result
        park=right.fk(rq);park[:3,3]=[-.05,-.55,1.25]
        park_goals=c['_park_role_cache'].get('park_goals')
        if park_goals is None:
            park_goals=propose_ik(c,right,park[:3,3],park[:3,:3],actual[ri],'empty_RIGHT_clear_holder_carry');c['_park_role_cache']['park_goals']=park_goals
        if park_goal is not None:park_goals=[np.asarray(park_goal,float)]
        else:park_goals=park_goals[:1]
        result['RIGHT_park_endpoint_proposals']=len(park_goals);parked=None
        with rc.other_arm_at(lc,actual[li],wl):
            for goal in park_goals:
                route=None
                model_hint=c.get('_empty_park_model_proposal')
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
        branch=int(c['_forced_left_goal_index']);result['LEFT_branch_index']=branch
        if branch>=len(left_goals):
            result['failure']='No cached LEFT endpoint at requested index';save();return result
        goal=np.asarray(left_goals[branch])
        with lc.other_arm_at(rc,actual[ri],max(wr,float(cmd[rf].sum()))):
            legal,why=lc.clear(goal,float(cmd[lf].sum()))
            if not legal:
                result['failure']='Selected LEFT endpoint violates original geometry';result['LEFT_endpoint_collision']=why;save();return result
            candidate=None
            prior_path=c.get('_loaded_route_model_proposal')
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
        lq2=path[-1];prior=json.loads((here.parents[1]/'runs/native-roas-table-scatter-stable-v274-gpu-route-pin-yaw-all3-446/rgb-observations.json').read_text())[49];raw=np.linalg.inv(prior['hand_TCP_world'])@frame(prior['result']['pose']['center_m'],axis_frame(prior['result']['pose']['axis']))
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
                    search_started=time.perf_counter();route=planner.route(rq,goal,.080,10.)
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
    finally:lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative,lc.loaded_contact_aperture=saved


def execute(c):
    source=Path(__file__).resolve().parent.parents[1]/'runs/native-roas-table-scatter-stable-v369-whole-role-park-preflight-3491/whole-role-park-preflight.json'
    historical=json.loads(source.read_text());known=historical['branches'][3];seed=np.asarray(known['RIGHT_empty_park']['q'])
        # Same current-target original-FK seed, never an issued recorded pose.
    arm=c['arm'];target=arm.fk(seed);source_encoders=np.load(source.parent.parent/c['config']['handover_probe_source_run']/'executed-actuator-commands.npz');expected=arm.fk(source_encoders['command'][-1,c['ids']]);expected[:3,3]=[-.05,-.55,1.25]
    if np.linalg.norm(target[:3,3]-expected[:3,3])>1e-5 or np.linalg.norm(log(target[:3,:3]@expected[:3,:3].T))>1e-4:raise RuntimeError('Known park seed differs from current explicit park target')
    goal=arm.solve(expected[:3,3],expected[:3,:3],seed)
    if goal is None:raise RuntimeError('Known model park seed no longer satisfies original FK')
    c['_park_role_cache']={'park_goals':[goal]}
    # ONE physical held-assembly orientation, not a coordinate relabel:
    # H_RIGHT = P2 @ inv(G_RIGHT), so P2.R=G_RIGHT.R gives H_RIGHT.R=I.
    prior=json.loads((source.parent.parent/'native-roas-table-scatter-stable-v274-gpu-route-pin-yaw-all3-446/rgb-observations.json').read_text())[49]
    GR=np.linalg.inv(prior['hand_TCP_world'])@frame(prior['result']['pose']['center_m'],axis_frame(prior['result']['pose']['axis']))
    c['_model_part_R']=GR[:3,:3].copy()
    reference=Path(__file__).resolve().parent.parents[1]/'runs/native-roas-table-scatter-stable-v381-left-elbow-whole-role-3491/whole-LEFT-role-preflight.json'
    admitted=json.loads(reference.read_text())['branches'][2]
    c['_left_model_seed']=np.asarray(admitted['carry']['route'][-1])
    c['_loaded_route_model_proposal']=admitted['carry']['route']
    c['_empty_park_model_proposal']=known['RIGHT_empty_park']['route']
    c['_forced_left_goal_index']=0
    known_right_record=Path(__file__).resolve().parent.parents[1]/'runs/native-roas-table-scatter-stable-v395-boundary-EMPTY-reverse-role-3491/free-EMPTY-approach-role-preflight.json'
    known_right=json.loads(known_right_record.read_text())['branch']['receiver_candidates'][0]['native_clear_endpoint_qs'][0]
    c['_known_right_seed']=np.asarray(known_right)
    try:
        result=_execute_for_park(c,goal)
        output=dict(state='ONE_native_clear_endpoint_ORIGINAL_FCL_10s_model_preflight',source_reference=str(reference),robot_motion_executed=False,uses_free_body_poses=False,full_task_pass=False,accepted=result.get('accepted'),complete_model_pass=result.get('accepted')is not None,branch=result,proposal_only_other_robot_OBB_scale=.85,original_full_CAD_self_pair_margin_and_exact_issued_rows_unchanged=True)
        (c['vision'].out/'one-FCL-role-preflight.json').write_text(json.dumps(output,indent=2));return output
    finally:
        c['checkers']['right'].gpu_other_arm_proposal_scale=1.
        c['checkers']['right'].gpu_other_arm_full_scale_sources=()
