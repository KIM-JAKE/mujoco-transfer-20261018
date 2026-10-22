"""SIM component: stationary axial-face holder -> positive-axis RIGHT regrasp.

Only saved public RGB-D/encoder grasp priors, current RGB-D and robot encoders
enter targets. No object actor poses, attachments, welds, mass or force changes.
"""
from pathlib import Path
from contextlib import contextmanager
import json,time,types
import numpy as np
from isaac_kinematics import frame
from rgbd_geometry import axis_frame
from grasp_transfer_feasibility import sequential_transfer_clear,receiver_approach_route,StagedReceiverRoute
from receiver_pad_contact import contact_state
from loaded_contact_aperture import LoadedContactAperture

def _endpoint_candidates(c,arm,target,current,seeds,label,seed_key=None):
    """Optional model seed, then GPU; every proposal retains original guards."""
    from fr3_gpu_ik import propose_ik
    seen=[]
    if seed_key is not None and seed_key in c['config']:
        seed=np.asarray(c['config'][seed_key],float)
        if seed.shape!=(7,)or not np.isfinite(seed).all()or np.any(seed<arm.bounds[:,0])or np.any(seed>arm.bounds[:,1]):raise RuntimeError('Invalid original-model receiver IK seed')
        #Re-solve the CURRENT measured target. Never issue a recorded q.
        candidate=arm.solve(target[:3,3],target[:3,:3],seed)
        if candidate is not None:
            seen.append(candidate);yield candidate
    for q in propose_ik(c,arm,target[:3,3],target[:3,:3],current,label):
        seen.append(q);yield q
    for seed in seeds:
        q=arm.solve(target[:3,3],target[:3,:3],seed)
        if q is not None and all(np.linalg.norm(q-old)>.05 for old in seen):
            seen.append(q);yield q

def prepare_holder(c,part,donor_G):
    """Actually move empty LEFT before recomputing the donor handover plan."""
    from grasp_transfer_feasibility import receiving_poses,donor_aperture,large_donor_pad_support
    if part!='ring1':raise RuntimeError('Temporary holder requires LARGE class')
    art=c['art'];cmd=c['cmd'];left=c['left_arm'];lc,rc=c['checkers']['left'],c['checkers']['right'];li,ri=c['left_ids'],c['ids'];lf,rf=c['left_fi'],c['fi'];template=c['part_collision_templates'][part]
    if not c.get('left_hand_known_empty',False):raise RuntimeError('LEFT must be known empty before preposition')
    P=np.asarray(c['handoff_reference'][part]['part_H']).copy();P[:3,:3]=np.eye(3);P[:3,3]+=[-.15,0.,-.20]
    if 'temporary_axial_air_part_center_m'in c['config']:
        center=np.asarray(c['config']['temporary_axial_air_part_center_m'],dtype=float)
        if center.shape!=(3,)or not np.all(np.isfinite(center)):raise RuntimeError('Temporary air center must be three finite calibrated world coordinates')
        P[:3,3]=center
    target,_=next(receiving_poses(c,part,P));pre=target.copy();pre[:3,3]-=pre[:3,2]*.020
    actual=art.get_dof_positions()[0];r_actual=actual[ri].copy();right_actual=float(actual[rf].sum());right_command=float(cmd[0,rf].sum());opening=.0235
    seeds=[actual[li].copy(),np.asarray(c['handoff_reference'][part]['left_receive_q'])]+list(np.random.default_rng(317).uniform(left.bounds[:,0],left.bounds[:,1],(8,7)))
    def preposition_proposals():
        # A previous original-model role proof is a seed, never a replayed
        # command. Solve the CURRENT target with the original arm model;
        # every ensuing endpoint/route/role guard remains mandatory.
        if 'temporary_axial_preposition_seed'in c['config']:
            seed=np.asarray(c['config']['temporary_axial_preposition_seed'],float)
            if seed.shape!=(7,)or not np.isfinite(seed).all()or np.any(seed<left.bounds[:,0])or np.any(seed>left.bounds[:,1]):raise RuntimeError('Invalid temporary-holder original-model IK seed')
            candidate=left.solve(pre[:3,3],pre[:3,:3],seed)
            if candidate is not None:yield candidate
        yield from _endpoint_candidates(c,left,pre,actual[li],seeds,'temporary_LEFT_standoff')
    previews=[]
    preview_path=c['vision'].out/'temporary-holder-preposition-role-preview.json'
    issued=None;rejections=[];saved=(rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative);rc.attach_payload(template,frame([0,0,.1034],np.eye(3))@donor_G)
    try:
        for goal in preposition_proposals():
            with lc.other_arm_at(rc,r_actual,max(right_actual,right_command)):
                clear,why=lc.clear(goal,opening)
                if not clear:rejections.append(why);continue
                route=lc.route(cmd[0,li],goal,opening,seconds=.5)
            if route is None:continue
            rows=c['component_trajectory']('left',route,.4,opening,checked_edges=True);accepted=True
            for other_width in np.linspace(min(right_actual,right_command),max(right_actual,right_command),5):
                with lc.other_arm_at(rc,r_actual,float(other_width)):
                    for index,row in enumerate(rows):
                        clear,why=lc.clear(row[:7],float(row[7:].sum()))
                        if not clear:rejections.append(dict(row=index,other_opening_m=float(other_width),collision=why));accepted=False;break
                if not accepted:break
            if not accepted:continue
            # An empty-clear elbow can still block ALL future RIGHT donor
            # endpoints. Model this particular LEFT candidate before moving;
            # this changes no actual robot state. After actual preposition,
            # the caller STILL refreshes RGB-D and replans from encoders.
            prospective_cmd=cmd.copy();prospective_cmd[0,li]=goal;prospective_cmd[0,lf]=opening/2
            prospective_actual=art.get_dof_positions().copy();prospective_actual[0,li]=goal;prospective_actual[0,lf]=opening/2
            class ProspectiveEncoders:
                def get_dof_positions(self):return prospective_actual
            prospective=dict(c,cmd=prospective_cmd,art=ProspectiveEncoders(),config=dict(c['config'],handover_plan_budget_s=min(15.,float(c['config'].get('handover_plan_budget_s',15.)))))
            planner=types.ModuleType('temporary_holder_role_preview');planner.__file__=str(Path(__file__).resolve().parent/'paired_handover.py');exec(compile(Path(planner.__file__).read_text(),planner.__file__,'exec'),planner.__dict__)
            payload_before_preview=(rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative)
            evidence=dict(model_only=True,robot_motion_executed=False,uses_free_body_poses=False,proposed_LEFT_q=goal.tolist(),air_part_P=P.tolist(),actual_move_and_fresh_replan_still_required=True)
            try:
                with rc.other_arm_at(lc,goal,opening):proposal=planner.plan(prospective,part,donor_G,donor_release_width=.080)
                evidence.update(complete_future_pair_route_admitted=True,donor_route=np.asarray(proposal['donor_route']).tolist(),receiver_route=np.asarray(proposal['receiver_route']).tolist(),receiver_q=np.asarray(proposal['receiver_q']).tolist())
            except RuntimeError as error:
                evidence.update(complete_future_pair_route_admitted=False,reason=str(error));rejections.append(dict(prospective_pair_rejected=str(error)))
            finally:rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative=payload_before_preview
            previews.append(evidence);preview_path.write_text(json.dumps(previews,indent=2));print('DOUBLE_HANDOVER_PREPOSITION_ROLE_PREVIEW',{k:v for k,v in evidence.items()if k not in ['donor_route','receiver_route']},flush=True)
            if not evidence['complete_future_pair_route_admitted']:continue
            issued=rows;break
    finally:rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative=saved
    if issued is None:raise RuntimeError('No full-guard actual empty LEFT preposition '+str(rejections[:4]))
    for row in issued:cmd[0,li]=row[:7];cmd[0,lf]=row[7:];c['step']('temporary_LEFT_actual_empty_preposition')
    new_G,pose=c['component_observe']('right',part,donor_G,'RIGHT_after_actual_LEFT_preposition_RGBD')
    clean=dict(c,config=dict(c['config'],receiver_axial_only=False));width=float(art.get_dof_positions()[0,rf].sum());contact=contact_state(clean,part,new_G,width);support=large_donor_pad_support(clean,new_G,width);translation=float(np.linalg.norm(new_G[:3,3]-donor_G[:3,3]));angle=float(np.arccos(np.clip(new_G[:3,2]@donor_G[:3,2],-1,1)))
    evidence=dict(event='temporary_LEFT_actual_empty_preposition_verified',part=part,robot_motion_executed=True,physical_motion_s=len(issued)*c['dt'],air_part_P=P.tolist(),actual_LEFT_TCP=left.fk(art.get_dof_positions()[0,li]).tolist(),RIGHT_grip_translation_change_m=translation,RIGHT_grip_axis_change_rad=angle,RIGHT_contact=contact,RIGHT_support=support,endpoint_rejections=rejections[:4],uses_free_body_poses=False)
    c['planning_diagnostics'].append(evidence);print('DOUBLE_HANDOVER_PREPOSITION',evidence,flush=True)
    if not contact['opposed_contact_compatible']or not support['broad_opposed_flat_pad_support']or donor_aperture(clean,part,new_G)is None or translation>.002 or angle>.05:
        raise RuntimeError('Loaded RIGHT did not retain its validated grasp during actual empty LEFT preposition')
    return new_G

def execute(c,part,left_G):
    if part!='ring1':raise RuntimeError('Double-handover component currently requires LARGE class')
    art=c['art'];cmd=c['cmd'];dt=c['dt'];arms={'left':c['left_arm'],'right':c['arm']};ids={'left':c['left_ids'],'right':c['ids']};fi={'left':c['left_fi'],'right':c['fi']};checkers=c['checkers'];template=c['part_collision_templates'][part];events=[]
    move=c['component_move'];observe=c['component_observe'];trajectory=c['component_trajectory'];step=c['step'];bindings=c['component_bindings']
    def q(side):return art.get_dof_positions()[0,ids[side]].copy()
    def w(side):return float(art.get_dof_positions()[0,fi[side]].sum())
    def wait(duration,label):
        for _ in range(int(np.ceil(duration/dt))):step(label)
    def event(name,**data):
        row=dict(event=name,wall_s=time.perf_counter()-c['started'],uses_free_body_poses=False,**data);events.append(row);c['planning_diagnostics'].append(row);print('DOUBLE_HANDOVER',row,flush=True)
    @contextmanager
    def payload(checker,G):
        saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative)
        checker.attach_payload(template,frame([0,0,.1034],np.eye(3))@G)
        try:yield
        finally:checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved
    @contextmanager
    def no_payload(checker):
        saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative);checker.detach_payload()
        try:yield
        finally:checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved
    def issue(side,path,width,label,*,holder=None,Gholder=None):
        """Check every exact retimed command with real fixed/pair/payload scope."""
        rows=trajectory(side,path,.4,width,checked_edges=True);other='left'if side=='right'else'right';checker=checkers[side];otherchecker=checkers[other]
        context=payload(otherchecker,Gholder)if holder==other else no_payload(otherchecker)
        with context:
            other_actual=w(other);other_command=float(cmd[0,fi[other]].sum())
            other_openings=np.linspace(min(other_actual,other_command),max(other_actual,other_command),5)
            actual_own=q(side);actual_fingers=art.get_dof_positions()[0,fi[side]].copy()
            guarded_rows=np.vstack([np.r_[actual_own,actual_fingers],np.r_[cmd[0,ids[side]],cmd[0,fi[side]]],np.asarray(rows)])
            other_states=[q(other),cmd[0,ids[other]].copy()]
            if np.max(abs(other_states[0]-other_states[1]))<1e-12:other_states=other_states[:1]
            if c['config'].get('reuse_actual_other_context_checks',False):
                from dual_aperture_path_check import rows_other_contexts,path_other_contexts
                contexts=[(other_q,float(opening))for other_q in other_states for opening in other_openings]
                statistics={};ok,why=rows_other_contexts(checker,otherchecker,contexts,guarded_rows[:,:7],guarded_rows[:,7:].sum(axis=1),diagnostics=statistics)
                event('double_exact_issued_other_context_reuse',label=label,statistics=statistics,clear=ok)
                if not ok:raise RuntimeError('Double-handover exact issued robot/pair/held-CAD blocked '+str(dict(label=label,**why)))
                if holder==side:
                    native_values=np.r_[actual_fingers.sum(),guarded_rows[:,7:].sum(axis=1)]
                    native_widths=np.unique(np.linspace(native_values.min(),native_values.max(),5))
                    stats={};ok,why=path_other_contexts(checker,otherchecker,contexts,guarded_rows[:,:7],native_widths,diagnostics=stats)
                    event('double_loaded_actual_requested_native_context_guard',label=label,own_openings_m=native_widths.tolist(),statistics=stats,clear=ok)
                    if not ok:raise RuntimeError('Loaded double-handover actual/requested native-width sweep blocked '+str(why))
            else:
                for other_q in other_states:
                    if c['config'].get('reuse_dual_aperture_issued_checks',False):
                        from dual_aperture_path_check import rows_dual_openings
                        statistics={};ok,why=rows_dual_openings(checker,otherchecker,other_q,other_openings,guarded_rows[:,:7],guarded_rows[:,7:].sum(axis=1),diagnostics=statistics)
                        event('double_exact_issued_aperture_reuse',label=label,other_q=other_q.tolist(),statistics=statistics,clear=ok)
                        if not ok:raise RuntimeError('Double-handover exact issued robot/pair/held-CAD blocked '+str(dict(label=label,**why)))
                        if holder==side:
                            from dual_aperture_path_check import path_dual_openings
                            native_values=np.r_[actual_fingers.sum(),guarded_rows[:,7:].sum(axis=1)]
                            native_widths=np.unique(np.linspace(native_values.min(),native_values.max(),5))
                            stats={};ok,why=path_dual_openings(checker,otherchecker,other_q,other_openings,guarded_rows[:,:7],native_widths,diagnostics=stats)
                            event('double_loaded_actual_requested_native_width_guard',label=label,own_openings_m=native_widths.tolist(),other_q=other_q.tolist(),statistics=stats,clear=ok)
                            if not ok:raise RuntimeError('Loaded double-handover actual/requested native-width sweep blocked '+str(why))
                    else:
                        for other_opening in other_openings:
                            with checker.other_arm_at(otherchecker,other_q,float(other_opening)):
                                for i,r in enumerate(guarded_rows):
                                    ok,why=checker.clear(r[:7],float(r[7:].sum()))
                                    if not ok:raise RuntimeError('Double-handover exact issued robot/pair/held-CAD blocked '+str(dict(label=label,row=i,other_opening_m=float(other_opening),collision=why)))
        for r in rows:cmd[0,ids[side]]=r[:7];cmd[0,fi[side]]=r[7:];step(label)
        event(label,control_steps=len(rows),physical_motion_s=len(rows)*dt)
    def shared_stationary_sweep(P,widths):
        """Selected-ring distal contact only; BOTH full arms/palms/proximal strict.

        The duplicated held CAD is omitted only from the cross-arm set; each
        actual arm independently checks it with its original distal-pad-only
        OwnPayloadClearance. Robot-vs-robot still checks ALL complete fingers.
        """
        qs={s:q(s)for s in arms}
        for wl,wr in widths:
            with no_payload(checkers['left']),no_payload(checkers['right']):
                with checkers['right'].other_arm_at(checkers['left'],qs['left'],wl):
                    ok,why=checkers['right'].clear(qs['right'],wr)
                    if not ok:raise RuntimeError('Double-handover stationary full robot/pair/fixed sweep blocked '+str(why))
            for side,width in [('left',wl),('right',wr)]:
                G=np.linalg.inv(arms[side].fk(qs[side]))@P
                with payload(checkers[side],G):
                    old_binding=getattr(checkers[side],'loaded_contact_aperture',None);checkers[side].loaded_contact_aperture=None
                    other='left'if side=='right'else'right';other_width=wl if other=='left'else wr
                    try:
                        # Full robot pair was checked above. Here the selected
                        # ring is checked against each own arm/palm/proximal
                        # independently, retaining ONLY its distal-pad contact.
                        # Skip duplicate opposite-holder actors in the fixed
                        # query; their full geometry was covered by the pair
                        # check and the other own-ring check, not discarded.
                        with checkers[side].other_arm_at(checkers[other],qs[other],other_width):
                            ok,why=checkers[side].same_arm_clearance.clear(qs[side],width)
                            if ok:
                                transforms=arms[side].bodies(qs[side],width)
                                transforms[checkers[side].payload_path]=transforms[arms[side].hand_path]@checkers[side].payload_relative
                                ok,why=checkers[side]._clear_transforms(transforms,None)
                    finally:checkers[side].loaded_contact_aperture=old_binding
                    if not ok:raise RuntimeError('Double-handover selected-CAD vs full arm/palm/proximal/fixed blocked '+str(dict(side=side,collision=why)))
    left_G,left_pose=observe('left',part,left_G,'double_holder_fresh_RGBD');P=arms['left'].fk(q('left'))@left_G
    if not contact_state(dict(c,config=dict(c['config'],receiver_axial_only=True)),part,left_G,w('left'))['opposed_contact_compatible']:
        raise RuntimeError('Temporary LEFT axial holder lost fresh bilateral geometry')
    relocation=None
    if c['config'].get('temporary_axial_two_location_handover',False):
        from two_location_native_handover import relocate
        left_G,relocation=relocate(c,part,left_G,issue=issue,observe=observe,payload=payload,event=event)
        P=arms['left'].fk(q('left'))@left_G
    # Prior is a successful public RGB-D + Hand encoder grasp, not an actor pose.
    here=Path(__file__).resolve().parent;source=here.parents[1]/'runs/native-roas-table-scatter-stable-v274-gpu-route-pin-yaw-all3-446/rgb-observations.json';prior=json.loads(source.read_text())[49]
    right_G=np.linalg.inv(np.asarray(prior['hand_TCP_world']))@frame(prior['result']['pose']['center_m'],axis_frame(prior['result']['pose']['axis']))
    if relocation is not None:right_G=np.asarray(relocation['RIGHT_G'],float).copy()
    from grasp_transfer_feasibility import donor_aperture
    clean=dict(c,config=dict(c['config'],receiver_axial_only=False))
    expected=donor_aperture(clean,part,right_G)
    if expected is None or right_G[2,2]<=.1:raise RuntimeError('Saved positive-axis RIGHT prior lacks current native class support')
    right_target=P@np.linalg.inv(right_G);left_target=arms['left'].fk(q('left'))
    if not sequential_transfer_clear(checkers['left'],left_target,w('left'),checkers['right'],right_target,expected,.080,donor_release_width=.0235):
        raise RuntimeError('Fresh temporary-holder/RIGHT regrasp sequential native pair is blocked')
    seeds=[q('right'),np.asarray(c['handoff_reference'][part]['right_seed'])]+list(np.random.default_rng(317).uniform(arms['right'].bounds[:,0],arms['right'].bounds[:,1],(12,7)))
    route=None;positive_endpoint_proposals=[]
    if relocation is not None:
        model_goal=np.asarray(relocation['RIGHT_goal_q'],float)
        current_goal=arms['right'].solve(right_target[:3,3],right_target[:3,:3],model_goal)
        if current_goal is None:raise RuntimeError('Fresh actual RIGHT receiver target lost original-model IK')
        hint=[np.asarray(row,float).copy()for row in relocation['RIGHT_route']];hint[0]=cmd[0,ids['right']].copy();hint[-1]=current_goal.copy()
        with payload(checkers['left'],left_G),checkers['right'].other_arm_at(checkers['left'],q('left'),w('left')):
            old=checkers['right'].resolution_rad;checkers['right'].resolution_rad=min(old,.002)
            try:hint_clear,hint_why=checkers['right'].path(hint,.080)
            finally:checkers['right'].resolution_rad=old
        if not hint_clear:raise RuntimeError('Fresh actual second-air RIGHT proposal original native path blocked '+str(hint_why))
        # Free-joint approach has a checked inspection waypoint before its
        # last edge. Fresh image feedback still retargets BEFORE closure.
        if len(hint)==2:
            hint.insert(1,(hint[0]+hint[1])/2)
            with payload(checkers['left'],left_G),checkers['right'].other_arm_at(checkers['left'],q('left'),w('left')):
                if not checkers['right'].path(hint,.080)[0]:raise RuntimeError('Fresh inspection split changed native route clearance')
        route=StagedReceiverRoute(hint,len(hint)-2)
        event('two_location_current_RIGHT_model_hint_admitted',joint_knots=len(route),whole_original_geometry=True,actual_motion_not_yet_issued=True)
    for target_q in ([]if route is not None else _endpoint_candidates(c,arms['right'],right_target,q('right'),seeds,'double_RIGHT_positive_receiver',seed_key='temporary_axial_right_receiver_seed')):
        positive_endpoint_proposals.append(np.asarray(target_q).copy())
        with payload(checkers['left'],left_G),checkers['right'].other_arm_at(checkers['left'],q('left'),w('left')):
            route=receiver_approach_route(c,'double_RIGHT_positive_receiver',checkers['right'],cmd[0,ids['right']],target_q,.080,seconds=.5)
        if route is not None:break
    if route is None:
        #343's final/standoff/terminal were clear, but a direct wrist flip
        #swept the palm through fixedM80/0157. Reorient the EMPTY hand in
        #free air first. This is a fresh IK/route proposal, not recorded q.
        via=right_target.copy();via[:3,3]=[0.,-.35,1.40]
        if 'temporary_axial_right_via_center_m'in c['config']:
            position=np.asarray(c['config']['temporary_axial_right_via_center_m'],float)
            if position.shape!=(3,)or not np.isfinite(position).all():raise RuntimeError('Finite calibrated empty RIGHT via position required')
            via[:3,3]=position
        checker=checkers['right'];other=checkers['left'];fallback_evidence=[]
        with payload(other,left_G),checker.other_arm_at(other,q('left'),w('left')):
            old_resolution=getattr(checker,'resolution_rad',.035);checker.resolution_rad=min(old_resolution,.002)
            try:
                for via_q in _endpoint_candidates(c,arms['right'],via,q('right'),seeds,'double_RIGHT_empty_positive_via',seed_key='temporary_axial_right_via_seed'):
                    ok,why=checker.clear(via_q,.080)
                    if not ok:fallback_evidence.append(dict(stage='via_endpoint',collision=why));continue
                    prefix=checker.route(cmd[0,ids['right']],via_q,.080,seconds=.5)
                    if prefix is None:fallback_evidence.append(dict(stage='protected_prefix_route'));continue
                    #Concatenation deliberately discards cubic semantics:
                    #recheck the original joint edges and later retime those
                    #same edges, including the terminal corner stops.
                    prefix=[np.asarray(value).copy()for value in prefix]
                    if not checker.path(prefix,.080)[0]:continue
                    for target_q in positive_endpoint_proposals:
                        suffix=receiver_approach_route(c,'double_RIGHT_positive_from_via',checker,via_q,target_q,.080,seconds=.5)
                        if suffix is None:continue
                        combined=prefix+list(suffix)[1:]
                        ok,why=checker.path(combined,.080)
                        if not ok:fallback_evidence.append(dict(stage='combined_original_edges',collision=why));continue
                        route=StagedReceiverRoute(combined,len(prefix)-1+getattr(suffix,'standoff_index',len(suffix)-1));event('double_RIGHT_guarded_empty_via_route',via_TCP=via.tolist(),via_q=np.asarray(via_q).tolist(),joint_knots=len(route),model_only_until_issue=True,full_original_fixed_self_pair_selected_CAD=True);break
                    if route is not None:break
            finally:checker.resolution_rad=old_resolution
        (c['vision'].out/'double-positive-via-route-diagnostics.json').write_text(json.dumps(dict(accepted=route is not None,via_target=via.tolist(),rejections=fallback_evidence,route=None if route is None else np.asarray(route).tolist(),robot_motion_executed=False,uses_free_body_poses=False,original_exact_issued_row_checks_still_required=True),indent=2))
    if route is None:raise RuntimeError('No full fixed/pair/selected-CAD clear RIGHT positive receiver approach')
    if c.get('set_empty_donor_contact_guard'):c['set_empty_donor_contact_guard']('right',False)
    if c['config'].get('refresh_reverse_handover_standoff',False):
        index=getattr(route,'standoff_index',None)
        if index is None or not 0<=index<len(route)-1:raise RuntimeError('Fresh reverse approach requires checked stand-off metadata')
        proposed_goal=np.asarray(route[-1]).copy();old_target=right_target.copy()
        issue('right',list(route[:index+1]),.080,'double_RIGHT_positive_standoff',holder='left',Gholder=left_G)
        previous_G=left_G.copy();left_G,left_pose=observe('left',part,left_G,'double_holder_fresh_at_RIGHT_standoff_RGBD')
        holder_contact=contact_state(dict(c,config=dict(c['config'],receiver_axial_only=True)),part,left_G,w('left'))
        if not holder_contact['opposed_contact_compatible']:raise RuntimeError('Fresh stand-off image does not confirm temporary holder contact')
        P=arms['left'].fk(q('left'))@left_G;right_target=P@np.linalg.inv(right_G)
        displacement=float(np.linalg.norm(right_target[:3,3]-old_target[:3,3]));rotation=float(np.arccos(np.clip((np.trace(old_target[:3,:3].T@right_target[:3,:3])-1)/2,-1,1)))
        if displacement>.003 or rotation>.05:raise RuntimeError('Temporary holder drift exceeds bounded fresh terminal replan')
        goal=arms['right'].solve(right_target[:3,3],right_target[:3,:3],proposed_goal)
        if goal is None:raise RuntimeError('Fresh terminal RIGHT target has no original-model IK')
        with payload(checkers['left'],left_G),checkers['right'].other_arm_at(checkers['left'],q('left'),w('left')):
            if relocation is not None:
                refreshed=checkers['right'].route(cmd[0,ids['right']],goal,.080,seconds=.7)
                if refreshed is not None and not checkers['right'].path(np.asarray(refreshed),.080)[0]:refreshed=None
            else:refreshed=receiver_approach_route(c,'double_RIGHT_fresh_terminal',checkers['right'],cmd[0,ids['right']],goal,.080,seconds=.5)
        if refreshed is None:raise RuntimeError('Fresh observed RIGHT terminal approach blocked by original full geometry')
        event('double_RIGHT_standoff_fresh_terminal_admitted',target_translation_change_m=displacement,target_rotation_change_rad=rotation,holder_contact=holder_contact,exact_issued_checks_still_required=True)
        issue('right',refreshed,.080,'double_RIGHT_fresh_terminal_approach',holder='left',Gholder=left_G)
    else:
        issue('right',route,.080,'double_RIGHT_positive_approach',holder='left',Gholder=left_G)
    left_G,left_pose=observe('left',part,left_G,'double_holder_before_RIGHT_close_RGBD');P=arms['left'].fk(q('left'))@left_G
    fresh_right_G=np.linalg.inv(arms['right'].fk(q('right')))@P;expected=donor_aperture(clean,part,fresh_right_G)
    if expected is None:raise RuntimeError('Fresh camera does not confirm proposed RIGHT opposing pad geometry')
    closing=max(.001,expected-.003)
    shared_stationary_sweep(P,[(w('left'),gap)for gap in np.linspace(.080,closing,9)])
    c['set_grip_hold']('right',True)
    # The arm is stationary; only a finite native single-width motor closes.
    close_rows=trajectory('right',[cmd[0,ids['right']].copy()]*2,.15,closing,checked_edges=True)
    for r in close_rows:cmd[0,ids['right']]=r[:7];cmd[0,fi['right']]=r[7:];step('double_RIGHT_close')
    window=[];times=[];settled=False
    for _ in range(120):
        window.append(art.get_dof_positions()[0,fi['right']].copy());times.append(time.perf_counter());window=window[-5:]
        gap=w('right');stationary=len(window)==5 and np.max(np.ptp(window,axis=0))<=.00015 and np.ptp(np.asarray(window).sum(1))<=.00015
        if stationary and closing+.0005<gap<expected+.002:settled=True;break
        wait(.025,'double_RIGHT_contact_plateau')
    if not settled:raise RuntimeError('Positive RIGHT grasp failed bounded native encoder plateau; LEFT stays closed')
    right_G,right_pose=observe('right',part,fresh_right_G,'double_RIGHT_bilateral_contact_RGBD')
    contact=contact_state(clean,part,right_G,w('right'));event('double_RIGHT_bilateral_contact',contact=contact,measured_grasp=right_G.tolist())
    if not contact['opposed_contact_compatible']:raise RuntimeError('Fresh RIGHT bilateral pads not confirmed; LEFT stays closed')
    c['set_grip_hold']('right',True,preload_m_s=float(c['config'].get('native_loaded_preload_after_contact_m_s',.015)))
    wait(float(c['config'].get('native_loaded_preload_ramp_s',0))+.05,'double_RIGHT_existing_force_ramp')
    right_G,right_pose=observe('right',part,right_G,'double_RIGHT_force_confirmed_RGBD');P=arms['right'].fk(q('right'))@right_G
    contact=contact_state(clean,part,right_G,w('right'))
    if not contact['opposed_contact_compatible']:raise RuntimeError('RIGHT opposing contact lost during original force ramp')
    shared_stationary_sweep(P,[(gap,wr)for gap in np.linspace(w('left'),.0235,9)for wr in np.linspace(min(float(cmd[0,fi['right']].sum()),w('right')),max(float(cmd[0,fi['right']].sum()),w('right')),5)])
    binding=bindings.pop('left',None)
    if binding is not None:binding.release()
    checkers['left'].loaded_contact_aperture=None;c['set_grip_hold']('left',False)
    open_rows=trajectory('left',[cmd[0,ids['left']].copy()]*2,.15,.0235,checked_edges=True)
    for r in open_rows:cmd[0,ids['left']]=r[:7];cmd[0,fi['left']]=r[7:];step('double_LEFT_release')
    released=False;release_window=[]
    for _ in range(120):
        release_window.append(art.get_dof_positions()[0,fi['left']].copy());release_window=release_window[-5:]
        stationary=len(release_window)==5 and np.max(np.ptp(release_window,axis=0))<=.00015 and np.ptp(np.asarray(release_window).sum(1))<=.00015
        if stationary and w('left')>=.0235-.0005:released=True;break
        wait(.025,'double_LEFT_actual_release_plateau')
    event('double_LEFT_actual_release',confirmed=released,actual_aperture_m=w('left'),stationary_encoder_samples=[r.tolist()for r in release_window])
    if not released:raise RuntimeError('LEFT actual opening did not establish23mm stationary release; no withdrawal issued')
    previous_right_G=right_G.copy();right_G,right_pose=observe('right',part,right_G,'double_RIGHT_after_LEFT_actual_release_RGBD')
    contact=contact_state(clean,part,right_G,w('right'))
    release_translation=float(np.linalg.norm(right_G[:3,3]-previous_right_G[:3,3]));release_angle=float(np.arccos(np.clip(right_G[:3,2]@previous_right_G[:3,2],-1,1)))
    event('double_RIGHT_holds_before_LEFT_withdraw',contact=contact,translation_change_m=release_translation,axis_change_rad=release_angle)
    if not contact['opposed_contact_compatible']or release_translation>.0015 or release_angle>.025:
        raise RuntimeError('RIGHT fresh hold not confirmed after actual LEFT release; no withdrawal issued')
    from paired_handover import empty_giver_route
    with payload(checkers['right'],right_G),checkers['left'].other_arm_at(checkers['right'],q('right'),w('right')):
        route=empty_giver_route(checkers['left'],cmd[0,ids['left']],.0235)
    if route is None:raise RuntimeError('No LEFT empty withdrawal with received RIGHT held-CAD protection')
    issue('left',route,.0235,'double_LEFT_empty_withdraw',holder='right',Gholder=right_G);c['left_hand_known_empty']=True
    # Stationary physical encoder window first, then a fresh image collected
    # while the same actual encoder callback remains live. Original .25s TTL.
    samples=[]
    def collect(t):samples.append((t,art.get_dof_positions()[0,fi['right']].copy()))
    callbacks=c['component_encoder_capture_callbacks'];callbacks.append(collect)
    try:
        wait(.105,'double_RIGHT_encoder_window');right_G,right_pose=observe('right',part,right_G,'double_RIGHT_final_fresh_RGBD')
    finally:callbacks.remove(collect)
    contact=contact_state(clean,part,right_G,w('right'))
    if donor_aperture(clean,part,right_G)is None:raise RuntimeError('Final positive RIGHT camera grip lacks stable class/native geometry')
    capture_age=float(c['component_image_age_steps']('right',part))*dt
    binding=LoadedContactAperture(side='right',part=part,payload_path=template,encoder_samples=[r[1]for r in samples],sample_times_s=[r[0]for r in samples],contact=contact,measured_grasp=right_G,control_dt_s=dt,capture_age_physics_s=capture_age)
    checkers['right'].loaded_contact_aperture=binding;bindings['right']=binding
    with payload(checkers['right'],right_G):
        ok,why=checkers['right'].clear(q('right'),max(w('right'),float(cmd[0,fi['right']].sum())))
        if not ok:raise RuntimeError('Received RIGHT actual interval/full own arm or fixed clearance rejected '+str(why))
    prior_G=right_G.copy();wait(.15,'double_RIGHT_camera_retention_hold');right_G,retained=observe('right',part,right_G,'double_RIGHT_retention_RGBD');translation=float(np.linalg.norm(right_G[:3,3]-prior_G[:3,3]));angle=float(np.arccos(np.clip(right_G[:3,2]@prior_G[:3,2],-1,1)))
    if translation>.0015 or angle>.025:raise RuntimeError('Positive RIGHT regrasp did not retain stable camera-measured grasp')
    with payload(checkers['right'],right_G):
        ok,why=checkers['right'].clear(q('right'),max(w('right'),float(cmd[0,fi['right']].sum())))
        if not ok:raise RuntimeError('Retained RIGHT fresh-G own/full clearance failed '+str(why))
    event('double_RIGHT_retained',translation_change_m=translation,axis_change_rad=angle,measured_grasp=right_G.tolist(),actual_aperture_m=w('right'),loaded_geometry_binding=binding.record)
    return dict(state='physical_double_handover_RIGHT_regrasp_camera_confirmed',part=part,measured_grasp=right_G.tolist(),grasp_type='outer',full_task_pass=False,physical_insertion_validated=False,uses_free_body_poses=False,events=events,retention_RGBD=retained)
