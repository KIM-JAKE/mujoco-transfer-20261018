"""Current RGB-D external pickup followed by right-arm ring insertion.

The skill never reads archived trial commands or live free-body transforms.
All motion uses the current camera grasp, robot encoders, native geometry,
known jig calibration and unchanged physical contacts/actuator limits.
"""
from contextlib import contextmanager
from pathlib import Path
import json,time,types
import numpy as np
from isaac_kinematics import frame,exp,log
from rgbd_geometry import axis_frame
from fr3_path_timing import retime,retime_safe_polyline
from grasp_transfer_feasibility import pad_aperture,donor_aperture,hinted_route
from receiver_pad_contact import contact_state
from probe_native_right_ring import fixture_clear
from aperture_path_check import path_openings


class PrecarryPlanUnavailable(RuntimeError):
    """Only first fixture preflight failed; no loaded RIGHT move was issued."""
    def __init__(self, reason, measured_grasp, observed_pose):
        super().__init__(reason)
        self.measured_grasp=np.asarray(measured_grasp,float).copy()
        self.observed_pose=dict(observed_pose)


def execute(c,pickup=None):
    here=Path(__file__).resolve().parent
    arm,art,cmd,ids,fi,dt,vision,checker=[c[k]for k in ['arm','art','cmd','ids','fi','dt','vision','checker']]
    part=c['config'].get('pick_part','ring1');events=[]
    if part not in ['ring1','ring2']:raise ValueError('Direct ring insertion requires a ring class')
    contact_context=dict(c,config=dict(c['config'],receiver_axial_only=False))
    c.setdefault('robot_route_hints',json.loads((here/'bin-picking/live-robot-route-hints.json').read_text()))
    def step(label):c['step'](part+'_right_'+label)
    def wait(seconds,label):
        began=time.perf_counter()
        for _ in range(int(np.ceil(seconds/dt))):step(label)
        while time.perf_counter()-began<seconds:step(label)
    def record(label,**values):
        row=dict(event=part+'_right_'+label,wall_s=time.perf_counter()-c['started'],**values);events.append(row)
        print('RIGHT_RING_INSERT',row,flush=True)
    @contextmanager
    def query_context(G=None):
        saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative)
        checker.detach_payload()
        if G is not None:checker.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@G)
        try:
            with checker.other_arm_at(c['checkers']['left'],art.get_dof_positions()[0,c['left_ids']],float(art.get_dof_positions()[0,c['left_fi']].sum())):yield
        finally:checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved
    def move(qs,requested,label,width=None,G=None,smooth=False):
        qs=np.asarray(qs,float).copy();qs[0]=cmd[0,ids];start=cmd[0,fi].copy();target=start if width is None else np.full(2,float(width)/2)
        scale=max(1.,float(c['config'].get('stability_time_scale',1.)))
        measured=float(art.get_dof_positions()[0,fi].sum());low=min(measured,float(start.sum()),float(target.sum()));high=max(measured,float(start.sum()),float(target.sum()))
        openings=np.unique(np.linspace(low,high,5));actual=art.get_dof_positions()[0,ids].copy()
        reuse_width_invariants=bool(c['config'].get('reuse_aperture_invariant_checks',False))
        def verify(q):
            stats={}
            with query_context(G):
                if reuse_width_invariants:
                    ok,why=path_openings(checker,np.vstack([actual,q]),openings,diagnostics=stats)
                    return ok,why,stats
                for opening in openings:
                    ok,why=checker.path(np.vstack([actual,q]),float(opening))
                    if not ok:return ok,why,stats
            return True,None,stats
        smooth_accepted=False;smooth_rejection=None;aperture_stats={}
        # Only a planner-marked continuous cubic may propose this timing.
        # EVERY resulting float32 command is still checked in the live scene.
        # Failure here precedes ALL motion and retains the old checked edges.
        if smooth and len(qs)>2:
            try:
                q,h,timing=retime(qs,requested,dt,start,target,optimize=bool(c['config'].get('optimized_motion_timing')))
                if scale>1:q,h,timing=retime(qs,timing['duration_s']*scale,dt,start,target,optimize=False)
            except RuntimeError as error:smooth_rejection=dict(stage='retiming',reason=str(error))
            else:
                ok,why,aperture_stats=verify(q)
                if ok:smooth_accepted=True
                else:smooth_rejection=dict(stage='full_issued_geometry',reason=why)
        if not smooth_accepted:
            if len(qs)>2:q,h,timing=retime_safe_polyline(qs,dt,start,target,scale=scale)
            else:
                q,h,timing=retime(qs,requested,dt,start,target,optimize=bool(c['config'].get('optimized_motion_timing')))
                if scale>1:q,h,timing=retime(qs,timing['duration_s']*scale,dt,start,target,optimize=False)
            clear,why,aperture_stats=verify(q)
            if not clear:raise RuntimeError('Right ring executed curve hits fixed structure '+label+' '+str(why))
        c['planning_diagnostics'].append(dict(event='right_ring_checked_executed_curve',part=part,label=label,checked_openings_m=openings.tolist(),payload_checked=G is not None,aperture_invariants_reused=reuse_width_invariants,aperture_query_diagnostics=aperture_stats,smooth_cubic_proposed=bool(smooth),smooth_cubic_accepted=smooth_accepted,smooth_pre_motion_rejection=smooth_rejection,**timing))
        for row,u in zip(q[1:],h[1:]):cmd[0,ids]=row;cmd[0,fi]=start+(target-start)*u;step(label)
        record(label,physical_duration_s=(len(q)-1)*dt,smooth_cubic_accepted=smooth_accepted)
    def line(target,requested,label,G=None,width=None):
        initial=arm.fk(cmd[0,ids]);qs=[cmd[0,ids].copy()]
        for u in np.linspace(0,1,9)[1:]:
            R=exp(log(target[:3,:3]@initial[:3,:3].T)*u)@initial[:3,:3]
            q=arm.solve(initial[:3,3]*(1-u)+target[:3,3]*u,R,qs[-1])
            if q is None:raise RuntimeError('Right ring Cartesian IK failed '+label)
            qs.append(q)
        move(qs,requested,label,width=width,G=G)
    def observe(G,label):
        H=arm.fk(art.get_dof_positions()[0,ids]);predicted=H@G;n=predicted[:3,2]
        root=(H@frame([0,0,-.1034],np.eye(3)))[:3,3];side=1 if (root-predicted[:3,3])@n>=0 else -1
        tangent=np.array([-.25,.12,.03]);tangent-=n*(tangent@n);tangent/=max(np.linalg.norm(tangent),1e-9)
        views=[-side*n*.25+tangent*.24,-side*n*.25-tangent*.24,np.array([-.25,.12,.20])]
        if predicted[1,3]<-.5:
            # Use the same low annular views already verified by pickup;
            # above-hand views can hide the ring behind the native palm.
            short=H[:3,0].copy();short[2]=0.;short/=max(np.linalg.norm(short),1e-9)
            other=np.cross([0.,0.,1.],short)
            dz=max(-.015,1.210918206667893+.005-predicted[2,3]);vertical=np.array([0.,0.,dz])
            views=[-short*.25+vertical,other*.25+vertical,-other*.25+vertical,short*.25+vertical]+views
        else:
            # Look beneath the fixed housing, using the accepted insertion
            # camera directions before trying cap-normal fallbacks.
            views=[np.array(v)for v in [[-.12,-.20,-.28],[-.22,.16,-.25],[-.20,-.25,-.12]]]+views
            if c['config'].get('right_fixture_oblique_camera_views'):
                # Opposite-hand views look upward at the lower annular face.
                # Eye and crop follow only current encoder FK and the last
                # directly measured RGB-D grasp; all pixel gates remain.
                views=[np.array(v)for v in [[.12,.18,-.20],[-.10,.20,-.22]]]+views
        for index,offset in enumerate(views):
            event=dict(mode='held',part=part,tracked_grasp=G.tolist(),camera_eye_offset=offset.tolist(),label=part+'_right_'+label,require_annular_face=True,view_index=index)
            future=vision.request(event,arm.fk(art.get_dof_positions()[0,ids]));deadline=time.perf_counter()+3
            while not future.done():
                vision.poll();step(label)
                if time.perf_counter()>deadline:raise RuntimeError('Right ring RGB-D deadline '+label)
            value=future.result();event['delivered_wall_s']=time.perf_counter()-c['started']
            pose=value['result'].get('pose')
            if value['result'].get('error') or not pose or not pose.get('front_back_from_RGBD'):continue
            measured=np.linalg.inv(value['H'])@frame(pose['center_m'],axis_frame(pose['axis']))
            record(label,observed_center_m=pose['center_m'],observed_axis=pose['axis'],captured_wall_s=value['captured_wall_s'])
            return measured,pose
        raise RuntimeError('No directly observed signed annular face '+label)
    def check_hold(G,label):
        expected=pad_aperture(c,part,G)
        if expected is None:raise RuntimeError('Current right ring grasp lacks valid opposing native pad geometry '+label)
        width=float(art.get_dof_positions()[0,fi].sum());proof=contact_state(contact_context,part,G,width)
        record(label,native_contact=proof)
        if not proof['opposed_contact_compatible']:raise RuntimeError('Current RGB-D does not confirm both native outer pads '+label)
        return max(float(expected),width)
    def park_empty_left(G):
        if not (c.get('left_hand_known_empty') is True or c['config'].get('left_hand_known_empty') is True):
            raise RuntimeError('Empty-left parking requires an explicit logical empty-hand invariant')
        left=c['left_arm'];lc=c['checkers']['left'];li=c['left_ids'];lf=c['left_fi']
        actual_left=art.get_dof_positions()[0,li].copy();actual_right=art.get_dof_positions()[0,ids].copy()
        right_width=float(art.get_dof_positions()[0,fi].sum());left_width=float(art.get_dof_positions()[0,lf].sum());H=left.fk(actual_left)
        proposals=[[-.10,-.10,-.03],[-.20,-.10,-.03],[-.10,0.,-.03],[-.20,0.,-.03],[0.,-.10,-.03]]
        saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative)
        checker.detach_payload();checker.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@G)
        chosen=None
        try:
            with lc.other_arm_at(checker,actual_right,right_width):
                for delta in proposals:
                    target=H.copy();target[:3,3]+=delta
                    seeds=[actual_left,cmd[0,li],np.asarray(c['handoff_reference'][part]['left_receive_q'])]
                    for seed in seeds:
                        park_q=left.solve(target[:3,3],target[:3,:3],seed)
                        if park_q is None or not lc.clear(park_q,left_width)[0]:continue
                        start=cmd[0,li].copy()
                        route=[start,park_q] if lc.path([actual_left,start,park_q],left_width)[0] else lc.route(start,park_q,left_width,seconds=.35)
                        if route is None:continue
                        knots=np.asarray(route,float).copy();knots[0]=start;finger_start=cmd[0,lf].copy();scale=max(1.,float(c['config'].get('stability_time_scale',1.)))
                        if len(knots)>2:q,h,timing=retime_safe_polyline(knots,dt,finger_start,finger_start,scale=scale)
                        else:
                            q,h,timing=retime(knots,.5,dt,finger_start,finger_start,optimize=bool(c['config'].get('optimized_motion_timing')))
                            if scale>1:q,h,timing=retime(knots,timing['duration_s']*scale,dt,finger_start,finger_start,optimize=False)
                        openings=np.unique(np.linspace(min(left_width,float(finger_start.sum())),max(left_width,float(finger_start.sum())),5))
                        curve=np.vstack([actual_left,q])
                        aperture_stats={};reuse_width_invariants=bool(c['config'].get('reuse_aperture_invariant_checks',False))
                        if reuse_width_invariants:
                            if not path_openings(lc,curve,openings,diagnostics=aperture_stats)[0]:continue
                        elif not all(lc.path(curve,float(opening))[0]for opening in openings):continue
                        timing=dict(timing,aperture_invariants_reused=reuse_width_invariants,aperture_query_diagnostics=aperture_stats)
                        chosen=(q,delta,openings,timing);break
                    if chosen is not None:break
                if chosen is None:raise RuntimeError('No fully checked empty-left parking route around the currently held right ring')
                q,delta,openings,timing=chosen
                c['planning_diagnostics'].append(dict(event='right_ring_empty_left_parking',part=part,logical_empty_hand_asserted=True,checked_openings_m=openings.tolist(),stationary_right_actual_q=actual_right.tolist(),stationary_right_actual_width_m=right_width,measured_right_grasp=G.tolist(),left_TCP_delta_m=delta,**timing))
                for row in q[1:]:cmd[0,li]=row;step('empty_left_park')
        finally:checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved
        # A fixture rejection/approval depends on the other arm's parked
        # configuration. Never reuse a result from its previous location.
        c.pop('_right_direct_fixture_cache',None);c.pop('_right_direct_fixture_counts',None);c.pop('_right_direct_fixture_examples',None)
        record('empty_left_park',logical_empty_hand_asserted=True,left_TCP_delta_m=delta,physical_duration_s=(len(q)-1)*dt)
    if pickup is None:
        path=here/'loose_ring_pick.py';module=types.ModuleType('loose_ring_pick');module.__file__=str(path);exec(compile(path.read_text(),str(path),'exec'),module.__dict__)
        pick_context=dict(c,config=dict(c['config'],pick_part=part,prefer_transfer_compatible=False,verify_lifted_pad_contact=True),grasp_pad_aperture=lambda pose,H:donor_aperture(c,part,np.linalg.inv(H)@frame(pose['center_m'],axis_frame(pose['axis']))))
        pick_context.pop('handover_feasible_grasp',None)
        try:pickup=module.execute(pick_context)
        except RuntimeError as error:
            if not c['config'].get('allow_single_finger_rearrangement') or not str(error).startswith(('No structure-clear grasp','No visible accessible measurable')):raise
            path=here/'loose_bin_rearrange.py';rearrange=types.ModuleType('loose_bin_rearrange');rearrange.__file__=str(path);exec(compile(path.read_text(),str(path),'exec'),rearrange.__dict__)
            # The helper physically rearranges, then returns its own freshly
            # observed pickup. Consume that result once; do not pick again or
            # allow a recursive rearrangement attempt inside the helper.
            rearrange_context=dict(pick_context,config=dict(pick_context['config'],rearrange_then_pick=True,allow_single_finger_rearrangement=False))
            rearrange_context.pop('prefetched_initial',None)
            pickup=rearrange.execute(rearrange_context)
    if pickup.get('grasp_type','outer')!='outer':raise RuntimeError('Right direct insertion currently requires an external native closing grasp')
    G=np.asarray(pickup['measured_grasp'],float)
    G,pose=observe(G,'lifted_contact_RGBD');width=check_hold(G,'lifted_opposed_contact')
    if G[2,2]<=.1:raise RuntimeError('Right direct insertion needs the palm below the signed CAD face; use the left handover skill for this grasp')
    if c['config'].get('right_direct_empty_left_parking'):
        park_empty_left(G)
        G,pose=observe(G,'after_empty_left_park_RGBD');width=check_hold(G,'after_empty_left_park_opposed_contact')
    def plan_current_fixture(G,width,replan=False):
        actual_start=art.get_dof_positions()[0,ids].copy();command_start=cmd[0,ids].copy()
        parked_left=art.get_dof_positions()[0,c['left_ids']].copy();parked_width=float(art.get_dof_positions()[0,c['left_fi']].sum());actual_width=float(art.get_dof_positions()[0,fi].sum())
        context_key=(np.round(actual_start,8).tobytes(),np.round(command_start,8).tobytes(),np.round(parked_left,8).tobytes(),round(actual_width,8),round(parked_width,8))
        search=dict(c,_right_direct_current_heading_first=bool(replan),_right_direct_minimum_joint_margin_rad=.03 if replan else 0.,_right_direct_measured_q=actual_start,_right_direct_fixture_context_key=context_key,_right_direct_fixture_cache={},_right_direct_fixture_counts={},_right_direct_fixture_examples={})
        deadline=time.perf_counter()+float(c['config'].get('axial_receiver_fixture_budget_s',45.));carry_cache={};carry_counts=dict(attempts=0,accepted=0)
        def carry_candidate(pre_q,grasp,opening):
            key=np.round(pre_q,7).tobytes()
            if key in carry_cache:return carry_cache[key]
            if time.perf_counter()>deadline:return None
            carry_counts['attempts']+=1;carry_started=time.perf_counter()
            with query_context(grasp):
                # Optional greedy shortcut search can exhaust the fixture budget
                # after cuRobo already supplied a full-geometry clear route.
                route_context=dict(c,config=dict(c['config'],shortcut_checked_routes=False)) if c['config'].get('right_direct_skip_greedy_shortcuts') else c
                route=hinted_route(route_context,part+'_direct_right_carry',checker,command_start,pre_q,opening,seconds=min(3.,max(.05,deadline-time.perf_counter())))
                if route is not None:
                    carry_ok,carry_why=checker.path(np.vstack([actual_start,np.asarray(route)]),opening)
                    if not carry_ok:
                        record('carry_candidate_rejected',reason=carry_why,wall_check_s=time.perf_counter()-carry_started)
                        route=None
            if route is not None:carry_counts['accepted']+=1
            carry_cache[key]=route;return route
        search['_right_direct_carry_route_callback']=carry_candidate
        with query_context():approved=fixture_clear(search,part,G,width,deadline=deadline)
        diagnostic=dict(event='right_ring_fixture_search',part=part,fresh_RGBD_alignment_replan=bool(replan),approved=approved is not None,counts=dict(search.get('_right_direct_fixture_counts',{})),examples=dict(search.get('_right_direct_fixture_examples',{})),carry_route_counts=carry_counts,current_measured_grasp=G.tolist(),checked_width_m=width)
        c['planning_diagnostics'].append(diagnostic);record('fixture_search',**{k:v for k,v in diagnostic.items()if k!='event'})
        if approved is None:raise RuntimeError('Current camera grasp has no complete payload-carry and right-arm fixed-CAD insertion/release/withdraw route')
        record('fixture_approved',fixture=approved)
        return approved
    try:
        approved=plan_current_fixture(G,width)
    except RuntimeError as error:
        if str(error)!='Current camera grasp has no complete payload-carry and right-arm fixed-CAD insertion/release/withdraw route':raise
        # No RIGHT carry has started. A stopped empty LEFT parking motion may
        # have completed; downstream handover uses its CURRENT encoder state.
        raise PrecarryPlanUnavailable(str(error),G,pose) from error
    route=approved.get('carry_route')
    if route is None:raise RuntimeError('Carry-aware fixture approval omitted its checked current-start route')
    move(route,.05 if c['config'].get('optimized_motion_timing') else 1.4,'carry_to_preinsert',G=G,smooth=bool(c['config'].get('right_direct_smooth_verified_carry') and approved.get('carry_route_verified_cubic')))
    goal=np.asarray(c['goals'][part]['P']);clearance=float(approved['preinsert_clearance_m']);pre_position=goal-np.array([0.,0.,clearance])
    def aligned_hand(position,grasp):
        H=arm.fk(cmd[0,ids]);n=H[:3,:3]@grasp[:3,2];cross=np.cross(n,[0.,0.,1.]);length=np.linalg.norm(cross)
        R=exp(cross/max(length,1e-12)*np.arctan2(length,n[2]))@H[:3,:3]
        return frame(np.asarray(position)-R@grasp[:3,3],R)
    preinsert_replans=0
    for iteration in range(10):
        G,pose=observe(G,'preinsert_RGBD');width=check_hold(G,'preinsert_opposed_contact')
        P=np.asarray(pose['center_m']);error=float(np.linalg.norm(P-pre_position));angle=float(np.arccos(np.clip(pose['axis'][2],-1,1)))
        aligned=error<.0007 and angle<.010
        if error>.025 or angle>.35 or (iteration==9 and not aligned):raise RuntimeError('Right free-space alignment did not converge within the measured correction bounds')
        try:
            if not aligned:
                line(aligned_hand(pre_position,G),.4,'preinsert_visual_correction',G=G)
                wait(.08,'alignment_settle')
                continue
            # line() first builds ALL Cartesian IK knots and checks the exact
            # retimed curve. An IK failure therefore issues no axial motion.
            line(aligned_hand(goal,G),.8,'axial_insert')
            break
        except RuntimeError as failure:
            if str(failure) not in ['Right ring Cartesian IK failed preinsert_visual_correction','Right ring Cartesian IK failed axial_insert'] or not c['config'].get('right_fixture_replan_on_alignment_failure') or preinsert_replans>=2 or iteration==9:raise
            # Replan the full insertion/release/withdraw AND loaded carry from
            # current camera G/current both-arm encoders, with extra nominal
            # joint headroom. No pickup-era fixture approval is reused.
            preinsert_replans+=1
            record('preinsert_current_grasp_replan',attempt=preinsert_replans,reason=str(failure),measured_grasp=G.tolist(),measured_position_error_m=error,measured_axis_error_rad=angle)
            approved=plan_current_fixture(G,width,replan=True)
            route=approved.get('carry_route')
            if route is None:raise RuntimeError('Current-grasp fixture replan omitted its verified current-start carry route')
            move(route,.05 if c['config'].get('optimized_motion_timing') else 1.4,'preinsert_current_grasp_replan_carry',G=G)
            clearance=float(approved['preinsert_clearance_m']);pre_position=goal-np.array([0.,0.,clearance])
    else:raise RuntimeError('Right measured prealignment/axial insertion exhausted its bounded observation loop')
    wait(.12,'full_depth_hold')
    for correction in range(3):
        G,pose=observe(G,'full_depth_RGBD');P=np.asarray(pose['center_m']);error=float(np.linalg.norm(P-goal));angle=float(np.arccos(np.clip(pose['axis'][2],-1,1)));remaining=float(goal[2]-P[2]);radial=float(np.linalg.norm(P[:2]-goal[:2]))
        if error<.0015 and angle<.05:break
        if correction==2 or not .0005<remaining<.020 or radial>.0015 or angle>.05:raise RuntimeError('Right ring full seating not confirmed; fingers remain closed')
        H=arm.fk(cmd[0,ids]);H[2,3]+=remaining;line(H,.3,'axial_depth_correction');wait(.12,'depth_settle')
    seated_world=frame(pose['center_m'],axis_frame(pose['axis']))
    c['close_clamp'](part);wait(.12,'clamp_hold');c['set_grip_hold']('right',False)
    release=.08 if part=='ring1'else min(.08,width+.002)
    move([cmd[0,ids],cmd[0,ids]],.20,'release_after_full_depth',width=release)
    wait(.25,'release_settle');H=arm.fk(cmd[0,ids]);H[2,3]-=.08
    line(H,.6,'world_down_withdraw',width=release)
    wait(.8,'retention')
    # After release this is an optical world-pose seed from the last RGB-D
    # measurement, not a hand attachment or a claim that the part still moves
    # with the hand. The independent trial evaluator checks physical retention.
    tracking=np.linalg.inv(arm.fk(art.get_dof_positions()[0,ids]))@seated_world
    retention=None
    try:
        _,retained=observe(tracking,'released_retention_RGBD');retention=dict(position_error_m=float(np.linalg.norm(np.asarray(retained['center_m'])-goal)),axis_error_rad=float(np.arccos(np.clip(retained['axis'][2],-1,1))),pose=retained)
        if retention['position_error_m']>=.0015 or retention['axis_error_rad']>=.05:raise RuntimeError('Released ring moved outside the insertion tolerance')
    except RuntimeError as error:
        if retention is not None or not str(error).startswith(('No directly observed signed annular face','Right ring RGB-D deadline')):raise
        record('released_retention_not_visible',reason=str(error))
    return dict(state='right_native_ring_insert_executed_requires_physical_evaluation',part=part,pickup=pickup,grasp_type='outer',measured_grasp=G.tolist(),right_direct_insertion=True,native_ring_result=dict(part=part,full_depth_RGBD_pass=True,full_depth_pose=pose,release_opening_m=release,world_down_withdrawal_m=.08,retention_minimum_s=.8,retention_RGBD=retention,physical_retention_validated=False),events=events,elapsed_s=time.perf_counter()-c['started'],full_task_pass=False,uses_simulator_body_poses=False,workpiece_attachments=False,workpiece_teleports=False)
