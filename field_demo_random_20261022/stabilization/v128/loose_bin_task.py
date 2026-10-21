"""Compose live RGB-D pickup, physical handover and insertion skills.

No scene random seed or live free-body pose enters the motion controller.
"""
from pathlib import Path
import types,time,json,sys,numpy as np
from scipy.interpolate import CubicSpline
from isaac_kinematics import frame,exp,log
from rgbd_geometry import axis_frame
from fr3_motion_control import position_velocity_bounds
from fr3_path_timing import retime,retime_safe_polyline

def execute(c):
    HERE=Path(__file__).resolve().parent;c['robot_route_hints']=json.loads((HERE/'bin-picking/live-robot-route-hints.json').read_text());cmd=c['cmd'];art=c['art'];dt=c['dt'];vision=c['vision'];phase='initial';events=[]
    arms={'right':c['arm'],'left':c['left_arm']};indices={'right':c['ids'],'left':c['left_ids']};fingers={'right':c['fi'],'left':c['left_fi']};checkers=c['checkers'];left_program=None;control_steps=0;left_finished_step=0;pin_retention_start=None
    def step(label=None):
        nonlocal left_program,control_steps,left_finished_step,pin_retention_start
        if label=='pin_retention' and pin_retention_start is None:pin_retention_start=control_steps
        if left_program is not None:
            try:next(left_program)
            except StopIteration:left_program=None;left_finished_step=control_steps
        c['step'](label or phase)
        control_steps+=1
    def wait(t,label):
        for _ in range(int(np.ceil(t/dt))):step(label)
    def trajectory(side,qs,requested,width=None,checked_edges=False):
        ids=indices[side];fi=fingers[side];checked_edges=checked_edges and not getattr(qs,'verified_cubic',False);qs=np.array(qs);qs[0]=cmd[0,ids];start=cmd[0,fi].copy();target=start if width is None else np.full(2,width/2)
        scale=max(1.,float(c['config'].get('stability_time_scale',1.)))
        if checked_edges and len(qs)>2:
            q,h,timing=retime_safe_polyline(qs,dt,start,target,scale=scale)
            c['planning_diagnostics'].append(dict(event='checked_joint_edge_execution',side=side,**timing));print('LOOSE_CHECKED_EDGES',side,timing,flush=True)
            return np.c_[q[1:],start+(target-start)*h[1:,None]]
        try:
            q,h,timing=retime(qs,requested,dt,start,target,optimize=c['config'].get('optimized_motion_timing',False))
            if scale>1:q,h,timing=retime(qs,timing['duration_s']*scale,dt,start,target,optimize=False)
        except RuntimeError as error:
            if 'Joint path leaves FR3 position limits' not in str(error):raise
            # The planner checked the polyline's edges; use those exact
            # edges rather than a cubic that bows outside their envelope.
            q,h,timing=retime_safe_polyline(qs,dt,start,target,scale=scale)
            c['planning_diagnostics'].append(dict(event='joint_edge_execution_fallback',side=side,**timing));print('LOOSE_SAFE_POLYLINE',side,timing,flush=True)
        return np.c_[q[1:],start+(target-start)*h[1:,None]]
    def move(side,qs,t,label,width=None,checked_edges=True):
        nonlocal phase
        phase=label
        if c['config'].get('optimized_motion_timing') and any(word in label for word in ['carry','approach','preposition','corridor','donor_clear','pull_from']):t=.05
        rows=trajectory(side,qs,t,width,checked_edges=checked_edges)
        for row in rows:cmd[0,indices[side]]=row[:7];cmd[0,fingers[side]]=row[7:];step()
        events.append(dict(event=label,wall_s=time.perf_counter()-c['started'],motion_s=len(rows)*dt));print('LOOSE_TASK',label,events[-1]['wall_s'],flush=True)
    def line(side,target,t,label,width=None):
        arm=arms[side];ids=indices[side];H=arm.fk(cmd[0,ids]);qs=[cmd[0,ids].copy()]
        for u in np.linspace(0,1,5)[1:]:
            q=arm.solve(H[:3,3]*(1-u)+target[:3,3]*u,target[:3,:3],qs[-1])
            if q is None:raise RuntimeError('No Cartesian IK '+label)
            qs.append(q)
        move(side,qs,t,label,width,checked_edges=False)
    def observe(side,part,G,label,background=None):
        # Select an unobstructed cap view from the encoder and last measured
        # grasp, never from a free-body pose. A weak one-sided ICP fit must
        # not authorize a transfer. All re-exposures count in the wall clock.
        H=arms[side].fk(art.get_dof_positions()[0,indices[side]]);predicted=H@G;n=predicted[:3,2]
        root=(H@frame([0,0,-.1034],np.eye(3)))[:3,3];palm_side=1 if (root-predicted[:3,3])@n>=0 else -1
        lateral=np.array([-.25,.12,.03]);lateral-=n*(lateral@n);lateral/=max(np.linalg.norm(lateral),1e-9)
        views=[-palm_side*n*.25+lateral*.24,-palm_side*n*.25-lateral*.24,np.array([-.25,.12,.20])]
        for view_index,offset in enumerate(views):
            e=dict(mode='held' if side=='right' else 'held_left',part=part,tracked_grasp=G.tolist(),camera_eye_offset=offset.tolist(),label=label,require_annular_face=True,view_index=view_index);v=vision.request(e,arms[side].fk(art.get_dof_positions()[0,indices[side]]));deadline=time.perf_counter()+3
            while not v.done():
                vision.poll()
                if background is not None:
                    try:next(background)
                    except StopIteration:background=None
                step(label)
                if time.perf_counter()>deadline:raise RuntimeError('Dual-arm camera timeout')
            value=v.result();e['delivered_wall_s']=time.perf_counter()-c['started']
            if value['result'].get('error'):continue
            p=value['result']['pose'];Gobs=np.linalg.inv(value['H'])@frame(p['center_m'],axis_frame(p['axis']));return Gobs,p
        raise RuntimeError('No directly measured annular face in transfer camera views')
    ref=c['handoff_reference'];results=[];next_initial=None;pin_initial=None;left_standby_q=cmd[0,indices['left']].copy()
    transfer_path=HERE/'grasp_transfer_feasibility.py';transfer=types.ModuleType('grasp_transfer_feasibility');transfer.__file__=str(transfer_path);sys.modules['grasp_transfer_feasibility']=transfer;exec(compile(transfer_path.read_text(),str(transfer_path),'exec'),transfer.__dict__)
    ring_order=['ring2','ring1'] if c['config'].get('small_ring_first') else ['ring1','ring2']
    def clear_left_work_zone_for_pin(pin_grasp):
        nonlocal next_initial
        if c['config'].get('pin_first') and not results and next_initial is None:
            next_initial=vision.request(dict(mode='initial',part=None),arms['right'].fk(art.get_dof_positions()[0,indices['right']]))
        while left_program is not None:step('pin_waits_for_final_ring_release')
        # With the pin lifted clear of its holder, park the empty receiver.
        # Leaving it under ring 2 blocks the adjacent pin insertion corridor.
        if np.max(abs(cmd[0,indices['left']]-left_standby_q))<1e-4:return dict(empty_left_arm_parked=True,already_at_standby=True)
        rc=checkers['right'];lc=checkers['left'];width=float(art.get_dof_positions()[0,fingers['left']].sum())
        rc.attach_payload(c['pin_context']['paths']['workpiece_pin'],frame([0,0,.1034],np.eye(3))@pin_grasp)
        try:
            with lc.other_arm_at(rc,cmd[0,indices['right']],float(art.get_dof_positions()[0,fingers['right']].sum())):
                route=transfer.hinted_route(c,'pin_left_clearance',lc,cmd[0,indices['left']],left_standby_q,width,seconds=2.)
                if route is None and c['config'].get('local_left_park'):
                    startH=arms['left'].fk(cmd[0,indices['left']])
                    for delta in [[-.10,-.10,0.],[0.,-.15,-.03],[-.15,0.,-.03],[-.20,-.10,0.]]:
                        target=startH.copy();target[:3,3]+=delta;q=arms['left'].solve(target[:3,3],target[:3,:3],cmd[0,indices['left']])
                        if q is None:continue
                        route=lc.route(cmd[0,indices['left']],q,width,seconds=.5)
                        if route is not None:break
        finally:rc.detach_payload()
        if route is None:raise RuntimeError('No empty left-arm clearance route before pin transport')
        move('left',route,.8,'pin_left_clear_work_zone')
        return dict(empty_left_arm_parked=True,right_held_pin_in_collision_query=True)
    def run_pin(prefetched=None):
        if c['config'].get('wait_for_ring_transport_clear'):
            while left_program is not None and np.linalg.norm(arms['left'].fk(art.get_dof_positions()[0,indices['left']])[:3,3]-arms['right'].fk(art.get_dof_positions()[0,indices['right']])[:3,3])<.24:step('receiver_clears_empty_pin_preshape_volume')
        pin_path=HERE/'direct_pin_skill.py';pin=types.ModuleType('direct_pin_skill');pin.__file__=str(pin_path);exec(compile(pin_path.read_text(),str(pin_path),'exec'),pin.__dict__)
        return pin.execute(dict(c['pin_context'],step=step,config=c['config'],robot_route_hints=c['robot_route_hints'],prefetched_initial=prefetched,before_transport=clear_left_work_zone_for_pin,goal_aware_pickup=bool(c['config'].get('pin_first') or c['config'].get('pin_between_rings')),defer_retention_to_recipe=bool(c['config'].get('pin_first') or c['config'].get('pin_between_rings'))))
    pin_first=bool(c['config'].get('pin_first'));pin_report=run_pin() if pin_first else None
    for part in (['ring1'] if c['config'].get('one_ring_only') else ring_order):
        if c['config'].get('stability_serial'):
            while left_program is not None:step('stable_receiver_finishes_before_next_pick')
        # The right hand keeps working while the queued left insertion runs.
        pick_path=HERE/'loose_ring_pick.py';pick=types.ModuleType('loose_ring_pick');pick.__file__=str(pick_path);exec(compile(pick_path.read_text(),str(pick_path),'exec'),pick.__dict__)
        pickup_ctx=dict(c,step=step,config=dict(c['config'],pick_part=part,prefer_transfer_compatible=True),handover_feasible_grasp=lambda pose,H:transfer.compatible(c,part,pose,H),grasp_pad_aperture=lambda pose,H:transfer.pad_aperture(c,part,np.linalg.inv(H)@frame(pose['center_m'],axis_frame(pose['axis']))))
        if next_initial is not None:pickup_ctx['prefetched_initial']=next_initial;next_initial=None
        try:p=pick.execute(pickup_ctx)
        except RuntimeError as error:
            if c['config'].get('allow_single_finger_rearrangement') and str(error).startswith(('No structure-clear grasp','No visible accessible measurable')):
                rearrange_path=HERE/'loose_bin_rearrange.py';rearrange=types.ModuleType('loose_bin_rearrange');rearrange.__file__=str(rearrange_path);exec(compile(rearrange_path.read_text(),str(rearrange_path),'exec'),rearrange.__dict__)
                rearranged=rearrange.execute(pickup_ctx);pickup_ctx.pop('prefetched_initial',None)
                p=pick.execute(pickup_ctx);p['rearrangement']=rearranged
            elif left_program is not None and str(error).startswith(('No structure-clear grasp','No visible accessible measurable')):
                # Keep the active insertion moving before retrying a plan
                # that may currently be obstructed by the receiving arm.
                latest=next((f for e,f in reversed(vision.jobs) if e['mode']=='initial'),None)
                while left_program is not None:step('receiver_finishes_before_replan')
                if latest is not None:pickup_ctx['prefetched_initial']=latest
                p=pick.execute(pickup_ctx)
            else:raise
        G=np.array(p['measured_grasp']);D=.075 if part=='ring1'else .062
        while left_program is not None:step('await_receiver_clear')
        receiver_opening=min(.08,D+.006)
        if abs(float(cmd[0,fingers['left']].sum())-receiver_opening)>.0005:
            move('left',[cmd[0,indices['left']],cmd[0,indices['left']]],.10,part+'_receiver_empty_preshape',receiver_opening)
        row=ref[part]
        planner_path=HERE/'paired_handover.py';planner=types.ModuleType('paired_handover');planner.__file__=str(planner_path);exec(compile(planner_path.read_text(),str(planner_path),'exec'),planner.__dict__)
        release_width=.001 if p.get('grasp_type')=='inner' else .08
        handover=planner.plan(c,part,G,donor_release_width=release_width);incoming=handover['incoming']
        if handover.get('approach_order')=='receiver_first':move('left',handover['receiver_route'],.4,part+'_receiver_preposition',receiver_opening)
        parallel=False
        if c['config'].get('parallel_transfer_approach') and handover.get('approach_order')!='receiver_first':
            rr=trajectory('right',handover['donor_route'],.05,checked_edges=True)
            ll=trajectory('left',handover['receiver_route'],.05,receiver_opening,checked_edges=True)
            module_path=HERE/'parallel_native_transfer.py';pair_module=types.ModuleType('parallel_native_transfer');pair_module.__file__=str(module_path);exec(compile(module_path.read_text(),str(module_path),'exec'),pair_module.__dict__)
            rc=checkers['right'];rc.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@G)
            try:paired=pair_module.schedule(rc,checkers['left'],rr,ll,cmd[0,indices['right']],cmd[0,indices['left']],float(art.get_dof_positions()[0,fingers['right']].sum()),receiver_opening)
            finally:rc.detach_payload()
            if paired is not None:
                delay=paired['delay_steps'];r0=cmd[0,indices['right']].copy();l0=cmd[0,indices['left']].copy()
                for k in range(max(len(rr),delay+len(ll))):
                    if k<len(rr):cmd[0,indices['right']]=rr[k,:7];cmd[0,fingers['right']]=rr[k,7:]
                    if delay<=k<delay+len(ll):cmd[0,indices['left']]=ll[k-delay,:7];cmd[0,fingers['left']]=ll[k-delay,7:]
                    step(part+'_parallel_transfer_approach')
                parallel=True;events.append(dict(event=part+'_parallel_transfer_approach',wall_s=time.perf_counter()-c['started'],**paired));print('LOOSE_PARALLEL_TRANSFER',part,paired,flush=True)
        if not parallel:move('right',handover['donor_route'],.8,part+'_carry_to_handover')
        # Transport can change the grasp through real frictional slip. Adapt
        # the receiving hand to a NEW sensor pose, preserving its intended
        # hand-relative grasp, instead of closing at the old nominal location.
        intended_receiver_grasp=np.linalg.inv(arms['left'].fk(handover['receiver_q']))@incoming
        G,donor_pose=observe('right',part,G,part+'_donor_before_transfer_RGBD')
        incoming=frame(donor_pose['center_m'],axis_frame(donor_pose['axis']))
        receiving_H=incoming@np.linalg.inv(intended_receiver_grasp)
        route=None
        candidates=[(receiving_H,None)]+list(transfer.receiving_poses(c,part,incoming))
        for candidate_H,metadata in candidates:
            receiving_q=arms['left'].solve(candidate_H[:3,3],candidate_H[:3,:3],handover['receiver_q'])
            if receiving_q is None:continue
            # The giver must also be able to OPEN after the receiver closes.
            # Checking only the incoming held width missed that swept volume.
            candidate_G=np.linalg.inv(candidate_H)@incoming;expected_receiver_width=transfer.pad_aperture(c,part,candidate_G)
            if expected_receiver_width is None:continue
            donor_H=arms['right'].fk(cmd[0,indices['right']]);donor_width=float(art.get_dof_positions()[0,fingers['right']].sum())
            if not transfer.sequential_transfer_clear(checkers['right'],donor_H,donor_width,checkers['left'],candidate_H,expected_receiver_width,receiver_opening,donor_release_width=release_width):continue
            with checkers['left'].other_arm_at(checkers['right'],cmd[0,indices['right']],donor_width):
                if not all(checkers['left'].clear(receiving_q,w)[0] for w in np.linspace(receiver_opening,max(.001,expected_receiver_width-.003),5)):continue
            # The fresh-image replan must include the HELD RING as an
            # obstacle too. The paired planner's payload was detached on
            # return, which let a later receiver path sweep through the rim.
            checkers['right'].attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@G)
            try:
                with checkers['left'].other_arm_at(checkers['right'],cmd[0,indices['right']],donor_width):
                    route=transfer.hinted_route(c,part+'_receiver_approach',checkers['left'],cmd[0,indices['left']],receiving_q,receiver_opening,seconds=.50)
            finally:checkers['right'].detach_payload()
            if route is not None:
                receiving_H=candidate_H;intended_receiver_grasp=np.linalg.inv(receiving_H)@incoming;break
        if route is None:raise RuntimeError('No fresh-RGBD receiver route that permits giver opening')
        move('left',route,.4,part+'_receive_approach',receiver_opening)
        G,preclose_pose=observe('right',part,G,part+'_donor_after_receiver_approach_RGBD')
        preclose_part=frame(preclose_pose['center_m'],axis_frame(preclose_pose['axis']))
        if np.linalg.norm(preclose_part[:3,3]-incoming[:3,3])>.003 or np.arccos(np.clip(preclose_part[:3,2]@incoming[:3,2],-1,1))>.05:
            raise RuntimeError('Receiver approach disturbed held ring; closing is blocked')
        actual_receiving_H=arms['left'].fk(art.get_dof_positions()[0,indices['left']])
        expected_receiver_width=transfer.pad_aperture(c,part,np.linalg.inv(actual_receiving_H)@preclose_part)
        if expected_receiver_width is None:raise RuntimeError('Fresh preclose RGB-D does not confirm opposing receiver pad geometry')
        incoming=preclose_part
        c['stiffness'][0,fingers['left']]=0.;c['damping'][0,fingers['left']]=[4000.,.001];art.set_dof_stiffnesses(c['stiffness'],c['idx']);art.set_dof_dampings(c['damping'],c['idx'])
        c['set_left_force'](float(c['config'].get('native_grip_pad_force_n',30.)))
        receiver_target=max(.001,expected_receiver_width-.003)
        for attempt in range(2):
            c['set_grip_hold']('left',True)
            move('left',[cmd[0,indices['left']],cmd[0,indices['left']]],.15,part+'_receiver_close',receiver_target);wait(.08,part+'_receiver_contact_check')
            for _ in range(12):
                gap=float(art.get_dof_positions()[0,fingers['left']].sum())
                if gap<=expected_receiver_width+.001 and np.max(abs(art.get_dof_velocities()[0,fingers['left']]))<.010:break
                if gap<=receiver_target+.001:break
                wait(.025,part+'_receiver_grasp_controller_completion')
            aperture=float(art.get_dof_positions()[0,fingers['left']].sum())
            events.append(dict(event=part+'_receiver_contact',aperture_m=aperture,attempt=attempt,wall_s=time.perf_counter()-c['started']))
            if receiver_target+.0005<aperture<min(D+.0035,expected_receiver_width+.002):break
            if attempt==1:raise RuntimeError('Receiver pad contact not confirmed; donor stays closed')
            c['set_grip_hold']('left',False);move('left',[cmd[0,indices['left']],cmd[0,indices['left']]],.15,part+'_receiver_retry_open',receiver_opening)
            G,donor_pose=observe('right',part,G,part+'_donor_contact_retry_RGBD');incoming=frame(donor_pose['center_m'],axis_frame(donor_pose['axis']));receiving_H=incoming@np.linalg.inv(intended_receiver_grasp)
            receiving_q=arms['left'].solve(receiving_H[:3,3],receiving_H[:3,:3],cmd[0,indices['left']])
            route=checkers['left'].route(cmd[0,indices['left']],receiving_q,receiver_opening,seconds=.25) if receiving_q is not None else None
            if route is None:raise RuntimeError('No image-corrected receiver retry route; donor stays closed')
            move('left',route,.25,part+'_receiver_contact_retry')
        c['set_grip_hold']('left',True);wait(.12,part+'_bilateral_handover_hold');c['set_grip_hold']('right',False);move('right',[cmd[0,indices['right']],cmd[0,indices['right']]],.15,part+'_donor_open',.001 if p.get('grasp_type')=='inner' else .08)
        left_H=arms['left'].fk(art.get_dof_positions()[0,indices['left']]);left_G=np.linalg.inv(left_H)@incoming
        def protect_received_ring(query):
            # Empty giver retreat must avoid the workpiece now carried by
            # the receiver. Robot-only queries can otherwise push through it
            # and wrench it out of the receiving pads without arm-arm contact.
            lc=checkers['left'];rc=checkers['right']
            lc.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@left_G)
            try:
                with rc.other_arm_at(lc,cmd[0,indices['left']],float(art.get_dof_positions()[0,fingers['left']].sum())):
                    return query()
            finally:lc.detach_payload()
        route=protect_received_ring(lambda:planner.empty_giver_route(checkers['right'],cmd[0,indices['right']],release_width))
        if route is None:
            # Pull the received ring out of the open giver instead of forcing
            # the giver's elbow through the receiver's arm. Contact and all
            # static/pair collision queries stay active.
            donor_root=arms['right'].fk(cmd[0,indices['right']])@frame([0,0,-.1034],np.eye(3));away=incoming[:3,3]-donor_root[:3,3];away/=max(np.linalg.norm(away),1e-9)
            pull=arms['left'].fk(cmd[0,indices['left']]);pull[:3,3]+=away*.045;pull_q=arms['left'].solve(pull[:3,3],pull[:3,:3],cmd[0,indices['left']])
            checkers['left'].attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@left_G)
            try:pull_route=checkers['left'].route(cmd[0,indices['left']],pull_q,D,seconds=.25) if pull_q is not None else None
            finally:checkers['left'].detach_payload()
            if pull_route is not None:
                move('left',pull_route,.45,part+'_receiver_pull_from_open_giver');route=protect_received_ring(lambda:planner.empty_giver_route(checkers['right'],cmd[0,indices['right']],release_width))
        if route is None:raise RuntimeError('No collision-free giver withdrawal; receiver remains closed')
        move('right',route,.45,part+'_donor_clear')
        H=arms['right'].fk(cmd[0,indices['right']]);H[:3,3]+=[.10,-.10,0.];q=arms['right'].solve(H[:3,3],H[:3,:3],cmd[0,indices['right']])
        route=protect_received_ring(lambda:checkers['right'].route(cmd[0,indices['right']],q,release_width,seconds=.15)) if q is not None else None
        extra_lateral_clearance=route is not None
        # The short giver escape was already collision-checked and executed.
        # An optional extra waypoint is not a task requirement. Hold that
        # valid pose when it is unreachable; all subsequent carried/empty
        # motions still plan against the actual other arm before execution.
        if route is None:route=[cmd[0,indices['right']].copy(),cmd[0,indices['right']].copy()]
        clearing_rows=trajectory('right',route,.05 if c['config'].get('optimized_motion_timing') else .45,release_width,checked_edges=True)
        def empty_clearance_program():
            for row_value in clearing_rows:
                cmd[0,indices['right']]=row_value[:7];cmd[0,fingers['right']]=row_value[7:];yield
        clearing=empty_clearance_program()
        left_G,pose=observe('left',part,left_G,part+'_receiver_RGBD',background=clearing);results.append(dict(part=part,pickup=p,receiver_pose=pose,handover_planning=dict(elapsed_s=handover['elapsed_s'],counts=handover['counts'])))
        if part==ring_order[0] and not c['config'].get('one_ring_only'):next_initial=vision.request(dict(mode='initial',part=None),arms['right'].fk(art.get_dof_positions()[0,indices['right']]))
        if part==ring_order[-1] and not c['config'].get('one_ring_only') and not pin_first and not c['config'].get('pin_between_rings'):
            pin_event=dict(mode='pin_initial',part='pin',label='pin_initial');pin_initial=(pin_event,vision.request(pin_event,arms['right'].fk(art.get_dof_positions()[0,indices['right']])))
        for _ in clearing:step(part+'_donor_leave_receiver_corridor')
        if part=='ring2':
            previous_G=left_G.copy();wait(.15,part+'_receiver_stable_hold')
            left_G,pose=observe('left',part,left_G,part+'_receiver_stable_hold_RGBD')
            if np.linalg.norm(left_G[:3,3]-previous_G[:3,3])>.0015 or np.arccos(np.clip(left_G[:3,2]@previous_G[:3,2],-1,1))>.025:
                raise RuntimeError('Small ring slipped in receiver after giver withdrawal')
            if transfer.pad_aperture(c,part,left_G) is None:
                raise RuntimeError('Small ring lacks stable receiver pad geometry after withdrawal')
            results[-1]['receiver_stable_hold_RGBD']=pose
        events.append(dict(event=part+'_donor_leave_receiver_corridor',wall_s=time.perf_counter()-c['started'],overlapped_receiver_RGBD=True,extra_lateral_clearance_used=extra_lateral_clearance,motion_s=len(clearing_rows)*dt))
        if c['config'].get('handover_only'):return dict(state='physical_handover_camera_confirmed',parts=results,events=events,full_task_pass=False)
        # Adapt the already taught approach shape to the newly observed grasp.
        # These poses come only from stored robot commands + calibrated goals.
        # No recorded or live free-workpiece transforms are used.
        arm=arms['left'];ring_path=np.array(row['approach_ring_poses']);seed_path=np.array(row['approach_robot_seeds']);pre_index=int(row['pre_index']);approach=[]
        goal=np.asarray(c['goals'][part]['P']);goal_R=np.eye(3);route=None;carry_started=time.perf_counter()
        lc=checkers['left'];lc.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@left_G)
        free_yaw=part=='ring2' and c['config'].get('ring_yaw_free')
        try:
            if free_yaw:
                # Class/task IK hint from a saved RGB-D + encoder stroke probe.
                # It contains robot joints, never a scene seed or stock pose.
                insertion_hint=np.asarray(c['robot_route_hints']['ring2_preinsert_q'])
                seeds=[insertion_hint,seed_path[pre_index],row['left_insert_seed'],cmd[0,indices['left']]]+list(np.random.default_rng(7812).uniform(arm.bounds[:,0],arm.bounds[:,1],(48,7)))
                insertion_counts={};insertion_examples={}
                def reject_insertion(key,why=None):
                    insertion_counts[key]=insertion_counts.get(key,0)+1
                    if why is not None and key not in insertion_examples:insertion_examples[key]=why
                for final_H,azimuth in transfer.insertion_pose_options(c,part,left_G):
                    candidate_R=(final_H@left_G)[:3,:3]
                    for clearance in [.080,.060,.045,.100]:
                        pre_hand=frame(goal-[0,0,clearance],candidate_R)@np.linalg.inv(left_G);solutions=[]
                        hand_ok,hand_why=lc.hand_clear(pre_hand,D)
                        if not hand_ok:reject_insertion('fixed_pre_hand',hand_why);continue
                        hand_ok,hand_why=lc.hand_clear(final_H,D)
                        if not hand_ok:reject_insertion('fixed_final_hand',hand_why);continue
                        for seed in seeds:
                            q=arm.solve(pre_hand[:3,3],pre_hand[:3,:3],seed)
                            if q is not None and all(np.linalg.norm(q-old)>.05 for old in solutions):solutions.append(q)
                        solutions.sort(key=lambda q:np.linalg.norm(q-insertion_hint))
                        if not solutions:reject_insertion('pre_IK')
                        for q in solutions:
                            # Intentional ring/jig contact is simulated during
                            # insertion. Verify the full ROBOT stroke here;
                            # retain payload checks on the free carry route.
                            saved=(lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative);lc.detach_payload();stroke=[q];valid=True
                            try:
                                for u in np.linspace(0,1,9)[1:]:
                                    position=pre_hand[:3,3]*(1-u)+final_H[:3,3]*u;nq=arm.solve(position,final_H[:3,:3],stroke[-1])
                                    if nq is None:reject_insertion('stroke_IK');valid=False;break
                                    clear,why=lc.path([stroke[-1],nq],D)
                                    if not clear:reject_insertion('stroke_robot_collision',why);valid=False;break
                                    stroke.append(nq)
                            finally:lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative=saved
                            if not valid:continue
                            endpoint_clear,endpoint_why=lc.clear(q,D)
                            if not endpoint_clear:reject_insertion('carry_endpoint_collision',endpoint_why);continue
                            candidate_route=transfer.hinted_route(c,'ring2_left_carry',lc,cmd[0,indices['left']],q,D,seconds=1.)
                            if candidate_route is None:reject_insertion('carry_route')
                            if candidate_route is not None:route=candidate_route;goal_R=candidate_R;results[-1]['ring_goal_azimuth_rad']=azimuth;results[-1]['preinsert_clearance_m']=clearance;break
                        if route is not None:break
                    if route is not None:break
            else:
                pre_hand=frame(goal-[0,0,.080],np.eye(3))@np.linalg.inv(left_G)
                for seed in [seed_path[pre_index],row['left_insert_seed'],cmd[0,indices['left']]]:
                    q=arm.solve(pre_hand[:3,3],pre_hand[:3,:3],seed)
                    if q is not None:route=lc.route(cmd[0,indices['left']],q,D,seconds=.20)
                    if route is not None:break
        finally:lc.detach_payload()
        if free_yaw:
            diagnostics=dict(event='ring2_insertion_search',counts=insertion_counts,examples=insertion_examples,found=route is not None,planning_s=time.perf_counter()-carry_started)
            c['planning_diagnostics'].append(diagnostics);print('LOOSE_INSERTION_SEARCH',diagnostics,flush=True)
        shortcut=route is not None
        if route is None and free_yaw:raise RuntimeError('No payload-clear carry and robot-clear axial stroke over ring azimuth search')
        if route is None:
            for ring_H,seed_q in zip(ring_path[:pre_index+1],seed_path[:pre_index+1]):
                hand_H=ring_H@np.linalg.inv(left_G);q=arm.solve(hand_H[:3,3],hand_H[:3,:3],seed_q)
                if q is None:raise RuntimeError('No adapted taught ring approach IK')
                approach.append(q)
            route=[cmd[0,indices['left']].copy()]+approach
        results[-1]['left_carry_shortcut']=shortcut
        print('LOOSE_LEFT_CARRY_PLAN',part,shortcut,time.perf_counter()-carry_started,flush=True)
        end=ring_path[-1]@np.linalg.inv(left_G)
        def insert_program(part=part,G=left_G.copy(),route=route,end=end.copy(),D=D,row=row,free_yaw=free_yaw):
            arm=arms['left'];ids=indices['left'];fi=fingers['left'];goal=np.array(c['goals'][part]['P'])
            def rows(qs,t,width=None,checked_edges=False):
                for r in trajectory('left',qs,t,width,checked_edges=checked_edges):
                    cmd[0,ids]=r[:7];cmd[0,fi]=r[7:];yield
            def hold(t):
                for _ in range(round(t/dt)):yield
            def photo(label):
                # A weak side-only ICP orientation must not steer a precision
                # insertion. Re-expose the actual cap from bounded clear views.
                offsets=[[-.12,-.20,-.28],[-.22,.16,-.25],[-.20,-.25,-.12]]
                for view_index,offset in enumerate(offsets):
                    e=dict(mode='held_left',part=part,tracked_grasp=G.tolist(),camera_eye_offset=offset,label=label,require_annular_face=True,view_index=view_index);future=vision.request(e,arm.fk(art.get_dof_positions()[0,ids]));deadline=time.perf_counter()+3
                    while not future.done():
                        vision.poll()
                        if time.perf_counter()>deadline:raise RuntimeError('Async left RGB-D deadline')
                        yield
                    value=future.result();e['delivered_wall_s']=time.perf_counter()-c['started']
                    if not value['result'].get('error'):return value
                raise RuntimeError('No directly observed annular face for insertion correction')
            def grasp_from(v):
                p=v['result']['pose'];return np.linalg.inv(v['H'])@frame(p['center_m'],axis_frame(p['axis'])),p
            def cartesian(target,t):
                H=arm.fk(cmd[0,ids]);qs=[cmd[0,ids].copy()]
                for u in np.linspace(0,1,5)[1:]:
                    R=exp(log(target[:3,:3]@H[:3,:3].T)*u)@H[:3,:3]
                    q=arm.solve(H[:3,3]*(1-u)+target[:3,3]*u,R,qs[-1])
                    if q is None:raise RuntimeError('Async left Cartesian IK')
                    qs.append(q)
                if free_yaw:
                    curve=CubicSpline(np.linspace(0,1,len(qs)),qs)(np.linspace(0,1,33));opening=max(float(art.get_dof_positions()[0,fi].sum()),float(cmd[0,fi].sum()))
                    clear,rejection=checkers['left'].path(curve,opening)
                    if not clear:raise RuntimeError('Image-corrected insertion/withdrawal robot path hits structure '+str(dict(opening_m=opening,collision=rejection)))
                yield from rows(qs,t)
            def aligned_hand(position):
                H=arm.fk(cmd[0,ids]);n=H[:3,:3]@G[:3,2];cross=np.cross(n,[0.,0.,1.]);length=np.linalg.norm(cross)
                rotation=exp(cross/max(length,1e-12)*np.arctan2(length,n[2]))@H[:3,:3]
                return frame(np.asarray(position)-rotation@G[:3,3],rotation)
            yield from rows(route,.05 if c['config'].get('optimized_motion_timing') else 1.4,checked_edges=True)
            c['set_left_force'](30.)
            for alignment in range(8):
                v=yield from photo(part+'_preinsert_RGBD');G,p=grasp_from(v)
                P=np.asarray(p['center_m']);radial=float(np.linalg.norm(P[:2]-goal[:2]));angle=float(np.arccos(np.clip(p['axis'][2],-1,1)))
                print('LOOSE_LEFT_PREALIGN',part,radial,angle,flush=True)
                if radial<.0005 and angle<.010:break
                if alignment==7:raise RuntimeError('Free-space ring alignment not confirmed by RGB-D')
                safe_z=min(float(P[2]),float(goal[2]-.045))
                target=aligned_hand([goal[0],goal[1],safe_z]) if free_yaw else frame([goal[0],goal[1],safe_z],np.eye(3))@np.linalg.inv(G)
                yield from cartesian(target,.40);yield from hold(.08)
            final_hand=aligned_hand(goal) if free_yaw else frame(goal,np.eye(3))@np.linalg.inv(G)
            yield from cartesian(final_hand,.8);yield from hold(.12)
            for correction in range(3):
                v=yield from photo(part+'_full_depth_RGBD');G,p=grasp_from(v);P=np.array(p['center_m']);error=float(np.linalg.norm(P-goal));angle=float(np.arccos(np.clip(p['axis'][2],-1,1)));remaining=float(goal[2]-P[2]);radial=float(np.linalg.norm(P[:2]-goal[:2]));print('LOOSE_LEFT_DEPTH',part,error,angle,flush=True)
                if error<.0015 and angle<.05:break
                if correction==2 or not .0005<remaining<.020 or radial>.0015 or angle>.05:raise RuntimeError('Ring release blocked: full seating not visible '+part+str((error,angle,remaining)))
                H=arm.fk(cmd[0,ids]);H[2,3]+=remaining;yield from cartesian(H,.30);yield from hold(.12)
            c['close_clamp'](part);yield from hold(.12);c['set_grip_hold']('left',False)
            # Release with clearance around the measured 62 mm ring rather
            # than spreading to 80 mm inside its narrow fixture. Reverse the
            # insertion direction before a later empty-hand reposition.
            release_opening=min(.08,D+.002) if part=='ring2' else .08
            yield from rows([cmd[0,ids],cmd[0,ids]],.20,release_opening);yield from hold(.25)
            H=arm.fk(cmd[0,ids])
            if part=='ring2':H[2,3]-=.08
            else:H=H@frame([0,0,-.08],np.eye(3))
            yield from cartesian(H,.6);c['set_left_force'](float(c['config'].get('native_grip_pad_force_n',30.)));results[-1]['insertion_fresh_RGBD']=p
            if part==ring_order[0] and not c['config'].get('one_ring_only') and not c['config'].get('pin_between_rings'):
                return_route=checkers['left'].route(cmd[0,ids],left_standby_q,.08,seconds=.35)
                if return_route is None:raise RuntimeError('No empty receiver return route to handover standby')
                yield from rows(return_route,.05 if c['config'].get('optimized_motion_timing') else .8,checked_edges=True)
                next_opening=min(.08,(.075 if ring_order[1]=='ring1' else .062)+.006)
                yield from rows([cmd[0,ids],cmd[0,ids]],.10,next_opening)
        left_program=insert_program()
        if c['config'].get('pin_between_rings') and part==ring_order[0] and not c['config'].get('one_ring_only'):
            pin_report=run_pin()
            next_initial=vision.request(dict(mode='initial',part=None),arms['right'].fk(art.get_dof_positions()[0,indices['right']]))
    if c['config'].get('one_ring_only'):
        while left_program is not None:step('one_ring_component_withdraw')
        wait(.55,'one_ring_component_retention')
        return dict(state='one_ring_component_executed',parts=results,events=events,full_task_pass=False,one_ring_only=True)
    # The pin module shares the same step callback; the queued final ring
    # insertion continues while the right hand approaches the existing holder.
    if c['config'].get('stability_serial') or c['config'].get('stability_finish_rings_before_pin'):
        while left_program is not None:step('stable_final_ring_finishes_before_pin')
    if not pin_first and pin_report is None:pin_report=run_pin(pin_initial)
    while left_program is not None:step('finish_final_ring_insertion')
    retention_begin=max(left_finished_step,pin_retention_start if pin_retention_start is not None else control_steps)
    wait(max(0.,.55-(control_steps-retention_begin)*dt),'loose_all_parts_retention')
    return dict(state='loose_bin_recipe_executed',parts=results,pin=pin_report,events=events,full_task_pass=False,elapsed_s=time.perf_counter()-c['started'])
