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


def direct_right_insert_allowed(config,part):
    """Choose a task role per size; geometry/pose gates remain mandatory."""
    overrides=config.get('allow_direct_right_insert_by_part')
    if overrides is not None:
        if not isinstance(overrides,dict) or any(key not in ['ring1','ring2'] or type(value) is not bool for key,value in overrides.items()):
            raise ValueError('Direct RIGHT per-part policy must map ring1/ring2 to booleans')
        if part in overrides:return overrides[part]
    return bool(config.get('allow_direct_right_insert'))


def _left_endpoint_candidates(c,arm,target,seeds,current,preferred,label):
    """GPU endpoint proposals first; lazy original CPU branches on rejection."""
    from fr3_gpu_ik import propose_ik
    seen=[]
    gpu=propose_ik(c,arm,target[:3,3],target[:3,:3],current,label)
    gpu.sort(key=lambda q:np.linalg.norm(q-preferred))
    for q in gpu:
        seen.append(q);yield q
    fallback=[]
    for seed in seeds:
        q=arm.solve(target[:3,3],target[:3,:3],seed)
        if q is not None and all(np.linalg.norm(q-old)>.05 for old in seen):
            seen.append(q);fallback.append(q)
    fallback.sort(key=lambda q:np.linalg.norm(q-preferred))
    yield from fallback


def _encoder_bias_compensated_target(command_H,actual_H,desired_actual_H):
    """One bounded SE(3) command correction using robot FK only.

    The measured grasp defines desired_actual_H. A locally constant tracking
    error actual_H @ inv(command_H) would remain after an absolute command;
    applying its inverse cancels that bias without changing the motion limits.
    """
    command_H=np.asarray(command_H,float);actual_H=np.asarray(actual_H,float);desired_actual_H=np.asarray(desired_actual_H,float)
    if any(T.shape!=(4,4) or not np.isfinite(T).all() for T in [command_H,actual_H,desired_actual_H]):
        raise RuntimeError('Invalid encoder-bias feedback transform')
    translation=float(np.linalg.norm(command_H[:3,3]-actual_H[:3,3]))
    angle=float(np.linalg.norm(log(command_H[:3,:3]@actual_H[:3,:3].T)))
    if translation>.002 or angle>.01:
        raise RuntimeError('Large receiver encoder pose bias exceeds bounded2mm/.01rad correction '+str(dict(translation_m=translation,angle_rad=angle)))
    target=command_H@np.linalg.inv(actual_H)@desired_actual_H
    applied_translation=float(np.linalg.norm(target[:3,3]-desired_actual_H[:3,3]))
    if applied_translation>.002:
        raise RuntimeError('Large receiver applied pose-bias translation exceeds2mm')
    return target,dict(command_vs_actual_translation_m=translation,command_vs_actual_rotation_rad=angle,applied_target_translation_m=applied_translation,source='Current robot command and encoder FK; desired actual hand pose from fresh RGB-D',uses_simulator_body_poses=False)

def _annular_cap_camera_view_bank(hand_world,measured_grasp):
    """Virtual cap views from robot encoders and last direct RGB-D only.

    Six azimuths on each cap-axis hemisphere expose either signed face as
    hands or fixtures occlude an earlier view. These are movable simulated
    camera poses, not a claim that one fixed physical RealSense sees them all.
    """
    H=np.asarray(hand_world,float);G=np.asarray(measured_grasp,float)
    if any(T.shape!=(4,4) or not np.isfinite(T).all() for T in [H,G]):
        raise RuntimeError('Invalid image/encoder transform for annular camera views')
    axis=(H@G)[:3,2];axis=axis/max(np.linalg.norm(axis),1e-12)
    reference=np.eye(3)[int(np.argmin(abs(axis)))];u=reference-axis*(reference@axis)
    u/=max(np.linalg.norm(u),1e-12);v=np.cross(axis,u)
    views=[(.20*sign*axis+.23*(np.cos(angle)*u+np.sin(angle)*v)).tolist()
            for sign in [1.,-1.] for angle in np.arange(6)*np.pi/3]
    # Retain all original view priorities; only after those fail, expose the
    # same directly measured cap from axial and near-axial camera positions.
    # Poses depend on last public RGB-D + SDK only, never engine object state.
    views += [(sign*.28*axis).tolist() for sign in [1.,-1.]]
    views += [(sign*.26*axis+.08*(np.cos(angle)*u+np.sin(angle)*v)).tolist()
              for sign in [1.,-1.] for angle in np.arange(4)*np.pi/2]
    return views

def execute(c):
    HERE=Path(__file__).resolve().parent;c['robot_route_hints']=json.loads((HERE/'bin-picking/live-robot-route-hints.json').read_text());cmd=c['cmd'];art=c['art'];dt=c['dt'];vision=c['vision'];phase='initial';events=[]
    arms={'right':c['arm'],'left':c['left_arm']};indices={'right':c['ids'],'left':c['left_ids']};fingers={'right':c['fi'],'left':c['left_fi']};checkers=c['checkers'];left_program=None;control_steps=0;left_finished_step=0;pin_retention_start=None
    c['_collision_physics_epoch']=control_steps;c['_last_loaded_left_issued_full_proof']=None
    from placed_part_clearance import PlacedPartClearance
    completed_geometry=PlacedPartClearance()
    for planning_checker in checkers.values():planning_checker.placed_part_clearance=completed_geometry
    def register_completed(part,pose,source):
        if not pose or pose.get('part')!=part:raise RuntimeError('Completed part requires a directly observed class pose')
        target=np.asarray(c['goals'][part]['P'] if part!='pin' else c['config']['pin_axis_goal_world_m'],float)
        P=np.asarray(pose['center_m'],float);axis=np.asarray(pose['axis'],float)
        if np.linalg.norm(P-target)>=.0015 or np.arccos(np.clip(axis[2],-1,1))>=.05:raise RuntimeError('Completed part pose outside insertion tolerance')
        T=frame(P,axis_frame(axis))
        if part=='pin':
            from pin_native_contact import axis_to_body_grasp
            T=axis_to_body_grasp(T);template=c['pin_context']['paths']['workpiece_pin']
        else:template=c['part_collision_templates'][part]
        completed_geometry.register(checkers['right'],part,template,T,uncertainty_m=.0035)
        c['planning_diagnostics'].append(dict(event='completed_part_added_to_both_arm_collision_worlds',part=part,source=source,pose_from_RGBD=pose,uncertainty_m=.0035,uses_free_body_poses=False))
    loaded_contact_bindings={}
    overlap=None
    if c['config'].get('fixed_supply_paired_overlap'):
        if not c['config'].get('stability_serial'):raise RuntimeError('Paired fixed-supply policy retains the serial baseline guards')
        from paired_issued_overlap import PairedIssuedOverlap
        overlap=PairedIssuedOverlap(c)
    loaded_contact_capture_window=None
    last_annular_observation_steps={}
    def check_loaded_contact_bindings():
        if overlap is not None:overlap.check_right_binding()
        for carrier,binding in loaded_contact_bindings.items():
            binding.check(art.get_dof_positions()[0,fingers[carrier]])
    def step(label=None):
        nonlocal left_program,control_steps,left_finished_step,pin_retention_start
        check_loaded_contact_bindings()
        if label=='pin_retention' and pin_retention_start is None:pin_retention_start=control_steps
        if overlap is not None:overlap.prepare_idle_left_step()
        if left_program is not None and (overlap is None or overlap.advance_left_this_step()):
            try:next(left_program)
            except StopIteration:left_program=None;left_finished_step=control_steps
        c['_last_loaded_left_issued_full_proof']=None
        c['step'](label or phase)
        control_steps+=1
        c['_collision_physics_epoch']=control_steps
        if overlap is not None:overlap.step_completed()
        for callback in c.get('_encoder_capture_callbacks',()):callback(control_steps*dt)
        if loaded_contact_capture_window is not None:
            loaded_contact_capture_window.append((control_steps*dt,art.get_dof_positions()[0,fingers['left']].copy()))
        check_loaded_contact_bindings()
    def prime_left_chunk():
        nonlocal left_program,left_finished_step
        if left_program is not None and overlap.chunk is None:
            try:next(left_program)
            except StopIteration:left_program=None;left_finished_step=control_steps
    def finish_left_before_new_contact():
        while left_program is not None:
            overlap.before_stationary_left()
            step('paired_receiver_finishes_before_new_donor_contact')
    def issue_checked_left_lead(count):
        for _ in range(count):step('paired_LEFT_lead_before_RIGHT_pickup')
    if overlap is not None:
        overlap.prime=prime_left_chunk;overlap.drain=finish_left_before_new_contact;overlap.lead_steps=issue_checked_left_lead
    def execute_pick(context):
        if overlap is None:return pick.execute(context)
        context=dict(context,_before_right_pick_rows=overlap.before_right,_after_right_pick_rows=overlap.after_right,_before_right_pick_contact=overlap.before_contact,_bind_right_pick_payload=overlap.bind_right_payload,_needs_right_pick_payload_binding=lambda:overlap.supports_loaded_resume,_refresh_right_pick_payload=overlap.refresh_right_payload,_overlap_control_time_s=lambda:control_steps*dt)
        overlap.begin_pick()
        try:
            result=pick.execute(context)
            overlap.end_pick()
            return result
        except BaseException:
            overlap.abort_pick()
            raise
    def wait(t,label):
        for _ in range(int(np.ceil(t/dt))):step(label)
    def trajectory(side,qs,requested,width=None,checked_edges=False):
        from fixed_small_left_executor import SavedCommandRoute
        if isinstance(qs,SavedCommandRoute):return qs.commands(c,side,width)
        from fixed_small_donor_executor import SavedDonorRoute
        if isinstance(qs,SavedDonorRoute):return qs.commands(c,side,width)
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
    def collision_template(part):
        if part=='pin':return c['pin_context']['paths']['workpiece_pin']
        return c['part_collision_templates'][part]
    def move(side,qs,t,label,width=None,checked_edges=True,carried_part=None,carried_grasp=None,other_carried_part=None,other_carried_grasp=None,_allow_saved_donor=True):
        nonlocal phase
        phase=label
        original_route=qs;saved_donor=False
        if _allow_saved_donor and side=='right' and label=='ring2_carry_to_handover' and carried_part=='ring2' and carried_grasp is not None:
            from fixed_small_donor_executor import candidate as donor_candidate
            proposal=donor_candidate(c,carried_part,carried_grasp,np.asarray(qs[-1],float))
            if proposal is not None:qs=proposal;saved_donor=True
        if c['config'].get('optimized_motion_timing') and any(word in label for word in ['carry','approach','preposition','corridor','donor_clear','pull_from']):t=.05
        rows=trajectory(side,qs,t,width,checked_edges=checked_edges)
        if carried_part is not None or other_carried_part is not None:
            if left_program is not None:raise RuntimeError('Serial approach preflight requires the opposite queued program to be finished')
            # Validate the ACTUAL serial execution curve, in the current
            # opposite-arm encoder context, before issuing its first row.
            # Existing route approval alone is not an execution certificate.
            from dual_aperture_path_check import path_dual_openings
            other_side='left' if side=='right' else 'right'
            checked,opposite=checkers[side],checkers[other_side]
            measured=art.get_dof_positions()[0].copy()
            actual_q=measured[indices[side]].copy()
            actual_other=measured[indices[other_side]].copy()
            own_values=np.r_[measured[fingers[side]].sum(),cmd[0,fingers[side]].sum(),rows[:,7:].sum(axis=1)]
            other_values=np.array([measured[fingers[other_side]].sum(),cmd[0,fingers[other_side]].sum()])
            own_widths=np.unique(np.linspace(own_values.min(),own_values.max(),5))
            other_widths=np.unique(np.linspace(other_values.min(),other_values.max(),5))
            path=np.vstack([actual_q,cmd[0,indices[side]],rows[:,:7]])
            saved=[(obj,obj.payload_path,obj.payload_rows,obj.payload_boxes,obj.payload_relative) for obj in [checked,opposite]]
            from contextlib import ExitStack
            proof_stack=ExitStack();saved_stats={};saved_error=None
            try:
                if carried_part is not None:
                    if carried_grasp is None:raise RuntimeError('Serial loaded guard requires an image-derived grasp')
                    checked.attach_payload(collision_template(carried_part),frame([0,0,.1034],np.eye(3))@np.asarray(carried_grasp))
                if other_carried_part is not None:
                    if other_carried_grasp is None:raise RuntimeError('Other-arm payload guard requires an image-derived grasp')
                    opposite.attach_payload(collision_template(other_carried_part),frame([0,0,.1034],np.eye(3))@np.asarray(other_carried_grasp))
                if saved_donor:
                    from fixed_small_donor_executor import static_scope as donor_static_scope
                    saved_stats=proof_stack.enter_context(donor_static_scope(c,checked))
                if c['config'].get('reuse_actual_other_context_checks',False):
                    from dual_aperture_path_check import path_other_contexts
                    contexts=[(other_q,float(opening))for other_q in [actual_other,cmd[0,indices[other_side]].copy()]for opening in other_widths]
                    from serial_gpu_pair_scope import serial_path_scope
                    stats={}
                    with serial_path_scope(c,checked,opposite,contexts,path,own_widths):
                        clear,why=path_other_contexts(checked,opposite,contexts,path,own_widths,diagnostics=stats)
                    c['planning_diagnostics'].append(dict(event='serial_handover_exact_issued_other_context_guard',side=side,label=label,clear=bool(clear),rejection=why,rows=len(rows),own_openings_m=own_widths.tolist(),carried_part=carried_part,other_carried_part=other_carried_part,uses_free_body_poses=False,**stats))
                    if not clear:raise RuntimeError('Serial handover actual issued curve is blocked before motion '+label+' '+str(why))
                else:
                    for other_q in [actual_other,cmd[0,indices[other_side]].copy()]:
                        stats={}
                        clear,why=path_dual_openings(checked,opposite,other_q,other_widths,path,own_widths,diagnostics=stats)
                        c['planning_diagnostics'].append(dict(event='serial_handover_exact_issued_guard',side=side,label=label,clear=bool(clear),rejection=why,rows=len(rows),actual_other_q=other_q.tolist(),own_openings_m=own_widths.tolist(),other_openings_m=other_widths.tolist(),carried_part=carried_part,other_carried_part=other_carried_part,uses_free_body_poses=False,**stats))
                        if not clear:raise RuntimeError('Serial handover actual issued curve is blocked before motion '+label+' '+str(why))
            except RuntimeError as error:
                if not saved_donor or not str(error).startswith('Serial handover actual issued curve is blocked before motion'):raise
                saved_error=error
            finally:
                try:proof_stack.close()
                finally:
                    for obj,pp,pr,pb,pt in saved:obj.payload_path,obj.payload_rows,obj.payload_boxes,obj.payload_relative=pp,pr,pb,pt
            if saved_error is not None:
                c['planning_diagnostics'].append(dict(event='saved_SMALL_donor_rejected',stage='original_full_current_guard',reason=str(saved_error),physical_rows_issued=0))
                return move(side,original_route,t,label,width,checked_edges,carried_part,carried_grasp,other_carried_part,other_carried_grasp,_allow_saved_donor=False)
            if saved_donor:c['planning_diagnostics'].append(dict(event='saved_SMALL_donor_admitted',original_actual_encoders_all_widths_both_other_contexts_and_payload_guards=True,**saved_stats))
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
        if c['config'].get('fixed_supply_mode') and side=='right' and part=='ring2' and label in [part+'_donor_after_receiver_approach_RGBD',part+'_bilateral_contact_RGBD',part+'_receiver_holding_force_RGBD',part+'_receiver_final_contact_RGBD']:
            views=[views[2],views[0],views[1]]
        if c['config'].get('robust_annular_camera_view_bank'):
            views+=_annular_cap_camera_view_bank(H,G)
        for view_index,offset in enumerate(views):
            observed_request_step=control_steps
            e=dict(mode='held' if side=='right' else 'held_left',part=part,tracked_grasp=G.tolist(),camera_eye_offset=np.asarray(offset,dtype=float).tolist(),label=label,require_annular_face=True,view_index=view_index);v=vision.request(e,arms[side].fk(art.get_dof_positions()[0,indices[side]]));deadline=time.perf_counter()+3
            while not v.done():
                vision.poll()
                if background is not None:
                    try:next(background)
                    except StopIteration:background=None
                step(label)
                if time.perf_counter()>deadline:raise RuntimeError('Dual-arm camera timeout')
            value=v.result();e['delivered_wall_s']=time.perf_counter()-c['started']
            if value['result'].get('error'):continue
            p=value['result']['pose'];Gobs=np.linalg.inv(value['H'])@frame(p['center_m'],axis_frame(p['axis']))
            last_annular_observation_steps[(side,part)]=observed_request_step
            return Gobs,p
        raise RuntimeError('No directly measured annular face in transfer camera views')
    def bind_initial_right_donor(part,G,label):
        # The spring target is deliberately inside the rigid ring. Only own
        # finger/held-CAD geometry may use this freshly certified encoder
        # interval; fixed/other/placed sweeps still use requested AND actual.
        from loaded_contact_aperture import LoadedContactAperture
        from receiver_pad_contact import contact_state
        from grasp_transfer_feasibility import large_donor_pad_support
        samples=[(control_steps*dt,art.get_dof_positions()[0,fingers['right']].copy())]
        callbacks=c.setdefault('_encoder_capture_callbacks',[])
        def sample_encoder(physics_s):
            samples.append((physics_s,art.get_dof_positions()[0,fingers['right']].copy()))
        callbacks.append(sample_encoder)
        try:
            wait(.1,label+'_stationary_encoder_window')
            fresh_G,pose=observe('right',part,G,label+'_fresh_RGBD')
            current=(control_steps*dt,art.get_dof_positions()[0,fingers['right']].copy())
            if current[0]>samples[-1][0]:samples.append(current)
            else:samples[-1]=current
        finally:callbacks.remove(sample_encoder)
        clean=dict(c,config=dict(c['config'],receiver_axial_only=False))
        gap=float(samples[-1][1].sum())
        contact=contact_state(clean,part,fresh_G,gap,contact_tolerance_m=.0008)
        support=large_donor_pad_support(clean,fresh_G,gap)
        if transfer.donor_aperture(clean,part,fresh_G) is None or not support['broad_opposed_flat_pad_support']:
            raise RuntimeError('Initial RIGHT donor lacks fresh stable native outer pad support')
        binding=LoadedContactAperture(side='right',part=part,
            payload_path=c['part_collision_templates'][part],
            encoder_samples=[r[1]for r in samples],sample_times_s=[r[0]for r in samples],
            contact=contact,measured_grasp=fresh_G,control_dt_s=dt,
            capture_age_physics_s=(control_steps-last_annular_observation_steps[('right',part)])*dt)
        old=loaded_contact_bindings.get('right')
        if old is not None:old.release()
        checkers['right'].loaded_contact_aperture=binding;loaded_contact_bindings['right']=binding
        evidence=dict(event='initial_RIGHT_donor_encoder_geometry_bound',label=label,support=support,**binding.record)
        c['planning_diagnostics'].append(evidence);print('LOOSE_INITIAL_DONOR_BOUND',evidence,flush=True)
        return fresh_G
    ref=c['handoff_reference'];results=[];next_initial=None;pin_initial=None;left_standby_q=cmd[0,indices['left']].copy()
    c['_fixed_supply_empty_left_standby_q']=left_standby_q.copy()
    transfer_path=HERE/'grasp_transfer_feasibility.py';transfer=types.ModuleType('grasp_transfer_feasibility');transfer.__file__=str(transfer_path);sys.modules['grasp_transfer_feasibility']=transfer;exec(compile(transfer_path.read_text(),str(transfer_path),'exec'),transfer.__dict__)
    direct_preview_cache={}
    def pickup_destination_feasible(part,pose,H):
        G=np.linalg.inv(H)@frame(pose['center_m'],axis_frame(pose['axis']))
        if direct_right_insert_allowed(c['config'],part) and pose.get('grasp_type','outer')=='outer' and G[2,2]>.1:
            key=(part,np.round(G,7).tobytes(),np.round(art.get_dof_positions()[0,indices['left']],6).tobytes(),round(float(art.get_dof_positions()[0,fingers['left']].sum()),6))
            if key in direct_preview_cache:return direct_preview_cache[key]
            width=transfer.donor_aperture(c,part,G)
            if width is None:direct_preview_cache[key]=False;return False
            from receiver_pad_contact import contact_state
            contact_context=dict(c,config=dict(c['config'],receiver_axial_only=False))
            if not contact_state(contact_context,part,G,width)['opposed_contact_compatible']:
                direct_preview_cache[key]=False;return False
            # Necessary fixed-hand preview only. Current grasp, full arm IK,
            # complete stroke and carry are checked again after actual pickup.
            release=.08 if part=='ring1' else min(.08,width+.002)
            from probe_native_right_ring import insertion_pose_options
            result=False
            for goal_H,_ in insertion_pose_options(c,part,G):
                if checkers['right'].hand_clear(goal_H,width)[0] and all(checkers['right'].hand_clear(goal_H,float(gap))[0] for gap in np.linspace(width,release,5)):
                    result=True;break
            direct_preview_cache[key]=result;return result
        return transfer.compatible(c,part,pose,H)
    ring_order=['ring2','ring1'] if c['config'].get('small_ring_first') else ['ring1','ring2']
    def initial_ring_event():
        # Fixed CaP task class, never a simulator observation or instance ID.
        expected=next((name for name in ring_order if name not in {row['part'] for row in results}),None) if c['config'].get('fixed_supply_mode') else None
        return dict(mode='initial',part=expected)
    def close_empty_left_for_pin(pin_grasp):
        if not c['config'].get('close_empty_left_for_pin'):return dict(empty_left_closed_for_pin=False,configured=False)
        # The normal pin preinsert can be blocked by an otherwise parked,
        # open empty receiver. Certify its whole closure before commanding it.
        if c.get('left_hand_known_empty') is not True:
            raise RuntimeError('Pin clearance closure requires a confirmed empty left hand')
        from predictive_pair_clearance import at_pose,shape_overlap,proxy
        from scipy.spatial import ConvexHull
        from pxr import UsdGeom,Gf,PhysicsSchemaTools,Sdf
        lc,rc=checkers['left'],checkers['right'];li,lf=indices['left'],fingers['left']
        measured=art.get_dof_positions()[0];left_q=measured[li].copy();right_q=measured[indices['right']].copy()
        current_width=float(measured[lf].sum());command_width=float(cmd[0,lf].sum());target_width=.010
        if max(current_width,command_width)<=target_width+.0005:
            return dict(empty_left_closed_for_pin=True,already_closed=True,actual_opening_m=current_width)
        start_width=max(current_width,command_width);right_width=float(measured[fingers['right']].sum())
        hand_H=arms['left'].fk(left_q);command_H=arms['left'].fk(cmd[0,li])
        a=transfer.swept_gripper_boxes(lc,hand_H,start_width,target_width)
        b=transfer.swept_gripper_boxes(lc,command_H,start_width,target_width)
        swept=[proxy(np.r_[x[3],y[3]]) for x,y in zip(a,b)]
        own_bodies={lc.arm.hand_path}|{lc.arm.hand_path+'/left_'+label+'_finger' for label in ['left','right']}
        sources=[(body,source)for body,source,_ in lc.pair_boxes if body in own_bodies]
        if not swept or len(sources)!=len(swept):raise RuntimeError('Missing continuous empty-left native closure geometry')
        saved_payload=(rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative)
        rc.attach_payload(c['pin_context']['paths']['workpiece_pin'],frame([0,0,.1034],np.eye(3))@pin_grasp)
        try:
            other=at_pose(rc,right_q,right_width)
            if any(shape_overlap(box,other_box,margin=.0005) for box in swept for _,_,other_box in other):
                raise RuntimeError('Empty-left closure sweep intersects current right arm or camera-estimated held pin')
            # Query conservative continuous finger unions, not just sampled
            # widths. These invisible meshes have no Collision/RigidBody API.
            query_rows=[];generation=c.get('_empty_pin_sweep_generation',0)+1;c['_empty_pin_sweep_generation']=generation
            for source_index,((body,source),box) in enumerate(zip(sources,swept)):
                vertices=box[3];hull=ConvexHull(vertices);faces=hull.simplices.copy()
                normals=np.cross(vertices[faces[:,1]]-vertices[faces[:,0]],vertices[faces[:,2]]-vertices[faces[:,0]])
                flip=np.einsum('ij,ij->i',normals,hull.equations[:,:3])<0;faces[flip]=faces[flip][:,[0,2,1]]
                path='/World/PlanningQueries/left_empty_pin_sweep_'+str(generation)+'_'+str(source_index)
                mesh=UsdGeom.Mesh.Define(lc.stage,path);mesh.CreatePointsAttr(vertices.astype(np.float32));mesh.CreateFaceVertexCountsAttr([3]*len(faces));mesh.CreateFaceVertexIndicesAttr(faces.ravel());mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateVisibilityAttr('invisible')
                xf=UsdGeom.Xformable(mesh);xf.ClearXformOpOrder();op=xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble)
                query_rows.append((body,source,op,PhysicsSchemaTools.encodeSdfPath(Sdf.Path(path))))
            with lc.other_arm_at(rc,right_q,right_width):
                if not lc.clear(left_q,start_width)[0] or not lc.clear(cmd[0,li],start_width)[0]:
                    raise RuntimeError('Current empty-left robot configuration is not clear before pin closure')
                saved_rows=lc.rows,lc.payload_rows
                try:
                    lc.rows=query_rows;lc.payload_rows=[]
                    clear,why=lc._clear_transforms({body:np.eye(4)for body,_ in sources},None)
                finally:lc.rows,lc.payload_rows=saved_rows
                if not clear:raise RuntimeError('Continuous empty-left closure sweep hits fixed structure '+str(why))
        finally:rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative=saved_payload
        c['set_grip_hold']('left',False)
        move('left',[cmd[0,li].copy(),cmd[0,li].copy()],.3,'pin_empty_left_close_for_clearance',target_width,other_carried_part='pin',other_carried_grasp=pin_grasp)
        deadline=time.perf_counter()+12.;window=[]
        for sample in range(120):
            window.append(art.get_dof_positions()[0,lf].copy());window=window[-5:];values=np.asarray(window)
            gap=float(values[-1].sum());stationary=len(window)==5 and np.max(np.ptp(values,axis=0))<=.00015 and np.ptp(values.sum(1))<=.00015
            if stationary and 0<=gap<=target_width+.001:break
            if time.perf_counter()>=deadline or sample==119:raise RuntimeError('Empty-left closure did not reach a stationary safe aperture before pin transport')
            wait(.025,'pin_empty_left_close_encoder_completion')
        result=dict(empty_left_closed_for_pin=True,actual_opening_m=gap,target_opening_m=target_width,continuous_native_sweep_checked=True,current_right_arm_and_RGBD_pin_checked=True,force_and_speed_caps_unchanged=True)
        c['planning_diagnostics'].append(dict(event='pin_empty_left_clearance_closure',**result));return result

    def clear_left_work_zone_for_pin(pin_grasp):
        nonlocal next_initial
        if c['config'].get('pin_first') and not results and next_initial is None:
            next_initial=vision.request(initial_ring_event(),arms['right'].fk(art.get_dof_positions()[0,indices['right']]))
        while left_program is not None:step('pin_waits_for_final_ring_release')
        if c['config'].get('pin_goal_aware_empty_left_park'):
            module=types.ModuleType('pin_goal_aware_park');module.__file__=str(HERE/'pin_goal_aware_park.py');exec(compile(Path(module.__file__).read_text(),module.__file__,'exec'),module.__dict__)
            return module.execute(c,pin_grasp,move,close_empty_left_for_pin,left_standby_q)
        # With the pin lifted clear of its holder, park the empty receiver.
        # Leaving it under ring 2 blocks the adjacent pin insertion corridor.
        if np.max(abs(cmd[0,indices['left']]-left_standby_q))<1e-4:return dict(empty_left_arm_parked=True,already_at_standby=True,**close_empty_left_for_pin(pin_grasp))
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
        if route is None and c['config'].get('close_empty_left_for_pin'):
            # Preserve the existing open-hand parking route when available.
            # Otherwise certify and close the logically empty hand HERE,
            # before retrying the same parking search at its measured width.
            closure=close_empty_left_for_pin(pin_grasp)
            measured=art.get_dof_positions()[0];width=float(measured[fingers['left']].sum())
            c['planning_diagnostics'].append(dict(event='pin_left_parking_retry_after_empty_closure',closure=closure,measured_opening_m=width))
            rc.attach_payload(c['pin_context']['paths']['workpiece_pin'],frame([0,0,.1034],np.eye(3))@pin_grasp)
            try:
                with lc.other_arm_at(rc,measured[indices['right']].copy(),float(measured[fingers['right']].sum())):
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
        return dict(empty_left_arm_parked=True,right_held_pin_in_collision_query=True,**close_empty_left_for_pin(pin_grasp))
    def run_pin(prefetched=None):
        if c['config'].get('wait_for_ring_transport_clear'):
            while left_program is not None and np.linalg.norm(arms['left'].fk(art.get_dof_positions()[0,indices['left']])[:3,3]-arms['right'].fk(art.get_dof_positions()[0,indices['right']])[:3,3])<.24:step('receiver_clears_empty_pin_preshape_volume')
        if c.get('set_empty_donor_contact_guard'):c['set_empty_donor_contact_guard']('right',False)
        pin_path=HERE/'direct_pin_skill.py';pin=types.ModuleType('direct_pin_skill');pin.__file__=str(pin_path);exec(compile(pin_path.read_text(),str(pin_path),'exec'),pin.__dict__)
        pin_ctx=dict(c['pin_context'],step=step,config=c['config'],robot_route_hints=c['robot_route_hints'],prefetched_initial=prefetched,before_transport=clear_left_work_zone_for_pin,goal_aware_pickup=bool(c['config'].get('pin_first') or c['config'].get('pin_between_rings')),defer_retention_to_recipe=bool(c['config'].get('pin_first') or c['config'].get('pin_between_rings')))
        if c['config'].get('pin_saved_canonical_carry',False):
            pin_ctx['_pin_saved_carry_cache_dir']=Path(c['config'].get('fixed_route_cache_dir',str(HERE.parents[2]/'.local/fixed-route-cache')))
            pin_ctx['report']['pin_saved_carry_preparation']=dict(deferred_until_current_public_G_admitted=True,compile_excluded_from_cycle=False)
        pin_result=pin.execute(pin_ctx)
        if c['config'].get('pin_grip_transport_only'):return pin_result
        seated=next((e['result']['pose']for e in reversed(pin_result['events'])if e['event']=='pin_full_depth_RGBD'and e.get('result',{}).get('pose')),None)
        register_completed('pin',seated,'Last fresh full-depth RGB-D; conservative placement obstacle, retention not certified')
        return pin_result
    if c['config'].get('pin_only_component',False):
        # Exercise the CURRENT pin skill in the CURRENT full scene. Both rings
        # remain in their ORIGINAL stock locations; neither is installed/held
        # synthetically, and no ring task or old-physics script is executed.
        if c['config'].get('one_ring_only') or c['config'].get('pin_first') or c['config'].get('pin_between_rings'):
            raise ValueError('PIN-only diagnostic cannot combine task-order/component flags')
        if c.get('left_hand_known_empty') is not True or left_program is not None or results:
            raise RuntimeError('PIN-only component must start before all ring tasks with known EMPTY LEFT')
        pin_component=run_pin()
        return dict(state='pin_grip_transport_component_executed'if c['config'].get('pin_grip_transport_only')else'pin_only_component_executed',component_only=True,full_task_pass=False,vision_end_to_end_pass=False,pin=pin_component,parts=[],ring_tasks_executed=0,rings_initialization='Original stock; no initially installed or held workpieces',events=events,elapsed_s=time.perf_counter()-c['started'],requires_whole_three_part_trial=True)
    if c['config'].get('park_before_bin_pick'):
        if c['config'].get('pin_first'):raise ValueError('Initial known-empty park must precede ring-first task')
        park=types.ModuleType('probe_empty_park_observe');park.__file__=str(HERE/'probe_empty_park_observe.py');exec(compile(Path(park.__file__).read_text(),park.__file__,'exec'),park.__dict__)
        parked=park.execute(c)
        if parked.get('state')!='empty_robot_park_then_single_fixed_RGBD_observed':raise RuntimeError('No same-arm-clear known-empty camera parking route')
        next_initial=vision.jobs[-1][1]
        vision.calibration['initial_eye_offset_m']=[0.,0.,.80]
        vision.calibration.update(initial_camera_width_px=768,initial_camera_height_px=768,initial_camera_span_m=.55)
        events.append(dict(event='initial_empty_robot_park_for_fixed_overhead_RGBD',wall_s=time.perf_counter()-c['started'],physical_robot_motion_s=parked['physical_robot_motion_s'],uses_free_body_poses=False))
    pin_first=bool(c['config'].get('pin_first'));pin_report=run_pin() if pin_first else None
    if c['config'].get('adaptive_initial_ring_order') and not c['config'].get('one_ring_only') and not c['config'].get('sequential_initial_ring_selection',True):
        preview_path=HERE/'loose_ring_pick.py';preview=types.ModuleType('pickup_order_preview');preview.__file__=str(preview_path);exec(compile(preview_path.read_text(),str(preview_path),'exec'),preview.__dict__)
        trials=[]
        for candidate_part in list(ring_order):
            context=dict(c,step=step,config=dict(c['config'],pick_part=candidate_part,pickup_preview_only=True,prefer_transfer_compatible=True),handover_feasible_grasp=lambda pose,H:pickup_destination_feasible(candidate_part,pose,H),grasp_pad_aperture=lambda pose,H:transfer.donor_aperture(c,candidate_part,np.linalg.inv(H)@frame(pose['center_m'],axis_frame(pose['axis']))))
            try:
                planned=preview.execute(context)
            except RuntimeError as error:
                if not str(error).startswith(('No structure-clear grasp','No visible accessible measurable')):raise
                trials.append(dict(part=candidate_part,pickup_route_available=False,reason=str(error)));continue
            if planned.get('state')!='RGBD_pickup_route_preview_only':raise RuntimeError('Pickup ordering preview unexpectedly executed a task')
            trials.append(dict(part=candidate_part,pickup_route_available=True,selected_image_pose=planned['selected_instance']))
            ring_order=[candidate_part]+[name for name in ring_order if name!=candidate_part]
            break
        order_evidence=dict(event='initial_RGBD_pickup_order',trials=trials,selected_order=list(ring_order),uses_free_body_poses=False,preview_robot_motion_executed=False,wall_s=time.perf_counter()-c['started'])
        events.append(order_evidence);c['planning_diagnostics'].append(order_evidence);print('LOOSE_INITIAL_PICKUP_ORDER',order_evidence,flush=True)
    for task_index,part in enumerate([c['config'].get('pick_part','ring1')] if c['config'].get('one_ring_only') else ring_order):
        if c['config'].get('stability_serial') and overlap is None:
            while left_program is not None:step('stable_receiver_finishes_before_next_pick')
        # The right hand keeps working while the queued left insertion runs.
        if c.get('set_empty_donor_contact_guard'):c['set_empty_donor_contact_guard']('right',False)
        pick_path=HERE/'loose_ring_pick.py';pick=types.ModuleType('loose_ring_pick');pick.__file__=str(pick_path);exec(compile(pick_path.read_text(),str(pick_path),'exec'),pick.__dict__)
        pickup_ctx=dict(c,step=step,config=dict(c['config'],pick_part=part,prefer_transfer_compatible=True),handover_feasible_grasp=lambda pose,H:pickup_destination_feasible(part,pose,H),grasp_pad_aperture=lambda pose,H:transfer.donor_aperture(c,part,np.linalg.inv(H)@frame(pose['center_m'],axis_frame(pose['axis']))))
        if next_initial is not None:pickup_ctx['prefetched_initial']=next_initial;next_initial=None
        try:p=execute_pick(pickup_ctx)
        except RuntimeError as error:
            # Select and execute one fresh plan instead of previewing and then
            # recomputing that same plan. Try the other size before rearranging
            # stock; all sensing, CAD/route, retention and issued-curve checks
            # remain inside the same pickup skill.
            eligible=str(error).startswith(('No structure-clear grasp','No visible accessible measurable'))
            if task_index==0 and eligible and c['config'].get('adaptive_initial_ring_order') and c['config'].get('sequential_initial_ring_selection',True) and not c['config'].get('one_ring_only'):
                original_part=part;alternate=next(name for name in ring_order if name!=part)
                alternate_ctx=dict(pickup_ctx,config=dict(pickup_ctx['config'],pick_part=alternate),handover_feasible_grasp=lambda pose,H,selected=alternate:pickup_destination_feasible(selected,pose,H),grasp_pad_aperture=lambda pose,H,selected=alternate:transfer.donor_aperture(c,selected,np.linalg.inv(H)@frame(pose['center_m'],axis_frame(pose['axis']))))
                alternate_ctx.pop('prefetched_initial',None)
                try:alternative_pick=execute_pick(alternate_ctx)
                except RuntimeError as alternate_error:
                    if not str(alternate_error).startswith(('No structure-clear grasp','No visible accessible measurable')):raise
                    events.append(dict(event='initial_size_alternative_unavailable',part=alternate,reason=str(alternate_error),wall_s=time.perf_counter()-c['started']))
                else:
                    p=alternative_pick;part=alternate;pickup_ctx=alternate_ctx;ring_order[:]=[alternate,original_part];error=None
                    evidence=dict(event='initial_RGBD_pickup_order',selected_order=list(ring_order),executed_single_fresh_plan=True,discarded_size_reason='No current accessible checked grasp',uses_free_body_poses=False,wall_s=time.perf_counter()-c['started'])
                    events.append(evidence);c['planning_diagnostics'].append(evidence);print('LOOSE_INITIAL_PICKUP_ORDER',evidence,flush=True)
            if error is None:pass
            elif c['config'].get('allow_single_finger_rearrangement') and str(error).startswith(('No structure-clear grasp','No visible accessible measurable')):
                rearrange_path=HERE/'loose_bin_rearrange.py';rearrange=types.ModuleType('loose_bin_rearrange');rearrange.__file__=str(rearrange_path);exec(compile(rearrange_path.read_text(),str(rearrange_path),'exec'),rearrange.__dict__)
                limit=int(c['config'].get('rearrangement_retry_limit',1))
                if not 1<=limit<=3:raise ValueError('Rearrangement retry limit must be between 1 and 3')
                history=[]
                for rearrangement_attempt in range(limit):
                    rearranged=rearrange.execute(pickup_ctx);history.append(rearranged);pickup_ctx.pop('prefetched_initial',None)
                    pickup_ctx['config']=dict(pickup_ctx['config'],initial_pickup_extra_eye_offsets_m=c['config'].get('rearrangement_pickup_extra_eye_offsets_m',[]))
                    if rearranged.get('state')=='RGBD_grasp_and_lift_confirmed':p=rearranged;break
                    try:
                        p=execute_pick(pickup_ctx);p['rearrangement']=rearranged;break
                    except RuntimeError as retry_error:
                        eligible=str(retry_error).startswith(('No structure-clear grasp','No visible accessible measurable'))
                        if not eligible or rearrangement_attempt+1>=limit:raise
                        events.append(dict(event=part+'_fresh_rearrangement_retry',attempt=rearrangement_attempt+2,reason=str(retry_error),wall_s=time.perf_counter()-c['started']))
                p['rearrangement_history']=history
            elif left_program is not None and str(error).startswith(('No structure-clear grasp','No visible accessible measurable')):
                # Keep the active insertion moving before retrying a plan
                # that may currently be obstructed by the receiving arm.
                latest=next((f for e,f in reversed(vision.jobs) if e['mode']=='initial'),None)
                while left_program is not None:step('receiver_finishes_before_replan')
                if latest is not None:pickup_ctx['prefetched_initial']=latest
                p=execute_pick(pickup_ctx)
            else:raise
        G=np.array(p['measured_grasp']);D=.075 if part=='ring1'else .062
        if direct_right_insert_allowed(c['config'],part) and p.get('grasp_type','outer')=='outer' and G[2,2]>.1:
            while left_program is not None:step('receiver_finishes_before_right_insertion')
            direct_path=HERE/'native_right_ring_insert.py';direct=types.ModuleType('native_right_ring_insert');direct.__file__=str(direct_path);exec(compile(direct_path.read_text(),str(direct_path),'exec'),direct.__dict__)
            direct_context=dict(c,left_hand_known_empty=c.get('left_hand_known_empty',False),step=step,config=dict(c['config'],pick_part=part,right_direct_empty_left_parking=bool(part=='ring2' and c['config'].get('right_direct_empty_left_parking'))))
            try:
                placed=direct.execute(direct_context,pickup=p)
            except direct.PrecarryPlanUnavailable as unavailable:
                if part in c['config'].get('require_direct_right_insert_parts',[]):raise
                # A geometric planning failure before any loaded RIGHT move
                # selects the already checked handover skill. An execution or
                # sensing failure has no catch here and still stops the task.
                G=unavailable.measured_grasp.copy()
                p=dict(p,measured_grasp=G.tolist(),direct_precarry_unavailable=str(unavailable))
                evidence=dict(event=part+'_right_precarry_select_handover',reason=str(unavailable),current_RGBD_pose=unavailable.observed_pose,loaded_right_motion_issued=False,uses_free_body_poses=False,wall_s=time.perf_counter()-c['started'])
                events.append(evidence);c['planning_diagnostics'].append(evidence)
                print('LOOSE_PRE_CARRY_HANDOVER_FALLBACK',evidence,flush=True)
            else:
                retained=placed['native_ring_result'].get('retention_RGBD')
                if retained is None:raise RuntimeError('Released ring retention must be visible before planning the next part')
                register_completed(part,retained['pose'],'Fresh RGB-D after release and withdrawal')
                results.append(dict(part=part,pickup=p,right_direct=placed,insertion_fresh_RGBD=placed['native_ring_result']['full_depth_pose']))
                events.append(dict(event=part+'_right_direct_insertion_complete',wall_s=time.perf_counter()-c['started']))
                if c['config'].get('pin_between_rings') and part==ring_order[0] and not c['config'].get('one_ring_only'):
                    pin_report=run_pin()
                    next_initial=vision.request(initial_ring_event(),arms['right'].fk(art.get_dof_positions()[0,indices['right']]))
                continue
        while left_program is not None:step('await_receiver_clear')
        receiver_opening=transfer.receiver_opening(c,part)
        if abs(float(cmd[0,fingers['left']].sum())-receiver_opening)>.0005:
            move('left',[cmd[0,indices['left']],cmd[0,indices['left']]],.10,part+'_receiver_empty_preshape',receiver_opening)
        row=ref[part]
        planner_path=HERE/'paired_handover.py';planner=types.ModuleType('paired_handover');planner.__file__=str(planner_path);exec(compile(planner_path.read_text(),str(planner_path),'exec'),planner.__dict__)
        release_width=float(p.get('release_width_m',.001 if p.get('grasp_type')=='inner' else .08))
        if p.get('grasp_type','outer')=='outer' and 'donor_release_opening_cap_m' in c['config']:
            cap=float(c['config']['donor_release_opening_cap_m'])
            contact_span=max(float(p['selected_instance']['planned_grip_width_m']),float(art.get_dof_positions()[0,fingers['right']].sum()))
            if not .010<cap<=.080:
                raise RuntimeError('Donor release preference must remain within native opening limits')
            required_release=contact_span+.002
            if not np.isfinite(required_release) or required_release>.080:
                raise RuntimeError('Measured donor grip cannot preserve 2 mm release clearance within native80mm opening')
            if required_release>cap:
                # The configured78mm preference avoids the hard endstop;
                # it is not the native80mm hardware aperture limit. A broad
                # but admissible CAD grip may need slightly more clearance.
                preferred_cap=cap;cap=min(.080,required_release+.00025)
                evidence=dict(event=part+'_measured_width_release_clearance',configured_preference_m=preferred_cap,measured_contact_span_m=contact_span,required_clearance_m=.002,selected_release_width_m=cap,native_hardware_limit_m=.080,uses_free_body_poses=False)
                events.append(evidence);c['planning_diagnostics'].append(evidence)
            release_width=min(release_width,cap)
            if release_width<required_release:
                raise RuntimeError('Donor grasp release command lacks measured2mm clearance')
        if part=='ring1' and (c['config'].get('temporary_axial_double_handover_component') or c['config'].get('large_double_handover_full_task')):
            preparation=types.ModuleType('double_native_handover_prepare');preparation.__file__=str(HERE/'double_native_handover.py');exec(compile(Path(preparation.__file__).read_text(),preparation.__file__,'exec'),preparation.__dict__)
            preparation_context=dict(c,step=step,component_trajectory=trajectory,component_observe=observe)
            G=preparation.prepare_holder(preparation_context,part,G)
        if part=='ring1' and c['config'].get('measured_initial_donor_contact_geometry',False):
            G=bind_initial_right_donor(part,G,part+'_initial_donor')
        handover=planner.plan(c,part,G,donor_release_width=release_width);incoming=handover['incoming']
        if handover.get('approach_order')=='receiver_first':move('left',handover['receiver_route'],.4,part+'_receiver_preposition',receiver_opening,other_carried_part=part,other_carried_grasp=G)
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
        if not parallel:move('right',handover['donor_route'],.8,part+'_carry_to_handover',carried_part=part,carried_grasp=G)
        # Transport can change the grasp through real frictional slip. Adapt
        # the receiving hand to a NEW sensor pose, preserving its intended
        # hand-relative grasp, instead of closing at the old nominal location.
        intended_receiver_grasp=np.linalg.inv(arms['left'].fk(handover['receiver_q']))@incoming
        G,donor_pose=observe('right',part,G,part+'_donor_before_transfer_RGBD')
        if part=='ring1' and c['config'].get('verify_large_donor_after_carry'):
            from receiver_pad_contact import contact_state as donor_contact_state
            from grasp_transfer_feasibility import large_donor_pad_support
            actual_width=float(art.get_dof_positions()[0,fingers['right']].sum())
            contact=donor_contact_state(dict(c,config=dict(c['config'],receiver_axial_only=False)),part,G,actual_width)
            support=large_donor_pad_support(c,G,actual_width)
            prior_G=np.asarray(p['measured_grasp'],float)
            translation=float(np.linalg.norm(G[:3,3]-prior_G[:3,3]))
            angle=float(np.arccos(np.clip(G[:3,2]@prior_G[:3,2],-1,1)))
            retained=bool(contact['opposed_contact_compatible'] and support['broad_opposed_flat_pad_support'] and translation<=.002 and angle<=.05)
            evidence=dict(event=part+'_donor_after_carry_retention',retained=retained,native_contact=contact,support=support,translation_change_m=translation,axis_change_rad=angle,translation_limit_m=.002,axis_limit_rad=.05,uses_free_body_poses=False)
            events.append(evidence);c['planning_diagnostics'].append(evidence)
            if not retained:raise RuntimeError('Large donor lost fresh stable pad support during carry; receiver approach blocked')
        incoming=frame(donor_pose['center_m'],axis_frame(donor_pose['axis']))
        receiving_H=incoming@np.linalg.inv(intended_receiver_grasp)
        route=None
        balanced_small=bool(part=='ring2' and c['config'].get('small_balanced_receiver_axis30',False))
        if balanced_small:
            if c.get('left_hand_known_empty') is not True:raise RuntimeError('Balanced SMALL receiver proposal requires confirmed EMPTY LEFT Hand')
            from balanced_small_receiver import propose as propose_balanced_small
            balanced_H,balanced_metadata=propose_balanced_small(receiving_H,incoming)
            # One bounded physical candidate. If its original full gates fail,
            # stop while RIGHT holds; never silently close at the rejected pose.
            candidates=[(balanced_H,balanced_metadata)]
        else:candidates=[(receiving_H,None)]+list(transfer.receiving_poses(c,part,incoming))
        for candidate_H,metadata in candidates:
            receiving_q=arms['left'].solve(candidate_H[:3,3],candidate_H[:3,:3],handover['receiver_q'])
            if receiving_q is None:continue
            if c['config'].get('safe_float32_receiver_endpoints'):
                from float32_joint_endpoint import receiver_candidate
                try:receiving_q,rounding=receiver_candidate(arms['left'],receiving_q,candidate_H)
                except ValueError:continue
                if rounding is not None:c['planning_diagnostics'].append(dict(event='new_receiver_endpoint_float32_interior',part=part,**rounding))
            # The giver must also be able to OPEN after the receiver closes.
            # Checking only the incoming held width missed that swept volume.
            candidate_G=np.linalg.inv(candidate_H)@incoming;expected_receiver_width=transfer.receiver_aperture(c,part,candidate_G)
            if expected_receiver_width is None:continue
            if balanced_small:
                # Additional ORIGINAL full native-self/held-CAD proximal proof
                # at the proposed endpoint. Intended distal pad contact retains
                # its original exemption; no physical actor or achievedG changes.
                lc=checkers['left'];saved_small=(lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative)
                planned_G=np.linalg.inv(arms['left'].fk(receiving_q))@incoming
                lc.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@planned_G)
                try:native_ok,native_why=lc.same_arm_clearance.clear(receiving_q,float(expected_receiver_width))
                finally:lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative=saved_small
                if not native_ok:
                    c['planning_diagnostics'].append(dict(event='balanced_SMALL_receiver_native_rejected',collision=native_why));continue
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
                    route=transfer.receiver_approach_route(c,part+'_receiver_approach',checkers['left'],cmd[0,indices['left']],receiving_q,receiver_opening,seconds=.50)
            finally:checkers['right'].detach_payload()
            if route is not None:
                receiving_H=candidate_H;intended_receiver_grasp=np.linalg.inv(receiving_H)@incoming
                if balanced_small:c['planning_diagnostics'].append(dict(event='balanced_SMALL_receiver_target_proposed',**metadata,original_native_proximal_preflight=True,original_dual_release_and_current_route_preflight=True,original_actual_command_allwidth_issued_guard_still_required=True))
                break
        if route is None:raise RuntimeError('No fresh-RGBD receiver route that permits giver opening')
        move('left',route,.4,part+'_receive_approach',receiver_opening,other_carried_part=part,other_carried_grasp=G)
        G,preclose_pose=observe('right',part,G,part+'_donor_after_receiver_approach_RGBD')
        preclose_part=frame(preclose_pose['center_m'],axis_frame(preclose_pose['axis']))
        if np.linalg.norm(preclose_part[:3,3]-incoming[:3,3])>.003 or np.arccos(np.clip(preclose_part[:3,2]@incoming[:3,2],-1,1))>.05:
            raise RuntimeError('Receiver approach disturbed held ring; closing is blocked')
        small_balanced_feedback=bool(part=='ring2' and c['config'].get('small_balanced_receiver_axis30',False))
        if small_balanced_feedback or (part=='ring1' and c['config'].get('temporary_holder_preclose_pose_feedback',False) and (c['config'].get('temporary_axial_double_handover_component') or c['config'].get('large_double_handover_full_task'))):
            # Match the actual EMPTY receiver to its original intended grasp.
            # Targets use only the fresh donor image and current robot encoders;
            # no new grasp family, force target, speed or clearance is selected.
            left_arm=arms['left'];left_ids=indices['left'];right_ids=indices['right']
            actual_H=left_arm.fk(art.get_dof_positions()[0,left_ids]);command_H=left_arm.fk(cmd[0,left_ids])
            if small_balanced_feedback:
                # Correct the actual EMPTY Hand from this LAST preclose image,
                # rather than assuming its earlier planned target was achieved.
                # The existing full guarded correction and fresh-image policy
                # below is shared unchanged with the proven LARGE mechanism.
                if c.get('left_hand_known_empty') is not True:raise RuntimeError('Balanced SMALL preclose feedback requires confirmed EMPTY LEFT Hand')
                # Keep the BEFORE-approach selected grasp fixed: use the new
                # image to move the Hand toward it, not a moving grasp reference.
                balanced_feedback_metadata=dict(selected_intended_grasp=intended_receiver_grasp.tolist(),fixed_before_approach=True)
            desired_H=preclose_part@np.linalg.inv(intended_receiver_grasp)
            translation=float(np.linalg.norm(desired_H[:3,3]-actual_H[:3,3]));angle=float(np.linalg.norm(log(desired_H[:3,:3]@actual_H[:3,:3].T)))
            bias_translation=float(np.linalg.norm(command_H[:3,3]-actual_H[:3,3]));bias_angle=float(np.linalg.norm(log(command_H[:3,:3]@actual_H[:3,:3].T)))
            if max(translation,bias_translation)>.002 or max(angle,bias_angle)>.05:
                raise RuntimeError('Empty temporary receiver feedback exceeds2mm/.05rad; no closing issued')
            target=command_H@np.linalg.inv(actual_H)@desired_H
            path=[cmd[0,left_ids].copy()]
            for u in np.linspace(0.,1.,5)[1:]:
                orientation=exp(u*log(target[:3,:3]@command_H[:3,:3].T))@command_H[:3,:3]
                qq=left_arm.solve(command_H[:3,3]*(1-u)+target[:3,3]*u,orientation,path[-1])
                if qq is None:raise RuntimeError('Empty temporary receiver feedback has no original-model IK')
                path.append(qq)
            rc,lc=checkers['right'],checkers['left'];saved=(rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative);resolution=getattr(lc,'resolution_rad',.035)
            rc.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@G)
            try:
                lc.resolution_rad=.002
                with lc.other_arm_at(rc,art.get_dof_positions()[0,right_ids],float(art.get_dof_positions()[0,fingers['right']].sum())):
                    clear,why=lc.path(path,receiver_opening)
                    if not clear:raise RuntimeError('Empty temporary receiver feedback original path blocked '+str(why))
            finally:rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative=saved;lc.resolution_rad=resolution
            move('left',path,.4,part+'_receiver_actual_pose_feedback',receiver_opening,other_carried_part=part,other_carried_grasp=G)
            prior_part=preclose_part.copy();G,preclose_pose=observe('right',part,G,part+'_receiver_feedback_fresh_RGBD');preclose_part=frame(preclose_pose['center_m'],axis_frame(preclose_pose['axis']))
            if np.linalg.norm(preclose_part[:3,3]-prior_part[:3,3])>.003 or np.arccos(np.clip(preclose_part[:3,2]@prior_part[:3,2],-1,1))>.05:
                raise RuntimeError('Empty receiver feedback disturbed donor-held ring; no closing issued')
            after_H=left_arm.fk(art.get_dof_positions()[0,left_ids]);actual_G=np.linalg.inv(after_H)@preclose_part
            evidence=dict(event='balanced_SMALL_EMPTY_LEFT_actual_pose_feedback' if small_balanced_feedback else 'temporary_EMPTY_LEFT_actual_pose_feedback',requested_translation_m=translation,requested_rotation_rad=angle,command_actual_bias_m=bias_translation,command_actual_bias_rad=bias_angle,actual_grasp=actual_G.tolist(),intended_grasp=intended_receiver_grasp.tolist(),uses_free_body_poses=False,force_speed_geometry_limits_unchanged=True)
            if small_balanced_feedback:evidence.update(balanced_target=balanced_feedback_metadata,exact_original2mm_50mrad_correction_bounds=True,single_bounded_correction=True,one_additional_fresh_image=True)
            c['planning_diagnostics'].append(evidence);print('LOOSE_TEMPORARY_RECEIVER_FEEDBACK',evidence,flush=True)
        actual_receiving_H=arms['left'].fk(art.get_dof_positions()[0,indices['left']])
        expected_receiver_width=transfer.receiver_aperture(c,part,np.linalg.inv(actual_receiving_H)@preclose_part)
        if expected_receiver_width is None:raise RuntimeError('Fresh preclose RGB-D does not confirm opposing receiver pad geometry')
        if small_balanced_feedback:
            # The corrected physical endpoint changes the future donor-opening
            # geometry. Requalify it NOW, before either gripper starts closing.
            positions=art.get_dof_positions()[0].copy()
            donor_widths=[float(positions[fingers['right']].sum()),float(cmd[0,fingers['right']].sum())]
            receiver_openings=[receiver_opening,float(positions[fingers['left']].sum()),float(cmd[0,fingers['left']].sum())]
            lc=checkers['left'];rc=checkers['right'];saved=(lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative)
            try:
                for left_q in [positions[indices['left']],cmd[0,indices['left']]]:
                    left_H=arms['left'].fk(left_q);fresh_left_G=np.linalg.inv(left_H)@preclose_part
                    fresh_width=transfer.receiver_aperture(c,part,fresh_left_G)
                    if fresh_width is None:raise RuntimeError('Balanced SMALL corrected receiver aperture invalid; donor stays closed')
                    lc.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@fresh_left_G)
                    native_ok,native_why=lc.same_arm_clearance.clear(left_q,float(fresh_width))
                    if not native_ok:raise RuntimeError('Balanced SMALL corrected native proximal geometry blocked '+str(native_why))
                    for right_q in [positions[indices['right']],cmd[0,indices['right']]]:
                        for donor_width in donor_widths:
                            for opening in receiver_openings:
                                if not transfer.sequential_transfer_clear(rc,arms['right'].fk(right_q),donor_width,lc,left_H,fresh_width,opening,donor_release_width=release_width):
                                    raise RuntimeError('Balanced SMALL corrected future donor opening blocked; donor stays closed')
            finally:lc.payload_path,lc.payload_rows,lc.payload_boxes,lc.payload_relative=saved
            c['planning_diagnostics'].append(dict(event='balanced_SMALL_corrected_preclose_future_opening_admitted',actual_command_both_arms=True,current_actual_command_and_planned_openings=True,fresh_RGBD_native_proximal=True,donor_release_width_m=float(release_width),uses_free_body_poses=False))
        incoming=preclose_part
        c['stiffness'][0,fingers['left']]=0.;c['damping'][0,fingers['left']]=[4000.,.001];art.set_dof_stiffnesses(c['stiffness'],c['idx']);art.set_dof_dampings(c['damping'],c['idx'])
        c['set_left_force'](float(c['config'].get('native_grip_pad_force_n',30.)))
        receiver_target=max(.001,expected_receiver_width-.003)
        for attempt in range(2):
            c['left_hand_known_empty']=False;c['set_grip_hold']('left',True)
            move('left',[cmd[0,indices['left']],cmd[0,indices['left']]],.15,part+'_receiver_close',receiver_target);wait(.08,part+'_receiver_contact_check')
            # CPU PhysX may advance <1 s of physical closure in 3 s wall
            # time. Preserve the 120-sample (~3 s physics) contact bound;
            # allow its unchanged motion to finish on this slower host.
            receiver_wait_started_step=control_steps
            receiver_deadline=time.perf_counter()+12.;receiver_window=[];receiver_times=[]
            receiver_plateau=False;receiver_completion='bounded_wait_expired'
            for sample_index in range(120):
                receiver_window.append(art.get_dof_positions()[0,fingers['left']].copy())
                receiver_times.append(control_steps*dt);receiver_window=receiver_window[-5:];receiver_times=receiver_times[-5:]
                measured_window=np.asarray(receiver_window);gap=float(measured_window[-1].sum())
                if gap<=receiver_target+.0005:
                    receiver_completion='empty_closure_reached_target';break
                near_contact=receiver_target+.0005<gap<min(D+.0035,expected_receiver_width+.002)
                stationary=len(receiver_window)==5 and receiver_times[-1]-receiver_times[0]>=.1-1e-9 and np.max(np.ptp(measured_window,axis=0))<=.00015 and np.ptp(measured_window.sum(1))<=.00015
                if stationary and near_contact:
                    receiver_plateau=True;receiver_completion='stationary_near_CAD';break
                if time.perf_counter()>=receiver_deadline:break
                wait(.025,part+'_receiver_grasp_controller_completion')
            aperture=float(art.get_dof_positions()[0,fingers['left']].sum())
            receiver_evidence=dict(event=part+'_receiver_contact',aperture_m=aperture,attempt=attempt,stationary_near_CAD=bool(receiver_plateau),completion=receiver_completion,encoder_samples=sample_index+1,maximum_wall_wait_s=12.,maximum_encoder_samples=120,physics_wait_s=float((control_steps-receiver_wait_started_step)*dt),encoder_window_span_s=float(receiver_times[-1]-receiver_times[0]),per_finger_position_range_m=np.ptp(measured_window,axis=0).tolist(),gap_position_range_m=float(np.ptp(measured_window.sum(1))),wall_s=time.perf_counter()-c['started'])
            events.append(receiver_evidence);c['planning_diagnostics'].append(receiver_evidence)
            opposed=receiver_plateau and receiver_target+.0005<aperture<min(D+.0035,expected_receiver_width+.002)
            if opposed and c['config'].get('verify_receiver_pad_contact'):
                from receiver_pad_contact import contact_state as receiver_contact_state
                G,contact_pose=observe('right',part,G,part+'_bilateral_contact_RGBD')
                contact_part=frame(contact_pose['center_m'],axis_frame(contact_pose['axis']))
                receiver_H=arms['left'].fk(art.get_dof_positions()[0,indices['left']])
                receiver_G=np.linalg.inv(receiver_H)@contact_part
                contact=receiver_contact_state(c,part,receiver_G,float(art.get_dof_positions()[0,fingers['left']].sum()))
                evidence=dict(event=part+'_receiver_both_pad_contact',contact=contact,wall_s=time.perf_counter()-c['started'])
                events.append(evidence);c['planning_diagnostics'].append(evidence);print('LOOSE_RECEIVER_CONTACT',evidence,flush=True)
                opposed=bool(contact['opposed_contact_compatible'])
                if opposed:incoming=contact_part;intended_receiver_grasp=receiver_G
            if opposed:break
            if attempt==1:raise RuntimeError('Receiver pad contact not confirmed; donor stays closed')
            c['set_grip_hold']('left',False);move('left',[cmd[0,indices['left']],cmd[0,indices['left']]],.15,part+'_receiver_retry_open',receiver_opening)
            G,donor_pose=observe('right',part,G,part+'_donor_contact_retry_RGBD');incoming=frame(donor_pose['center_m'],axis_frame(donor_pose['axis']));receiving_H=incoming@np.linalg.inv(intended_receiver_grasp)
            receiving_q=arms['left'].solve(receiving_H[:3,3],receiving_H[:3,:3],cmd[0,indices['left']])
            route=None
            if receiving_q is not None:
                checkers['right'].attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@G)
                try:
                    with checkers['left'].other_arm_at(checkers['right'],cmd[0,indices['right']],float(art.get_dof_positions()[0,fingers['right']].sum())):
                        route=transfer.receiver_approach_route(c,part+'_receiver_contact_retry',checkers['left'],cmd[0,indices['left']],receiving_q,receiver_opening,seconds=.5)
                finally:checkers['right'].detach_payload()
            if route is None:raise RuntimeError('No image-corrected receiver retry route; donor stays closed')
            move('left',route,.25,part+'_receiver_contact_retry')
        c['left_hand_known_empty']=False;c['set_grip_hold']('left',True)
        if c['config'].get('verify_receiver_pad_contact') and c['config'].get('native_loaded_preload_after_contact_m_s'):
            c['set_grip_hold']('left',True,preload_m_s=float(c['config']['native_loaded_preload_after_contact_m_s']))
            wait(float(c['config'].get('native_loaded_preload_ramp_s',0.))+.05,part+'_receiver_holding_force_ramp')
            G,contact_pose=observe('right',part,G,part+'_receiver_holding_force_RGBD')
            incoming=frame(contact_pose['center_m'],axis_frame(contact_pose['axis']))
            receiver_G=np.linalg.inv(arms['left'].fk(art.get_dof_positions()[0,indices['left']]))@incoming
            contact=receiver_contact_state(c,part,receiver_G,float(art.get_dof_positions()[0,fingers['left']].sum()))
            c['planning_diagnostics'].append(dict(event=part+'_receiver_contact_after_force_ramp',contact=contact));print('LOOSE_RECEIVER_CONTACT_AFTER_FORCE_RAMP',part,contact,flush=True)
            if not contact['opposed_contact_compatible']:raise RuntimeError('Receiver lost opposing pad geometry during force ramp; donor stays closed')
        wait(.12,part+'_bilateral_handover_hold')
        if c['config'].get('verify_receiver_pad_contact'):
            # A near-CAD width can still be an empty, moving closure. Wait
            # for a bounded measured plateau while the donor stays loaded.
            settle_deadline=time.perf_counter()+3.;receiver_window=[]
            while True:
                receiver_window.append(art.get_dof_positions()[0,fingers['left']].copy())
                receiver_window=receiver_window[-5:];measured_window=np.asarray(receiver_window)
                aperture=float(measured_window[-1].sum())
                near_contact=receiver_target+.0005<aperture<min(D+.0035,expected_receiver_width+.002)
                stationary=len(receiver_window)==5 and np.max(np.ptp(measured_window,axis=0))<=.00015 and np.ptp(measured_window.sum(1))<=.00015
                if stationary and near_contact:break
                if time.perf_counter()>=settle_deadline:raise RuntimeError('Receiver did not establish a stationary near-CAD aperture within 3 s; donor stays closed')
                wait(.025,part+'_receiver_final_contact_plateau')
            # Refresh after the bounded wait rather than certify a ring pose
            # that may predate several seconds of continued finite force.
            G,contact_pose=observe('right',part,G,part+'_receiver_final_contact_RGBD')
            incoming=frame(contact_pose['center_m'],axis_frame(contact_pose['axis']))
            receiver_window=[]
            for sample_index in range(5):
                if sample_index:wait(.025,part+'_receiver_final_encoder_window')
                receiver_window.append(art.get_dof_positions()[0,fingers['left']].copy())
            measured_window=np.asarray(receiver_window);aperture=float(measured_window[-1].sum())
            stationary=np.max(np.ptp(measured_window,axis=0))<=.00015 and np.ptp(measured_window.sum(1))<=.00015
            near_contact=receiver_target+.0005<aperture<min(D+.0035,expected_receiver_width+.002)
            if not stationary or not near_contact:raise RuntimeError('Receiver aperture moved through fresh RGB-D confirmation; donor stays closed')
            receiver_G=np.linalg.inv(arms['left'].fk(art.get_dof_positions()[0,indices['left']]))@incoming
            contact=receiver_contact_state(c,part,receiver_G,aperture,contact_tolerance_m=.0008)
            evidence=dict(event=part+'_receiver_final_stationary_contact',contact=contact,measured_grasp=receiver_G.tolist(),per_finger_position_range_m=np.ptp(measured_window,axis=0).tolist(),gap_position_range_m=float(np.ptp(measured_window.sum(1))),encoder_window_span_s=.1,stationary=bool(stationary),near_contact=bool(near_contact),force_contact_measured=False,uses_free_body_poses=False,wall_s=time.perf_counter()-c['started'])
            events.append(evidence);c['planning_diagnostics'].append(evidence);print('LOOSE_RECEIVER_FINAL_CONTACT',evidence,flush=True)
            if not contact['opposed_contact_compatible']:raise RuntimeError('Fresh RGB-D and final encoder aperture do not certify opposing receiver pads; donor stays closed')
            intended_receiver_grasp=receiver_G
        donor_binding=loaded_contact_bindings.pop('right',None)
        if donor_binding is not None:
            donor_binding.release();checkers['right'].loaded_contact_aperture=None
        c['set_grip_hold']('right',False);move('right',[cmd[0,indices['right']],cmd[0,indices['right']]],.15,part+'_donor_open',release_width)
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
        if route is None and c['config'].get('allow_guarded_full_donor_release',False) and p.get('grasp_type','outer')=='outer' and release_width<.080:
            # A78mm preference can leave a legal static pose but obstruct the
            # first empty retreat. Admit the REAL native80mm opening and its
            # future exit before changing the actual command; no margins or
            # intended-contact exclusions change.
            rc,lc=checkers['right'],checkers['left'];candidate_width=.080
            candidate_route=protect_received_ring(lambda:planner.empty_giver_route(rc,cmd[0,indices['right']],candidate_width))
            if candidate_route is not None:
                open_rows=trajectory('right',[cmd[0,indices['right']],cmd[0,indices['right']]],.15,candidate_width)
                left_widths=np.linspace(min(float(cmd[0,fingers['left']].sum()),float(art.get_dof_positions()[0,fingers['left']].sum())),max(float(cmd[0,fingers['left']].sum()),float(art.get_dof_positions()[0,fingers['left']].sum())),5)
                def exact_rows_clear(rows,right_widths=None):
                    lc.attach_payload(c['part_collision_templates'][part],frame([0,0,.1034],np.eye(3))@left_G)
                    try:
                        for lw in left_widths:
                            with rc.other_arm_at(lc,art.get_dof_positions()[0,indices['left']],float(lw)):
                                for index,row in enumerate(rows):
                                    for rw in ([float(row[7:].sum())]if right_widths is None else right_widths):
                                        ok,why=rc.clear(row[:7],float(rw))
                                        if not ok:return False,dict(row=index,LEFT_width_m=float(lw),RIGHT_width_m=float(rw),collision=why)
                        return True,None
                    finally:lc.detach_payload()
                ok,why=exact_rows_clear(open_rows)
                if ok:
                    phase=part+'_donor_guarded_native_full_open'
                    for r in open_rows:cmd[0,indices['right']]=r[:7];cmd[0,fingers['right']]=r[7:];step()
                    window=[]
                    for attempt in range(41):
                        window.append(art.get_dof_positions()[0,fingers['right']].copy());window=window[-5:];samples=np.asarray(window)
                        if len(window)==5 and float(samples[-1].sum())>=candidate_width-.00015 and np.max(np.ptp(samples,axis=0))<=.00015 and np.ptp(samples.sum(1))<=.00015:break
                        if attempt==40:raise RuntimeError('Native full donor release did not reach actual stationary aperture')
                        wait(.025,part+'_donor_full_open_encoder_plateau')
                    release_width=candidate_width
                    actual_width=float(art.get_dof_positions()[0,fingers['right']].sum());route=protect_received_ring(lambda:planner.empty_giver_route(rc,cmd[0,indices['right']],min(actual_width,release_width)))
                    if route is not None:
                        requested=.05 if c['config'].get('optimized_motion_timing')else .45
                        exit_rows=trajectory('right',route,requested,release_width)
                        right_widths=np.linspace(min(actual_width,release_width),max(actual_width,release_width),5)
                        ok,why=exact_rows_clear(exit_rows,right_widths)
                        if not ok:raise RuntimeError('Actual fully opened donor exit exact issued rows blocked '+str(why))
                    evidence=dict(event=part+'_guarded_native_full_donor_release',native_opening_limit_m=.080,actual_aperture_m=actual_width,opening_control_rows=len(open_rows),actual_encoder_plateau=True,future_exit_admitted=route is not None,uses_free_body_poses=False)
                    events.append(evidence);c['planning_diagnostics'].append(evidence);print('LOOSE_GUARDED_DONOR_RELEASE',evidence,flush=True)
                else:
                    c['planning_diagnostics'].append(dict(event=part+'_native_full_release_rejected_before_motion',collision=why))
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
        if part==ring_order[0] and not c['config'].get('one_ring_only') and not c['config'].get('defer_next_initial_until_receiver_bound',False):next_initial=vision.request(initial_ring_event(),arms['right'].fk(art.get_dof_positions()[0,indices['right']]))
        if part==ring_order[-1] and not c['config'].get('one_ring_only') and not pin_first and not c['config'].get('pin_between_rings'):
            pin_event=dict(mode='pin_initial',part='pin',label='pin_initial');pin_initial=(pin_event,vision.request(pin_event,arms['right'].fk(art.get_dof_positions()[0,indices['right']])))
        for _ in clearing:step(part+'_donor_leave_receiver_corridor')
        if c.get('set_empty_donor_contact_guard'):c['set_empty_donor_contact_guard']('right',True,part)
        if part=='ring1' and c['config'].get('measured_loaded_contact_geometry'):
            # A finite-force target deliberately lies inside the rigid ring.
            # Certify actual encoder geometry, never replace the force target.
            from loaded_contact_aperture import LoadedContactAperture
            # Establish stationarity FIRST, then capture. Continue recording
            # every actual encoder sample while the real RGB-D worker runs;
            # no extra 100 ms dwell is added to the newly captured image.
            loaded_contact_capture_window=[(control_steps*dt,art.get_dof_positions()[0,fingers['left']].copy())]
            try:
                wait(.1,part+'_loaded_contact_encoder_window')
                left_G,pose=observe('left',part,left_G,part+'_loaded_contact_fresh_RGBD')
                # The last step sample and current delivery share a timestamp
                # if the future completed without advancing physics.
                current=(control_steps*dt,art.get_dof_positions()[0,fingers['left']].copy())
                if current[0]>loaded_contact_capture_window[-1][0]:loaded_contact_capture_window.append(current)
                else:loaded_contact_capture_window[-1]=current
                window_times=[row[0]for row in loaded_contact_capture_window]
                finger_window=[row[1]for row in loaded_contact_capture_window]
            finally:loaded_contact_capture_window=None
            gap=float(np.asarray(finger_window)[-1].sum())
            contact=receiver_contact_state(c,part,left_G,gap,contact_tolerance_m=.0008)
            if transfer.receiver_aperture(c,part,left_G) is None:
                raise RuntimeError('Fresh loaded receiver pose lacks stable native pad geometry')
            binding=LoadedContactAperture(side='left',part=part,
                payload_path=c['part_collision_templates'][part],
                encoder_samples=finger_window,sample_times_s=window_times,
                contact=contact,measured_grasp=left_G,control_dt_s=dt,
                capture_age_physics_s=(control_steps-last_annular_observation_steps[('left',part)])*dt)
            checkers['left'].loaded_contact_aperture=binding;loaded_contact_bindings['left']=binding
            evidence=dict(event='loaded_receiver_encoder_geometry_bound',**binding.record)
            c['planning_diagnostics'].append(evidence);print('LOOSE_LOADED_APERTURE_BOUND',evidence,flush=True)
        if part=='ring2':
            previous_G=left_G.copy()
            bind_small_geometry=bool(c['config'].get('measured_loaded_contact_geometry') and c['config'].get('small_measured_loaded_contact_geometry',False))
            # Reuse the existing stationary hold and its existing fresh image.
            # The finite-force target and every other collision width stay intact.
            if bind_small_geometry:
                loaded_contact_capture_window=[(control_steps*dt,art.get_dof_positions()[0,fingers['left']].copy())]
            try:
                wait(.15,part+'_receiver_stable_hold')
                left_G,pose=observe('left',part,left_G,part+'_receiver_stable_hold_RGBD')
                if bind_small_geometry:
                    current=(control_steps*dt,art.get_dof_positions()[0,fingers['left']].copy())
                    if current[0]>loaded_contact_capture_window[-1][0]:loaded_contact_capture_window.append(current)
                    else:loaded_contact_capture_window[-1]=current
                    small_window_times=[r[0]for r in loaded_contact_capture_window]
                    small_finger_window=[r[1]for r in loaded_contact_capture_window]
            finally:
                if bind_small_geometry:loaded_contact_capture_window=None
            if np.linalg.norm(left_G[:3,3]-previous_G[:3,3])>.0015 or np.arccos(np.clip(left_G[:3,2]@previous_G[:3,2],-1,1))>.025:
                if not c['config'].get('small_receiver_one_shot_settled_reconfirmation',False):
                    raise RuntimeError('Small ring slipped in receiver after giver withdrawal')
                # A failed original comparison never enters motion. Requalify
                # ONE stationary current grasp with two fresh images; no
                # repeated moving reference, force change or threshold increase.
                from loaded_contact_aperture import LoadedContactAperture
                from receiver_pad_contact import contact_state as receiver_contact_state
                failed_G=left_G.copy();confirmation_rows=[];fresh_grasps=[]
                for confirmation_index in range(2):
                    loaded_contact_capture_window=[(control_steps*dt,art.get_dof_positions()[0,fingers['left']].copy())]
                    try:
                        wait(.1,part+'_receiver_reconfirmation_encoder_window')
                        fresh_G,fresh_pose=observe('left',part,failed_G,part+'_receiver_reconfirmation_RGBD')
                        current=(control_steps*dt,art.get_dof_positions()[0,fingers['left']].copy())
                        if current[0]>loaded_contact_capture_window[-1][0]:loaded_contact_capture_window.append(current)
                        else:loaded_contact_capture_window[-1]=current
                        sample_times=[r[0]for r in loaded_contact_capture_window]
                        encoder_samples=[r[1]for r in loaded_contact_capture_window]
                    finally:loaded_contact_capture_window=None
                    gap=float(np.asarray(encoder_samples)[-1].sum())
                    contact=receiver_contact_state(c,part,fresh_G,gap,contact_tolerance_m=.0008)
                    if transfer.receiver_aperture(c,part,fresh_G) is None:
                        raise RuntimeError('Small receiver reconfirmation lacks native pad support')
                    # This validates measured encoders/contact only. It is
                    # never installed as a planning geometry certificate.
                    binding=LoadedContactAperture(side='left',part=part,payload_path=c['part_collision_templates'][part],encoder_samples=encoder_samples,sample_times_s=sample_times,contact=contact,measured_grasp=fresh_G,control_dt_s=dt,capture_age_physics_s=(control_steps-last_annular_observation_steps[('left',part)])*dt)
                    binding.release()
                    reference_G=failed_G if confirmation_index==0 else fresh_grasps[0]
                    delta_m=float(np.linalg.norm(fresh_G[:3,3]-reference_G[:3,3]));delta_rad=float(np.arccos(np.clip(fresh_G[:3,2]@reference_G[:3,2],-1,1)))
                    row=dict(index=confirmation_index,translation_delta_m=delta_m,axis_delta_rad=delta_rad,translation_limit_m=.00025,axis_limit_rad=.005,stationary_encoder_contact=binding.record,uses_free_body_poses=False)
                    confirmation_rows.append(row)
                    if delta_m>.00025 or delta_rad>.005:
                        raise RuntimeError('Small receiver continues changing during one-shot settled reconfirmation')
                    fresh_grasps.append(fresh_G.copy());pose=fresh_pose
                left_G=fresh_grasps[-1]
                if bind_small_geometry:
                    # The final EXISTING reconfirmation image/window supersedes
                    # the first hold; a failed original gate never installs it.
                    small_window_times=sample_times;small_finger_window=encoder_samples
                evidence=dict(event='small_receiver_one_shot_settled_reconfirmation',original_reference_G=previous_G.tolist(),failed_original_G=failed_G.tolist(),confirmed_G=left_G.tolist(),fresh_frames=confirmation_rows,maximum_reconfirmation_frames=2,force_and_speed_unchanged=True,uses_free_body_poses=False,wall_s=time.perf_counter()-c['started'])
                events.append(evidence);c['planning_diagnostics'].append(evidence);print('LOOSE_RECEIVER_SETTLED_RECONFIRMATION',evidence,flush=True)
            if transfer.receiver_aperture(c,part,left_G) is None:
                raise RuntimeError('Small ring lacks stable receiver pad geometry after withdrawal')
            if bind_small_geometry:
                from loaded_contact_aperture import LoadedContactAperture
                from receiver_pad_contact import contact_state as receiver_contact_state
                gap=float(np.asarray(small_finger_window)[-1].sum())
                contact=receiver_contact_state(c,part,left_G,gap,contact_tolerance_m=.0008)
                binding=LoadedContactAperture(side='left',part=part,
                    payload_path=c['part_collision_templates'][part],
                    encoder_samples=small_finger_window,sample_times_s=small_window_times,
                    contact=contact,measured_grasp=left_G,control_dt_s=dt,
                    capture_age_physics_s=(control_steps-last_annular_observation_steps[('left',part)])*dt,
                    # Expand the PROVED swept CAD interval before any route
                    # proof; never add epsilon to encoder acceptance itself.
                    half_interval_m=.0005 if c['config'].get('small_loaded_contact_wide_proof',False)else .00025)
                checkers['left'].loaded_contact_aperture=binding;loaded_contact_bindings['left']=binding
                evidence=dict(event='small_loaded_receiver_encoder_geometry_bound',**binding.record)
                c['planning_diagnostics'].append(evidence);print('LOOSE_SMALL_LOADED_APERTURE_BOUND',evidence,flush=True)
            results[-1]['receiver_stable_hold_RGBD']=pose
        # Mandatory fresh receiver/binding capture precedes optional next-part
        # prefetch on the single camera worker; no extra image or dwell.
        if part==ring_order[0] and not c['config'].get('one_ring_only') and c['config'].get('defer_next_initial_until_receiver_bound',False):next_initial=vision.request(initial_ring_event(),arms['right'].fk(art.get_dof_positions()[0,indices['right']]))
        events.append(dict(event=part+'_donor_leave_receiver_corridor',wall_s=time.perf_counter()-c['started'],overlapped_receiver_RGBD=True,extra_lateral_clearance_used=extra_lateral_clearance,motion_s=len(clearing_rows)*dt))
        if part=='ring1' and (c['config'].get('temporary_axial_double_handover_component') or c['config'].get('large_double_handover_full_task')):
            module=types.ModuleType('double_native_handover');module.__file__=str(HERE/'double_native_handover.py');exec(compile(Path(module.__file__).read_text(),module.__file__,'exec'),module.__dict__)
            component_context=dict(c,step=step,component_move=move,component_observe=observe,component_trajectory=trajectory,component_bindings=loaded_contact_bindings,component_encoder_capture_callbacks=c.setdefault('_encoder_capture_callbacks',[]),component_image_age_steps=lambda side,part:control_steps-last_annular_observation_steps[(side,part)])
            completed=module.execute(component_context,part,left_G)
            if not c['config'].get('large_double_handover_full_task'):
                return dict(completed,parts=results,initial_handover_events=events,full_task_pass=False)
            if completed.get('state')!='physical_double_handover_RIGHT_regrasp_camera_confirmed' or completed.get('part')!=part or completed.get('grasp_type')!='outer':
                raise RuntimeError('Full task lacks successful physical positive RIGHT regrasp evidence')
            # component_context is a shallow context copy: propagate its
            # logical empty state only after the actual LEFT release/withdraw.
            if component_context.get('left_hand_known_empty') is not True:
                raise RuntimeError('Physical double handover did not establish empty LEFT')
            c['left_hand_known_empty']=True
            right_G=np.asarray(completed['measured_grasp'],float)
            if right_G.shape!=(4,4) or not np.isfinite(right_G).all() or right_G[2,2]<=.1 or c.get('left_hand_known_empty') is not True:
                raise RuntimeError('Full task requires fresh positive RIGHT grasp and actually empty LEFT')
            binding=loaded_contact_bindings.get('right')
            if binding is None or not binding.active or binding.part!=part or checkers['right'].loaded_contact_aperture is not binding:
                raise RuntimeError('Full task lost measured RIGHT loaded-contact geometry binding')
            binding.check(art.get_dof_positions()[0,fingers['right']])
            # Keep the SAME image-selected workpiece identity. Only its new,
            # physically received camera/encoder grasp replaces the old grasp.
            received_pickup=dict(p,measured_grasp=right_G.tolist(),grasp_type='outer',physical_double_handover=completed)
            received_pickup['actual_grip_width_m']=float(art.get_dof_positions()[0,fingers['right']].sum())
            received_pickup['selected_instance']=dict(p['selected_instance'],planned_grip_width_m=received_pickup['actual_grip_width_m'],grasp_type='outer',measured_grasp=right_G.tolist())
            def release_loaded_contact_binding(side,released_part):
                active=loaded_contact_bindings.get(side)
                if side!='right' or released_part!=part or active is not binding or active.part!=released_part:
                    raise RuntimeError('Unexpected loaded-contact release ownership')
                active.release();loaded_contact_bindings.pop(side);checkers[side].loaded_contact_aperture=None
            direct_path=HERE/'native_right_ring_insert.py';direct=types.ModuleType('native_right_ring_insert_after_double');direct.__file__=str(direct_path);exec(compile(direct_path.read_text(),str(direct_path),'exec'),direct.__dict__)
            direct_context=dict(c,left_hand_known_empty=True,step=step,release_loaded_contact_binding=release_loaded_contact_binding,config=dict(c['config'],pick_part=part,temporary_axial_double_handover_component=False,large_double_handover_full_task=False,receiver_axial_only=False))
            # This is a physical regrasp already. Any fresh route/sensor failure
            # aborts holding; it MUST NOT loop back into another handover.
            placed=direct.execute(direct_context,pickup=received_pickup)
            retained=placed['native_ring_result'].get('retention_RGBD')
            if retained is None:raise RuntimeError('Double-handover insertion release retention must be freshly visible')
            if binding.active or loaded_contact_bindings.get('right') is not None:
                raise RuntimeError('RIGHT binding still active after completed physical release')
            register_completed(part,retained['pose'],'Fresh RGB-D after physical double handover, insertion, release and withdrawal')
            results[-1].update(physical_double_handover=completed,right_direct=placed,insertion_fresh_RGBD=placed['native_ring_result']['full_depth_pose'],released_retention_RGBD=retained['pose'])
            events.append(dict(event=part+'_double_handover_right_insertion_complete',wall_s=time.perf_counter()-c['started'],same_original_selected_instance=True,uses_free_body_poses=False))
            c['config']=dict(c['config'],receiver_axial_only=False,temporary_axial_double_handover_component=False)
            pin_initial=None
            next_initial=None
            if c['config'].get('pin_between_rings') and part==ring_order[0] and not c['config'].get('one_ring_only'):
                pin_report=run_pin()
            if part==ring_order[0] and not c['config'].get('one_ring_only'):
                next_initial=vision.request(initial_ring_event(),arms['right'].fk(art.get_dof_positions()[0,indices['right']]))
            continue
        if c['config'].get('handover_only'):return dict(state='physical_handover_camera_confirmed',parts=results,events=events,full_task_pass=False)
        # Adapt the already taught approach shape to the newly observed grasp.
        # These poses come only from stored robot commands + calibrated goals.
        # No recorded or live free-workpiece transforms are used.
        arm=arms['left'];ring_path=np.array(row['approach_ring_poses']);seed_path=np.array(row['approach_robot_seeds']);pre_index=int(row['pre_index']);approach=[]
        goal=np.asarray(c['goals'][part]['P']);goal_R=np.eye(3);route=None;carry_started=time.perf_counter()
        from left_loaded_path_guard import current_query as loaded_left_query,verify_issued as verify_loaded_left_issued
        lc=checkers['left'];carry_scope=loaded_left_query(c,part,left_G);carry_scope.__enter__()
        free_yaw=part=='ring2' and c['config'].get('ring_yaw_free')
        insertion_counts={};insertion_examples={}
        try:
            if free_yaw:
                from fixed_small_left_executor import candidate as saved_SMALL_LEFT_candidate
                saved_candidate=saved_SMALL_LEFT_candidate(c,part,left_G,trajectory,verify_loaded_left_issued)
                if saved_candidate is not None:
                    route=saved_candidate['route'];goal_R=saved_candidate['goal_R']
                    results[-1]['ring_goal_azimuth_rad']=saved_candidate['azimuth'];results[-1]['preinsert_clearance_m']=saved_candidate['clearance']
            if free_yaw and route is None:
                # Class/task IK hint from a saved RGB-D + encoder stroke probe.
                # It contains robot joints, never a scene seed or stock pose.
                insertion_hint=np.asarray(c['robot_route_hints']['ring2_preinsert_q'])
                seeds=[insertion_hint,seed_path[pre_index],row['left_insert_seed'],cmd[0,indices['left']]]+list(np.random.default_rng(7812).uniform(arm.bounds[:,0],arm.bounds[:,1],(48,7)))
                insertion_counts={};insertion_examples={};small_exact_route_fallbacks=0
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
                        if c['config'].get('gpu_dual_arm_reference'):
                            solutions=_left_endpoint_candidates(c,arm,pre_hand,seeds,art.get_dof_positions()[0,indices['left']].copy(),insertion_hint,'left_ring2_preinsert')
                        else:
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
                            if candidate_route is not None:
                                # SMALL needs the same exact issued-curve
                                # admission as LARGE: a coarse route can miss
                                # a narrow held-ring / open-giver fingertip
                                # interference, then fail only when primed.
                                issued=trajectory('left',candidate_route,.05 if c['config'].get('optimized_motion_timing') else 1.4,checked_edges=True)
                                try:verify_loaded_left_issued(c,part,left_G,issued)
                                except RuntimeError as error:
                                    if not str(error).startswith('Loaded LEFT issued curve'):raise
                                    reject_insertion('carry_exact_issued_collision',str(error))
                                    candidate_route=None
                                    if small_exact_route_fallbacks<1:
                                        small_exact_route_fallbacks+=1
                                        old_resolution=getattr(lc,'resolution_rad',.035);lc.resolution_rad=min(old_resolution,.005)
                                        try:
                                            alternative=lc.route(cmd[0,indices['left']],q,D,seconds=1.,skip_direct=True,use_fast=False)
                                        finally:lc.resolution_rad=old_resolution
                                        if alternative is not None:
                                            alternative_issued=trajectory('left',alternative,.05 if c['config'].get('optimized_motion_timing') else 1.4,checked_edges=True)
                                            try:verify_loaded_left_issued(c,part,left_G,alternative_issued)
                                            except RuntimeError as alternative_error:
                                                if not str(alternative_error).startswith('Loaded LEFT issued curve'):raise
                                                reject_insertion('carry_exact_issued_alternative_collision',str(alternative_error))
                                            else:
                                                candidate_route=alternative
                                                c['planning_diagnostics'].append(dict(event='SMALL_carry_exact_issued_guarded_route_alternative',resolution_rad=.005,original_pose_candidate=True,original_fixed_pair_payload_guards=True,uses_free_body_poses=False))
                                    if candidate_route is None:continue
                                route=candidate_route;goal_R=candidate_R;results[-1]['ring_goal_azimuth_rad']=azimuth;results[-1]['preinsert_clearance_m']=clearance;break
                        if route is not None:break
                    if route is not None:break
            elif not free_yaw:
                pre_hand=frame(goal-[0,0,.080],np.eye(3))@np.linalg.inv(left_G)
                carry_width=max(float(art.get_dof_positions()[0,fingers['left']].sum()),float(cmd[0,fingers['left']].sum()))
                carry_seeds=[seed_path[pre_index],row['left_insert_seed'],cmd[0,indices['left']]]+list(np.random.default_rng(231035).uniform(arm.bounds[:,0],arm.bounds[:,1],(32,7)))
                carry_solutions=[]
                if c['config'].get('gpu_dual_arm_reference'):
                    carry_solutions=_left_endpoint_candidates(c,arm,pre_hand,carry_seeds,art.get_dof_positions()[0,indices['left']].copy(),cmd[0,indices['left']],part+'_left_preinsert')
                else:
                    for seed in carry_seeds:
                        q=arm.solve(pre_hand[:3,3],pre_hand[:3,:3],seed)
                        if q is not None and all(np.linalg.norm(q-old)>.05 for old in carry_solutions):carry_solutions.append(q)
                    carry_solutions.sort(key=lambda q:np.linalg.norm(q-cmd[0,indices['left']]))
                for q in carry_solutions:
                    if not lc.clear(q,carry_width)[0]:continue
                    candidate_route=transfer.hinted_route(c,part+'_left_carry',lc,cmd[0,indices['left']],q,carry_width,seconds=3.)
                    if candidate_route is None:continue
                    issued=trajectory('left',candidate_route,.05 if c['config'].get('optimized_motion_timing') else 1.4,checked_edges=True)
                    try:verify_loaded_left_issued(c,part,left_G,issued)
                    except RuntimeError as error:
                        if not str(error).startswith('Loaded LEFT issued curve'):raise
                        rejection=dict(event='large_left_carry_candidate_rejected',reason=str(error));c['planning_diagnostics'].append(rejection);print('LOOSE_LEFT_CARRY_CANDIDATE_REJECTED',rejection,flush=True);continue
                    route=candidate_route;break
        finally:carry_scope.__exit__(*sys.exc_info())
        if free_yaw:
            diagnostics=dict(event='ring2_insertion_search',counts=insertion_counts,examples=insertion_examples,found=route is not None,planning_s=time.perf_counter()-carry_started)
            c['planning_diagnostics'].append(diagnostics);print('LOOSE_INSERTION_SEARCH',diagnostics,flush=True)
        shortcut=route is not None
        if route is None and free_yaw:raise RuntimeError('No payload-clear carry and robot-clear axial stroke over ring azimuth search')
        if route is None:
            # The legacy taught fallback may not bypass current pair/payload
            # clearance merely because no fresh planner route was found.
            for ring_H,seed_q in zip(ring_path[:pre_index+1],seed_path[:pre_index+1]):
                hand_H=ring_H@np.linalg.inv(left_G);q=arm.solve(hand_H[:3,3],hand_H[:3,:3],seed_q)
                if q is None:raise RuntimeError('No adapted taught ring approach IK')
                approach.append(q)
            route=[cmd[0,indices['left']].copy()]+approach
            with loaded_left_query(c,part,left_G):
                clear,why=lc.path(route,max(float(art.get_dof_positions()[0,fingers['left']].sum()),float(cmd[0,fingers['left']].sum())))
                if not clear:raise RuntimeError('Adapted taught LEFT carry is blocked with current payload/RIGHT '+str(why))
        results[-1]['left_carry_shortcut']=shortcut
        print('LOOSE_LEFT_CARRY_PLAN',part,shortcut,time.perf_counter()-carry_started,flush=True)
        end=ring_path[-1]@np.linalg.inv(left_G)
        def insert_program(part=part,G=left_G.copy(),route=route,end=end.copy(),D=D,row=row,free_yaw=free_yaw):
            arm=arms['left'];ids=indices['left'];fi=fingers['left'];goal=np.array(c['goals'][part]['P'])
            holding_ring=True
            def rows(qs,t,width=None,checked_edges=False,loaded_grasp=None,allow_fixture_contact=False):
                issued=trajectory('left',qs,t,width,checked_edges=checked_edges)
                protected_right=None
                if loaded_grasp is not None:protected_right=verify_loaded_left_issued(c,part,loaded_grasp,issued,allow_fixture_contact)
                if overlap is not None:
                    overlap.expose_left(issued,part,loaded_grasp,allow_fixture_contact)
                    # Expose the immutable chunk before any command is issued.
                    yield
                try:
                    for r in issued:
                        if protected_right is not None and np.max(abs(np.r_[cmd[0,indices['right']],cmd[0,fingers['right']]]-protected_right))>1e-6:
                            if overlap is None or not overlap.authorize_left(r):
                                raise RuntimeError('RIGHT command changed during serial loaded LEFT curve; paired execution was not approved')
                        cmd[0,ids]=r[:7];cmd[0,fi]=r[7:]
                        if overlap is not None:overlap.left_row_issued(r)
                        yield
                finally:
                    if overlap is not None:overlap.finish_left()
            def hold(t):
                for _ in range(round(t/dt)):yield
            def photo(label):
                # A weak side-only ICP orientation must not steer a precision
                # insertion. Re-expose the actual cap from bounded clear views.
                offsets=[[-.12,-.20,-.28],[-.22,.16,-.25],[-.20,-.25,-.12]]
                if c['config'].get('robust_annular_camera_view_bank'):
                    offsets+=_annular_cap_camera_view_bank(arm.fk(art.get_dof_positions()[0,ids]),G)
                if c['config'].get('fixed_supply_mode') and part=='ring2':
                    # Reorder existing views only. The third fixed direction
                    # exposed this cap in the live fixed-shelf insertion run.
                    offsets=([offsets[2],offsets[3]]+offsets[:2]+offsets[4:]) if len(offsets)>3 else [offsets[2]]+offsets[:2]
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
            def cartesian(target,t,pose_feedback=False,allow_fixture_contact=False):
                H=arm.fk(cmd[0,ids])
                if pose_feedback and part=='ring1' and c['config'].get('large_encoder_pose_bias_compensation'):
                    actual_H=arm.fk(art.get_dof_positions()[0,ids]);target,bias=_encoder_bias_compensated_target(H,actual_H,target)
                    c['planning_diagnostics'].append(dict(event='large_receiver_encoder_pose_bias_compensation',**bias))
                qs=[cmd[0,ids].copy()]
                for u in np.linspace(0,1,5)[1:]:
                    R=exp(log(target[:3,:3]@H[:3,:3].T)*u)@H[:3,:3]
                    q=arm.solve(H[:3,3]*(1-u)+target[:3,3]*u,R,qs[-1])
                    if q is None:raise RuntimeError('Async left Cartesian IK')
                    qs.append(q)
                curve=CubicSpline(np.linspace(0,1,len(qs)),qs)(np.linspace(0,1,33));opening=max(float(art.get_dof_positions()[0,fi].sum()),float(cmd[0,fi].sum()))
                with loaded_left_query(c,part):clear,rejection=checkers['left'].path(curve,opening)
                if not clear:raise RuntimeError('Image-corrected insertion/withdrawal robot path hits structure '+str(dict(opening_m=opening,collision=rejection)))
                yield from rows(qs,t,loaded_grasp=G if holding_ring else None,allow_fixture_contact=allow_fixture_contact)
            def aligned_hand(position):
                actual_feedback=part=='ring1' and c['config'].get('large_encoder_pose_bias_compensation')
                H=arm.fk(art.get_dof_positions()[0,ids] if actual_feedback else cmd[0,ids]);n=H[:3,:3]@G[:3,2];cross=np.cross(n,[0.,0.,1.]);length=np.linalg.norm(cross)
                rotation=exp(cross/max(length,1e-12)*np.arctan2(length,n[2]))@H[:3,:3]
                return frame(np.asarray(position)-rotation@G[:3,3],rotation)
            yield from rows(route,.05 if c['config'].get('optimized_motion_timing') else 1.4,checked_edges=True,loaded_grasp=G)
            c['set_left_force'](30.)
            for alignment in range(8):
                v=yield from photo(part+'_preinsert_RGBD');G,p=grasp_from(v)
                P=np.asarray(p['center_m']);radial=float(np.linalg.norm(P[:2]-goal[:2]));angle=float(np.arccos(np.clip(p['axis'][2],-1,1)))
                print('LOOSE_LEFT_PREALIGN',part,radial,angle,flush=True)
                if radial<(.0001 if part=='ring1' else .0005) and angle<(.005 if part=='ring1' else .010):break
                if alignment==0 and part=='ring2' and c['config'].get('fixed_supply_mode') and c['config'].get('small_preinsert_one_shot_settled_reconfirmation',False):
                    # One fresh exposure at the SAME commanded pose can avoid
                    # steering a just-settling grasp from a transient sample.
                    # No extra correction iteration or relaxed gate is added.
                    before=dict(radial_m=radial,signed_axis_error_rad=angle)
                    yield from hold(.08)
                    v=yield from photo(part+'_preinsert_settled_reconfirm_RGBD');G,p=grasp_from(v)
                    P=np.asarray(p['center_m']);radial=float(np.linalg.norm(P[:2]-goal[:2]));angle=float(np.arccos(np.clip(p['axis'][2],-1,1)))
                    passed=radial<.0005 and angle<.010
                    c['planning_diagnostics'].append(dict(event='SMALL_preinsert_one_shot_settled_reconfirmation',before=before,after=dict(radial_m=radial,signed_axis_error_rad=angle),qualified=bool(passed),original_radial_limit_m=.0005,original_signed_axis_limit_rad=.010,extra_correction_iterations=0,fresh_RGBD_and_public_encoders=True,uses_free_body_poses=False))
                    if passed:break
                if alignment==7:raise RuntimeError('Free-space ring alignment not confirmed by RGB-D')
                safe_z=min(float(P[2]),float(goal[2]-.045))
                target=aligned_hand([goal[0],goal[1],safe_z]) if free_yaw else frame([goal[0],goal[1],safe_z],np.eye(3))@np.linalg.inv(G)
                yield from cartesian(target,.40,pose_feedback=True);yield from hold(.08)
            observed_preinsert_hand=arm.fk(cmd[0,ids]).copy()
            final_hand=aligned_hand(goal) if free_yaw else frame(goal,np.eye(3))@np.linalg.inv(G)
            yield from cartesian(final_hand,.8,pose_feedback=True,allow_fixture_contact=True);yield from hold(.12)
            angular_repairs=0
            for correction in range(6):
                v=yield from photo(part+'_full_depth_RGBD');G,p=grasp_from(v);P=np.array(p['center_m']);error=float(np.linalg.norm(P-goal));angle=float(np.arccos(np.clip(p['axis'][2],-1,1)));remaining=float(goal[2]-P[2]);radial=float(np.linalg.norm(P[:2]-goal[:2]));print('LOOSE_LEFT_DEPTH',part,error,angle,flush=True)
                if error<(.0007 if part=='ring1' else .0015) and angle<.05:break
                if correction==5 or not .00025<remaining<.020 or radial>.0015 or angle>.05:raise RuntimeError('Ring release blocked: full seating not visible '+part+str((error,angle,remaining)))
                if part=='ring1' and angle>.010 and angular_repairs<2:
                    # A tilted rim can seat on one edge. Clear the contact,
                    # remeasure the held transform, and align the observed
                    # centre/axis before retrying the axial stroke.
                    angular_repairs+=1
                    c['planning_diagnostics'].append(dict(event=part+'_contact_tilt_recovery',attempt=angular_repairs,observed_error_m=error,observed_axis_error_rad=angle,backoff_m=.020,uses_free_body_poses=False))
                    backoff=arm.fk(cmd[0,ids]);backoff[2,3]-=.020
                    yield from cartesian(backoff,.30,allow_fixture_contact=True);yield from hold(.08)
                    for realignment in range(5):
                        rv=yield from photo(part+'_retracted_alignment_RGBD');G,rp=grasp_from(rv);rP=np.asarray(rp['center_m']);rangle=float(np.arccos(np.clip(rp['axis'][2],-1,1)));rradial=float(np.linalg.norm(rP[:2]-goal[:2]))
                        if rradial<.0001 and rangle<.005:break
                        if realignment==4:raise RuntimeError('Retracted large ring alignment not confirmed; receiver remains closed')
                        target=aligned_hand([goal[0],goal[1],min(float(rP[2]),float(goal[2]-.010))])
                        yield from cartesian(target,.40,pose_feedback=True);yield from hold(.08)
                    yield from cartesian(aligned_hand(goal),.8,pose_feedback=True,allow_fixture_contact=True);yield from hold(.12)
                    continue
                H=arm.fk(cmd[0,ids]);H[2,3]+=remaining;yield from cartesian(H,.30,allow_fixture_contact=True);yield from hold(.12)
            c['close_clamp'](part);yield from hold(.12)
            binding=loaded_contact_bindings.pop('left',None)
            if binding is not None:
                binding.release();checkers['left'].loaded_contact_aperture=None
            c['set_grip_hold']('left',False);holding_ring=False
            # Release with clearance around the measured 62 mm ring rather
            # than spreading to 80 mm inside its narrow fixture. Reverse the
            # insertion direction before a later empty-hand reposition.
            release_opening=min(.08,D+.002) if part=='ring2' else .08
            yield from rows([cmd[0,ids],cmd[0,ids]],.20,release_opening);yield from hold(.25)
            H=arm.fk(cmd[0,ids])
            if part=='ring2':H=observed_preinsert_hand.copy()
            # Return to the checked preinsert pose instead of extending
            # below the validated approach and into the machine.
            else:H=H@frame([0,0,-.08],np.eye(3))
            yield from cartesian(H,.6);c['left_hand_known_empty']=True;c['set_left_force'](float(c['config'].get('native_grip_pad_force_n',30.)));results[-1]['insertion_fresh_RGBD']=p
            released_world=frame(p['center_m'],axis_frame(p['axis']));G=np.linalg.inv(arm.fk(art.get_dof_positions()[0,ids]))@released_world
            released=yield from photo(part+'_released_retention_RGBD');_,released_pose=grasp_from(released)
            register_completed(part,released_pose,'Fresh RGB-D after release and withdrawal');results[-1]['released_retention_RGBD']=released_pose
            if part==ring_order[0] and not c['config'].get('one_ring_only') and not c['config'].get('pin_between_rings'):
                if c['config'].get('fixed_supply_mode') and direct_right_insert_allowed(c['config'],ring_order[1]):
                    # The next part is carried/inserted by RIGHT. The actual
                    # LEFT withdrawal and fresh released-retention observation
                    # already completed; an unused old handover standby may
                    # cross that newly loaded RIGHT and is not a task goal.
                    if not c.get('left_hand_known_empty'):raise RuntimeError('Cannot retain LEFT pose without confirmed empty-hand ownership')
                    c['planning_diagnostics'].append(dict(event='fixed_supply_LEFT_retains_verified_empty_postwithdraw_pose',next_part=ring_order[1],reason='Next original skill uses direct RIGHT insertion; fresh current-context planning remains mandatory',actual_withdrawal_and_released_retention_completed=True,uses_free_body_poses=False))
                    return
                return_route=checkers['left'].route(cmd[0,ids],left_standby_q,.08,seconds=.35)
                if return_route is None:raise RuntimeError('No empty receiver return route to handover standby')
                yield from rows(return_route,.05 if c['config'].get('optimized_motion_timing') else .8,checked_edges=True)
                next_opening=transfer.receiver_opening(c,ring_order[1])
                yield from rows([cmd[0,ids],cmd[0,ids]],.10,next_opening)
        left_program=insert_program()
        if overlap is not None:
            overlap.awaiting_prime=True
            from fixed_small_left_executor import SavedCommandRoute
            if isinstance(route,SavedCommandRoute):
                # Reuse the just-passed exact epoch proof. Priming stops at the
                # existing exposure yield; future paired guards stay fresh.
                prime_epoch=control_steps;prime_command=cmd.copy()
                prime_left_chunk()
                if control_steps!=prime_epoch or not np.array_equal(cmd,prime_command):
                    raise RuntimeError('Saved LEFT chunk exposure changed commands or advanced physics')
        if c['config'].get('pin_between_rings') and part==ring_order[0] and not c['config'].get('one_ring_only'):
            pin_report=run_pin()
            next_initial=vision.request(initial_ring_event(),arms['right'].fk(art.get_dof_positions()[0,indices['right']]))
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
    final_retreat=None
    if c['config'].get('final_empty_right_clearance'):
        if len(results)!=2 or pin_report is None:raise RuntimeError('Final empty retreat requires completed two-ring and pin sequence')
        module=types.ModuleType('final_empty_clearance');module.__file__=str(HERE/'final_empty_clearance.py');exec(compile(Path(module.__file__).read_text(),module.__file__,'exec'),module.__dict__)
        final_retreat=module.execute(c,trajectory,step)
    retention_begin=control_steps if final_retreat and final_retreat['performed'] else max(left_finished_step,pin_retention_start if pin_retention_start is not None else control_steps)
    wait(max(0.,.55-(control_steps-retention_begin)*dt),'loose_all_parts_retention')
    return dict(state='loose_bin_recipe_executed',parts=results,pin=pin_report,events=events,full_task_pass=False,elapsed_s=time.perf_counter()-c['started'])
