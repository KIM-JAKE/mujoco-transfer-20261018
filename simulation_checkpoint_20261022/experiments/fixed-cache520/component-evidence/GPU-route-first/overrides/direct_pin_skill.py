"""One uninterrupted tail-grasp pin skill; fresh RGB-D plus encoders only.

The caller advances the other arm and records the shared physical/wall clock.
No scene initialization, pose resets or workpiece attachments occur here.
"""
from pathlib import Path
import time,json,numpy as np
from pxr import UsdPhysics
from scipy.interpolate import CubicSpline
from isaac_kinematics import frame,exp,log
from rgbd_geometry import axis_frame
from fr3_motion_control import position_velocity_bounds
from fr3_path_timing import retime,retime_safe_polyline


class PinCartesianPreflightBlocked(RuntimeError):
    """Geometry rejected before motion(); never a partial-stroke error."""


def _pin_visual_preinsert_target(command_H,actual_H,measured_G,goal,goal_axis,observed_z,gain=1.,translation_deadband_m=0.,axis_deadband_rad=0.,axis_gain=None,preinsert_depth_m=.045):
    """Bounded geometric center/axis servo; only images, encoders and jig CAD."""
    preinsert_depth_m=float(preinsert_depth_m)
    if preinsert_depth_m not in(.045,.018):raise ValueError('PIN feedback depth plane must be original45mm or explicit18mm entry')
    C,A,G=[np.asarray(x,float)for x in [command_H,actual_H,measured_G]]
    goal=np.asarray(goal,float);goal_axis=np.asarray(goal_axis,float)
    if any(T.shape!=(4,4)or not np.isfinite(T).all()for T in[C,A,G]) or goal.shape!=(3,) or goal_axis.shape!=(3,) or not np.isfinite(np.r_[goal,goal_axis,observed_z]).all():raise ValueError('Invalid pin visual-servo input')
    goal_axis=goal_axis/max(np.linalg.norm(goal_axis),1e-12);axis=A[:3,:3]@G[:3,2];cross=np.cross(axis,goal_axis);length=np.linalg.norm(cross)
    translation_deadband_m=float(translation_deadband_m);axis_deadband_rad=float(axis_deadband_rad)
    if not 0<=translation_deadband_m<=.000075 or not 0<=axis_deadband_rad<=.003:raise ValueError('PIN component deadbands exceed original precision acceptance limits')
    observed=A@G;radial=float(np.linalg.norm(observed[:2,3]-goal[:2]));axis_error=float(np.arccos(np.clip(axis@goal_axis,-1,1)))
    translation_held=translation_deadband_m>0 and radial<=translation_deadband_m
    rotation_held=axis_deadband_rad>0 and axis_error<=axis_deadband_rad
    # Keep a component already inside its ORIGINAL final acceptance gate.
    # Other components still correct, and the coupled shaft gate remains live.
    rotation=A[:3,:3].copy() if rotation_held else exp(cross/max(length,1e-12)*np.arctan2(length,axis@goal_axis))@A[:3,:3]
    gain=float(gain)
    if not .3<=gain<=1.:raise ValueError('Pin geometric feedback gain must be0.3--1')
    component_gains=axis_gain is not None
    if component_gains:
        axis_gain=float(axis_gain)
        if not .3<=axis_gain<=1.:raise ValueError('Pin axis feedback gain must be0.3--1')
        # Apply independent gains in OBSERVED PART space. Scaling the Hand
        # translation separately from its rotation would move the pin center
        # around the grasp lever arm and break a previously qualified XY.
        rotation=exp(axis_gain*log(rotation@A[:3,:3].T))@A[:3,:3]
        target_xy=observed[:2,3] if translation_held else observed[:2,3]+gain*(goal[:2]-observed[:2,3])
        target_z=float(observed_z)+gain*(min(float(observed_z),float(goal[2]-preinsert_depth_m))-float(observed_z))
    else:
        target_xy=observed[:2,3] if translation_held else goal[:2]
        target_z=min(float(observed_z),float(goal[2]-preinsert_depth_m))
    desired=frame(np.r_[target_xy,target_z]-rotation@G[:3,3],rotation)
    bias_translation=float(np.linalg.norm(C[:3,3]-A[:3,3]));bias_angle=float(np.linalg.norm(log(C[:3,:3]@A[:3,:3].T)))
    if bias_translation>.002 or bias_angle>.01:raise RuntimeError('Pin encoder tracking bias exceeds bounded2mm/.01rad prealignment correction')
    target=C@np.linalg.inv(A)@desired
    if not component_gains:
        target[:3,3]=C[:3,3]+gain*(target[:3,3]-C[:3,3])
        target[:3,:3]=exp(gain*log(target[:3,:3]@C[:3,:3].T))@C[:3,:3]
    if np.linalg.norm(target[:3,3]-C[:3,3])>.008 or np.linalg.norm(log(target[:3,:3]@C[:3,:3].T))>.1:raise RuntimeError('Pin prealignment correction exceeds8mm/.1rad local envelope')
    return target,dict(command_vs_actual_translation_m=bias_translation,command_vs_actual_rotation_rad=bias_angle,geometric_feedback_gain=gain,independent_axis_feedback_gain=axis_gain,part_space_component_feedback=component_gains,qualified_XY_preserved=translation_held,qualified_axis_preserved=rotation_held,component_translation_deadband_m=translation_deadband_m,component_axis_deadband_rad=axis_deadband_rad)

def _pin_bounded_xy_gain(gain,radial,previous,streak,ceiling=.65):
    """Recover bounded XY response only after two measured strong improvements."""
    gain,radial,ceiling=float(gain),float(radial),float(ceiling)
    if not .3<=gain<=ceiling<=.65 or radial<0 or not np.isfinite([gain,radial,ceiling]).all():raise ValueError('Invalid bounded PIN XY feedback state')
    if previous is None:return gain,0
    previous=float(previous)
    if not np.isfinite(previous)or previous<0:raise ValueError('Invalid previous PIN radial error')
    if radial<=.000075:return gain,0
    if radial>=.95*previous:return max(.3,gain*.65),0
    streak=int(streak)+1 if radial<.9*previous else 0
    if streak>=2:return min(ceiling,gain/.65),0
    return gain,streak

def _pin_engine_readonly_diagnostics(stage,sim,arm):
    """Evaluator metadata only: engine state NEVER enters the servo policy."""
    from pxr import UsdUtils,Sdf,PhysicsSchemaTools,PhysxSchema
    data=dict(scope='Privileged engine diagnostics only; NOT a controller sensor',used_for_control=False,scene_configuration=[],articulation_configuration=[],body_sleeping={})
    try:
        for prim in stage.Traverse():
            if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
                api=PhysxSchema.PhysxSceneAPI(prim);a=api.GetEnableStabilizationAttr();solver=api.GetSolverTypeAttr()
                data['scene_configuration'].append(dict(path=str(prim.GetPath()),enable_stabilization=a.Get(),authored=bool(a.HasAuthoredValueOpinion()),solver_type=solver.Get(),solver_type_authored=bool(solver.HasAuthoredValueOpinion())))
            if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
                api=PhysxSchema.PhysxArticulationAPI(prim);sleep=api.GetSleepThresholdAttr();stabilization=api.GetStabilizationThresholdAttr()
                data['articulation_configuration'].append(dict(path=str(prim.GetPath()),sleep_threshold=sleep.Get(),sleep_threshold_authored=bool(sleep.HasAuthoredValueOpinion()),stabilization_threshold=stabilization.Get(),stabilization_threshold_authored=bool(stabilization.HasAuthoredValueOpinion())))
        stage_id=UsdUtils.StageCache.Get().GetId(stage).ToLongInt();data['stage_id']=int(stage_id)
        for name,path in [('hand',arm.hand_path),('link7',arm.link_paths[-1])]:
            prim=stage.GetPrimAtPath(path);is_rigid=bool(prim and prim.HasAPI(UsdPhysics.RigidBodyAPI))
            data['body_sleeping'][name]=dict(path=path,is_rigid_body=is_rigid,is_sleeping=bool(sim.is_sleeping(stage_id,PhysicsSchemaTools.sdfPathToInt(Sdf.Path(path))))if is_rigid else None)
    except Exception as error:
        # Diagnostic availability cannot affect motion/acceptance decisions.
        data['diagnostic_error']=type(error).__name__+': '+str(error)
    return data

def _pin_rgbd_sdk_hold_evidence(pose,reference_G,measured_G,opening,stationary=True):
    """Confirm observed retention using circle/axis + encoder aperture, no yaw."""
    A,B=[np.asarray(x,float)for x in [reference_G,measured_G]];gap=float(opening)
    if A.shape!=(4,4)or B.shape!=(4,4)or not np.isfinite(A).all()or not np.isfinite(B).all()or not np.isfinite(gap):raise ValueError('Invalid pin RGB-D/SDK hold evidence')
    translation=float(np.linalg.norm(B[:3,3]-A[:3,3]));a=A[:3,2]/max(np.linalg.norm(A[:3,2]),1e-12);b=B[:3,2]/max(np.linalg.norm(B[:3,2]),1e-12);axis=float(np.arccos(np.clip(a@b,-1,1)));radius=float(pose.get('visible_end_radius_m',float('nan')));expected=float(pose.get('cap_expected_radius_m',float('nan')))
    direct=bool(pose.get('robust_cap_circle_from_pixels')and pose.get('lateral_from_previous_RGBD_grasp_and_encoders')is False and pose.get('uses_depth_sensor')and pose.get('cap_edge_coverage_deg',0)>=240 and np.isfinite(radius)and np.isfinite(expected)and min(abs(expected-.00525),abs(expected-.0025))<.0001 and abs(radius-expected)<=.0004)
    aperture=bool(.0097<gap<.0135)
    return dict(confirmed=bool(direct and aperture and stationary and translation<=.002 and axis<=.05),direct_class_end_circle=direct,actual_encoder_aperture_m=gap,required_aperture_band_m=[.0097,.0135],encoder_position_stationary=bool(stationary),relative_center_drift_m=translation,relative_axis_drift_rad=axis,center_drift_limit_m=.002,axis_drift_limit_rad=.05,unobservable_axial_yaw_used=False,native_sample_normals_advisory_only=True,source='Fresh RGB-D end-circle size/center/axis and actual joint encoders; no force sensor or simulator pose',uses_free_body_poses=False)

def _pin_ordered_camera_views(offsets,label,config):
    """Reorder existing virtual camera candidates; every capture stays fresh.

    Preferred indices are calibrated from successful RGB-D images for this
    skill phase. All original candidates remain as fallback and retain their
    original indices in evidence. No sensed pose or geometric gate is reused.
    """
    orders=config.get('pin_rgbd_view_order_by_label')
    if orders is None:return list(enumerate(offsets))
    if not isinstance(orders,dict):raise ValueError('PIN RGB-D view ordering must map phase labels to original view indices')
    preferred=orders.get(label,[])
    if not isinstance(preferred,list)or any(type(i)is not int or not 0<=i<len(offsets)for i in preferred)or len(set(preferred))!=len(preferred):
        raise ValueError('Invalid PIN RGB-D preferred view indices for '+label)
    order=preferred+[i for i in range(len(offsets))if i not in preferred]
    return [(i,offsets[i])for i in order]

def _pin_gpu_cubic_route_before_hint(checker,start,goal,opening,selected):
    """Existing GPU proposal plus unchanged live native continuous admission."""
    if not getattr(checker,'gpu_route_enabled',False):return None
    if not checker.clear(goal,opening,selected)[0]:return None
    from fr3_gpu_route import propose_route
    proposed=propose_route(checker,start,goal,opening,selected)
    if proposed is None:return None
    from gpu_route_curve_validation import cubic_geometry
    from grasp_transfer_feasibility import VerifiedCubicRoute
    continuous,curve_evidence=cubic_geometry(proposed,checker.arm.bounds,step_rad=.010)
    if continuous is None:return None
    old_resolution=getattr(checker,'resolution_rad',.035);checker.resolution_rad=.010
    try:continuous_clear,continuous_why=checker.path(continuous,opening,selected)
    finally:checker.resolution_rad=old_resolution
    if not continuous_clear:return None
    checker.gpu_route_stats['PhysX_verified_continuous_proposals']=checker.gpu_route_stats.get('PhysX_verified_continuous_proposals',0)+1
    print('PIN_GPU_ROUTE_BEFORE_HINT_NATIVE_VERIFIED',dict(curve_evidence,fixed_pair_payload_checked=True,issued_curve_and_FR3_retiming_still_required=True),flush=True)
    return VerifiedCubicRoute(proposed)


def execute(ctx):
    preclose_gain=float(ctx.get('config',{}).get('pin_preclose_servo_gain',.5))
    if not np.isfinite(preclose_gain)or not .5<=preclose_gain<=.8:raise ValueError('PIN preclose gain must remain within0.5--0.8; original error and motion envelopes remain mandatory')
    if ctx.get("config",{}).get("pin_prefer_canonical_pickup_branch") and not ctx.get("config",{}).get("fixed_supply_mode"):
        raise ValueError("Canonical PIN pickup branch preference requires fixed_supply_mode")
    HERE=Path(__file__).resolve().parent
    cand=ctx['candidate'];flat_tail_proposal=None
    flat_tail_enabled=bool(ctx.get('config',{}).get('pin_tail_flat_contact_target',False))
    parallel_tail_enabled=bool(ctx.get('config',{}).get('pin_parallel_tail_contact_target',False))
    collar_flat_enabled=bool(ctx.get('config',{}).get('pin_collar_flat_contact_target',False))
    if int(flat_tail_enabled)+int(parallel_tail_enabled)+int(collar_flat_enabled)>1:raise ValueError('PIN physical target proposals are mutually exclusive')
    if collar_flat_enabled:
        from pin_collar_contact_target import retarget
        cand,flat_tail_proposal=retarget(cand,offset_m=ctx.get('config',{}).get('pin_collar_flat_offset_m',-.002))
    elif parallel_tail_enabled:
        from pin_parallel_tail_target import retarget
        cand,flat_tail_proposal=retarget(cand)
    elif flat_tail_enabled:
        from pin_tail_contact_target import retarget
        cand,flat_tail_proposal=retarget(cand)
    art=ctx['art'];arm=ctx['arm'];ids=ctx['ids'];fi=ctx['fi'];cmd=ctx['cmd'];idx=ctx['idx'];vision=ctx['vision'];checker=ctx['checker'];stage=ctx['stage'];sim=ctx['sim'];paths=ctx['paths'];goal=np.array(ctx['goal']);truth=ctx['truth'];started=ctx['started'];stiff=ctx['stiffness'];damping=ctx['damping'];DT=ctx['dt'];G=np.array(cand['grasp_transform']);pickupH=np.array(cand['pickup_H']);preH=pickupH.copy();preH[:3,3]+=[0,0,.07];preq=arm.solve(preH[:3,3],preH[:3,:3],cand['pickup_q']);phase='pin_start';report=ctx.setdefault('report',{});report.update(events=[],pin_single_stroke_pass=False)
    if flat_tail_proposal is not None:report['pin_tail_flat_contact_target']=flat_tail_proposal;print('PIN_TAIL_FLAT_CONTACT_TARGET',flat_tail_proposal,flush=True)
    body_goal=goal.copy();pin_config=ctx.get('config',{})
    visual_preinsert=bool(pin_config.get('pin_visual_preinsert_alignment'))
    qualified_full_depth=bool(pin_config.get('pin_full_depth_require_robust_circle'))
    if qualified_full_depth and not(visual_preinsert and pin_config.get('robust_pin_cap_circle')):raise ValueError('Qualified PIN full-depth observation requires original visual preinsert and robust circle settings')
    pin_axis_geometry=bool(pin_config.get('pin_axis_frame_consistent_geometry') or cand.get('pose_origin')=='RGB-D physical symmetry-axis midpoint')
    pin_sensor_hold=bool(pin_config.get('pin_rgbd_sdk_hold_confirmation'))
    entry_feedback_enabled=bool(pin_config.get('pin_two_stage_entry_feedback'))
    if entry_feedback_enabled and not(visual_preinsert and pin_sensor_hold and qualified_full_depth and pin_config.get('pin_terminal_stroke_preflight')):raise ValueError('Two-stage PIN entry needs original visual/hold/full-terminal and qualified-depth policies')
    pickup_opening_m=.024 if pin_config.get('pin_preclose_wide_opening')else.014
    if pin_sensor_hold and not pin_axis_geometry:raise ValueError('RGB-D/SDK pin hold confirmation requires consistent physical-axis geometry')
    if pin_axis_geometry:
        from pin_native_contact import native_support,axis_to_body_grasp
        report['pin_native_geometry_origin']='RGB-D physical axis; CADbody−[0,−.25mm,0]; no physical model change'
    def payload_grasp():return axis_to_body_grasp(G) if pin_axis_geometry else G
    def measured_grasp(value,prior):
        p=value['result']['pose'];measured=np.linalg.inv(value['H'])@frame(p['center_m'],axis_frame(p['axis']));a=prior[:3,2];b=measured[:3,2];v=np.cross(a,b);measured[:3,:3]=exp(v/max(np.linalg.norm(v),1e-12)*np.arctan2(np.linalg.norm(v),a@b))@prior[:3,:3];return measured
    if visual_preinsert:
        axis_goal=np.asarray(pin_config.get('pin_axis_goal_world_m',[]),float)
        if axis_goal.shape!=(3,) or not np.isfinite(axis_goal).all():raise ValueError('Pin visual servo requires a calibrated physical symmetry-axis midpoint goal')
        goal=axis_goal
        report.update(body_origin_goal_world_m=body_goal.tolist(),physical_axis_midpoint_goal_world_m=goal.tolist(),control_pose_origin='RGB-D physical symmetry-axis midpoint; fixed CAD jig calibration')
    def endpoint_candidates(H,seeds,label):
        from fr3_gpu_ik import propose_ik
        seen=[]
        def terminal_admitted(q):
            if not pin_config.get('pin_terminal_stroke_preflight') or label not in ['pin_preinsert_carry','pin_tail_carry_to_preinsert']:return True
            from pin_terminal_geometry import check_terminal
            width=max(float(art.get_dof_positions()[0,fi].sum()),float(cmd[0,fi].sum()))
            ok,evidence=check_terminal(ctx,arm,checker,q,.045,width,paths['workpiece_pin'],margin=.03)
            evidence.update(label=label,accepted=bool(ok));report.setdefault('pin_terminal_branch_preflight',[]).append(evidence);print('PIN_TERMINAL_BRANCH_PREFLIGHT',evidence,flush=True)
            return ok
        for q in propose_ik(ctx,arm,H[:3,3],H[:3,:3],art.get_dof_positions()[0,ids].copy(),label):
            seen.append(q)
            if terminal_admitted(q):yield q
        for seed in seeds:
            if seed is None:continue
            q=arm.solve(H[:3,3],H[:3,:3],seed)
            if q is not None and all(np.linalg.norm(q-old)>.05 for old in seen):
                seen.append(q)
                if terminal_admitted(q):yield q
    def step():ctx['step'](phase)
    def wait(seconds,label):
        nonlocal phase
        phase=label
        for _ in range(round(seconds/DT)):step()
    def await_precision_encoder_window(label):
        # Consecutive actual control steps, never elapsed camera/CPU wall time.
        nonlocal phase
        phase=label;samples=[];velocity_samples=[];joint_speed_limit=float(pin_config.get('pin_precision_max_joint_speed_rad_s',0.))
        if not 0<=joint_speed_limit<=.01:raise ValueError('Invalid PIN precision public joint speed limit')
        for k in range(int(np.ceil(2./DT))+1):
            value=arm.fk(art.get_dof_positions()[0,ids]);samples.append(value);samples=samples[-49:]
            velocity_samples.append(float(np.max(abs(art.get_dof_velocities()[0,ids]))));velocity_samples=velocity_samples[-49:]
            if len(samples)==49:
                translation=max(float(np.linalg.norm(v[:3,3]-samples[0][:3,3]))for v in samples)
                rotation=max(float(np.linalg.norm(log(v[:3,:3]@samples[0][:3,:3].T)))for v in samples)
                if translation<=.000025 and rotation<=.0005 and (joint_speed_limit==0. or max(velocity_samples)<=joint_speed_limit):
                    evidence=dict(control_steps=k,physics_window_s=48*DT,translation_span_m=translation,rotation_span_rad=rotation,source='Actual encoder FK; no object truth; speeds/accuracy gates unchanged',maximum_actual_joint_speed_rad_s=max(velocity_samples),actual_joint_speed_limit_rad_s=joint_speed_limit)
                    if pin_config.get('pin_engine_readonly_diagnostics',False):evidence['engine_diagnostics_only']=_pin_engine_readonly_diagnostics(stage,sim,arm)
                    report.setdefault('precision_encoder_windows',[]).append(evidence);print('PIN_PRECISION_ENCODER_WINDOW',evidence,flush=True);return
            step()
        raise RuntimeError('Pin precision encoder pose did not become stationary within2physical seconds')
    def observe(label,grasp=None,require_circle=False):
        H=arm.fk(art.get_dof_positions()[0,ids]);prefetched=ctx.pop('prefetched_initial',None) if grasp is None else None
        offsets=[None] if grasp is None else [([-.22,0,.12] if label=="pin_lift_RGBD" else [-.22,-.15,-.12])]
        if grasp is not None and visual_preinsert:
            # Virtual simulated views, not one fixed physical RealSense.
            axis=(H@grasp)[:3,2];axis/=np.linalg.norm(axis);reference=np.eye(3)[int(np.argmin(abs(axis)))];u=reference-axis*(reference@axis);u/=np.linalg.norm(u);v=np.cross(axis,u)
            offsets += [(.20*sign*axis+.23*(np.cos(angle)*u+np.sin(angle)*v)).tolist() for sign in [1.,-1.] for angle in np.arange(6)*np.pi/3]
        errors=[]
        views=_pin_ordered_camera_views(offsets,label,pin_config)
        if pin_config.get('fixed_supply_mode') and label=='pin_holder_preclose_RGBD' and pin_config.get('fixed_pin_preclose_overhead_views',True):
            # Public encoder Hand FK times the camera-measured/catalog grasp
            # sets the target as before. Original443 branch occluded every
            # distant oblique view; these0.25--0.30m overhead lenses exposed
            # the complete physical cap in recorded-frame image-only replay.
            # Keep all13 old views, original circle/coverage/precision gates.
            overhead=[[0.,0.,.25],[.10,0.,.25],[0.,0.,.30]]
            views=[(len(offsets)+i,value)for i,value in enumerate(overhead)]+views
        for view_index,offset in views:
            e=dict(mode='pin_initial'if grasp is None else'held',part='pin',label=label,view_index=view_index,robust_pin_cap_circle=bool(require_circle and pin_config.get('robust_pin_cap_circle')))
            if grasp is not None:e.update(tracked_grasp=grasp.tolist(),camera_eye_offset=offset)
            if require_circle and visual_preinsert and pin_config.get('pin_precision_image_width_px') is not None:e['pin_precision_image_width_px']=int(pin_config['pin_precision_image_width_px'])
            if prefetched is not None:e,f=prefetched;prefetched=None
            else:f=vision.request(e,arm.fk(art.get_dof_positions()[0,ids]))
            # Same3s capture-to-ready ceiling used by ring observations.
            # Poll after a potentially long physics/queued-proof step before
            # declaring a camera late; delivery delay is not inference delay.
            deadline=time.perf_counter()+3.
            while not f.done():
                vision.poll()
                if f.done():break
                step();vision.poll()
                if not f.done() and time.perf_counter()>deadline:raise RuntimeError('RGB-D response timeout')
            value=f.result();captured=value.get('captured_wall_s');ready=value.get('ready_wall_s')
            if captured is not None and ready is not None:
                latency=float(ready)-float(captured)
                if not np.isfinite(latency) or latency<0 or latency>3.:raise RuntimeError('RGB-D capture-to-ready exceeded3s')
            elif time.perf_counter()>deadline:raise RuntimeError('RGB-D response timeout without timely-ready metadata')
            e['delivered_wall_s']=time.perf_counter()-started
            error=value['result'].get('error');pose=value['result'].get('pose')
            if not error and require_circle and (not pose or pose.get('lateral_from_previous_RGBD_grasp_and_encoders',True)):
                error='Pin precision alignment needs directly observed end-circle XY, not propagated XY'
            if qualified_full_depth and label=='pin_full_depth_RGBD' and not(pose and pose.get('robust_cap_circle_from_pixels')is True):
                error=error or 'PIN full-depth requires the original robust class-radius circular-cap proof'
            if error:
                errors.append(str(error).splitlines()[-1]);continue
            if qualified_full_depth and label=='pin_full_depth_RGBD':
                report.setdefault('pin_full_depth_qualified_observations',[]).append(dict(view_index=view_index,captured_wall_s=captured,ready_wall_s=ready,delivered_wall_s=e['delivered_wall_s'],robust_cap_circle_from_pixels=True,original_cap_quality={key:pose.get(key)for key in ['cap_expected_radius_m','visible_end_radius_m','cap_edge_coverage_deg','cap_circular_edge_inliers','cap_total_hull_edge_points','cap_circular_edge_rms_m','cap_points','depth_points','plane_rms_m']},same_existing_camera_view_bank=True,original_depth_axis_precision_and_motor_gates_unchanged=True,uses_free_body_poses=False))
            report['events'].append(dict(event=label,wall_s=e['delivered_wall_s'],result=value['result'],view_index=view_index));print('PIN_CONTACT_VISION',label,e['delivered_wall_s'],flush=True);return value
        raise RuntimeError('No valid pin RGB-D view '+label+' '+str(errors))
    def motion(qs,requested,label,checked_edges=True):
        nonlocal phase
        checked_edges=checked_edges and not getattr(qs,'verified_cubic',False);qs=np.array(qs);qs[0]=cmd[0,ids];T=requested
        # Smooth joint spline sampled at the real control rate, checked against
        # position-dependent bounds, acceleration and jerk before execution.
        opening=cmd[0,fi].copy();preshape=label=='pin_tail_approach';optimized=ctx.get('optimized_motion_timing',False)
        if optimized and any(name in label for name in ['approach','carry','orient_free','lift']):T=.05
        scale=max(1.,float(ctx.get('stability_time_scale',1.)))
        if checked_edges and len(qs)>2:
            q,h,timing=retime_safe_polyline(qs,DT,opening,np.full(2,pickup_opening_m/2) if preshape else opening,scale=scale,velocity_fraction=.95,acceleration=9.5,jerk=4750.)
        else:
            q,h,timing=retime(qs,T,DT,opening,np.full(2,pickup_opening_m/2) if preshape else opening,velocity_fraction=.95,acceleration=9.5,jerk=4750.,finger_command_speed=.0475,optimize=optimized)
            if scale>1:q,h,timing=retime(qs,timing['duration_s']*scale,DT,opening,np.full(2,pickup_opening_m/2) if preshape else opening,velocity_fraction=.95,acceleration=9.5,jerk=4750.,finger_command_speed=.0475,optimize=False)
        phase=label
        for row,hvalue in zip(q[1:],h[1:]):
            cmd[0,ids]=row
            if preshape:cmd[0,fi]=opening+(pickup_opening_m/2-opening)*hvalue
            step()
        report['events'].append(dict(event=label,wall_s=time.perf_counter()-started,physical_duration_s=(len(q)-1)*DT));print('PIN_CONTACT_MOVE',label,time.perf_counter()-started,flush=True)
    def line(H0,H1,requested,label):
        H0=arm.fk(cmd[0,ids]);rows=[cmd[0,ids].copy()];last=rows[0]
        for u in np.linspace(0,1,7)[1:]:
            T=H0.copy();T[:3,3]=H0[:3,3]*(1-u)+H1[:3,3]*u;T[:3,:3]=exp(log(H1[:3,:3]@H0[:3,:3].T)*u)@H0[:3,:3] if label in ['pin_preinsert_visual_correction','pin_holder_regrasp_alignment'] else H0[:3,:3];q=arm.solve(T[:3,3],T[:3,:3],last)
            if q is None:raise RuntimeError('Axial IK failed '+label)
            rows.append(q);last=q
        opening=max(float(art.get_dof_positions()[0,fi].sum()),float(cmd[0,fi].sum()))
        curve=CubicSpline(np.linspace(0,1,len(rows)),rows)(np.linspace(0,1,41));clear,why=checker.path(curve,opening,paths['workpiece_pin'])
        if not clear:raise PinCartesianPreflightBlocked('Pin Cartesian robot path obstructed '+label+' '+str(why))
        motion(rows,requested,label,checked_edges=False)
    def fingers(width,label):
        nonlocal phase
        phase=label;start=cmd[0,fi].copy();target=np.array([width/2]*2);T=max(.14,float(abs(target-start).max())/.0475*1.875);N=int(np.ceil(T/DT))
        for k in range(1,N+1):u=k/N;h=u**3*(10-15*u+6*u*u);cmd[0,fi]=start+(target-start)*h;step()
    v=observe('pin_initial')
    if ctx.get('handoff_withdraw'):
        H=arm.fk(art.get_dof_positions()[0,ids]);withdraw=H@frame([0,0,-.07],np.eye(3));line(H,withdraw,.65,'pin_donor_handoff_withdraw')
    # Empty stock pickup needs a narrow aperture near the existing holder.
    # Preshape at the already clear transfer pose, before planning the route.
    if abs(float(cmd[0,fi].sum())-pickup_opening_m)>.0005:
        H=arm.fk(cmd[0,ids]);opening=float(art.get_dof_positions()[0,fi].sum())
        if pin_config.get('pin_preclose_wide_opening'):
            from pin_empty_aperture_guard import check as preshape_guard
            report['pin_empty_preshape_sweep']=preshape_guard(ctx,pickup_opening_m)
        deadline=time.perf_counter()+2.5
        while not all(checker.hand_clear(H,w)[0] for w in np.linspace(opening,pickup_opening_m,9)):
            if time.perf_counter()>deadline:raise RuntimeError('No clear empty-hand preshape before pin pickup')
            wait(.025,'pin_waits_for_receiving_arm_clearance')
        fingers(pickup_opening_m,'pin_empty_preshape')
    P=np.array(v['result']['pose']['center_m']);pickup_origin=np.asarray(cand.get('pickup_center_from_RGBD',json.load(open(HERE.parents[1]/'runs/native-roas-table-pin-tail01/pin-direct-probe.json'))['pickup_center_from_RGBD']),float)
    if pickup_origin.shape!=(3,) or not np.isfinite(pickup_origin).all():raise ValueError('Pin candidate requires finite RGB-D pickup origin')
    pickupH[:3,3]+=P-pickup_origin;report['pickup_retarget_origin_RGBD_m']=pickup_origin.tolist();report['pickup_retarget_delta_RGBD_m']=(P-pickup_origin).tolist()
    newpre=pickupH.copy();newpre[:3,3]+=[0,0,.07]
    robot_hint_data=json.loads((HERE/'bin-picking/direct-pin-robot-seeds.json').read_text());robot_seeds=robot_hint_data['seeds'];solutions=[];route=None;rejections=[];opening=max(pickup_opening_m,float(cmd[0,fi].sum()))
    # Preserve the current elbow branch before considering a distant taught
    # branch. Both satisfy the same fresh image target, bounds and geometry.
    seeds=[robot_seeds.get('pin_tail_approach',cmd[0,ids]),cmd[0,ids],preq,cand['pickup_q']]+list(np.random.default_rng(4121).uniform(arm.bounds[:,0],arm.bounds[:,1],(32,7)))
    from fr3_gpu_ik import propose_ik
    GPU_pre=propose_ik(ctx,arm,newpre[:3,3],newpre[:3,:3],art.get_dof_positions()[0,ids].copy(),'pin_pregrasp')
    next_goal=np.asarray(robot_seeds.get('pin_direct_preinsert',cmd[0,ids]));joint_speed=np.array([2.62,2.62,2.62,2.62,5.26,4.18,5.26]);seen=[]
    canonical=[];canonical_evidence=None
    if pin_config.get('pin_prefer_canonical_pickup_branch'):
        reference=np.asarray(robot_seeds.get('pin_tail_approach',cand['pickup_q']),float)
        canonical_evidence=dict(requested=True,fixed_supply_only=True,same_fresh_RGBD_target=newpre.tolist(),candidates=[],fallback_to_original_groups=True,uses_free_body_poses=False)
        # These three seeds already exist in the original CPU group. Only
        # their priority changes; every candidate keeps the original gates.
        for source,seed in [('robot_seeds.pin_tail_approach',robot_seeds.get('pin_tail_approach')),('catalogue.pickup_q',cand['pickup_q']),('catalogue.pre_q',preq)]:
            entry=dict(source=source,requested_seed=None if seed is None else np.asarray(seed).tolist(),admitted=False)
            canonical_evidence['candidates'].append(entry)
            if seed is None:entry['reason']='existing_seed_unavailable';continue
            q=arm.solve(newpre[:3,3],newpre[:3,:3],seed)
            entry['solved_q']=None if q is None else q.tolist()
            if q is None:entry['reason']='original_IK_no_solution';continue
            if any(np.linalg.norm(q-old)<=.05 for old in canonical):entry['reason']='duplicate_existing_candidate';continue
            canonical.append(q)
        canonical.sort(key=lambda q:float(np.linalg.norm(q-reference)))
        report['pin_canonical_pickup_branch']=canonical_evidence
    groups=([canonical]if canonical else[])+([GPU_pre,None]if GPU_pre else[None])
    for proposed in groups:
        if proposed is None:
            proposed=[]
            for seed in seeds:
                if seed is None:continue
                q=arm.solve(newpre[:3,3],newpre[:3,:3],seed)
                if q is not None and all(np.linalg.norm(q-old)>.05 for old in proposed+seen):proposed.append(q)
            proposed.sort(key=lambda q:float(np.max(abs(q-cmd[0,ids])/joint_speed)+np.max(abs(q-next_goal)/joint_speed)) if ctx.get('goal_aware_pickup') else np.linalg.norm(q-cmd[0,ids]))
        solutions.extend(proposed);seen.extend(proposed)
        for q in proposed:
            canonical_entry=None
            if canonical_evidence is not None and proposed is canonical:
                canonical_entry=next(e for e in canonical_evidence['candidates']if e.get('solved_q') is not None and np.array_equal(e['solved_q'],q))
            clear,why=checker.clear(q,opening,paths['workpiece_pin'])
            if not clear:
                if canonical_entry is not None:canonical_entry.update(reason='original_endpoint_blocked',evidence=why)
                rejections.append(why);continue
            route=checker.route(cmd[0,ids],q,opening,paths['workpiece_pin'],seconds=.75)
            if canonical_entry is not None:canonical_entry.update(admitted=route is not None,reason='original_full_route_admitted'if route is not None else'original_route_unavailable')
            if route is not None:
                if canonical_evidence is not None:canonical_evidence.update(selected_source=canonical_entry['source']if canonical_entry is not None else'original_GPU_or_CPU_fallback',selected_pre_q=np.asarray(q).tolist(),fallback_to_original_groups=canonical_entry is None)
                break
        if route is not None:break
    if route is None:raise RuntimeError('No pin pregrasp route across IK branches '+str(rejections[:3]))
    if canonical_evidence is not None:print('PIN_CANONICAL_PICKUP_BRANCH',canonical_evidence,flush=True)
    report['pregrasp_IK_solutions']=len(solutions);motion(route,1.1,'pin_tail_approach')
    if canonical_evidence is not None:canonical_evidence.update(actual_pre_q=np.asarray(art.get_dof_positions()[0,ids]).tolist(),command_pre_q=cmd[0,ids].tolist())
    H=arm.fk(art.get_dof_positions()[0,ids]);line(H,pickupH,.6,'pin_tail_descend')
    planned_G=G.copy()
    if pin_axis_geometry and pin_config.get('pin_preclose_visual_alignment'):
        # Open-hand descent can contact/nudge a loose pin before closure.
        # Observe its actual holder pose instead of treating a nonempty
        # encoder gap as proof of the catalog-relative contact geometry.
        for preclose_attempt in range(4):
            value=observe('pin_holder_preclose_RGBD',planned_G,require_circle=True)
            measured=measured_grasp(value,planned_G)
            current_part=np.asarray(value['H'])@measured
            C=arm.fk(cmd[0,ids]);A=arm.fk(art.get_dof_positions()[0,ids])
            desired=current_part@np.linalg.inv(planned_G)
            translation=float(np.linalg.norm(desired[:3,3]-A[:3,3]));angle=float(np.linalg.norm(log(desired[:3,:3]@A[:3,:3].T)))
            evidence=dict(attempt=preclose_attempt,translation_m=translation,axis_rotation_rad=angle,required_translation_m=.0003,required_rotation_rad=.01,source='Fresh directly measured pin cap/axis + actual encoder FK before closure',uses_free_body_poses=False)
            report.setdefault('pin_holder_preclose_alignment',[]).append(evidence);print('PIN_HOLDER_PRECLOSE',evidence,flush=True)
            if translation<=.0003 and angle<=.01:break
            if preclose_attempt==3:raise RuntimeError('Fresh pin holder preclose alignment not confirmed within bounded3 corrections')
            bias_translation=float(np.linalg.norm(C[:3,3]-A[:3,3]));bias_angle=float(np.linalg.norm(log(C[:3,:3]@A[:3,:3].T)))
            if bias_translation>.002 or bias_angle>.01:raise RuntimeError('Pin holder preclose tracking bias exceeds2mm/.01rad')
            target=C@np.linalg.inv(A)@desired
            target[:3,3]=C[:3,3]+preclose_gain*(target[:3,3]-C[:3,3])
            target[:3,:3]=exp(preclose_gain*log(target[:3,:3]@C[:3,:3].T))@C[:3,:3]
            delta_p=float(np.linalg.norm(target[:3,3]-C[:3,3]));delta_r=float(np.linalg.norm(log(target[:3,:3]@C[:3,:3].T)))
            if delta_p>.002 or delta_r>.05:raise RuntimeError('Pin holder preclose correction exceeds2mm/.05rad per-step envelope')
            evidence.update(issued_translation_m=delta_p,issued_rotation_rad=delta_r,encoder_bias_translation_m=bias_translation,encoder_bias_rotation_rad=bias_angle,gain=preclose_gain)
            line(A,target,.3,'pin_holder_regrasp_alignment');wait(.05,'pin_holder_preclose_settle')
    def release_inside_existing_holder(label):
        Hactual=arm.fk(art.get_dof_positions()[0,ids]);current_opening=float(art.get_dof_positions()[0,fi].sum())
        if not all(checker.clear(art.get_dof_positions()[0,ids],float(w),paths['workpiece_pin'])[0]for w in np.linspace(current_opening,.014,13)):raise RuntimeError('Pin holder release aperture sweep blocked; fingers remain closed')
        if ctx.get('set_grip_hold'):ctx['set_grip_hold'](False)
        fingers(.014,label);wait(.1,label+'_settle')
    for grasp_attempt in range(2 if pin_axis_geometry else 1):
        stiff[0,fi]=0. if ctx.get('set_grip_hold') else 30000.;damping[0,fi]=[4000.,.001] if ctx.get('set_grip_hold') else 300.;art.set_dof_stiffnesses(stiff,idx);art.set_dof_dampings(damping,idx);fingers(.0095,'pin_tail_close')
        if ctx.get('set_grip_hold'):ctx['set_grip_hold'](True)
        wait(.15,'pin_tail_grasp_hold')
        if not pin_axis_geometry:break
        samples=[];plateau=False
        for _ in range(80):
            samples.append(art.get_dof_positions()[0,fi].copy());samples=samples[-5:]
            if len(samples)==5:
                s=np.asarray(samples);plateau=bool(np.max(np.ptp(s,axis=0))<=.00015 and np.ptp(s.sum(axis=1))<=.00015)
                if plateau:break
            wait(.025,'pin_native_close_settle')
        if pin_sensor_hold:
            gap=float(art.get_dof_positions()[0,fi].sum());close_ok=bool(plateau and .0097<gap<.0135);evidence=dict(attempt=grasp_attempt,encoder_position_plateau=plateau,actual_encoder_aperture_m=gap,required_aperture_band_m=[.0097,.0135],controlled_lift_allowed=close_ok,transport_allowed=False,direct_pose_confirmation='Deferred until pin is lifted clear of hand/holder occlusion',sensor_basis='Simulated SDK-like aperture/encoder plateau after controlled close; field adapter uses gripper is_grasped; no force sensor',uses_free_body_poses=False)
            report.setdefault('pin_SDK_postclose',[]).append(evidence);print('PIN_SDK_POSTCLOSE_LIFT_ONLY',evidence,flush=True)
            if close_ok:G=planned_G.copy();break
            if grasp_attempt==1:raise RuntimeError('SDK-like pin grip did not reach nonempty stationary aperture after bounded reclose')
            release_inside_existing_holder('pin_unconfirmed_aperture_holder_release');G=planned_G.copy();continue
        else:
            vclose=observe('pin_postclose_RGBD',G,require_circle=True);settled_G=measured_grasp(vclose,planned_G);gap=float(art.get_dof_positions()[0,fi].sum());support=native_support(ctx,settled_G,gap);evidence=dict(attempt=grasp_attempt,actual_opening_m=gap,encoder_position_plateau=plateau,support=support);report.setdefault('native_pin_postclose',[]).append(evidence);print('PIN_NATIVE_POSTCLOSE',dict(attempt=grasp_attempt,plateau=plateau,admitted=support['admitted'],gap_m=gap,flat_span_m=support['minimum_flat_pin_axis_span_m']),flush=True)
            if plateau and support['admitted']:G=settled_G;break
            current_part=np.asarray(vclose['H'])@settled_G;holder_error=float(np.linalg.norm(current_part[:3,3]-P))
            if holder_error>.002:raise RuntimeError('Weak native pin contact outside original holder2mm envelope; fingers remain closed')
            release_inside_existing_holder('pin_rejected_contact_holder_release')
            if grasp_attempt==1:raise RuntimeError('Native pin postclose certificate failed after bounded holder regrasp')
            reopened=observe('pin_holder_regrasp_RGBD',settled_G,require_circle=True);reopened_G=measured_grasp(reopened,planned_G);part_H=np.asarray(reopened['H'])@reopened_G;target=part_H@np.linalg.inv(planned_G);actual_H=arm.fk(art.get_dof_positions()[0,ids])
            if np.linalg.norm(target[:3,3]-actual_H[:3,3])>.002 or np.linalg.norm(log(target[:3,:3]@actual_H[:3,:3].T))>.05:raise RuntimeError('Pin holder regrasp exceeds2mm/.05rad bounded envelope')
            line(actual_H,target,.3,'pin_holder_regrasp_alignment');G=planned_G.copy()
    H=arm.fk(art.get_dof_positions()[0,ids]);lift=H.copy();lift[:3,3]+=[0,0,.07];line(H,lift,.6,'pin_tail_lift');wait(.15,'pin_lift_settle');v=observe('pin_lift_RGBD',G,require_circle=pin_axis_geometry);G=measured_grasp(v,G);report['measured_grasp']=G.tolist()
    if pin_sensor_hold:
        first_direct_lift_G=G.copy();wait(.05,'pin_lift_independent_confirmation');v=observe('pin_lift_confirm_RGBD',G,require_circle=True);G=measured_grasp(v,G)
    if pin_axis_geometry:
        gap=float(art.get_dof_positions()[0,fi].sum());support=native_support(ctx,G,gap);report['native_pin_postlift']=support;print('PIN_NATIVE_POSTLIFT',dict(admitted=support['admitted'],gap_m=gap,flat_span_m=support['minimum_flat_pin_axis_span_m']),flush=True);postlift_confirmed=support['admitted']
        if pin_sensor_hold:
            sensor_evidence=_pin_rgbd_sdk_hold_evidence(v['result']['pose'],first_direct_lift_G,G,gap);report['pin_RGBD_SDK_postlift']=sensor_evidence;postlift_confirmed=sensor_evidence['confirmed'];print('PIN_RGBD_SDK_POSTLIFT',sensor_evidence,flush=True)
        if not postlift_confirmed:raise RuntimeError('Fresh RGB-D/encoder pin postlift retention confirmation failed before carry; fingers remain closed')
    if pin_config.get('pin_qualified_postlift_preload',False):
        if not pin_sensor_hold or not pin_axis_geometry or not ctx.get('set_grip_hold'):raise ValueError('Qualified PIN preload requires fresh physical-axis RGB-D/SDK hold confirmation and native hold callback')
        target_preload=float(pin_config.get('native_loaded_preload_after_contact_m_s',.015));ramp_s=float(pin_config.get('native_loaded_preload_ramp_s',.3))
        if not 0<target_preload<=.015 or not 0<=ramp_s<=2.:raise ValueError('Invalid qualified PIN preload ramp')
        preload_reference_G=G.copy()
        # Reuse the same validated native postcontact ramp as ring pickups.
        # This changes velocity PRELOAD, not the30N/pad effort cap or model.
        ctx['set_grip_hold'](True,preload_m_s=target_preload)
        wait(ramp_s+.05,'pin_confirmed_postlift_preload_ramp')
        # Same existing calibrated lift-confirm view order and fresh image;
        # no cached pose and no transport until the original hold gate passes.
        v=observe('pin_lift_confirm_RGBD',G,require_circle=True);confirmed_G=measured_grasp(v,G);gap=float(art.get_dof_positions()[0,fi].sum())
        force_confirmation=_pin_rgbd_sdk_hold_evidence(v['result']['pose'],preload_reference_G,confirmed_G,gap)
        report['pin_qualified_postlift_preload']=dict(target_velocity_preload_m_s=target_preload,ramp_s=ramp_s,effort_cap_per_pad_n=float(pin_config.get('native_grip_pad_force_n',30.)),effort_cap_unchanged=True,actual_contact_force_measured=False,confirmation=force_confirmation,uses_free_body_poses=False)
        print('PIN_QUALIFIED_POSTLIFT_PRELOAD',report['pin_qualified_postlift_preload'],flush=True)
        if not force_confirmation['confirmed']:raise RuntimeError('Fresh RGB-D/SDK PIN hold changed after qualified native preload ramp; transport blocked')
        G=confirmed_G
    pin_hold_reference_G=G.copy()
    if ctx.get('before_transport'):report['left_work_zone_clearance']=ctx['before_transport'](payload_grasp())
    R=np.array(cand['goal_part_R']);safe=frame([-.10,-.25,1.35],R)@np.linalg.inv(G);pre=frame(goal-[0,0,.045],R)@np.linalg.inv(G);end=frame(goal,R)@np.linalg.inv(G)
    robot_seeds=json.loads((HERE/'bin-picking/direct-pin-robot-seeds.json').read_text())['seeds']
    transports=[(safe,'pin_tail_orient_free'),(pre,'pin_tail_carry_to_preinsert')]
    air_tangent_flip=bool(pin_config.get('pin_tangent_flip_air_waypoint',False))
    if air_tangent_flip:
        if not pin_config.get('pin_axial_yaw_free'):raise ValueError('Tangent-air PIN flip requires original axial-yaw-free class permission')
        # Keep the original fixture yaw0. Use the existing FREE-space waypoint
        # for the tangent-direction flip, then rotate axial yaw before approach.
        # These are TWO original fully guarded motions, no unverified shortcut.
        tangent_R=R@exp(np.array([0.,0.,np.pi]))
        safe=frame([-.10,-.25,1.35],tangent_R)@np.linalg.inv(G)
        transports=[(safe,'pin_tail_orient_free'),(pre,'pin_tail_carry_to_preinsert')]
        report['pin_tangent_air_flip']=dict(enabled=True,free_part_midpoint_world_m=[-.10,-.25,1.35],free_part_R=tangent_R.tolist(),fixture_part_R_unchanged=R.tolist(),original_two_fully_guarded_segments=True,direct_shortcut_disabled=True,force_speed_caps_unchanged=True,uses_free_body_poses=False,physical_retention_qualified=False)
    if ctx.get('optimized_motion_timing') and not air_tangent_flip:
        checker.attach_payload(paths['workpiece_pin'],frame([0,0,.1034],np.eye(3))@payload_grasp());shortcut=None
        try:
            # The shaft/cap are axially symmetric. Keep the calibrated physical
            # midpoint and signed axis fixed, but choose a reachable hand yaw.
            # No axial yaw is inferred from RGB-D or sent as object truth.
            nominal_R=R.copy()
            yaw_options=[0.,90.,-90.,180.] if pin_config.get('pin_axial_yaw_free') else [0.]
            if pin_config.get('pin_tangent_flip_yaw_first',False) and pin_config.get('pin_axial_yaw_free'):
                # Reorder the SAME originally allowed axial yaw candidates.
                # Endpoint flip may rotate about a pad tangent instead of the
                # opposed contact-normal line. Actual route/retention unproven.
                yaw_options=[180.,90.,-90.,0.]
            report['pin_axial_yaw_candidate_order_deg']=list(yaw_options)
            report['pin_tangent_flip_yaw_first_requested']=bool(pin_config.get('pin_tangent_flip_yaw_first',False))
            for yaw_deg in yaw_options:
                candidate_R=nominal_R@exp(np.array([0.,0.,np.deg2rad(yaw_deg)]))
                candidate_pre=frame(goal-[0,0,.045],candidate_R)@np.linalg.inv(G)
                for q in endpoint_candidates(candidate_pre,[robot_seeds.get('pin_direct_preinsert',robot_seeds['pin_tail_carry_to_preinsert']),robot_seeds['pin_tail_carry_to_preinsert'],cmd[0,ids]],'pin_preinsert_carry'):
                    if checker.path([cmd[0,ids],q],.0105,paths['workpiece_pin'])[0]:shortcut=[cmd[0,ids].copy(),q];report['direct_joint_edge_used']=True;break
                    if pin_config.get('pin_gpu_route_before_hint',False):
                        shortcut=_pin_gpu_cubic_route_before_hint(checker,cmd[0,ids].copy(),q,.0105,paths['workpiece_pin'])
                        if shortcut is not None:
                            report['pin_GPU_cubic_route_before_hint_used']=True;break
                    from grasp_transfer_feasibility import robot_route_hint,hint_path_clear
                    warm=robot_route_hint(ctx,'pin_direct_carry_to_preinsert',cmd[0,ids],q)
                    if warm is not None and getattr(warm,'verified_cubic',False) and hint_path_clear(checker,warm,.0105,paths['workpiece_pin']):shortcut=warm;report['warm_spline_robot_hint_used']=True;break
                    hint=robot_hint_data.get('routes',{}).get('pin_direct_carry_to_preinsert')
                    if hint:
                        candidate=np.asarray(hint).copy();candidate[0]=cmd[0,ids];candidate[-1]=q
                        if checker.path(candidate,.0105,paths['workpiece_pin'])[0]:shortcut=candidate;report['validated_robot_route_hint_used']=True
                    if shortcut is None:shortcut=checker.route(cmd[0,ids],q,.0105,paths['workpiece_pin'],seconds=1.)
                    if shortcut is not None:break
                if shortcut is not None:
                    R=candidate_R;pre=candidate_pre;end=frame(goal,R)@np.linalg.inv(G)
                    safe=frame([-.10,-.25,1.35],R)@np.linalg.inv(G)
                    transports=[(safe,'pin_tail_orient_free'),(pre,'pin_tail_carry_to_preinsert')]
                    report['selected_pin_axial_yaw_deg']=float(yaw_deg)
                    report['pin_axis_goal_unchanged_by_yaw']=goal.tolist()
                    break
        finally:checker.detach_payload()
        if shortcut is not None:
            from grasp_transfer_feasibility import shorten_checked_route
            checker.attach_payload(paths['workpiece_pin'],frame([0,0,.1034],np.eye(3))@payload_grasp())
            try:
                if not getattr(shortcut,'verified_cubic',False):shortcut=shorten_checked_route(checker,shortcut,.0105,paths['workpiece_pin'])
            finally:checker.detach_payload()
            report['pin_shortcut_nodes']=len(shortcut);motion(shortcut,.05,'pin_tail_direct_carry_to_preinsert');transports=[];report['direct_preinsert_shortcut']=True
    for target,label in transports:
        checker.attach_payload(paths['workpiece_pin'],frame([0,0,.1034],np.eye(3))@payload_grasp());route=None
        for q in endpoint_candidates(target,[robot_seeds[label],cand['insert_waypoints'][0] if label=='pin_tail_carry_to_preinsert' else cmd[0,ids],cmd[0,ids]],label):
            clear,why=checker.clear(q,.0105,paths['workpiece_pin']);print('PIN_CARRY_GOAL_CLEAR',label,(clear,why),flush=True)
            if not clear:continue
            route=checker.route(cmd[0,ids],q,.0105,paths['workpiece_pin'],seconds=2)
            if route is not None:break
        checker.detach_payload()
        if route is None:raise RuntimeError('No collision-free carry '+label)
        motion(route,.9,label)
    adaptive_pin_gain=bool(pin_config.get('pin_visual_adaptive_gain',False))
    alignment_budget=16 if adaptive_pin_gain and visual_preinsert else 8
    independent_component_feedback=bool(pin_config.get('pin_visual_independent_component_feedback',False))
    bounded_component_recovery=bool(pin_config.get('pin_visual_bounded_component_recovery',False))
    if bounded_component_recovery and not independent_component_feedback:raise ValueError('Bounded PIN recovery requires independent part-space feedback')
    correction_gain=float(pin_config.get('pin_visual_servo_gain',.65 if bounded_component_recovery else .5 if independent_component_feedback else 1.))
    recovery_ceiling=min(.65,correction_gain);radial_improvement_streak=0
    previous_alignment_score=None
    previous_radial_error=None
    terminal_replans=0;axial_inserted=False;entry_stage=False;far_preinsert_H=None;far_remaining_depth=None
    for alignment_index in range(alignment_budget if visual_preinsert else 1):
        if visual_preinsert and pin_config.get('pin_precision_encoder_settle'):await_precision_encoder_window('pin_precision_encoder_settle')
        v=observe('pin_entry_RGBD'if entry_stage else'pin_preinsert_RGBD',G,require_circle=visual_preinsert);pose=v['result']['pose'];Pobs=np.asarray(pose['center_m']);remaining_depth=float(goal[2]-Pobs[2])
        if pin_sensor_hold:
            current_G=measured_grasp(v,G);gap=float(art.get_dof_positions()[0,fi].sum());sensor_evidence=_pin_rgbd_sdk_hold_evidence(pose,pin_hold_reference_G,current_G,gap);sensor_evidence['alignment_index']=alignment_index;report.setdefault('pin_RGBD_SDK_postcarry',[]).append(sensor_evidence);print('PIN_RGBD_SDK_POSTCARRY',sensor_evidence,flush=True)
            if not sensor_evidence['confirmed']:raise RuntimeError('Fresh RGB-D/SDK pin postcarry retention changed beyond2mm/.05rad; correction blocked')
        if not visual_preinsert:break
        radial=float(np.linalg.norm(Pobs[:2]-goal[:2]));axis_error=float(np.arccos(np.clip(np.asarray(pose['axis'])@R[:,2],-1,1)));shaft_axis_max=float(max(np.linalg.norm((Pobs+z*np.asarray(pose['axis'])-goal-z*R[:,2])[:2])for z in [-.0035,.0135]))
        evidence=dict(iteration=alignment_index,radial_error_m=radial,axis_error_rad=axis_error,radial_limit_m=.000075,axis_limit_rad=.003,maximum_shaft_axis_radial_error_m=shaft_axis_max,maximum_shaft_axis_radial_limit_m=.00015,source='Fresh RGB-D end circle/plane plus encoder FK',uses_free_body_poses=False)
        if pin_config.get('pin_engine_readonly_diagnostics',False):evidence['engine_diagnostics_only']=_pin_engine_readonly_diagnostics(stage,sim,arm)
        report.setdefault('preinsert_visual_alignment',[]).append(evidence);print('PIN_VISUAL_PREALIGN',evidence,flush=True)
        if entry_stage:
            if not .014<remaining_depth<.022:raise RuntimeError('Pin entry RGB-D must remain14–22mm below the calibrated jig target')
        elif not .035<remaining_depth<.055:raise RuntimeError('Pin visual prealignment must remain35–55mm below the calibrated jig target')
        evidence['entry_stage']=entry_stage;evidence['original_shared_alignment_budget']=alignment_budget
        measured=np.linalg.inv(v['H'])@frame(Pobs,axis_frame(pose['axis']))
        old_axis=G[:3,2];new_axis=measured[:3,2];cross=np.cross(old_axis,new_axis);length=np.linalg.norm(cross);measured[:3,:3]=exp(cross/max(length,1e-12)*np.arctan2(length,old_axis@new_axis))@G[:3,:3];G=measured
        if radial<=.000075 and axis_error<=.003 and shaft_axis_max<=.00015:
            if not pin_config.get('pin_terminal_stroke_preflight'):break
            from pin_current_terminal import assess as assess_current_terminal,plan_rebranch
            try:
                entry_options=dict(entry_stage=True,withdraw_H=far_preinsert_H,requested_s=.8*remaining_depth/far_remaining_depth)if entry_stage else dict(partial_entry=True)if entry_feedback_enabled else{}
                admitted,terminal=assess_current_terminal(ctx,payload_grasp(),remaining_depth,**entry_options)
                report.setdefault('fresh_image_terminal_checks',[]).append(terminal)
                if not admitted:raise PinCartesianPreflightBlocked('Last qualified RGB-D pin terminal obstructed '+str(terminal))
                current_pre=arm.fk(cmd[0,ids])
                if entry_feedback_enabled and not entry_stage:
                    if alignment_index==alignment_budget-1:raise RuntimeError('PIN shared image budget exhausted before near-entry feedback; partial stroke blocked')
                    far_preinsert_H=current_pre.copy();far_remaining_depth=remaining_depth
                    entry_target=current_pre.copy();entry_distance=remaining_depth-.018;entry_target[2,3]+=entry_distance
                    line(current_pre,entry_target,.8*entry_distance/remaining_depth,'pin_axial_move_to_entry')
                    entry_stage=True
                    report['pin_entry_feedback_transition']=dict(alignment_index=alignment_index,remaining_original_images=alignment_budget-alignment_index-1,far_preinsert_Hand=far_preinsert_H.tolist(),far_remaining_depth_m=far_remaining_depth,entry_remaining_depth_m=.018,shaft_tip_below_original_fixture_bottom_m=.00175,original_full_far_terminal_admitted=True,fresh_entry_RGBD_required=True,no_budget_or_feedback_history_reset=True,uses_free_body_poses=False)
                    continue
                pre=far_preinsert_H.copy()if entry_stage else current_pre
                end=current_pre.copy();end[2,3]+=remaining_depth
                line(current_pre,end,.8*remaining_depth/far_remaining_depth if entry_stage else .8,'pin_single_axial_insert')
            except PinCartesianPreflightBlocked as rejected:
                # Only known pre-motion geometry rejection can rebranch.
                # Original precision iterations are shared, never reset.
                if terminal_replans>=2 or alignment_index==alignment_budget-1:raise
                recovery=plan_rebranch(ctx,payload_grasp(),remaining_depth,**(dict(entry_stage=True,withdraw_H=far_preinsert_H,requested_s=.8*remaining_depth/far_remaining_depth)if entry_stage else{}))
                if not np.array_equal(cmd,recovery['source_command_all']) or not np.array_equal(art.get_dof_positions()[0],recovery['source_actual_all']):raise RuntimeError('PIN terminal rebranch source changed after full field proof')
                phase='pin_current_G_terminal_rebranch'
                for command in recovery['commands'][1:]:cmd[0,ids]=command;step()
                terminal_replans+=1
                if pin_config.get('pin_reset_feedback_after_rebranch',False):
                    # A new elbow branch has a different command/SDK pose bias.
                    # Old-branch errors must not damp the first NEW-branch image.
                    # This is after complete original field proof AND execution;
                    # shared image/rebranch budgets and every final gate stay.
                    previous_feedback_state=dict(gain=float(correction_gain),previous_radial_error_m=previous_radial_error,previous_normalized_error=previous_alignment_score,radial_improvement_streak=int(radial_improvement_streak))
                    correction_gain=min(.65,float(recovery_ceiling))
                    previous_radial_error=None;previous_alignment_score=None;radial_improvement_streak=0
                    report.setdefault('pin_feedback_resets_after_rebranch',[]).append(dict(reason='Fully admitted and executed new IK branch changes encoder tracking bias; old branch error history is inapplicable',rebranch=terminal_replans,alignment_index=alignment_index,previous_state=previous_feedback_state,new_state=dict(gain=correction_gain,previous_radial_error_m=None,previous_normalized_error=None,radial_improvement_streak=0),original_alignment_budget=alignment_budget,remaining_original_images=alignment_budget-alignment_index-1,extra_images_or_motion=0,all_original_precision_and_motor_gates_unchanged=True,uses_free_body_poses=False))
                entry=dict(event=phase,wall_s=time.perf_counter()-started,physical_duration_s=(len(recovery['commands'])-1)*DT)
                report['events'].append(entry)
                report.setdefault('fresh_image_terminal_replans',[]).append(dict(reason=str(rejected),replan=terminal_replans,alignment_index=alignment_index,candidate=recovery['candidate'],timing=recovery['timing'],terminal=recovery['terminal'],original_all_limits_and_collision_checks=True,fresh_precision_RGBD_required_before_insertion=True,extra_alignment_iterations=0,uses_free_body_poses=False))
                print('PIN_CURRENT_G_TERMINAL_REBRANCH',entry,flush=True)
                wait(.08,'pin_terminal_rebranch_settle')
                continue
            axial_inserted=True
            break
        if alignment_index==alignment_budget-1:raise RuntimeError('Pin RGB-D precision center/axis alignment did not converge; insertion blocked')
        alignment_score=max(radial/.000075,axis_error/.003,shaft_axis_max/.00015)
        if independent_component_feedback:
            # Translation adaptation never damps the independent axis servo.
            if bounded_component_recovery and adaptive_pin_gain:
                correction_gain,radial_improvement_streak=_pin_bounded_xy_gain(correction_gain,radial,previous_radial_error,radial_improvement_streak,recovery_ceiling)
            elif adaptive_pin_gain and previous_radial_error is not None and radial>.000075 and radial>=.95*previous_radial_error:
                correction_gain=max(.3,correction_gain*.65)
        elif adaptive_pin_gain and previous_alignment_score is not None and alignment_score>=.95*previous_alignment_score:
            correction_gain=max(.3,correction_gain*.65)
        previous_radial_error=radial
        evidence.update(adaptive_gain_enabled=adaptive_pin_gain,alignment_budget=alignment_budget,normalized_error=alignment_score,previous_normalized_error=previous_alignment_score,selected_feedback_gain=correction_gain)
        previous_alignment_score=alignment_score
        actual_H=arm.fk(art.get_dof_positions()[0,ids]);command_H=arm.fk(cmd[0,ids]);target,bias=_pin_visual_preinsert_target(command_H,actual_H,G,goal,R[:,2],float(Pobs[2]),gain=correction_gain,translation_deadband_m=.000075 if pin_config.get('pin_precision_component_deadband') else 0.,axis_deadband_rad=(.001 if bounded_component_recovery else .003) if pin_config.get('pin_precision_component_deadband') else 0.,axis_gain=(.65 if bounded_component_recovery else 1.) if independent_component_feedback else None,**(dict(preinsert_depth_m=.018)if entry_stage else{}));evidence.update(bias);evidence.update(bounded_component_recovery=bounded_component_recovery,radial_improvement_streak=radial_improvement_streak,XY_recovery_ceiling=recovery_ceiling)
        saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative)
        checker.attach_payload(paths['workpiece_pin'],frame([0,0,.1034],np.eye(3))@payload_grasp())
        try:line(command_H,target,.35,'pin_preinsert_visual_correction')
        finally:checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved
        wait(.08,'pin_preinsert_visual_hold')
    report['preinsert_observed_center_m']=Pobs.tolist();report['RGBD_measured_remaining_axial_depth_m']=remaining_depth
    if visual_preinsert:report['measured_grasp_after_preinsert_alignment']=G.tolist()
    depth_range_ok=.014<remaining_depth<.022 if entry_stage else .035<remaining_depth<.055
    if np.linalg.norm(Pobs[:2]-goal[:2])>=.0015 or not depth_range_ok:raise RuntimeError('Preinsert RGB-D alignment/depth outside explicit insertion state range')
    if entry_feedback_enabled and not(entry_stage and axial_inserted):raise RuntimeError('Two-stage PIN entry did not qualify final insertion within original image budget')
    if not axial_inserted:
        pre=arm.fk(cmd[0,ids]);end=pre.copy();end[2,3]+=remaining_depth;line(pre,end,.8,'pin_single_axial_insert')
    wait(.12,'pin_full_depth_hold');v=observe('pin_full_depth_RGBD',G,require_circle=qualified_full_depth);pose=v['result']['pose'];error=float(np.linalg.norm(np.array(pose['center_m'])-goal));ae=float(np.arccos(np.clip(np.array(pose['axis'])[2],-1,1)));report['RGBD_full_depth_check']=dict(position_error_m=error,axis_error_rad=ae)
    for correction_index in range(2):
        remaining=float(goal[2]-pose['center_m'][2]);radial=float(np.linalg.norm(np.array(pose['center_m'])[:2]-goal[:2]))
        if not (.0005<remaining<.006 and radial<.0015 and ae<.05):break
        H=arm.fk(cmd[0,ids]);end=H.copy();end[2,3]+=remaining;line(H,end,.25,'pin_axial_depth_correction');wait(.08,'pin_depth_check_hold');v=observe('pin_full_depth_RGBD',G,require_circle=qualified_full_depth);pose=v['result']['pose'];error=float(np.linalg.norm(np.array(pose['center_m'])-goal));ae=float(np.arccos(np.clip(np.array(pose['axis'])[2],-1,1)));report['RGBD_full_depth_check']=dict(position_error_m=error,axis_error_rad=ae);report['axial_visual_corrections_with_grip_closed']=correction_index+1
    if error>=.0015 or ae>=.05:raise RuntimeError('Full depth not confirmed by RGB-D; fingers remain closed')
    for prim in stage.Traverse():
        if ctx.get('close_pin_clamp',True) and prim.IsA(UsdPhysics.PrismaticJoint)and str(prim.GetPath()).startswith('/World/Clamps/pin/'):
            joint=UsdPhysics.PrismaticJoint(prim);UsdPhysics.DriveAPI(prim,'linear').GetTargetPositionAttr().Set(joint.GetLowerLimitAttr().Get())
    sim.flush_changes();wait(.12,'pin_clamp_hold');
    if ctx.get('set_grip_hold'):ctx['set_grip_hold'](False)
    # Return to the same preinsert pose that was checked and just traversed.
    # The former extra 25 mm downward stroke drove link 4 into the machine.
    fingers(.018,'pin_release_after_full_depth');withdraw=pre.copy();line(end,withdraw,.65,'pin_axial_withdraw');wait(0. if ctx.get('defer_retention_to_recipe') else .5,'pin_retention');report['retention_deferred_to_recipe']=bool(ctx.get('defer_retention_to_recipe'));tr=truth['pin'].get_transforms()[0];ae=float(np.arccos(np.clip(1-2*(tr[3]**2+tr[4]**2),-1,1)));err=float(np.linalg.norm(tr[:3]-body_goal));report.update(state='pin_single_stroke_contact_pass'if err<.0015 and ae<.05 else'pin_retention_failed',pin_single_stroke_pass=bool(err<.0015 and ae<.05),final_position_error_m=err,final_axis_error_rad=ae,pin_trial_wall_s=time.perf_counter()-started)
    return report
