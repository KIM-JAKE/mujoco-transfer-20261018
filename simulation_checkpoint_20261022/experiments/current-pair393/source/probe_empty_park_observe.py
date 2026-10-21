"""Physically park an initially EMPTY right arm, then capture one fixed view.

All loose stock remains physical. Only encoder-controlled robot motion is
commanded. No free-body/actor transform is read by this controller. This probe
returns sensor evidence, not a completed pick or hardware-ready certificate.
"""
from contextlib import contextmanager
from pathlib import Path
import time,json,numpy as np
from isaac_kinematics import frame
from fr3_path_timing import retime,retime_safe_polyline
from aperture_path_check import path_openings
from predictive_pair_clearance import proxy,transform_proxy,shape_overlap


def execute(c):
    arm,art,cmd,ids,fi,checker,vision,dt=[c[k]for k in ['arm','art','cmd','ids','fi','checker','vision','dt']]
    cfg=c['config'];beg=time.perf_counter();diagnostics=[];rejections={}
    if getattr(checker,'same_arm_clearance',None)is None:raise RuntimeError('Empty park requires same-arm nonadjacent CAD clearance checks')
    if cfg.get('empty_park_known_empty')is not True:raise RuntimeError('Physical parking probe requires explicit initial right-Hand-known-empty invariant')
    if 'left'not in c['checkers'] or not c.get('left_hand_known_empty'):raise RuntimeError('Physical parking probe requires current empty LEFT and its full pair checker')
    if checker.payload_path or c['checkers']['left'].payload_path:raise RuntimeError('Physical empty parking cannot detach an existing planned payload')
    actual_q=art.get_dof_positions()[0,ids].astype(float);start_q=cmd[0,ids].astype(float);finger_start=cmd[0,fi].astype(float).copy();actual_width=float(art.get_dof_positions()[0,fi].sum())
    if min(actual_width,float(finger_start.sum()))<.077:raise RuntimeError('Known empty native Hand must already be open before parking')
    opening=max(actual_width,float(finger_start.sum()));openings=np.unique(np.linspace(min(actual_width,float(finger_start.sum())),opening,5));width_margin=abs(actual_width-float(finger_start.sum()))/2
    bounds=np.asarray(c['calibration']['outer_bounds_world_m'],float);floor=np.asarray(c['calibration']['floor_origin_world_m'],float)
    # All ten loose rings are known to be inside the calibrated basket. Bound
    # protrusions as well as its whole unobserved interior; no individual body
    # is localized, ignored or moved to plan the empty escape.
    low=bounds[0]-[.004,.004,.004];high=bounds[1]+[.004,.004,.025];high[2]=max(high[2],floor[2]+.143)
    corners=np.array([[x,y,z]for x in [low[0],high[0]]for y in [low[1],high[1]]for z in [low[2],high[2]]]);stock=proxy(corners)
    own_geometry=list(checker.pair_boxes)
    if not own_geometry:raise RuntimeError('Whole native robot collision geometry unavailable for untouched-stock guard')
    def reject(label,why=None):
        rejections[label]=rejections.get(label,0)+1
        if why is not None and len(diagnostics)<12:diagnostics.append(dict(rejection=label,details=why))
    def stock_clear(curve):
        cache=set();margin=.001+width_margin
        for qi,q in enumerate(np.asarray(curve,float)):
            key=q.tobytes()
            if key in cache:continue
            cache.add(key);transforms=arm.bodies(q,opening)
            for body,source,geometry in own_geometry:
                shape=transform_proxy(geometry,transforms[body]);extent=abs(shape[1])@(shape[2]+margin)
                if np.any(shape[0]+extent<low)or np.any(shape[0]-extent>high):continue
                if shape_overlap(shape,stock,margin=margin):return False,dict(query=source,curve_row=qi,reason='whole robot enters calibrated conservative stock region')
        return True,None
    @contextmanager
    def pair_context():
        other_q=art.get_dof_positions()[0,c['left_ids']].astype(float);other_width=float(art.get_dof_positions()[0,c['left_fi']].sum())
        with checker.other_arm_at(c['checkers']['left'],other_q,other_width):yield
    preshape=cfg.get('empty_park_preshape_m')
    preshape_record=None
    if preshape is not None:
        preshape=float(preshape)
        if not .04<=preshape<=.075:raise ValueError('Empty park preshape must remain40--75mm away from native hardstops')
        # Empty-Hand only: preposition the physical slider away from its80mm
        # hardstop before the SAME fast arm route. Force/speed guards remain.
        gaps=np.linspace(min(preshape,actual_width),max(preshape,actual_width),9)
        saved_opening=opening
        with pair_context():
            for gap in gaps:
                for current in [actual_q,start_q]:
                    if not checker.clear(current,float(gap))[0]:raise RuntimeError('Empty preshape width sweep would hit fixed/pair geometry')
                opening=float(gap)
                clear,why=stock_clear([actual_q,start_q])
                if not clear:raise RuntimeError('Empty preshape width sweep enters calibrated stock region: '+str(why))
        opening=saved_opening
        target_fingers=np.full(2,preshape/2)
        _,blend,preshape_timing=retime([start_q,start_q],.20,dt,finger_start,target_fingers,velocity_fraction=.91,optimize=True)
        if c.get('set_grip_hold'):c['set_grip_hold']('right',False)
        for u in blend[1:]:cmd[0,fi]=finger_start+(target_fingers-finger_start)*u;c['step']('empty_right_preshape_away_from_hardstop')
        history=[];settled=False
        for _ in range(int(np.ceil(1./dt))):
            positions=art.get_dof_positions()[0,fi].astype(float);history.append(positions)
            if len(history)>int(np.ceil(.10/dt)):history.pop(0)
            if len(history)>=int(np.ceil(.10/dt)) and abs(float(positions.sum())-preshape)<.00025 and np.max(np.ptp(history,axis=0))<.00010:
                settled=True;break
            c['step']('empty_right_preshape_encoder_completion')
        if not settled:raise RuntimeError('Known-empty intermediate Hand preshape did not settle within native encoder bounds')
        actual_q=art.get_dof_positions()[0,ids].astype(float);start_q=cmd[0,ids].astype(float);finger_start=cmd[0,fi].astype(float).copy();actual_width=float(art.get_dof_positions()[0,fi].sum());opening=max(actual_width,float(finger_start.sum()));width_margin=abs(actual_width-float(finger_start.sum()))/2;openings=np.unique(np.linspace(min(actual_width,float(finger_start.sum())),opening,5))
        preshape_record=dict(requested_width_m=preshape,actual_width_m=actual_width,encoder_settled=True,arm_trajectory_unchanged=True,force_and_velocity_caps_unchanged=True,**preshape_timing)
        c['planning_diagnostics'].append(dict(event='empty_right_stationary_intermediate_preshape',**preshape_record))
    initial_H=arm.fk(actual_q);reference=[np.asarray(v['right_seed'],float)for v in c.get('handoff_reference',{}).values()]
    rotations=[initial_H[:3,:3]]+[arm.fk(q)[:3,:3]for q in reference]+[np.diag([1.,-1.,-1.])]
    seeds=[actual_q,start_q]+reference+list(np.asarray(c.get('empty_return_waypoints',[])))
    seeds+=list(np.random.default_rng(27103).uniform(arm.bounds[:,0],arm.bounds[:,1],(8,7)))
    targets=cfg.get('empty_park_targets_world_m',[[0.,-.25,1.40],[-.15,-.30,1.35],[.15,-.30,1.40],[.10,-.35,1.45]])
    if len(targets)>8:raise ValueError('At most eight fixed-workspace parking targets')
    chosen=None;budget=float(cfg.get('empty_park_planning_budget_s',35.))
    with pair_context():
        for position in targets:
            position=np.asarray(position,float)
            if position.shape!=(3,)or not np.isfinite(position).all():raise ValueError('Finite fixed workspace parking target required')
            if np.all(position[:2]>=low[:2]-.12)and np.all(position[:2]<=high[:2]+.12):raise ValueError('Parking target must leave the basket XY region with120mm hand margin')
            for rotation in rotations:
                if time.perf_counter()-beg>budget:break
                branches=[]
                for seed in seeds:
                    q=arm.solve(position,rotation,seed)
                    if q is not None and all(np.linalg.norm(q-old)>.10 for old in branches):branches.append(q)
                    if len(branches)>=4:break
                if not branches:reject('parking_IK');continue
                for endpoint in branches:
                    ok,why=checker.clear(endpoint,opening)
                    if not ok:reject('parking_fixed_or_pair',why);continue
                    ok,why=stock_clear([actual_q,endpoint])
                    if not ok:reject('parking_stock_endpoint',why);continue
                    routes=[[start_q,endpoint]]+[[start_q,np.asarray(v),endpoint]for v in c.get('empty_return_waypoints',[])]
                    # Existing live FCL/PhysX route planner remains authoritative.
                    route=None
                    for proposal in routes:
                        if not checker.path(np.vstack([actual_q,proposal]),opening)[0]:continue
                        preview=np.vstack([actual_q]+[a*(1-u)+b*u for a,b in zip(proposal[:-1],proposal[1:])for u in np.linspace(0,1,33)])
                        if stock_clear(preview)[0]:route=proposal;break
                    if route is None:route=checker.route(start_q,endpoint,opening,seconds=.5)
                    if route is None:reject('parking_route');continue
                    route=np.asarray(route,float);route[0]=start_q
                    scale=max(1.,float(cfg.get('stability_time_scale',1.)))
                    try:
                        if len(route)>2:q,h,timing=retime_safe_polyline(route,dt,finger_start,finger_start,scale=scale,velocity_fraction=.91)
                        else:q,h,timing=retime(route,.25,dt,finger_start,finger_start,velocity_fraction=.91,optimize=True)
                    except RuntimeError as error:reject('parking_retime',str(error));continue
                    curve=np.vstack([actual_q,q]);query={}
                    ok,why=path_openings(checker,curve,openings,diagnostics=query)
                    if not ok:reject('parking_exact_fixed_or_pair_curve',why);continue
                    ok,why=stock_clear(curve)
                    if not ok:reject('parking_exact_stock_curve',why);continue
                    chosen=(q,timing,query,position);break
                if chosen is not None:break
            if chosen is not None or time.perf_counter()-beg>budget:break
    if chosen is None:
        result=dict(state='empty_robot_park_no_safe_route',robot_motion_executed=False,rejections=rejections,examples=diagnostics,uses_free_body_poses=False,full_task_pass=False)
        (vision.out/'empty-park-observation.json').write_text(json.dumps(result,indent=2)+'\n');return result
    q,timing,query,position=chosen
    c['planning_diagnostics'].append(dict(event='empty_right_park_exact_curve',target_world_m=position.tolist(),whole_robot_fixed_and_pair_checked=True,same_arm_nonadjacent_CAD_checked=True,same_arm_query_statistics=dict(checker.same_arm_clearance.stats),whole_robot_conservative_stock_volume_checked=True,stock_bounds_world_m=[low.tolist(),high.tolist()],checked_openings_m=openings.tolist(),**timing,aperture_query_diagnostics=query))
    if c.get('set_grip_hold'):c['set_grip_hold']('right',False)
    for row in q[1:]:cmd[0,ids]=row;c['step']('empty_right_park_before_single_camera')
    for _ in range(int(np.ceil(.15/dt))):c['step']('empty_right_park_settle')
    measured_H=arm.fk(art.get_dof_positions()[0,ids]);prior=vision.calibration.get('initial_eye_offset_m');vision.calibration['initial_eye_offset_m']=[0.,0.,.80]
    event=dict(mode='initial',part=None,initial_camera_width_px=768,initial_camera_height_px=768,initial_camera_span_m=.55,single_fixed_vertical_camera=True,empty_robot_parked=True)
    try:future=vision.request(event,measured_H)
    finally:
        if prior is None:vision.calibration.pop('initial_eye_offset_m',None)
        else:vision.calibration['initial_eye_offset_m']=prior
    deadline=time.perf_counter()+10.
    while not future.done():
        vision.poll();c['step']('empty_right_park_single_camera_wait')
        if time.perf_counter()>deadline:raise RuntimeError('Parked fixed-camera RGB-D timeout')
    value=future.result();event['delivered_wall_s']=time.perf_counter()-c['started']
    if value.get('result',{}).get('error'):raise RuntimeError(value['result']['error'])
    packet=value['packet']
    if packet['depth_m'].shape!=(768,768):raise RuntimeError('Initial full-bin camera request overrides are not installed')
    camera=np.asarray(packet['T_cell_camera']);expected=floor+[0.,0.,.025]
    if not np.allclose(camera[:3,2],[0.,0.,-1.],atol=1e-7):raise RuntimeError('Requested fixed basket view is not vertically downward')
    np.savez_compressed(vision.out/'empty-park-fixed-camera-rgbd.npz',**packet)
    result=dict(state='empty_robot_park_then_single_fixed_RGBD_observed',robot_motion_executed=True,physical_robot_motion_s=(len(q)-1)*dt,stock_motion_commanded=False,stock_untouched_requires_external_evaluator=True,logical_initial_right_hand_empty=True,intermediate_preshape=preshape_record,actual_parked_TCP_world=measured_H.tolist(),camera_eye_world_m=camera[:3,3].tolist(),camera_optical_axis_world=camera[:3,2].tolist(),camera_target_from_calibration_m=expected.tolist(),single_camera_views=1,captured_wall_s=value['captured_wall_s'],RGBD_result=value['result'],rejections=rejections,examples=diagnostics,uses_free_body_poses=False,attachments_created=False,whole_robot_fixed_pair_and_stock_volume_checked=True,hardware_caps_unchanged=True,full_task_pass=False,physical_grasp_validated=False)
    (vision.out/'empty-park-observation.json').write_text(json.dumps(result,indent=2)+'\n');print('EMPTY_PARK_SINGLE_CAMERA',json.dumps(result),flush=True);return result
