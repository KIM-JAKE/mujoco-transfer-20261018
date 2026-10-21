"""One image-guided finger sweep to expose a ring; no object state writes.

The open native hand inserts one finger through an observed bore, gently
takes up its clearance, and slides toward the existing basket center. Every
contact is resolved by the same PhysX trial. This is not a grasp or success.
"""
import time,json
from contextlib import nullcontext
import numpy as np
from scipy.interpolate import CubicSpline
from isaac_kinematics import frame,log
from fr3_motion_control import position_velocity_bounds
from fr3_path_timing import retime,retime_safe_polyline


def execute(c):
    arm,art,cmd,ids,fi,dt,vision,checker=[c[k] for k in ['arm','art','cmd','ids','fi','dt','vision','checker']]
    part=c['config']['pick_part'];events=[]
    event=dict(mode='initial',part=None,capture_native_hand_opening_m=float(art.get_dof_positions()[0,fi].sum()));future=vision.request(event,arm.fk(art.get_dof_positions()[0,ids]))
    deadline=time.perf_counter()+3
    while not future.done():
        vision.poll();c['step'](part+'_rearrange_RGBD')
        if time.perf_counter()>deadline:raise RuntimeError('Rearrangement image deadline')
    value=future.result();value['capture_native_hand_opening_m']=event['capture_native_hand_opening_m'];event['delivered_wall_s']=time.perf_counter()-c['started'];measurements=[]
    if not value['result'].get('error'):measurements.append(value)
    previous=vision.calibration.get('initial_eye_offset_m',[.04,-.04,.65])
    extra_offsets=c['config'].get('initial_pickup_extra_eye_offsets_m',[])
    if len(extra_offsets)>4:raise ValueError('At most four extra calibrated-bin camera views are supported')
    for offset in extra_offsets:
        vector=np.asarray(offset,float)
        if vector.shape!=(3,) or not np.isfinite(vector).all() or vector[2]<=.05:
            raise ValueError('Extra initial camera offsets must be finite XYZ vectors above the calibrated bin floor')
    # A new hook attempt must use the same extra live views as pickup. Stock
    # may now lie behind the withdrawn hand; no prior hook displacement is
    # treated as a measured part pose, and every packet joins the cloud union.
    for offset in [[.35,-.25,.50],[-.30,.25,.50]]+list(extra_offsets):
        vision.calibration['initial_eye_offset_m']=offset;event=dict(mode='initial',part=None,rearrangement_camera=True,capture_native_hand_opening_m=float(art.get_dof_positions()[0,fi].sum()))
        try:future=vision.request(event,arm.fk(art.get_dof_positions()[0,ids]))
        finally:vision.calibration['initial_eye_offset_m']=previous
        deadline=time.perf_counter()+3
        while not future.done():
            vision.poll();c['step'](part+'_rearrange_RGBD')
            if time.perf_counter()>deadline:raise RuntimeError('Rearrangement alternate image deadline')
        value=future.result();value['capture_native_hand_opening_m']=event['capture_native_hand_opening_m'];event['delivered_wall_s']=time.perf_counter()-c['started']
        if not value['result'].get('error'):measurements.append(value)
    if not measurements:raise RuntimeError('No directly measured stock in rearrangement camera views')
    poses=[]
    for value in measurements:
        for pose in value['result'].get('instances',[]):
            if pose['part']!=part or abs(pose['axis'][2])<.8:continue
            if c['config'].get('rearrange_front_up_only',c['config'].get('require_front_up_pickup',False)) and pose['axis'][2]<.35:continue
            if not any(np.linalg.norm(np.asarray(pose['center_m'])-old['center_m'])<.012 for old in poses):poses.append(pose)
    # High exposed stock leaves clearance for the native 205 mm palm.
    poses.sort(key=lambda p:-p['center_m'][2])
    clouds=[]
    for value in measurements:
        points=c['observed_grasp_clearance'].points(value['packet'],c['calibration'])
        if 'capture_native_hand_opening_m' in value:
            points=c['observed_grasp_clearance'].remove_known_hand_surface(points,value['H'],value['capture_native_hand_opening_m'])
        clouds.append(points)
    points=np.concatenate(clouds)
    floor=np.asarray(c['calibration']['floor_origin_world_m']);B=np.asarray(c['calibration']['basket_to_world_R']);normal=B[:,2]
    y=B[:,1].copy();y[2]=0;y/=np.linalg.norm(y);z=np.array([0.,0.,-1.]);R=np.column_stack([np.cross(y,z),y,z])
    rotations=[R,R@np.diag([-1.,-1.,1.])];current_R=arm.fk(cmd[0,ids])[:3,:3];rotations.sort(key=lambda r:np.linalg.norm(log(r@current_R.T)))
    solution=None
    counts={};examples={};pose_trials=[]
    def reject(kind,detail=None):
        counts[kind]=counts.get(kind,0)+1
        if detail is not None and len(examples.setdefault(kind,[]))<3:examples[kind].append(detail)
    def diagnostics():
        value=dict(part=part,counts=counts,examples=examples,pose_trials=pose_trials,uses_free_body_poses=False)
        (vision.out/'rearrange-plan-diagnostics.json').write_text(json.dumps(value,indent=2,default=str))
        print('SCATTERED_REARRANGE_PLANNING',json.dumps(dict(part=part,counts=counts,examples=examples),default=str),flush=True)
    tcp_offset=float(c['config'].get('rearrange_tcp_offset_m',.0475))
    if not .040<=tcp_offset<=.050:raise ValueError('One-finger bore offset must stay within the 40--50 mm validated proposal range')
    for pose in poses:
        P=np.asarray(pose['center_m']);toward=floor-P;toward-=normal*(toward@normal);distance=np.linalg.norm(toward)
        if distance<.015:continue
        direction=toward/distance;sign=1 if (floor-P)@y>=0 else -1
        position=P+sign*y*tcp_offset+normal*.006
        for rotation in rotations:
            H=frame(position,rotation);pre=H.copy();pre[2,3]=max(H[2,3]+.080,1.235)
            # The first 18 mm only takes up bore clearance. Up to another
            # 52 mm physically translates the stock toward the bin centre.
            pull=H.copy();pull[:3,3]+=direction*min(.070,distance+.015)
            trial=dict(image_center_m=P.tolist(),image_axis=pose['axis'],contact_TCP_world_m=H[:3,3].tolist(),pregrasp_TCP_world_m=pre[:3,3].tolist(),pull_TCP_world_m=pull[:3,3].tolist(),rotation=rotation.tolist(),pregrasp_ik_attempts=0);pose_trials.append(trial)
            clear,why=checker.hand_clear(H,.070)
            if not clear:reject('contact_hand_fixed',dict(trial=trial,collision=why));continue
            if not c['observed_grasp_clearance'].palm_clear(points,pose,pre,H):reject('approach_palm_observed_stock',trial);continue
            seeds=[cmd[0,ids],c['pickup_seed']]+list(np.random.default_rng(9127).uniform(arm.bounds[:,0],arm.bounds[:,1],size=(32,7)));seen=[]
            for seed_index,seed in enumerate(seeds):
                trial['pregrasp_ik_attempts']+=1
                preq=arm.solve(pre[:3,3],rotation,seed)
                if preq is None:reject('pregrasp_IK',dict(image_center_m=P.tolist(),target=pre[:3,3].tolist(),seed_index=seed_index));continue
                if any(np.linalg.norm(preq-old)<.05 for old in seen):reject('duplicate_pregrasp_IK_branch');continue
                seen.append(preq.copy());clear,why=checker.clear(preq,.070)
                if not clear:reject('pregrasp_robot_fixed',dict(image_center_m=P.tolist(),q=preq.tolist(),collision=why));continue
                qs=[preq];bad=False
                for u in np.linspace(0,1,6)[1:]:
                    q=arm.solve(pre[:3,3]*(1-u)+H[:3,3]*u,rotation,qs[-1])
                    if q is None:reject('descent_IK',dict(image_center_m=P.tolist(),progress=float(u)));bad=True;break
                    clear,why=checker.path([qs[-1],q],.070)
                    if not clear:reject('descent_robot_fixed',dict(image_center_m=P.tolist(),progress=float(u),collision=why));bad=True;break
                    qs.append(q)
                if bad:continue
                # Plan the entire contact sequence before entering the bore.
                # A partial pull is not a substitute for a reachable exit.
                contact_plan=None
                for angle in [0.,np.radians(25),-np.radians(25)]:
                    trial_direction=direction*np.cos(angle)+np.cross(normal,direction)*np.sin(angle)
                    for stroke in [min(.070,distance+.015),.055,.045,.035]:
                        if stroke<.025:continue
                        takeup=H.copy();takeup[:3,3]+=trial_direction*.018
                        endpoint=H.copy();endpoint[:3,3]+=trial_direction*stroke
                        unload=endpoint.copy();unload[:3,3]-=trial_direction*.012
                        withdrawn=unload.copy();withdrawn[2,3]=max(1.235,withdrawn[2,3]+.040)
                        if not c['observed_grasp_clearance'].palm_clear(points,pose,H,endpoint):reject('planned_pull_palm_observed_stock');continue
                        stages=[];last_H=H;last_q=qs[-1];blocked=False
                        for label,target in [('takeup',takeup),('pull',endpoint),('unload',unload),('withdraw',withdrawn)]:
                            stage=[last_q.copy()]
                            for u in np.linspace(0,1,6)[1:]:
                                q=arm.solve(last_H[:3,3]*(1-u)+target[:3,3]*u,rotation,stage[-1])
                                clear,why=checker.path([stage[-1],q],.070) if q is not None else (False,{'IK':False})
                                if not clear:reject('planned_'+label+'_robot_fixed',dict(image_center_m=P.tolist(),stroke_m=stroke,direction=trial_direction.tolist(),progress=float(u),collision=why));blocked=True;break
                                stage.append(q)
                            if blocked:break
                            # The executed cubic is checked again by move();
                            # corner-stop fallback retains these clear edges.
                            stages.append(np.asarray(stage));last_H=target;last_q=stage[-1]
                        if not blocked:contact_plan=stages;direction=trial_direction;pull=endpoint;trial['planned_stroke_m']=stroke;trial['planned_pull_TCP_world_m']=endpoint[:3,3].tolist();break
                    if contact_plan is not None:break
                if contact_plan is None:continue
                route=[cmd[0,ids].copy(),preq] if checker.path([cmd[0,ids],preq],.08)[0] else None
                if route is None:
                    for via in c.get('empty_return_waypoints',[]):
                        if checker.path([cmd[0,ids],via,preq],.08)[0]:route=[cmd[0,ids].copy(),via,preq];break
                if route is None:route=checker.route(cmd[0,ids],preq,.08,seconds=.35)
                if route is not None:solution=(pose,H,pre,qs,route,direction,pull,contact_plan);break
                reject('empty_robot_approach_route',dict(image_center_m=P.tolist(),q=preq.tolist()))
            if solution:break
        if solution:break
    diagnostics()
    tilted_proposal=None;left_anchor=None;left_opening_anchor=None
    if solution is None and part in ['ring1','ring2'] and c['config'].get('allow_tilted_bore_hook'):
        # Only an empty stationary other hand is represented by this query.
        # Measurements above are fresh live camera packets, never a saved run.
        if c.get('left_hand_known_empty') is not True:
            raise RuntimeError('Tilted hook requires an explicitly known empty left hand')
        if 'left' not in c.get('checkers',{}) or 'left_ids' not in c or 'left_fi' not in c:
            raise RuntimeError('Tilted hook requires current left-arm collision context')
        import types
        from pathlib import Path
        path=Path(__file__).resolve().parent/'probe_tilted_ring_hook.py'
        module=types.ModuleType('live_tilted_bore_hook');module.__file__=str(path)
        exec(compile(path.read_text(),str(path),'exec'),module.__dict__)
        vertical_diagnostics=vision.out/'rearrange-plan-diagnostics.json'
        (vision.out/'rearrange-vertical-plan-diagnostics.json').write_text(vertical_diagnostics.read_text())
        actual=art.get_dof_positions();left_anchor=np.asarray(actual[0,c['left_ids']],float).copy()
        left_opening_anchor=float(actual[0,c['left_fi']].sum())
        live=dict(c,cmd=cmd.copy());live['cmd'][0,ids]=actual[0,ids]
        with checker.other_arm_at(c['checkers']['left'],left_anchor,left_opening_anchor):
            tilted_proposal=module.plan(live,measurements)
        pose=tilted_proposal['pose'];H=np.asarray(tilted_proposal['contact_TCP']);pre=np.asarray(tilted_proposal['pre_TCP'])
        descent=np.asarray(tilted_proposal['descent']);route=np.asarray(tilted_proposal['pre_route'])
        direction=np.asarray(tilted_proposal['pull_direction']);pull=np.asarray(tilted_proposal['pull_TCP'])
        contact_plan=[np.asarray(stage) for stage in tilted_proposal['contact_stage_routes']]
        solution=(pose,H,pre,descent,route,direction,pull,contact_plan)
        c.setdefault('planning_diagnostics',[]).append(dict(event='live_tilted_bore_hook_planned',part=part,source='fresh live RGB-D and current robot encoders',left_hand_known_empty=True,left_actual_q=left_anchor.tolist(),left_actual_opening_m=left_opening_anchor,proposal=tilted_proposal,archived_execute_called=False))
    if solution is None:raise RuntimeError('No RGB-D single-finger rearrangement route')
    pose,H,pre,descent,route,direction,pull,contact_plan=solution
    def check_stationary_left():
        if tilted_proposal is None:return
        actual=art.get_dof_positions();left=np.asarray(actual[0,c['left_ids']],float)
        opening=float(actual[0,c['left_fi']].sum())
        if c.get('left_hand_known_empty') is not True or np.max(abs(left-left_anchor))>.002 or abs(opening-left_opening_anchor)>.001:
            raise RuntimeError('Tilted hook other-arm state changed after its collision-checked plan')
    def current_other_arm():
        if tilted_proposal is None:return nullcontext()
        check_stationary_left();actual=art.get_dof_positions()
        return checker.other_arm_at(c['checkers']['left'],actual[0,c['left_ids']],float(actual[0,c['left_fi']].sum()))
    def move(qs,requested,label,width=None):
        qs=np.array(qs);qs[0]=cmd[0,ids];start=cmd[0,fi].copy();target=start if width is None else np.full(2,width/2)
        count=max(65,int(np.ceil(np.linalg.norm(np.diff(qs,axis=0),axis=1).sum()/.003))+1);curve=CubicSpline(np.linspace(0,1,len(qs)),qs)(np.linspace(0,1,count));measured_width=float(art.get_dof_positions()[0,fi].sum());width_min=min(float(start.sum()),float(target.sum()),measured_width);width_max=max(float(start.sum()),float(target.sum()),measured_width);openings=np.unique(np.linspace(width_min,width_max,5))
        with current_other_arm():
            if all(checker.path(curve,float(opening))[0] for opening in openings):q,h,_=retime(qs,requested,dt,start,target,optimize=True)
            elif all(checker.path(qs,float(opening))[0] for opening in openings):q,h,_=retime_safe_polyline(qs,dt,start,target)
            else:raise RuntimeError('Rearrangement actual robot curve is blocked by fixed structure')
            if tilted_proposal is not None:
                issued=np.vstack([art.get_dof_positions()[0,ids],q])
                if not all(checker.path(issued,float(opening))[0] for opening in openings):
                    raise RuntimeError('Tilted hook exact emitted curve or actual-start edge is blocked')
        for row,weight in zip(q[1:],h[1:]):
            check_stationary_left()
            cmd[0,ids]=row;cmd[0,fi]=start+(target-start)*weight;c['step'](label)
        check_stationary_left()
        events.append(dict(event=label,wall_s=time.perf_counter()-c['started']));print('SCATTERED_REARRANGE',label,events[-1]['wall_s'],flush=True)
    def line(target,requested,label,allow_partial=False):
        initial=arm.fk(cmd[0,ids]);qs=[cmd[0,ids].copy()]
        for u in np.linspace(0,1,6)[1:]:
            q=arm.solve(initial[:3,3]*(1-u)+target[:3,3]*u,target[:3,:3],qs[-1])
            clear,why=checker.path([qs[-1],q],.070) if q is not None else (False,{'IK':False})
            if not clear:
                if allow_partial:
                    print('SCATTERED_REARRANGE_STOP_BEFORE_COLLISION',label,why,flush=True)
                    if len(qs)>1:move(qs,requested*(len(qs)-1)/5,label+'_shortened')
                    return False
                raise RuntimeError('Rearrangement Cartesian route invalidated '+str(why))
            qs.append(q)
        move(qs,requested,label)
    c['set_grip_hold']('right',False)
    move(route,.85,part+'_hook_pregrasp',.070)
    actual_width=float(art.get_dof_positions()[0,fi].sum())
    for opening in np.linspace(min(.070,actual_width),max(.070,actual_width),5):
        if not checker.clear(art.get_dof_positions()[0,ids],float(opening))[0]:raise RuntimeError('Hook pregrasp actual encoder opening would hit fixed structure')
    if abs(actual_width-.070)>.001:move([cmd[0,ids],cmd[0,ids]],.25,part+'_hook_stationary_width_completion',.070)
    if abs(float(art.get_dof_positions()[0,fi].sum())-.070)>.001:raise RuntimeError('Hook gripper did not reach its measured 70 mm opening before descent')
    move(descent,.65,part+'_hook_enter_bore')
    for qs,requested,label in zip(contact_plan,[.8,.7,.4,.7],['_hook_take_up_clearance','_hook_slide_toward_center','_hook_back_off_inside_bore','_hook_withdraw']):move(qs,requested,part+label)
    for _ in range(int(np.ceil(.35/dt))):c['step'](part+'_rearrange_settle')
    result=dict(state='physical_rearrangement_executed_requires_new_RGBD',part=part,observed_start=pose,events=events,object_teleports=False,full_task_pass=False)
    if tilted_proposal is not None:
        result.update(tilted_bore_hook=True,source='fresh live RGB-D and robot encoders',archived_execute_called=False,left_hand_known_empty=True,exact_issued_curve_and_actual_other_arm_checked=True)
    if c['config'].get('rearrange_then_pick'):
        import types
        from pathlib import Path
        path=Path(__file__).resolve().parent/'loose_ring_pick.py';module=types.ModuleType('loose_ring_pick');module.__file__=str(path);exec(compile(path.read_text(),str(path),'exec'),module.__dict__)
        fresh=dict(c,config=dict(c['config'],pickup_retry_attempt=0));fresh.pop('prefetched_initial',None);picked=module.execute(fresh);picked['physical_rearrangement']=result;return picked
    return result
