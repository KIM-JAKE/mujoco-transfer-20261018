"""One uninterrupted tail-grasp pin skill; fresh RGB-D plus encoders only.

The caller advances the other arm and records the shared physical/wall clock.
No scene initialization, pose resets or workpiece attachments occur here.
"""
from pathlib import Path
import time,json,numpy as np
from pxr import UsdPhysics
from scipy.interpolate import CubicSpline
from isaac_kinematics import frame,exp
from rgbd_geometry import axis_frame
from fr3_motion_control import position_velocity_bounds
from fr3_path_timing import retime,retime_safe_polyline

def execute(ctx):
    HERE=Path(__file__).resolve().parent
    art=ctx['art'];arm=ctx['arm'];ids=ctx['ids'];fi=ctx['fi'];cmd=ctx['cmd'];idx=ctx['idx'];vision=ctx['vision'];checker=ctx['checker'];stage=ctx['stage'];sim=ctx['sim'];paths=ctx['paths'];goal=np.array(ctx['goal']);truth=ctx['truth'];started=ctx['started'];stiff=ctx['stiffness'];damping=ctx['damping'];DT=ctx['dt'];cand=ctx['candidate'];G=np.array(cand['grasp_transform']);pickupH=np.array(cand['pickup_H']);preH=pickupH.copy();preH[:3,3]+=[0,0,.07];preq=arm.solve(preH[:3,3],preH[:3,:3],cand['pickup_q']);phase='pin_start';report=ctx.setdefault('report',{});report.update(events=[],pin_single_stroke_pass=False)
    def step():ctx['step'](phase)
    def wait(seconds,label):
        nonlocal phase
        phase=label
        for _ in range(round(seconds/DT)):step()
    def observe(label,grasp=None):
        e=dict(mode='pin_initial'if grasp is None else'held',part='pin',label=label)
        if grasp is not None:e.update(tracked_grasp=grasp.tolist(),camera_eye_offset=([-.22,0,.12] if label=="pin_lift_RGBD" else [-.22,-.15,-.12]))
        H=arm.fk(art.get_dof_positions()[0,ids]);prefetched=ctx.pop('prefetched_initial',None) if grasp is None else None
        if prefetched is not None:e,f=prefetched
        else:f=vision.request(e,H)
        deadline=time.perf_counter()+1.
        while not f.done():
            vision.poll();step()
            if time.perf_counter()>deadline:raise RuntimeError('RGB-D response timeout')
        value=f.result();e['delivered_wall_s']=time.perf_counter()-started
        if value['result'].get('error'):raise RuntimeError(value['result']['error'])
        report['events'].append(dict(event=label,wall_s=e['delivered_wall_s'],result=value['result']));print('PIN_CONTACT_VISION',label,e['delivered_wall_s'],flush=True);return value
    def motion(qs,requested,label,checked_edges=True):
        nonlocal phase
        checked_edges=checked_edges and not getattr(qs,'verified_cubic',False);qs=np.array(qs);qs[0]=cmd[0,ids];T=requested
        # Smooth joint spline sampled at the real control rate, checked against
        # position-dependent bounds, acceleration and jerk before execution.
        opening=cmd[0,fi].copy();preshape=label=='pin_tail_approach';optimized=ctx.get('optimized_motion_timing',False)
        if optimized and any(name in label for name in ['approach','carry','orient_free','lift']):T=.05
        scale=max(1.,float(ctx.get('stability_time_scale',1.)))
        if checked_edges and len(qs)>2:
            q,h,timing=retime_safe_polyline(qs,DT,opening,np.full(2,.007) if preshape else opening,scale=scale,velocity_fraction=.95,acceleration=9.5,jerk=4750.)
        else:
            q,h,timing=retime(qs,T,DT,opening,np.full(2,.007) if preshape else opening,velocity_fraction=.95,acceleration=9.5,jerk=4750.,finger_command_speed=.0475,optimize=optimized)
            if scale>1:q,h,timing=retime(qs,timing['duration_s']*scale,DT,opening,np.full(2,.007) if preshape else opening,velocity_fraction=.95,acceleration=9.5,jerk=4750.,finger_command_speed=.0475,optimize=False)
        phase=label
        for row,hvalue in zip(q[1:],h[1:]):
            cmd[0,ids]=row
            if preshape:cmd[0,fi]=opening+(.007-opening)*hvalue
            step()
        report['events'].append(dict(event=label,wall_s=time.perf_counter()-started,physical_duration_s=(len(q)-1)*DT));print('PIN_CONTACT_MOVE',label,time.perf_counter()-started,flush=True)
    def line(H0,H1,requested,label):
        H0=arm.fk(cmd[0,ids]);rows=[cmd[0,ids].copy()];last=rows[0]
        for u in np.linspace(0,1,7)[1:]:
            T=H0.copy();T[:3,3]=H0[:3,3]*(1-u)+H1[:3,3]*u;q=arm.solve(T[:3,3],T[:3,:3],last)
            if q is None:raise RuntimeError('Axial IK failed '+label)
            rows.append(q);last=q
        opening=max(float(art.get_dof_positions()[0,fi].sum()),float(cmd[0,fi].sum()))
        curve=CubicSpline(np.linspace(0,1,len(rows)),rows)(np.linspace(0,1,41));clear,why=checker.path(curve,opening,paths['workpiece_pin'])
        if not clear:raise RuntimeError('Pin Cartesian robot path obstructed '+label+' '+str(why))
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
    if float(cmd[0,fi].sum())>.0145:
        H=arm.fk(cmd[0,ids]);opening=float(art.get_dof_positions()[0,fi].sum())
        deadline=time.perf_counter()+2.5
        while not all(checker.hand_clear(H,w)[0] for w in np.linspace(opening,.014,9)):
            if time.perf_counter()>deadline:raise RuntimeError('No clear empty-hand preshape before pin pickup')
            wait(.025,'pin_waits_for_receiving_arm_clearance')
        fingers(.014,'pin_empty_preshape')
    P=np.array(v['result']['pose']['center_m']);pickupH[:3,3]+=P-np.array(json.load(open(HERE.parents[1]/'runs/native-roas-table-pin-tail01/pin-direct-probe.json'))['pickup_center_from_RGBD'])
    newpre=pickupH.copy();newpre[:3,3]+=[0,0,.07]
    robot_hint_data=json.loads((HERE/'bin-picking/direct-pin-robot-seeds.json').read_text());robot_seeds=robot_hint_data['seeds'];solutions=[];route=None;rejections=[];opening=max(.014,float(cmd[0,fi].sum()))
    # Preserve the current elbow branch before considering a distant taught
    # branch. Both satisfy the same fresh image target, bounds and geometry.
    seeds=[robot_seeds.get('pin_tail_approach',cmd[0,ids]),cmd[0,ids],preq,cand['pickup_q']]+list(np.random.default_rng(4121).uniform(arm.bounds[:,0],arm.bounds[:,1],(32,7)))
    for seed in seeds:
        if seed is None:continue
        q=arm.solve(newpre[:3,3],newpre[:3,:3],seed)
        if q is not None and all(np.linalg.norm(q-old)>.05 for old in solutions):solutions.append(q)
    next_goal=np.asarray(robot_seeds.get('pin_direct_preinsert',cmd[0,ids]));joint_speed=np.array([2.62,2.62,2.62,2.62,5.26,4.18,5.26])
    solutions.sort(key=lambda q:float(np.max(abs(q-cmd[0,ids])/joint_speed)+np.max(abs(q-next_goal)/joint_speed)) if ctx.get('goal_aware_pickup') else np.linalg.norm(q-cmd[0,ids]))
    for q in solutions:
        clear,why=checker.clear(q,opening,paths['workpiece_pin'])
        if not clear:rejections.append(why);continue
        route=checker.route(cmd[0,ids],q,opening,paths['workpiece_pin'],seconds=.75)
        if route is not None:break
    if route is None:raise RuntimeError('No pin pregrasp route across IK branches '+str(rejections[:3]))
    report['pregrasp_IK_solutions']=len(solutions);motion(route,1.1,'pin_tail_approach')
    H=arm.fk(art.get_dof_positions()[0,ids]);line(H,pickupH,.6,'pin_tail_descend');stiff[0,fi]=0. if ctx.get('set_grip_hold') else 30000.;damping[0,fi]=[4000.,.001] if ctx.get('set_grip_hold') else 300.;art.set_dof_stiffnesses(stiff,idx);art.set_dof_dampings(damping,idx);fingers(.0095,'pin_tail_close');
    if ctx.get('set_grip_hold'):ctx['set_grip_hold'](True)
    wait(.15,'pin_tail_grasp_hold');H=arm.fk(art.get_dof_positions()[0,ids]);lift=H.copy();lift[:3,3]+=[0,0,.07];line(H,lift,.6,'pin_tail_lift');wait(.15,'pin_lift_settle');v=observe('pin_lift_RGBD',G);p=v['result']['pose'];Gobs=np.linalg.inv(v['H'])@frame(p['center_m'],axis_frame(p['axis']));axis0=G[:3,2];axis1=Gobs[:3,2];cross=np.cross(axis0,axis1);angle=np.arctan2(np.linalg.norm(cross),axis0@axis1);Gobs[:3,:3]=exp(cross/max(np.linalg.norm(cross),1e-12)*angle)@G[:3,:3];G=Gobs;report['measured_grasp']=G.tolist()
    if ctx.get('before_transport'):report['left_work_zone_clearance']=ctx['before_transport'](G)
    R=np.array(cand['goal_part_R']);safe=frame([-.10,-.25,1.35],R)@np.linalg.inv(G);pre=frame(goal-[0,0,.045],R)@np.linalg.inv(G);end=frame(goal,R)@np.linalg.inv(G)
    robot_seeds=json.loads((HERE/'bin-picking/direct-pin-robot-seeds.json').read_text())['seeds']
    transports=[(safe,'pin_tail_orient_free'),(pre,'pin_tail_carry_to_preinsert')]
    if ctx.get('optimized_motion_timing'):
        checker.attach_payload(paths['workpiece_pin'],frame([0,0,.1034],np.eye(3))@G);shortcut=None
        try:
            for seed in [robot_seeds.get('pin_direct_preinsert',robot_seeds['pin_tail_carry_to_preinsert']),robot_seeds['pin_tail_carry_to_preinsert'],cmd[0,ids]]:
                q=arm.solve(pre[:3,3],pre[:3,:3],seed)
                if q is None:continue
                hint=robot_hint_data.get('routes',{}).get('pin_direct_carry_to_preinsert')
                if hint:
                    candidate=np.asarray(hint).copy();candidate[0]=cmd[0,ids];candidate[-1]=q
                    if checker.path(candidate,.0105,paths['workpiece_pin'])[0]:shortcut=candidate;report['validated_robot_route_hint_used']=True
                if shortcut is None:shortcut=checker.route(cmd[0,ids],q,.0105,paths['workpiece_pin'],seconds=1.)
                if shortcut is not None:break
        finally:checker.detach_payload()
        if shortcut is not None:
            from grasp_transfer_feasibility import shorten_checked_route
            checker.attach_payload(paths['workpiece_pin'],frame([0,0,.1034],np.eye(3))@G)
            try:shortcut=shorten_checked_route(checker,shortcut,.0105,paths['workpiece_pin'])
            finally:checker.detach_payload()
            report['pin_shortcut_nodes']=len(shortcut);motion(shortcut,.05,'pin_tail_direct_carry_to_preinsert');transports=[];report['direct_preinsert_shortcut']=True
    for target,label in transports:
        checker.attach_payload(paths['workpiece_pin'],frame([0,0,.1034],np.eye(3))@G);route=None
        for seed in [robot_seeds[label],cand['insert_waypoints'][0] if label=='pin_tail_carry_to_preinsert' else cmd[0,ids],cmd[0,ids]]:
            q=arm.solve(target[:3,3],target[:3,:3],seed)
            if q is None:continue
            clear,why=checker.clear(q,.0105,paths['workpiece_pin']);print('PIN_CARRY_GOAL_CLEAR',label,(clear,why),flush=True)
            if not clear:continue
            route=checker.route(cmd[0,ids],q,.0105,paths['workpiece_pin'],seconds=2)
            if route is not None:break
        checker.detach_payload()
        if route is None:raise RuntimeError('No collision-free carry '+label)
        motion(route,.9,label)
    v=observe('pin_preinsert_RGBD',G);Pobs=np.array(v['result']['pose']['center_m']);report['preinsert_observed_center_m']=Pobs.tolist();remaining_depth=float(goal[2]-Pobs[2]);report['RGBD_measured_remaining_axial_depth_m']=remaining_depth
    if np.linalg.norm(Pobs[:2]-goal[:2])>=.0015 or not .035<remaining_depth<.055:raise RuntimeError('Preinsert RGB-D alignment/depth outside direct insertion range')
    pre=arm.fk(cmd[0,ids]);end=pre.copy();end[2,3]+=remaining_depth;line(pre,end,.8,'pin_single_axial_insert');wait(.12,'pin_full_depth_hold');v=observe('pin_full_depth_RGBD',G);pose=v['result']['pose'];error=float(np.linalg.norm(np.array(pose['center_m'])-goal));ae=float(np.arccos(np.clip(np.array(pose['axis'])[2],-1,1)));report['RGBD_full_depth_check']=dict(position_error_m=error,axis_error_rad=ae)
    for correction_index in range(2):
        remaining=float(goal[2]-pose['center_m'][2]);radial=float(np.linalg.norm(np.array(pose['center_m'])[:2]-goal[:2]))
        if not (.0005<remaining<.006 and radial<.0015 and ae<.05):break
        H=arm.fk(cmd[0,ids]);end=H.copy();end[2,3]+=remaining;line(H,end,.25,'pin_axial_depth_correction');wait(.08,'pin_depth_check_hold');v=observe('pin_full_depth_RGBD',G);pose=v['result']['pose'];error=float(np.linalg.norm(np.array(pose['center_m'])-goal));ae=float(np.arccos(np.clip(np.array(pose['axis'])[2],-1,1)));report['RGBD_full_depth_check']=dict(position_error_m=error,axis_error_rad=ae);report['axial_visual_corrections_with_grip_closed']=correction_index+1
    if error>=.0015 or ae>=.05:raise RuntimeError('Full depth not confirmed by RGB-D; fingers remain closed')
    for prim in stage.Traverse():
        if ctx.get('close_pin_clamp',True) and prim.IsA(UsdPhysics.PrismaticJoint)and str(prim.GetPath()).startswith('/World/Clamps/pin/'):
            joint=UsdPhysics.PrismaticJoint(prim);UsdPhysics.DriveAPI(prim,'linear').GetTargetPositionAttr().Set(joint.GetLowerLimitAttr().Get())
    sim.flush_changes();wait(.12,'pin_clamp_hold');
    if ctx.get('set_grip_hold'):ctx['set_grip_hold'](False)
    # Return to the same preinsert pose that was checked and just traversed.
    # The former extra 25 mm downward stroke drove link 4 into the machine.
    fingers(.018,'pin_release_after_full_depth');withdraw=pre.copy();line(end,withdraw,.65,'pin_axial_withdraw');wait(0. if ctx.get('defer_retention_to_recipe') else .5,'pin_retention');report['retention_deferred_to_recipe']=bool(ctx.get('defer_retention_to_recipe'));tr=truth['pin'].get_transforms()[0];ae=float(np.arccos(np.clip(1-2*(tr[3]**2+tr[4]**2),-1,1)));err=float(np.linalg.norm(tr[:3]-goal));report.update(state='pin_single_stroke_contact_pass'if err<.0015 and ae<.05 else'pin_retention_failed',pin_single_stroke_pass=bool(err<.0015 and ae<.05),final_position_error_m=err,final_axis_error_rad=ae,pin_trial_wall_s=time.perf_counter()-started)
    return report
