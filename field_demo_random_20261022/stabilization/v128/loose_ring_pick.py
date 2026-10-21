"""Live RGB-D arbitrary bin pickup prototype, physically simulated fingers.

This diagnostic returns a camera-confirmed grasp. It never claims assembly
completion, and it never reads or moves a free-body transform.
"""
import time,numpy as np
from scipy.interpolate import CubicSpline
from rgbd_geometry import axis_frame
from isaac_kinematics import frame,log
from fr3_motion_control import position_velocity_bounds
from fr3_path_timing import retime, retime_checked_polyline, retime_safe_polyline

def execute(c):
    arm=c['arm'];art=c['art'];cmd=c['cmd'];ids=c['ids'];fi=c['fi'];dt=c['dt'];vision=c['vision'];checker=c['checker'];phase='initial';events=[]
    def step():c['step'](phase)
    def wait(t,label):
        nonlocal phase
        phase=label
        for _ in range(int(np.ceil(t/dt))):step()
    def observe(mode,part=None,G=None,allow_missing=False):
        e=dict(mode=mode,part=part)
        if G is not None:
            e['tracked_grasp']=G.tolist();short=arm.fk(art.get_dof_positions()[0,ids])[:3,0].copy();short[2]=0.;short/=max(np.linalg.norm(short),1e-8);e['camera_eye_offset']=(short*(.38 if mode=='pregrasp_ring' else .35)+np.array([0,0,.40 if mode=='pregrasp_ring' else .25])).tolist()
            if mode=='held':e['require_annular_face']=True
        future=c.pop('prefetched_initial',None) if mode=='initial' else None
        if future is None:future=vision.request(e,arm.fk(art.get_dof_positions()[0,ids]))
        else:e=vision.jobs[next(i for i,(_,f) in enumerate(vision.jobs) if f is future)][0]
        deadline=time.perf_counter()+3
        while not future.done():
            vision.poll();step()
            if time.perf_counter()>deadline:raise RuntimeError('Fresh scattered perception timeout')
        v=future.result();e['delivered_wall_s']=time.perf_counter()-c['started']
        if v['result'].get('error') and not allow_missing:raise RuntimeError(v['result']['error'])
        return v
    def move(qs,t,label,width=None):
        nonlocal phase
        qs=np.array(qs);qs[0]=cmd[0,ids];start=cmd[0,fi].copy();target=start if width is None else np.full(2,width/2)
        optimized=c['config'].get('optimized_motion_timing',False)
        if optimized and label in ['loose_ring_pregrasp','loose_ring_lift','loose_ring_RGBD_lift_clearance_repair']:t=.05
        scale=max(1.,float(c['config'].get('stability_time_scale',1.)))
        if label=='loose_ring_pregrasp' and len(qs)>2:
            q,h,timing=retime_safe_polyline(qs,dt,start,target,scale=scale,velocity_fraction=.91,jerk=4000.)
        else:
            q,h,timing=retime(qs,t,dt,start,target,velocity_fraction=.91,jerk=4000.,optimize=optimized)
            if scale>1:q,h,timing=retime(qs,timing['duration_s']*scale,dt,start,target,velocity_fraction=.91,jerk=4000.,optimize=False)
        count=len(q)-1
        phase=label
        for row,w in zip(q[1:],h[1:]):cmd[0,ids]=row;cmd[0,fi]=start+(target-start)*w;step()
        events.append(dict(event=label,wall_s=time.perf_counter()-c['started'],motion_s=count*dt,timing=timing));print('SCATTERED_PICK',label,events[-1]['wall_s'],flush=True)
    def line(target,t,label,width=None):
        H=arm.fk(cmd[0,ids]);qs=[cmd[0,ids].copy()]
        if label=='loose_ring_descend' and c['config'].get('gentle_descent') and c['config'].get('pick_part')=='ring2':
            distance=float(np.linalg.norm(target[:3,3]-H[:3,3]));slow=min(.025,distance)
            if distance>slow+.001:
                from isaac_kinematics import exp
                u=1-slow/distance;via=frame(H[:3,3]*(1-u)+target[:3,3]*u,exp(log(target[:3,:3]@H[:3,:3].T)*u)@H[:3,:3]);line(via,.05,label+'_free',width)
            return line(target,1.875*slow/float(c['config'].get('descent_contact_speed_m_s',.030)),label+'_contact',width)
        for u in np.linspace(0,1,5)[1:]:
            from isaac_kinematics import exp
            R=exp(log(target[:3,:3]@H[:3,:3].T)*u)@H[:3,:3]
            P=H[:3,3]*(1-u)+target[:3,3]*u;q=arm.solve(P,R,qs[-1])
            if q is None:raise RuntimeError('No bin line IK '+label)
            qs.append(q)
        move(qs,t,label,width)
    initial_value=observe('initial');initial=initial_value['result'];observed_points=c['observed_grasp_clearance'].points(initial_value['packet'],c['calibration']) if c.get('observed_grasp_clearance') else None;part=c['config'].get('pick_part','ring1');candidates=[p for p in initial['instances'] if p['part']==part and abs(p['axis'][2])>.35 and p.get('front_back_orientation_margin_m',0)>.0005]
    if not candidates:raise RuntimeError('No visible accessible measurable '+part+'')
    pickup_seed=np.array(c['pickup_seed']);postures=[(yaw,pickup_seed.copy(),None) for yaw in np.arange(0,2*np.pi,np.pi/8)]+[(yaw,pickup_seed.copy(),0.) for yaw in [0,np.pi/2,-np.pi/2,np.pi,np.pi/4,-np.pi/4,3*np.pi/4,-3*np.pi/4]]
    for elbow_delta in [-.50,.50]:
        seed=pickup_seed.copy();seed[2]+=elbow_delta
        postures.extend((yaw,seed.copy(),0.) for yaw in [0,np.pi/2,-np.pi/2,np.pi])
    tilts=[np.radians(25),-np.radians(25),np.radians(45),-np.radians(45),np.radians(60),-np.radians(60)]
    if c['config'].get('allow_side_entry'):tilts+=[np.radians(75),-np.radians(75),np.pi/2,-np.pi/2]
    for tilt in tilts:
        postures.extend((yaw,pickup_seed.copy(),tilt) for yaw in [0,np.pi/2,-np.pi/2,np.pi])
    planning_started=time.perf_counter();rejections={};checks=0
    def reject(why):
        key=str(why);rejections[key]=rejections.get(key,0)+1
    candidates.sort(key=lambda p:(bool(c['config'].get('prefer_transfer_compatible')) and p['axis'][2]<0,np.linalg.norm(np.array(p['center_m'])[:2]-np.array([.49,-.725]))-3.0*(p['center_m'][2]-1.10)));routes=[]
    # Try a centered, axis-aligned grasp before the deeper recovery poses.
    postures.sort(key=lambda row:0 if row[2]==0. else 1 if row[2] is None else 2)
    modes=(['inner','outer'] if part=='ring2' and c['config'].get('prefer_internal_small_grasp') else ['outer','inner']) if c['config'].get('allow_internal_grasp') else ['outer']
    # Offset along the finger pad's width, perpendicular to closing. This
    # grasps a chord of the rim and moves the bulky palm away from a bin wall.
    # The observed grasp transform is passed to transfer/insertion unchanged.
    offsets=[0.,.012,.024] if c['config'].get('allow_chord_grasp',True) else [0.]
    for grasp_type,depth_adjust,chord_offset,p in [(mode,d,x,p) for mode in modes for p in candidates for d in ([0.] if mode=='inner' else [-.004,-.002,0.,.002,.004] if part=='ring2' and c['config'].get('stable_small_grasp') else [.002,.004,0.,-.004] if part=='ring2' else [0.,-.004]) for x in ([0.] if mode=='inner' else offsets)]:
        P=np.array(p['center_m']);n=np.array(p['axis']);R0=axis_frame(-n*np.sign(n[2]))
        current_rotation=arm.fk(cmd[0,ids])[:3,:3]
        def posture_cost(posture):
            yaw,_,tilt=posture
            if tilt is None:
                z=np.array([0.,0.,-1.]);y=np.array([np.cos(yaw),np.sin(yaw),0.])
                if abs(n@y)>.25:return np.inf
                rotation=np.column_stack([np.cross(y,z),y,z])
            else:
                rotation=R0@np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1.]])@np.array([[np.cos(tilt),0,np.sin(tilt)],[0,1,0],[-np.sin(tilt),0,np.cos(tilt)]])
            return float(np.linalg.norm(log(rotation@current_rotation.T)))
        local_postures=postures
        if part=='ring2' and c['config'].get('stable_small_grasp') and np.linalg.norm(n[:2])>.005:
            tangent=float(np.arctan2(n[0],-n[1]))
            # Keep the closing direction exactly in the ring plane. A coarse
            # yaw grid otherwise contacts opposite axial rim corners first.
            local_postures=[(tangent,pickup_seed.copy(),None),(tangent+np.pi,pickup_seed.copy(),None)]+[r for r in postures if r[2] is not None]
        for yaw,ik_seed,tilt in sorted(local_postures,key=lambda row:((0 if row[2]==0. else 1 if row[2] is None else 2),posture_cost(row)) if part=='ring2' else (0,posture_cost(row))):
            if grasp_type=='inner' and tilt!=0.:continue
            if grasp_type=='outer' and chord_offset and tilt!=0.:continue
            opening=.001 if grasp_type=='inner' else min(.08,(.075 if part=='ring1' else .062)+.006)
            grip_width=(.038 if part=='ring1' else .025) if grasp_type=='inner' else (.075 if part=='ring1' else .062)
            tcp_offset=np.zeros(3)
            if grasp_type=='inner':
                toward=np.asarray(c['calibration']['floor_origin_world_m'])-P;toward-=n*(toward@n)
                distance=np.linalg.norm(toward);tcp_offset=np.zeros(3) if c['config'].get('prefer_internal_small_grasp') and part=='ring2' else toward/max(distance,1e-12)*min(distance,.013 if part=='ring1' else .008)
            if tilt is None:
                z=np.array([0.,0.,-1.]);y=np.array([np.cos(yaw),np.sin(yaw),0.]);R=np.column_stack([np.cross(y,z),y,z])
                if abs(n@y)>.25:continue
                contact_offset=.0028
            else:
                R=R0@np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1.]])
                R=R@np.array([[np.cos(tilt),0,np.sin(tilt)],[0,1,0],[-np.sin(tilt),0,np.cos(tilt)]])
                half=.00775 if part=='ring1' else .007;contact_offset=max(.0028,.0089*np.cos(tilt)+.00875*abs(np.sin(tilt))-half+.0012)
            contact_offset=.0045 if grasp_type=='inner' else max(-.003,contact_offset+depth_adjust)
            if grasp_type=='outer' and chord_offset:
                toward=np.asarray(c['calibration']['floor_origin_world_m'])-P
                tcp_offset=R[:,0]*np.sign(toward@R[:,0])*chord_offset
                nominal=.075 if part=='ring1' else .062
                near_pad_offset=max(0.,chord_offset-.0105)
                grip_width=2*np.sqrt(max(0.,(nominal/2)**2-near_pad_offset**2))
            H=frame(P+n*np.sign(n[2])*contact_offset+tcp_offset,R);pre=H.copy();pre[2,3]=max(pre[2,3]+.08,float(c['config'].get('pregrasp_floor_small_m',c['config'].get('pregrasp_floor_m',1.235)) if part=='ring2' else c['config'].get('pregrasp_floor_m',1.235)));
            if c['config'].get('allow_side_entry') and abs(R[2,2])<.45:
                pre=H.copy();pre[:3,3]-=R[:,2]*.10
            fixed_clear,fixed_why=checker.hand_clear(H,opening)
            if not fixed_clear:reject('fixed hand pose: '+str(fixed_why));continue
            if grasp_type=='outer' and c.get('grasp_pad_aperture'):
                predicted_width=c['grasp_pad_aperture'](p,H)
                if predicted_width is None:reject('Ring CAD does not reach both fingertip pads');continue
                grip_width=predicted_width
            if c.get('handover_feasible_grasp') and not c['handover_feasible_grasp'](dict(p,planned_grip_width_m=grip_width,grasp_type=grasp_type),H):
                reject('native hand pair cannot fit this grasp at handover');continue
            loose_clear=True
            if observed_points is not None:
                loose_clear=c['observed_grasp_clearance'].palm_clear(observed_points,p,pre,H) if c['config'].get('allow_fingertip_nudging') else c['observed_grasp_clearance'].clear(observed_points,p,pre,H,.08)
            if not loose_clear:
                print('SCATTERED_VISIBLE_CLUTTER_REJECT',part,yaw,None if tilt is None else float(tilt),flush=True);continue
            rng=np.random.default_rng(1127);seed_pool=[ik_seed,cmd[0,ids]]+list(rng.uniform(arm.bounds[:,0],arm.bounds[:,1],size=(48,7)));solutions=[]
            for seed in seed_pool:
                q=arm.solve(pre[:3,3],R,seed)
                if q is not None and all(np.linalg.norm(q-old)>.12 for old in solutions):solutions.append(q)
            solutions.sort(key=lambda q:np.max(abs(q-cmd[0,ids])/np.array([2.62,2.62,2.62,2.62,5.26,4.18,5.26])))
            for preq in solutions[:12]:
                finalq=arm.solve(H[:3,3],R,preq)
                if finalq is None:reject('final IK');continue
                ok,why=checker.clear(finalq,opening)
                if not ok:
                    reject(str(why))
                    # Every IK branch has the same final hand pose. A fixed
                    # hand/basket collision cannot be repaired by elbow seeds.
                    if why['query'].startswith(arm.hand_path+'/'):break
                    continue
                descent=[preq];bad=None
                for u in np.linspace(0,1,5)[1:]:
                    q=arm.solve(pre[:3,3]*(1-u)+H[:3,3]*u,R,descent[-1])
                    if q is None:bad='incremental Cartesian IK';break
                    clear,why=checker.path([descent[-1],q],opening)
                    if not clear:bad=why;break
                    descent.append(q)
                if bad is not None:reject(str(bad));continue
                D=.075 if part=='ring1' else .062;half=.00775 if part=='ring1' else .007;support=D/2*np.sqrt(max(0,1-n[2]**2))+half*abs(n[2]);lift_delta=max(.075,1.210918206667893+support+.010-P[2]);liftqs=[descent[-1]]
                for u in np.linspace(0,1,5)[1:]:
                    target=H[:3,3]+[0,0,lift_delta*u];q=arm.solve(target,R,liftqs[-1])
                    if q is None:bad='lift IK';break
                    if not checker.path([liftqs[-1],q],grip_width)[0]:bad='lift structure collision';break
                    liftqs.append(q)
                if bad is not None:reject(bad);continue
                route=None
                if checker.path([cmd[0,ids],preq],.08)[0]:route=[cmd[0,ids].copy(),preq]
                else:
                    for via in c.get('empty_return_waypoints',[]):
                        if checker.path([cmd[0,ids],via,preq],.08)[0]:route=[cmd[0,ids].copy(),via,preq];break
                if route is None:route=checker.route(cmd[0,ids],preq,.08,seconds=.15)
                if route is not None:routes.append((np.linalg.norm(preq-cmd[0,ids]),dict(p,grasp_yaw_rad=float(yaw),grasp_tilt_rad=None if tilt is None else float(tilt),grasp_type=grasp_type,planned_grip_width_m=grip_width,tcp_offset_m=tcp_offset.tolist()),H,route,contact_offset));break
            if routes:break
        if routes:break
    print('SCATTERED_PLANNING',part,'seconds',time.perf_counter()-planning_started,'rejections',rejections,flush=True)
    if not routes:raise RuntimeError('No structure-clear grasp among RGB-D instances')
    _,p,H,route,contact_offset=min(routes,key=lambda row:row[0]);grasp_type=p.get('grasp_type','outer');tcp_offset=np.asarray(p.get('tcp_offset_m',[0,0,0]));empty_width=.001 if grasp_type=='inner' else min(.08,(.075 if part=='ring1' else .062)+.006)
    print('SCATTERED_GRASP_TYPE',part,grasp_type,p.get('planned_grip_width_m'),'offset',tcp_offset.tolist(),'tilt',p.get('grasp_tilt_rad'),'image_center',p['center_m'],flush=True)
    if c.get('bind_validation_target'):c['bind_validation_target'](part,p)
    P=np.array(p['center_m']);n=np.array(p['axis']);current_H=arm.fk(art.get_dof_positions()[0,ids]);pre_event=dict(mode='pregrasp_ring',part=part,tracked_grasp=(np.linalg.inv(current_H)@frame(P,axis_frame(n))).tolist(),camera_eye_offset=[.06,-.06,.30]);pre_future=vision.request(pre_event,current_H)
    move(route,.8,'loose_ring_pregrasp',empty_width)
    if abs(float(cmd[0,fi].sum())-empty_width)>.0001:move([cmd[0,ids],cmd[0,ids]],.10,'loose_ring_empty_preshape',empty_width)
    while not pre_future.done():vision.poll();step()
    pre_value=pre_future.result();pre_event['delivered_wall_s']=time.perf_counter()-c['started']
    if pre_value['result'].get('error'):raise RuntimeError(pre_value['result']['error'])
    refined=pre_value['result']['pose'];newP=np.array(refined['center_m']);newN=np.array(refined['axis'])
    if np.linalg.norm(newP-P)>.008 or abs(newN[2])<.35:raise RuntimeError('Fresh pregrasp pose did not confirm an accessible measurable ring')
    from isaac_kinematics import exp
    a=n;b=newN*(1 if n@newN>=0 else -1);cross=np.cross(a,b);change=exp(cross/max(np.linalg.norm(cross),1e-12)*np.arctan2(np.linalg.norm(cross),a@b));H[:3,:3]=change@H[:3,:3];tcp_offset=change@tcp_offset;P=newP;n=newN;H[:3,3]=P+n*np.sign(n[2])*contact_offset+tcp_offset;line(H,.45,'loose_ring_descend')
    # Loose neighbours can move the target during descent. Reobserve before
    # closing; only image measurements and the hand encoder enter this repair.
    # An ungrasped ring stays in the world, not on the commanded TCP. Encode
    # its last IMAGE pose relative to the measured hand at this exposure.
    measured_H=arm.fk(art.get_dof_positions()[0,ids]);settled_result=observe('pregrasp_ring',part,np.linalg.inv(measured_H)@frame(P,axis_frame(n)),allow_missing=True)['result']
    visible_pose=settled_result.get('pose',{})
    directly_visible=bool(visible_pose.get('front_back_from_RGBD'))
    settled=visible_pose if directly_visible else dict(center_m=P.tolist(),axis=n.tolist());settledP=np.array(settled['center_m']);settledN=np.array(settled['axis'])
    events.append(dict(event='preclose_reobservation',new_pose_visible=directly_visible,fallback='Last directly measured RGB-D pose; lift requires a new visible ring' if not directly_visible else None,wall_s=time.perf_counter()-c['started']))
    if np.linalg.norm(settledP-P)>.025:raise RuntimeError('Ring moved outside local RGB-D grasp recovery range')
    if np.linalg.norm(settledP-P)>.0008 or np.arccos(np.clip(n@settledN,-1,1))>.035:
        aligned=settledN*(1 if n@settledN>=0 else -1);cross=np.cross(n,aligned);change=exp(cross/max(np.linalg.norm(cross),1e-12)*np.arctan2(np.linalg.norm(cross),n@aligned));H[:3,:3]=change@H[:3,:3];tcp_offset=change@tcp_offset
        P=settledP;n=settledN;H[:3,3]=P+n*np.sign(n[2])*contact_offset+tcp_offset
        if not checker.hand_clear(H,float(art.get_dof_positions()[0,fi].sum()))[0]:raise RuntimeError('Fresh RGB-D grasp correction would collide with fixed structure')
        line(H,.20,'loose_ring_observed_grasp_adjust')
    else:P=settledP;n=settledN
    # Native fingers close against the observed diameter with finite effort.
    c['stiffness'][0,fi]=0.;c['damping'][0,fi]=[4000.,.001];art.set_dof_stiffnesses(c['stiffness'],c['idx']);art.set_dof_dampings(c['damping'],c['idx']);D=.075 if part=='ring1'else .062
    close_width=p['planned_grip_width_m'] if grasp_type=='inner' else max(.001,p['planned_grip_width_m']-.003)
    if c.get('set_grip_hold'):c['set_grip_hold']('right',True,direction=1. if grasp_type=='inner' else -1.)
    move([cmd[0,ids],cmd[0,ids]],.14,'loose_ring_close',close_width)
    if c.get('set_grip_hold'):c['set_grip_hold']('right',True,direction=1. if grasp_type=='inner' else -1.)
    wait(float(c['config'].get('grasp_settle_s',.12)),'loose_ring_grasp_settle')
    for _ in range(12):
        gap=float(art.get_dof_positions()[0,fi].sum())
        if gap<=p['planned_grip_width_m']+.001 and np.max(abs(art.get_dof_velocities()[0,fi]))<.010:break
        if gap<=close_width+.001:break
        wait(.025,'loose_ring_grasp_controller_completion')
    aperture=float(art.get_dof_positions()[0,fi].sum())
    events.append(dict(event='encoder_grasp_aperture',aperture_m=aperture,expected_class_diameter_m=D,grasp_type=grasp_type))
    if grasp_type=='outer' and aperture>D+.0035:
        raise RuntimeError('Grasp aperture exceeds observed ring size; wrong class or neighbouring stock contact')
    if grasp_type=='outer' and aperture<close_width+.0005:
        raise RuntimeError('No opposing pad contact: fingers reached preload width without holding the rim')
    G=np.linalg.inv(arm.fk(art.get_dof_positions()[0,ids]))@frame(P,axis_frame(n));lift=H.copy();half=.00775 if part=='ring1' else .007;support=D/2*np.sqrt(max(0,1-n[2]**2))+half*abs(n[2]);requested_lift=max(.075,1.210918206667893+support+.010-P[2]);lift[2,3]+=requested_lift;line(lift,.6,'loose_ring_lift');wait(.10,'loose_ring_lift_settle');v=observe('held',part,G);pose=v['result']['pose'];Gobs=np.linalg.inv(v['H'])@frame(pose['center_m'],axis_frame(pose['axis']));predicted=arm.fk(art.get_dof_positions()[0,ids])@Gobs;rise=float(predicted[2,3]-P[2]);wait(.02,'loose_ring_camera_confirmed_hold')
    observed_axis=np.array(pose['axis']);observed_bottom=pose['center_m'][2]-D/2*np.sqrt(max(0,1-observed_axis[2]**2))-half*abs(observed_axis[2]);
    if rise>.04 and -.025<observed_bottom-1.210918206667893<.003:
        # A small measured slip is recoverable by a short extra axial lift.
        # The same fresh image gate must confirm clearance after this move.
        delta=1.210918206667893+.012-observed_bottom;repair=arm.fk(cmd[0,ids]);repair[2,3]+=delta
        line(repair,.25,'loose_ring_RGBD_lift_clearance_repair');wait(.08,'loose_ring_repair_settle')
        v=observe('held',part,Gobs);pose=v['result']['pose'];Gobs=np.linalg.inv(v['H'])@frame(pose['center_m'],axis_frame(pose['axis']));predicted=arm.fk(art.get_dof_positions()[0,ids])@Gobs;rise=float(predicted[2,3]-P[2]);observed_axis=np.array(pose['axis']);observed_bottom=pose['center_m'][2]-D/2*np.sqrt(max(0,1-observed_axis[2]**2))-half*abs(observed_axis[2])
    if rise<requested_lift-.015 or observed_bottom<1.210918206667893+.003:raise RuntimeError('RGB-D did not confirm selected ring fully above rim')
    if c.get('bind_validation_target'):c['bind_validation_target'](part,pose)
    return dict(state='RGBD_grasp_and_lift_confirmed',grasp_type=grasp_type,selected_instance=p,measured_grasp=Gobs.tolist(),observed_lift_m=rise,events=events,elapsed_s=time.perf_counter()-c['started'],full_task_pass=False,uses_simulator_body_poses=False)
