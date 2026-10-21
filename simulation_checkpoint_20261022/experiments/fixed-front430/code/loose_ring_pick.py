"""Live RGB-D arbitrary bin pickup prototype, physically simulated fingers.

This diagnostic returns a camera-confirmed grasp. It never claims assembly
completion, and it never reads or moves a free-body transform.
"""
import time,json,numpy as np
from scipy.interpolate import CubicSpline
from rgbd_geometry import axis_frame
from isaac_kinematics import frame,log
from fr3_motion_control import position_velocity_bounds
from fr3_path_timing import retime, retime_checked_polyline, retime_safe_polyline

def selected_outer_approach_clear(c,pose,start,end,opening):
    """No pre-close contact with selected RGB-D class CAD, native full hull.

    Constant-rotation translation uses the exact convex swept finger union.
    Rotating segments are subdivided to <=1deg and inflated by the maximum
    rotation-arc/chord deviation. No live object pose or actor ID is used.
    Surface samples are the existing fixed class-CAD geometry admission model.
    """
    from pathlib import Path
    from scipy.spatial import ConvexHull
    from isaac_kinematics import exp
    cache=c.setdefault('_selected_outer_native_geometry',{})
    if 'vertices' not in cache:cache['vertices']=np.load(Path(__file__).resolve().parent/'native-official-finger.npz')['vertices'].astype(float)
    vertices=cache['vertices'];start=np.asarray(start,float);end=np.asarray(end,float)
    P=np.asarray(pose['center_m'],float);n=np.asarray(pose['axis'],float)
    model=c['observed_grasp_clearance'].estimator.models[pose['part']]
    world=model@axis_frame(n).T+P
    rotvec=log(end[:3,:3]@start[:3,:3].T);angle=float(np.linalg.norm(rotvec));count=max(1,int(np.ceil(angle/np.radians(1.))))
    widths=np.atleast_1d(np.asarray(opening,float));opening=float(widths.max())
    radius=float(np.max(np.linalg.norm(vertices+np.array([0.,opening/2,-.045]),axis=1)))
    margin=.00055+radius*(1-np.cos(angle/(2*count)))
    minimum=float('inf')
    for index in range(count):
        poses=[]
        for u in [index/count,(index+1)/count]:
            R=exp(rotvec*u)@start[:3,:3];p=start[:3,3]*(1-u)+end[:3,3]*u;poses.append((R,p))
        for sign in [-1.,1.]:
            locals_=[vertices*np.array([sign,sign,1.])+np.array([0.,sign*gap/2,-.045])for gap in widths]
            swept=np.concatenate([local@R.T+p for R,p in poses for local in locals_]);planes=ConvexHull(swept,qhull_options='QJ').equations
            possible=np.all(world>=swept.min(0)-margin,axis=1)&np.all(world<=swept.max(0)+margin,axis=1)
            if not possible.any():continue
            distances=(world[possible]@planes[:,:3].T+planes[:,3]).max(1);clearance=float(distances.min());minimum=min(minimum,clearance)
            if clearance<=margin:return False,dict(minimum_selected_CAD_swept_hull_clearance_m=clearance,required_noncontact_margin_m=margin,opening_m=float(opening),jaw_sign=sign,uses_free_body_poses=False)
    return True,dict(minimum_selected_CAD_swept_hull_clearance_m=minimum,required_noncontact_margin_m=margin,opening_m=float(opening),uses_free_body_poses=False)


def outer_closure_clear(c,pose,H,points,opening,close_width,joint_positions=None,minimum_gap=None):
    """Same native aperture sweep before route selection and before closing.

    The selected measured CAD shell retains the existing contact exemption;
    all other observed stock and fixed geometry remain obstacles. No motion,
    gripper command, class/instance shortcut or free-body query is used.
    """
    lo=min(float(close_width),float(opening) if minimum_gap is None else float(minimum_gap));hi=float(opening)
    gaps=np.linspace(lo,hi,max(2,int(np.ceil(abs(hi-close_width)/.001))+1))
    for gap in gaps:
        if joint_positions is None:
            clear,why=c['checker'].hand_clear(H,float(gap))
            if not clear:return False,'fixed geometry: '+str(why)
        else:
            for q in joint_positions:
                clear,why=c['checker'].clear(q,float(gap))
                if not clear:return False,'fixed geometry: '+str(why)
    stock=c['observed_grasp_clearance']
    if not stock.palm_clear(points,pose,H,H):return False,'observed other stock at palm'
    for gap in gaps:
        if not stock.clear(points,pose,H,H,float(gap)):return False,'observed other stock at native fingers, aperture='+str(float(gap))
    return True,None


def execute(c):
    # Serial composition holds the other arm stationary. Snapshot its actual
    # encoders/native aperture rather than relying on stale scene query actors.
    # The original PhysX fixed-world queries and the 12N runtime abort remain.
    if c['config'].get('stability_serial') and not c['config'].get('fixed_supply_paired_overlap') and 'checkers' in c and 'left_ids' in c:
        measured=c['art'].get_dof_positions()[0]
        with c['checker'].other_arm_at(c['checkers']['left'],measured[c['left_ids']].copy(),float(measured[c['left_fi']].sum())):
            return _execute(c)
    return _execute(c)


def _execute(c):
    arm=c['arm'];art=c['art'];cmd=c['cmd'];ids=c['ids'];fi=c['fi'];dt=c['dt'];vision=c['vision'];checker=c['checker'];phase='initial';events=[];pickup_motion=[];lift_motion=[]
    inner_bore_mode=bool(c['config'].get('inner_only_pickup') or c['config'].get('native_inner_bore_grasp'))
    fixed_supply=bool(c['config'].get('fixed_supply_mode'))
    # The fixed shelf has no enclosing basket lip. Its calibrated original
    # support plane replaces that old extraction boundary; ROI height is only
    # camera cropping, not a physical obstacle. All native/world/stock/pair
    # guards still certify the actual breakaway and loaded lift below.
    extraction_boundary_z=float(np.asarray(c['calibration']['floor_origin_world_m'])[2]) if fixed_supply else 1.210918206667893
    if not np.isfinite(extraction_boundary_z):raise ValueError('Invalid calibrated extraction support plane')
    def record(event):
        events.append(event)
        value=dict(event,pick_part=c['config'].get('pick_part'))
        encoded=json.dumps(value,default=lambda v:v.tolist() if hasattr(v,'tolist') else str(v))
        with (vision.out/'pickup-events.jsonl').open('a') as stream:stream.write(encoded+'\n')
        print('SCATTERED_PICK_EVENT',encoded,flush=True)
    def step():c['step'](phase)
    def wait(t,label):
        nonlocal phase
        phase=label
        for _ in range(int(np.ceil(t/dt))):step()
    def measured_pose_supported(result,mode):
        pose=result.get('pose',{})
        return bool(not result.get('error') and (pose.get('front_back_from_RGBD') or (mode=='pregrasp_ring' and c['config'].get('allow_partial_preclose_tracking') and pose.get('fresh_partial_tracking'))))
    def observe(mode,part=None,G=None,allow_missing=False):
        capture_gap=float(art.get_dof_positions()[0,fi].sum())
        e=dict(mode=mode,part=part)
        allow_partial=bool(mode=='pregrasp_ring' and c['config'].get('allow_partial_preclose_tracking'))
        if allow_partial:e['allow_partial_preclose_tracking']=True
        if G is not None:
            e['tracked_grasp']=G.tolist();camera_H=arm.fk(art.get_dof_positions()[0,ids]);short=camera_H[:3,0].copy();short[2]=0.;short/=max(np.linalg.norm(short),1e-8)
            if mode=='pregrasp_ring' and part=='ring2':
                # A long oblique ray to stock below the bin rim hits its wall.
                # These nearer overhead rays were validated on the same pixels
                # and unchanged hand pose that defeated the wider views.
                predicted=camera_H@G;lip=float(np.asarray(c['calibration']['outer_bounds_world_m'])[1,2]);height=max(.30,lip+.005-predicted[2,3]);e['camera_eye_offset']=(short*.06+[0,0,height]).tolist()
            else:e['camera_eye_offset']=(short*(.38 if mode=='pregrasp_ring' else .35)+np.array([0,0,.40 if mode=='pregrasp_ring' else .25])).tolist()
            if mode=='held':
                # Look below the palm while staying above the known bin lip.
                # Aim from the preceding image estimate plus hand encoders.
                predicted=arm.fk(art.get_dof_positions()[0,ids])@G;lip=float(np.asarray(c['calibration']['outer_bounds_world_m'])[1,2]);height=max(-.015,lip+.005-predicted[2,3]);e['camera_eye_offset']=(-short*.25+[0,0,height]).tolist();e['require_annular_face']=True
        future=c.pop('prefetched_initial',None) if mode=='initial' else None
        if future is None:future=vision.request(e,arm.fk(art.get_dof_positions()[0,ids]))
        else:e=vision.jobs[next(i for i,(_,f) in enumerate(vision.jobs) if f is future)][0]
        deadline=time.perf_counter()+3
        while not future.done():
            vision.poll();step()
            if time.perf_counter()>deadline:raise RuntimeError('Fresh scattered perception timeout')
        v=future.result();e['delivered_wall_s']=time.perf_counter()-c['started']
        if mode=='initial' and c['config'].get('initial_pickup_multiview'):
            measurements=[]
            if not v['result'].get('error'):measurements.append(v)
            previous=vision.calibration.get('initial_eye_offset_m',[.40,-.12,.65])
            extra_offsets=c['config'].get('initial_pickup_extra_eye_offsets_m',[])
            if len(extra_offsets)>4:raise ValueError('At most four extra calibrated-bin camera views are supported')
            for offset in extra_offsets:
                value=np.asarray(offset,float)
                if value.shape!=(3,) or not np.isfinite(value).all() or value[2]<=.05:
                    raise ValueError('Extra initial camera offsets must be finite XYZ vectors above the calibrated bin floor')
            # Extra fixed bin views can expose stock behind the withdrawn
            # hand after a physical hook. Their aim uses calibration only;
            # inference keeps the same direct-annulus and free-space gates.
            for offset in [[.35,-.25,.50],[-.30,.25,.50]]+list(extra_offsets):
                # Each request immediately serializes this calibrated view.
                # Its aim is the fixed basket region, never a loose actor.
                vision.calibration['initial_eye_offset_m']=offset
                event=dict(mode='initial',part=None,pickup_camera_eye_offset_m=offset)
                try:extra=vision.request(event,arm.fk(art.get_dof_positions()[0,ids]))
                finally:vision.calibration['initial_eye_offset_m']=previous
                deadline=time.perf_counter()+3
                while not extra.done():
                    vision.poll();step()
                    if time.perf_counter()>deadline:raise RuntimeError('Alternate initial RGB-D camera timeout')
                value=extra.result();event['delivered_wall_s']=time.perf_counter()-c['started']
                if not value['result'].get('error'):measurements.append(value)
            if measurements:
                instances=[]
                for value in measurements:
                    for pose in value['result'].get('instances',[]):
                        match=next((j for j,old in enumerate(instances) if old['part']==pose['part'] and np.linalg.norm(np.asarray(old['center_m'])-pose['center_m'])<.012 and np.asarray(old['axis'])@pose['axis']>.98),None)
                        if match is None:instances.append(pose)
                        elif (pose.get('visible_arc_coverage_deg',0),pose.get('depth_points',0),-pose.get('rms_m',1.))>(instances[match].get('visible_arc_coverage_deg',0),instances[match].get('depth_points',0),-instances[match].get('rms_m',1.)):instances[match]=pose
                v=dict(measurements[0]);v['result']=dict(v['result'],instances=instances,initial_pickup_camera_count=len(measurements),merged_from_camera_images_only=True);v['supporting_packets']=[value['packet'] for value in measurements if 'packet'in value]
                record(dict(event='initial_multiview_ring_candidates',camera_count=len(measurements),instances_by_size={size:sum(p['part']==size for p in instances) for size in ['ring1','ring2']},wall_s=time.perf_counter()-c['started']))
        if G is not None and mode in ['held','pregrasp_ring'] and c['config'].get('pickup_multiview'):
            # Every view is aimed using the last image estimate and encoders.
            # The palm can hide a top view, so also inspect both cap sides.
            measured=arm.fk(art.get_dof_positions()[0,ids]);normal=(measured@G)[:3,2]
            good=[];consistent=None;last_success_event=None
            if measured_pose_supported(v['result'],mode):good.append(v);last_success_event=e
            if mode=='pregrasp_ring':
                # Signed CAD face normals can point below the bin floor.
                # Use above-bin oblique views along the narrow palm axis.
                short=measured[:3,0].copy();short[2]=0.;short/=max(np.linalg.norm(short),1e-8)
                other=np.cross([0.,0.,1.],short);predicted=measured@G;lip=float(np.asarray(c['calibration']['outer_bounds_world_m'])[1,2]);near_height=max(.25,lip+.005-predicted[2,3])
                offsets=([other*.12+[0,0,near_height],-other*.12+[0,0,near_height]] if part=='ring2' else [])+[short*.30+[0,0,.30],-short*.30+[0,0,.30],np.array([.28,.20,.32]),np.array([-.28,-.20,.32])]
                if c['config'].get('preclose_occlusion_max_age_s') is not None and not c['config'].get('require_fresh_preclose_rgbd'):offsets=offsets[:1]
            else:
                short=measured[:3,0].copy();short[2]=0.;short/=max(np.linalg.norm(short),1e-8);other=np.cross([0.,0.,1.],short);predicted=measured@G;lip=float(np.asarray(c['calibration']['outer_bounds_world_m'])[1,2]);height=max(-.015,lip+.005-predicted[2,3]);low=np.array([0,0,height])
                offsets=[other*.25+low,-other*.25+low,short*.25+low,normal*.32+np.array([.14,-.14,.16]),-normal*.32+np.array([-.14,.14,.16]),np.array([.0,.0,.48]),np.array([-.30,-.24,.32])]
            for offset in offsets:
                event=dict(mode=mode,part=part,tracked_grasp=G.tolist(),camera_eye_offset=offset.tolist(),require_annular_face=not allow_partial,allow_partial_preclose_tracking=allow_partial)
                extra=vision.request(event,arm.fk(art.get_dof_positions()[0,ids]));deadline=time.perf_counter()+3
                while not extra.done():
                    vision.poll();step()
                    if time.perf_counter()>deadline:raise RuntimeError('Alternate RGB-D pickup camera timeout')
                candidate=extra.result();event['delivered_wall_s']=time.perf_counter()-c['started']
                if not measured_pose_supported(candidate['result'],mode):continue
                good.append(candidate);last_success_event=event
                if len(good)>=2:
                    a,b=[entry['result']['pose'] for entry in good[-2:]]
                    if np.linalg.norm(np.asarray(a['center_m'])-b['center_m'])<.003 and np.asarray(a['axis'])@b['axis']>np.cos(.06):consistent=good[-2:];break
            if mode=='held' and good and consistent is None:
                # A formerly visible ring may have dropped during the other
                # exposures. Repeat the last successful view NOW rather than
                # returning its old pose after several later camera failures.
                last=good[-1];pose=last['result']['pose'];last_G=np.linalg.inv(last['H'])@frame(pose['center_m'],axis_frame(pose['axis']));event=dict(last_success_event,tracked_grasp=last_G.tolist(),require_annular_face=True);event.pop('delivered_wall_s',None)
                repeated=vision.request(event,arm.fk(art.get_dof_positions()[0,ids]));deadline=time.perf_counter()+3
                while not repeated.done():
                    vision.poll();step()
                    if time.perf_counter()>deadline:raise RuntimeError('Held-ring repeat camera timeout')
                value=repeated.result();event['delivered_wall_s']=time.perf_counter()-c['started'];fresh=value['result'].get('pose',{})
                if not value['result'].get('error') and fresh.get('front_back_from_RGBD'):
                    fresh_G=np.linalg.inv(value['H'])@frame(fresh['center_m'],axis_frame(fresh['axis']))
                    if np.linalg.norm(fresh_G[:3,3]-last_G[:3,3])<.0015 and fresh_G[:3,2]@last_G[:3,2]>np.cos(.04):consistent=[last,value]
                record(dict(event='held_camera_fresh_repeat',confirmed=consistent is not None,wall_s=time.perf_counter()-c['started']))
            if good and (mode!='held' or consistent is not None):
                # Use the latest directly measured pose, never the oldest
                # low-residual image while the physical object keeps moving.
                v=(consistent or good)[-1]
            elif c['config'].get('require_fresh_preclose_rgbd') or mode=='held':
                v=dict(result=dict(error='No directly visible annular face from pickup camera views'))
        if v['result'].get('error') and not allow_missing:raise RuntimeError(v['result']['error'])
        v['capture_native_hand_opening_m']=capture_gap
        return v
    def move(qs,t,label,width=None):
        nonlocal phase
        verified_cubic=bool(getattr(qs,'verified_cubic',False) and c['config'].get('gpu_route_proposals') and c['config'].get('stability_serial'))
        qs=np.array(qs);qs[0]=cmd[0,ids];start=cmd[0,fi].copy();target=start if width is None else np.full(2,width/2)
        if label.startswith(('loose_ring_descend','loose_ring_observed_grasp_adjust')):pickup_motion.append(qs.copy())
        if label in ['loose_ring_lift','loose_ring_lift_breakaway','loose_ring_RGBD_lift_clearance_repair']:lift_motion.append(qs.copy())
        optimized=c['config'].get('optimized_motion_timing',False)
        if optimized and label in ['loose_ring_pregrasp','loose_ring_lift','loose_ring_RGBD_lift_clearance_repair']:t=.05
        scale=max(1.,float(c['config'].get('stability_time_scale',1.)))
        if label=='loose_ring_pregrasp' and len(qs)>2 and not verified_cubic:
            q,h,timing=retime_safe_polyline(qs,dt,start,target,scale=scale,velocity_fraction=.91,jerk=4000.)
        else:
            q,h,timing=retime(qs,t,dt,start,target,velocity_fraction=.91,jerk=4000.,optimize=optimized)
            if scale>1:q,h,timing=retime(qs,timing['duration_s']*scale,dt,start,target,velocity_fraction=.91,jerk=4000.,optimize=False)
        count=len(q)-1
        if label=='loose_ring_pregrasp' and c['config'].get('stability_serial'):
            # Check the geometry actually issued, including the aperture ramp;
            # a route checked at only its final width is insufficient near the
            # other stationary Hand. This is robot-encoder evidence only.
            measured_gap=float(art.get_dof_positions()[0,fi].sum());last_checked=None
            for index,row in enumerate(q):
                if last_checked is not None and index!=len(q)-1 and np.max(np.abs(row-last_checked))<.003:continue
                commanded_gap=float((start+(target-start)*h[index]).sum())
                for gap in {measured_gap,commanded_gap,float(target.sum())}:
                    clear,why=checker.clear(row,gap)
                    if not clear:raise RuntimeError('Issued pickup pregrasp curve blocked by fixed/predictive other-arm geometry: '+str(why))
                last_checked=row
        if c['config'].get('verify_selected_outer_approach') and part=='ring2' and grasp_type=='outer' and label.startswith(('loose_ring_descend','loose_ring_observed_grasp_adjust')):
            measured_pose=dict(part=part,center_m=P.tolist(),axis=n.tolist());measured_gap=float(art.get_dof_positions()[0,fi].sum());command_gap=float(start.sum())
            opening_range=[min(measured_gap,command_gap,float(target.sum())),max(measured_gap,command_gap,float(target.sum()))]
            previous=arm.fk(art.get_dof_positions()[0,ids])
            for issued in q:
                current=arm.fk(issued);clear,proof=selected_outer_approach_clear(c,measured_pose,previous,current,opening_range)
                if not clear:raise RuntimeError('Selected RGB-D ring blocks exact issued native finger approach: '+str(proof))
                previous=current
        phase=label
        issued=np.c_[q[1:],start+(target-start)*h[1:,None]]
        before=c.get('_before_right_pick_rows');after=c.get('_after_right_pick_rows')
        if before is not None:before(issued,label)
        try:
            for row in issued:cmd[0,ids]=row[:7];cmd[0,fi]=row[7:];step()
        finally:
            if after is not None:after()
        record(dict(event=label,wall_s=time.perf_counter()-c['started'],motion_s=count*dt,timing=timing));print('SCATTERED_PICK',label,events[-1]['wall_s'],flush=True)
    def line(target,t,label,width=None):
        H=arm.fk(cmd[0,ids]);qs=[cmd[0,ids].copy()]
        if c['config'].get('verify_selected_outer_approach') and part=='ring2' and grasp_type=='outer' and label.startswith(('loose_ring_descend','loose_ring_observed_grasp_adjust')):
            measured_pose=dict(part=part,center_m=P.tolist(),axis=n.tolist())
            measured_H=arm.fk(art.get_dof_positions()[0,ids]);gap=max(float(cmd[0,fi].sum()),float(art.get_dof_positions()[0,fi].sum()))
            for start_H in [H,measured_H]:
                clear,proof=selected_outer_approach_clear(c,measured_pose,start_H,target,gap)
                if not clear:raise RuntimeError('Selected RGB-D ring blocks fresh open native finger approach: '+str(proof))
        if label=='loose_ring_descend' and c['config'].get('gentle_descent') and c['config'].get('pick_part')=='ring2':
            distance=float(np.linalg.norm(target[:3,3]-H[:3,3]));slow=min(.025,distance)
            if distance>slow+.001:
                from isaac_kinematics import exp
                u=1-slow/distance;via=frame(H[:3,3]*(1-u)+target[:3,3]*u,exp(log(target[:3,:3]@H[:3,:3].T)*u)@H[:3,:3]);line(via,.05,label+'_free',width)
            return line(target,1.875*slow/float(c['config'].get('descent_contact_speed_m_s',.030)),label+'_contact',width)
        for u in np.linspace(0,1,5)[1:]:
            from isaac_kinematics import exp
            R=exp(log(target[:3,:3]@H[:3,:3].T)*u)@H[:3,:3]
            waypoint_position=H[:3,3]*(1-u)+target[:3,3]*u;q=arm.solve(waypoint_position,R,qs[-1])
            if q is None:raise RuntimeError('No bin line IK '+label)
            qs.append(q)
        if c['config'].get('pickup_multiview') or c['config'].get('pickup_diagonal_offsets_m'):
            count=max(33,int(np.ceil(np.linalg.norm(np.diff(qs,axis=0),axis=1).sum()/.003))+1)
            curve=CubicSpline(np.linspace(0,1,len(qs)),qs)(np.linspace(0,1,count));opening=max(float(cmd[0,fi].sum()),float(art.get_dof_positions()[0,fi].sum()))
            clear,why=checker.path(curve,opening)
            if not clear:raise RuntimeError('Fresh RGB-D pickup Cartesian path blocked: '+str(why))
        move(qs,t,label,width)
    def stock_points(value,packet=None):
        stock=c['observed_grasp_clearance'];points=stock.points(value['packet']if packet is None else packet,c['calibration'])
        # The capture TCP is actual encoder FK, already part of the RGB-D
        # observation protocol. No scene actor pose is available to this mask.
        H=np.asarray(value.get('H',arm.fk(art.get_dof_positions()[0,ids])))
        gap=float(value.get('capture_native_hand_opening_m',art.get_dof_positions()[0,fi].sum()))
        filtered=stock.remove_known_hand_surface(points,H,gap)
        if stock.last_hand_surface_exclusion['surface_matches']:record(dict(event='RGBD_known_hand_surface_exclusion',**stock.last_hand_surface_exclusion,wall_s=time.perf_counter()-c['started']))
        return filtered

    initial_value=observe('initial');initial=initial_value['result'];observed_points=np.concatenate([stock_points(initial_value,packet) for packet in initial_value.get('supporting_packets',[initial_value['packet']])]) if c.get('observed_grasp_clearance') else None;part=c['config'].get('pick_part','ring1');candidates=[p for p in initial['instances'] if p['part']==part and abs(p['axis'][2])>.35 and p.get('front_back_orientation_margin_m',0)>.0005]
    if c['config'].get('require_front_up_pickup'):candidates=[p for p in candidates if p['axis'][2]>.35]
    avoid=c['config'].get('pickup_avoid_centers',[])
    if avoid:
        radius=float(c['config'].get('pickup_avoid_radius_m',.025));filtered=[p for p in candidates if all(np.linalg.norm(np.asarray(p['center_m'])-old)>radius for old in np.asarray(avoid))]
        if filtered:candidates=filtered
    if not candidates:raise RuntimeError('No visible accessible measurable '+part+'')
    if observed_points is not None:checker.gpu_rgbd_stock_points=observed_points.copy()
    pickup_seed=np.array(c['pickup_seed']);postures=[(yaw,pickup_seed.copy(),None) for yaw in np.arange(0,2*np.pi,np.pi/8)]+[(yaw,pickup_seed.copy(),0.) for yaw in [0,np.pi/2,-np.pi/2,np.pi,np.pi/4,-np.pi/4,3*np.pi/4,-3*np.pi/4]]
    for elbow_delta in [-.50,.50]:
        seed=pickup_seed.copy();seed[2]+=elbow_delta
        postures.extend((yaw,seed.copy(),0.) for yaw in [0,np.pi/2,-np.pi/2,np.pi])
    tilts=[np.radians(25),-np.radians(25),np.radians(45),-np.radians(45),np.radians(60),-np.radians(60)]
    if c['config'].get('allow_side_entry'):tilts+=[np.radians(75),-np.radians(75),np.pi/2,-np.pi/2]
    for tilt in tilts:
        postures.extend((yaw,pickup_seed.copy(),tilt) for yaw in [0,np.pi/2,-np.pi/2,np.pi])
    diagonal_offsets=[]
    for offset in c['config'].get('pickup_diagonal_offsets_m',[]):
        value=np.asarray(offset,float)
        if value.shape!=(3,) or not np.isfinite(value).all() or abs(value[2])>1e-9 or not .005<=np.linalg.norm(value)<=.12:
            raise ValueError('Diagonal pickup offsets must be horizontal finite vectors of5--120mm')
        if not any(np.linalg.norm(value-old)<1e-6 for old in diagonal_offsets):diagonal_offsets.append(value)
    if len(diagonal_offsets)>4:raise ValueError('At most four diagonal pickup offsets are supported')
    def temporary_payload(grasp):
        from contextlib import contextmanager
        @contextmanager
        def scope():
            template=c.get('part_collision_templates',{}).get(part)
            if template is None:raise RuntimeError('Diagonal pickup lift needs the fixed class payload collision template')
            saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative)
            try:
                checker.attach_payload(template,frame([0,0,.1034],np.eye(3))@grasp)
                yield
            finally:checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved
        return scope()
    planning_started=time.perf_counter();rejections={};checks=0
    def reject(why):
        key=str(why);rejections[key]=rejections.get(key,0)+1
    candidates.sort(key=lambda p:(bool(c['config'].get('prefer_transfer_compatible')) and p['axis'][2]<0,np.linalg.norm(np.array(p['center_m'])[:2]-np.array([.49,-.725]))-3.0*(p['center_m'][2]-1.10)));routes=[]
    # Try a centered, axis-aligned grasp before the deeper recovery poses.
    postures.sort(key=lambda row:0 if row[2]==0. else 1 if row[2] is None else 2)
    modes=(['inner','outer'] if part=='ring2' and c['config'].get('prefer_internal_small_grasp') else ['outer','inner']) if c['config'].get('allow_internal_grasp') else ['outer']
    if c['config'].get('inner_only_pickup'):modes=['inner']
    elif c['config'].get('rim_only_pickup'):modes=['rim']
    elif c['config'].get('allow_local_rim_grasp'):
        modes=(['rim']+modes) if c['config'].get('prefer_local_rim_grasp',True) else (modes+['rim'])
    # Offset along the finger pad's width, perpendicular to closing. This
    # grasps a chord of the rim and moves the bulky palm away from a bin wall.
    # The observed grasp transform is passed to transfer/insertion unchanged.
    offsets=[0.,.012,.024] if c['config'].get('allow_chord_grasp',True) else [0.]
    for grasp_type,depth_adjust,chord_offset,p in [(mode,d,x,p) for mode in modes for p in candidates for d in ([0.,.002,.004,.006] if mode=='rim' else [.0045,.0025,.006] if mode=='inner' and inner_bore_mode else [0.] if mode=='inner' else [-.004,-.002,0.,.002,.004] if part=='ring2' and c['config'].get('stable_small_grasp') else [.002,.004,0.,-.004] if part=='ring2' else ([0.,-.001,-.002,-.003,-.004] if c['config'].get('require_broad_large_donor_support') else [0.,-.004])) for x in ([0.] if mode in ['inner','rim'] else offsets)]:
        if c['config'].get('fixed_supply_mode') and (grasp_type!='outer' or abs(depth_adjust)>1e-9 or abs(chord_offset)>1e-9):continue
        P=np.array(p['center_m']);n=np.array(p['axis']);R0=axis_frame(-n*np.sign(n[2]))
        current_rotation=arm.fk(cmd[0,ids])[:3,:3]
        def posture_cost(posture):
            yaw,_,tilt=posture[:3];roll=float(posture[3]) if len(posture)>3 else 0.
            if tilt is None:
                z=np.array([0.,0.,-1.]);y=np.array([np.cos(yaw),np.sin(yaw),0.])
                if abs(n@y)>.25:return np.inf
                rotation=np.column_stack([np.cross(y,z),y,z])
            else:
                rotation=R0@np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1.]])@np.array([[1.,0,0],[0,np.cos(roll),-np.sin(roll)],[0,np.sin(roll),np.cos(roll)]])@np.array([[np.cos(tilt),0,np.sin(tilt)],[0,1,0],[-np.sin(tilt),0,np.cos(tilt)]])
            return float(np.linalg.norm(log(rotation@current_rotation.T)))
        local_postures=postures
        if grasp_type=='rim':
            # Tilt about closing Y preserves radial inner/outer-wall contact
            # while giving the palm a different route past the basket lip.
            local_postures=[(yaw,pickup_seed.copy(),angle) for yaw in np.arange(0,2*np.pi,np.pi/8) for angle in np.radians([0,15,-15,25,-25])]
        elif grasp_type=='inner' and inner_bore_mode:
            rolls=np.radians(c['config'].get('inner_bore_x_tilt_candidates_deg',[0.]));yaws=list(np.arange(0,2*np.pi,np.pi/8))
            if np.any(abs(rolls)>1e-8):
                # The palm is 205 mm along closing Y. Include exact calibrated
                # bin axes so the yaw grid does not miss its long opening.
                bin_rotation=np.asarray(c['calibration']['basket_to_world_R'])
                for direction in [bin_rotation[:,0],-bin_rotation[:,0],bin_rotation[:,1],-bin_rotation[:,1]]:
                    local=R0.T@direction;yaws.append(float(np.arctan2(-local[0],local[1])))
            local_postures=[(yaw,pickup_seed.copy(),angle,roll) for yaw in yaws for angle in np.radians([0,15,-15,25,-25]) for roll in rolls]
        elif part=='ring2' and c['config'].get('stable_small_grasp') and np.linalg.norm(n[:2])>.005:
            tangent=float(np.arctan2(n[0],-n[1]))
            # Keep the closing direction exactly in the ring plane. A coarse
            # yaw grid otherwise contacts opposite axial rim corners first.
            local_postures=[(tangent,pickup_seed.copy(),None),(tangent+np.pi,pickup_seed.copy(),None)]+[r for r in postures if r[2] is not None]
        if part=='ring1' and grasp_type=='outer' and c['config'].get('require_broad_large_donor_support') and np.linalg.norm(n[:2])>.005:
            # Add exact horizontal ring-plane closing axes. The coarse yaw
            # grid can otherwise load opposite cap corners on a leaned ring.
            # Every original posture and every native/fixed/cloud/IK gate
            # remains present; this adds two image-derived proposals only.
            tangent=float(np.arctan2(n[0],-n[1]))
            local_postures=list(local_postures)+[(tangent,pickup_seed.copy(),None),(tangent+np.pi,pickup_seed.copy(),None)]
        def posture_priority(row):
            category=(0 if row[2]==0. else 1 if row[2] is None else 2) if part=='ring2' else 0
            carry_preference=0
            if part=='ring2' and grasp_type=='outer' and n[2]<0. and c['config'].get('prefer_right_carry_small_posture',False):
                yaw,_,tilt=row[:3];roll=float(row[3]) if len(row)>3 else 0.
                # Signed ring axis in the proposed TOOL frame, derived only
                # from its RGB-D axis and candidate rotation. Mirrored native
                # jaws are both legal; try the carry-ready orientation first.
                axis_x=(-np.sin(yaw)*n[0]+np.cos(yaw)*n[1]) if tilt is None else -np.sin(tilt)*np.cos(roll)
                carry_preference=0 if axis_x<0. else 1
            return category,carry_preference,posture_cost(row)
        extra_open=bool(part=='ring2' and grasp_type=='outer' and c['config'].get('verify_selected_outer_approach'))
        if c['config'].get('fixed_supply_mode'):local_postures=[(-np.pi/2,pickup_seed.copy(),None),(np.pi/2,pickup_seed.copy(),None)]
        for posture,outer_opening in [(row,width) for row in sorted(local_postures,key=posture_priority) for width in ([.068,.080] if extra_open else [None])]:
            yaw,ik_seed,tilt=posture[:3];roll=float(posture[3]) if len(posture)>3 else 0.
            if grasp_type=='inner' and not inner_bore_mode and tilt!=0.:continue
            if grasp_type=='outer' and chord_offset and tilt!=0.:continue
            opening=outer_opening if outer_opening is not None else .001 if grasp_type=='inner' else min(.08,(.075 if part=='ring1' else .062)+.006)
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
                R=R@np.array([[1.,0,0],[0,np.cos(roll),-np.sin(roll)],[0,np.sin(roll),np.cos(roll)]])
                R=R@np.array([[np.cos(tilt),0,np.sin(tilt)],[0,1,0],[-np.sin(tilt),0,np.cos(tilt)]])
                half=.00775 if part=='ring1' else .007;contact_offset=max(.0028,.0089*np.cos(tilt)+.00875*abs(np.sin(tilt))-half+.0012)
            contact_offset=.0045 if grasp_type=='inner' else max(-.003,contact_offset+depth_adjust)
            if c['config'].get('fixed_supply_mode') and grasp_type=='outer':contact_offset=.0045 if part=='ring2' else .005
            if grasp_type=='outer' and chord_offset:
                toward=np.asarray(c['calibration']['floor_origin_world_m'])-P
                tcp_offset=R[:,0]*np.sign(toward@R[:,0])*chord_offset
                nominal=.075 if part=='ring1' else .062
                near_pad_offset=max(0.,chord_offset-.0105)
                grip_width=2*np.sqrt(max(0.,(nominal/2)**2-near_pad_offset**2))
            rim_details=None
            H=frame(P+n*np.sign(n[2])*contact_offset+tcp_offset,R)
            if grasp_type=='rim':
                from local_rim_grasp import candidate as rim_candidate
                rim_details=rim_candidate(c,part,p,R[:,1],axial_offset_m=depth_adjust,tilt_rad=float(tilt))
                if rim_details is None:reject('No two-pad native finger contact on this local rim');continue
                H=np.asarray(rim_details['tcp_world']).copy();R=H[:3,:3];opening=float(rim_details['opening_m']);grip_width=float(rim_details['contact_width_m']);contact_offset=depth_adjust;tcp_offset=H[:3,3]-P-n*np.sign(n[2])*contact_offset
            elif grasp_type=='inner' and inner_bore_mode:
                from inner_bore_grasp import candidate as bore_candidate
                # Supply the in-plane closing direction; the helper applies
                # the recorded roll exactly once when constructing the TCP.
                closing=R0@np.array([-np.sin(yaw),np.cos(yaw),0.])
                rim_details=bore_candidate(c,part,p,closing,axial_offset_m=depth_adjust,tilt_rad=float(tilt),roll_rad=roll)
                if rim_details is None:reject('No opposing native finger contact on the inner bore');continue
                H=np.asarray(rim_details['tcp_world']).copy();R=H[:3,:3];opening=float(rim_details['opening_m']);grip_width=float(rim_details['contact_width_m']);contact_offset=depth_adjust;tcp_offset=H[:3,3]-P-n*np.sign(n[2])*contact_offset
            pre=H.copy();pre[2,3]=max(pre[2,3]+.08,float(c['config'].get('pregrasp_floor_small_m',c['config'].get('pregrasp_floor_m',1.235)) if part=='ring2' else c['config'].get('pregrasp_floor_m',1.235)));
            if c['config'].get('allow_side_entry') and abs(R[2,2])<.45:
                pre=H.copy();pre[:3,3]-=R[:,2]*.10
            fixed_clear,fixed_why=checker.hand_clear(H,opening)
            if not fixed_clear:reject('fixed hand pose: '+str(fixed_why));continue
            if grasp_type=='outer' and c.get('grasp_pad_aperture'):
                predicted_width=c['grasp_pad_aperture'](p,H)
                if predicted_width is None:reject('Ring CAD does not reach both fingertip pads');continue
                grip_width=predicted_width
            transfer_pose=dict(p,planned_grip_width_m=grip_width,grasp_type=grasp_type)
            if rim_details is not None:transfer_pose['release_width_m']=float(rim_details['release_width_m'])
            defer_transfer=bool(c['config'].get('defer_handover_until_visible_clearance'))
            transfer_feasible=None
            if not defer_transfer and c.get('handover_feasible_grasp') and not c['handover_feasible_grasp'](transfer_pose,H):
                reject('native hand pair cannot fit this grasp at handover');continue
            original_pre=pre.copy();lift_points=observed_points
            if (diagonal_offsets or fixed_supply) and observed_points is not None:
                # Remove only this measured class-CAD surface shell, exactly
                # as the existing finger approach filter does. The old stock
                # image must not become a ghost obstacle after extraction.
                distance,_=c['observed_grasp_clearance'].estimator.trees[part].query((observed_points-P)@axis_frame(n))
                lift_points=observed_points[distance>.0025]
            for approach_offset in [np.zeros(3)]+(diagonal_offsets if grasp_type=='outer' else []):
                pre=original_pre.copy();pre[:3,3]+=approach_offset
                diagonal_approach=bool(np.linalg.norm(approach_offset)>1e-8)
                approach_stages=[pre,H]
                if diagonal_approach and c['config'].get('preclose_occlusion_max_age_s') is not None:
                    via=H.copy();via[2,3]+=.015;approach_stages=[pre,via,H]
                if extra_open:
                    selected_clear=True
                    for a,b in zip(approach_stages[:-1],approach_stages[1:]):
                        clear,proof=selected_outer_approach_clear(c,p,a,b,opening)
                        if not clear:reject('selected ring blocks open native finger approach: '+str(proof));selected_clear=False;break
                    if not selected_clear:continue
                if diagonal_approach and any(not checker.hand_clear(frame(a[:3,3]*(1-u)+b[:3,3]*u,R),opening)[0]for a,b in zip(approach_stages[:-1],approach_stages[1:])for u in np.linspace(0,1,13)):
                    reject('diagonal approach fixed hand collision');continue
                loose_clear=True
                if observed_points is not None:
                    for approach_start,approach_end in zip(approach_stages[:-1],approach_stages[1:]):
                        if grasp_type=='outer' and (diagonal_approach or c['config'].get('avoid_other_stock_finger_contact')):
                            segment_clear=c['observed_grasp_clearance'].palm_clear(observed_points,p,approach_start,approach_end) and c['observed_grasp_clearance'].clear(observed_points,p,approach_start,approach_end,opening)
                        else:
                            segment_clear=c['observed_grasp_clearance'].palm_clear(observed_points,p,approach_start,approach_end) if c['config'].get('allow_fingertip_nudging') and rim_details is None else c['observed_grasp_clearance'].clear(observed_points,p,approach_start,approach_end,opening if rim_details is not None else .08)
                        loose_clear=loose_clear and segment_clear
                if not loose_clear:
                    print('SCATTERED_VISIBLE_CLUTTER_REJECT',part,yaw,None if tilt is None else float(tilt),flush=True);continue
                if extra_open and observed_points is not None:
                    # Strict other-stock closure admission belongs before
                    # IK/route search and any physical approach. A grasp that
                    # is open-clear can still sweep a neighbour while closing.
                    closure_clear,closure_why=outer_closure_clear(c,p,H,observed_points,opening,max(.001,grip_width-.003))
                    if not closure_clear:
                        reject('native closure aperture sweep: '+str(closure_why));break
                if defer_transfer:
                    # The receiving grasp depends on H, not on which empty
                    # approach reaches it. Avoid catalog search for proposals
                    # already rejected by the SAME observed-stock checks.
                    if transfer_feasible is None:
                        transfer_feasible=not c.get('handover_feasible_grasp') or bool(c['handover_feasible_grasp'](transfer_pose,H))
                    if not transfer_feasible:
                        reject('native hand pair cannot fit this grasp at handover');break
                def pregrasp_endpoints():
                    from fr3_gpu_ik import propose_ik
                    # GPU endpoints are original USD-FK/bounds checked.
                    # Each still traverses all original final/descent/lift/
                    # stock/self/other-arm/route/issued-curve admission below.
                    gpu=propose_ik(c,arm,pre[:3,3],R,cmd[0,ids],'loose_'+part+'_pregrasp')
                    gpu.sort(key=lambda q:np.max(abs(q-cmd[0,ids])/np.array([2.62,2.62,2.62,2.62,5.26,4.18,5.26])))
                    for q in gpu[:12]:yield q
                    # Only compute expensive original CPU catalogue when all
                    # GPU alternatives failed. Preserve original seed set,
                    # CPU sorting and12-branch budget, even GPU duplicates.
                    started=time.perf_counter();rng=np.random.default_rng(1127)
                    seed_pool=[ik_seed,cmd[0,ids]]+list(rng.uniform(arm.bounds[:,0],arm.bounds[:,1],size=(48,7)));solutions=[]
                    for seed in seed_pool:
                        q=arm.solve(pre[:3,3],R,seed)
                        if q is not None and all(np.linalg.norm(q-old)>.12 for old in solutions):solutions.append(q)
                    if not solutions:reject('pregrasp IK')
                    solutions.sort(key=lambda q:np.max(abs(q-cmd[0,ids])/np.array([2.62,2.62,2.62,2.62,5.26,4.18,5.26])))
                    print('GPU_PICKUP_CPU_FALLBACK',dict(part=part,gpu_branches=len(gpu),CPU_branches=len(solutions),CPU_s=time.perf_counter()-started),flush=True)
                    yield from solutions[:12]
                for preq in pregrasp_endpoints():
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
                    for segment_start,segment_end in zip(approach_stages[:-1],approach_stages[1:]):
                        for u in np.linspace(0,1,5)[1:]:
                            q=arm.solve(segment_start[:3,3]*(1-u)+segment_end[:3,3]*u,R,descent[-1])
                            if q is None:bad='incremental Cartesian IK';break
                            clear,why=checker.path([descent[-1],q],opening)
                            if not clear:bad=why;break
                            descent.append(q)
                        if bad is not None:break
                    if bad is not None:reject(str(bad));continue
                    D=.075 if part=='ring1' else .062;half=.00775 if part=='ring1' else .007;support=D/2*np.sqrt(max(0,1-n[2]**2))+half*abs(n[2]);lift_delta=max(.075,extraction_boundary_z+support+.010-P[2]);selected_lift=None
                    for lift_offset in [np.zeros(3)]+(diagonal_offsets if grasp_type=='outer' else []):
                        diagonal_lift=bool(np.linalg.norm(lift_offset)>1e-8);liftqs=[descent[-1]];bad=None;last_H=H
                        targets=[]
                        if diagonal_lift or fixed_supply:
                            breakaway=H.copy();breakaway[2,3]+=.015;targets.append(breakaway)
                        endpoint=H.copy();endpoint[:3,3]+=lift_offset+[0.,0.,lift_delta];targets.append(endpoint)
                        for segment_index,target_H in enumerate(targets):
                            # The initial15mm breaks the known floor support,
                            # using the unchanged baseline robot-only lift
                            # checks. Every subsequent diagonal sample carries
                            # the fixed class payload derived from RGB-D.
                            loaded=(diagonal_lift or fixed_supply) and segment_index>0
                            if loaded and lift_points is not None:
                                if not c['observed_grasp_clearance'].palm_clear(lift_points,p,last_H,target_H) or not c['observed_grasp_clearance'].clear(lift_points,p,last_H,target_H,grip_width):bad='diagonal lifted observed stock';break
                            from contextlib import nullcontext
                            with temporary_payload(np.linalg.inv(H)@frame(P,axis_frame(n))) if loaded else nullcontext():
                                for u in np.linspace(0,1,5)[1:]:
                                    target=last_H[:3,3]*(1-u)+target_H[:3,3]*u;q=arm.solve(target,R,liftqs[-1])
                                    if q is None:bad='lift IK';break
                                    if not checker.path([liftqs[-1],q],grip_width)[0]:bad='diagonal payload lift structure collision' if loaded else 'lift structure collision';break
                                    liftqs.append(q)
                            if bad is not None:break
                            last_H=target_H
                        if bad is None:selected_lift=lift_offset.copy();break
                        reject(str(bad))
                    if selected_lift is None:continue
                    route=None
                    route_opening=opening if rim_details is not None else .08
                    if checker.path([cmd[0,ids],preq],route_opening)[0]:route=[cmd[0,ids].copy(),preq]
                    else:
                        for via in c.get('empty_return_waypoints',[]):
                            if checker.path([cmd[0,ids],via,preq],route_opening)[0]:route=[cmd[0,ids].copy(),via,preq];break
                    if route is None:route=checker.route(cmd[0,ids],preq,route_opening,seconds=.15)
                    if route is not None:
                        selected=dict(p,pickup_approach_offset_world_m=approach_offset.tolist(),pickup_lift_offset_world_m=selected_lift.tolist(),pickup_lift_breakaway_m=.015 if np.linalg.norm(selected_lift)>1e-8 or fixed_supply else 0.,grasp_yaw_rad=float(yaw),grasp_tilt_rad=None if tilt is None else float(tilt),grasp_roll_rad=roll,grasp_type=grasp_type,planned_grip_width_m=grip_width,tcp_offset_m=tcp_offset.tolist())
                        if extra_open:selected['outer_empty_opening_m']=float(opening)
                        if rim_details is not None:selected.update(opening_m=opening,command_width_m=float(rim_details['command_width_m']),release_width_m=float(rim_details['release_width_m']),native_local_contact_model=True,rim_contact_diagnostics=rim_details.get('diagnostics',{}))
                        routes.append((np.linalg.norm(preq-cmd[0,ids]),selected,H,route,contact_offset));break
                if routes:break
            if routes:break
        if routes:break
    print('SCATTERED_PLANNING',part,'seconds',time.perf_counter()-planning_started,'rejections',rejections,flush=True)
    if not routes:raise RuntimeError('No structure-clear grasp among RGB-D instances')
    _,p,H,route,contact_offset=min(routes,key=lambda row:row[0])
    if c['config'].get('pickup_preview_only'):
        # Selection uses the same fresh RGB-D, native contact and live
        # collision/IK planning as execution. Stop before binding a target,
        # requesting pregrasp tracking, moving an arm or changing a grip.
        packets=initial_value.get('supporting_packets',[initial_value['packet']])
        camera_evidence=dict(initial_detection=initial,captured_wall_s=initial_value.get('captured_wall_s'),source='Fresh live initial RGB-D and fixed bin calibration',camera_views=[dict(T_world_camera=np.asarray(packet['T_cell_camera']).tolist(),depth_shape=list(packet['depth_m'].shape))for packet in packets],uses_simulator_body_poses=False)
        record(dict(event='pickup_route_preview_selected',part=part,selected_image_center_m=p['center_m'],grasp_type=p.get('grasp_type','outer'),robot_motion_executed=False,wall_s=time.perf_counter()-c['started']))
        return dict(state='RGBD_pickup_route_preview_only',part=part,selected_instance=p,planned_TCP_world=np.asarray(H).tolist(),planned_route=np.asarray(route).tolist(),contact_offset_m=float(contact_offset),camera_evidence=camera_evidence,events=events,planning_rejections=rejections,elapsed_s=time.perf_counter()-c['started'],uses_simulator_body_poses=False,robot_motion_executed=False,grip_command_changed=False,full_task_pass=False)
    grasp_type=p.get('grasp_type','outer');local_contact=bool(p.get('native_local_contact_model'));tcp_offset=np.asarray(p.get('tcp_offset_m',[0,0,0]));empty_width=float(p['opening_m']) if local_contact else .001 if grasp_type=='inner' else float(p.get('outer_empty_opening_m',min(.08,(.075 if part=='ring1' else .062)+.006)))
    def retry_pick(reason,loaded=False,avoid_radius_m=None):
        attempt=int(c['config'].get('pickup_retry_attempt',0));limit=int(c['config'].get('pickup_retry_limit',0))
        if attempt>=limit:raise RuntimeError(reason+'; image-based pickup recovery limit reached')
        record(dict(event='pickup_recovery',reason=reason,attempt=attempt,loaded=bool(loaded),wall_s=time.perf_counter()-c['started']))
        # Reverse the actual reachable lift rather than inventing an inward
        # set-down target that may be outside this arm's IK branch.
        def reverse_checked(path,label):
            reverse=path[::-1].copy();reverse[0]=cmd[0,ids]
            # Check the executed cubic, including between its original knots.
            count=max(33,int(np.ceil(np.linalg.norm(np.diff(reverse,axis=0),axis=1).sum()/.003))+1)
            curve=CubicSpline(np.linspace(0,1,len(reverse)),reverse)(np.linspace(0,1,count));width=max(float(cmd[0,fi].sum()),float(art.get_dof_positions()[0,fi].sum()))
            clear,why=checker.path(curve,width)
            if not clear:raise RuntimeError('Physical pickup recovery path blocked: '+str(why))
            move(reverse,.8,label)
        if loaded:
            # Gobs is the freshly measured grasp from the enclosing pickup.
            held=Gobs
            template=c.get('part_collision_templates',{}).get(part)
            if template is not None:checker.attach_payload(template,frame([0,0,.1034],np.eye(3))@held)
            try:
                for path in reversed(lift_motion):reverse_checked(path,'loose_ring_recovery_return_to_stock')
            finally:
                if template is not None:checker.detach_payload()
            if c.get('set_grip_hold'):c['set_grip_hold']('right',False)
            move([cmd[0,ids],cmd[0,ids]],.25,'loose_ring_recovery_release',empty_width)
            wait(.3,'loose_ring_recovery_stock_settle')
        for path in reversed(pickup_motion):reverse_checked(path,'loose_ring_recovery_withdraw')
        wait(.15,'loose_ring_recovery_reobserve')
        config=dict(c['config'],pickup_retry_attempt=attempt+1,pickup_avoid_centers=list(avoid)+[p['center_m']])
        if avoid_radius_m is not None:config['pickup_avoid_radius_m']=float(avoid_radius_m)
        retry=dict(c,config=config);retry.pop('prefetched_initial',None)
        result=execute(retry);result['events']=events+result['events'];return result
    def retry_uncertain_hold(reason,last_image_grasp):
        """Open only over the calibrated basket; never assume a payload pose."""
        attempt=int(c['config'].get('pickup_retry_attempt',0));limit=int(c['config'].get('pickup_retry_limit',0))
        if attempt>=limit:raise RuntimeError(reason+'; uncertain-hold bin recovery limit reached')
        measured_q=art.get_dof_positions()[0,ids].copy();measured_H=arm.fk(measured_q);predicted_center=(measured_H@last_image_grasp)[:3,3];bounds=np.asarray(c['calibration']['outer_bounds_world_m']);rim=float(bounds[1,2]);margin=.045
        within=lambda point:bool(np.all(point[:2]>=bounds[0,:2]+margin) and np.all(point[:2]<=bounds[1,:2]-margin))
        if not within(measured_H[:3,3]) or not within(predicted_center) or measured_H[2,3]<rim+.005:raise RuntimeError(reason+'; gripper is not safely above the calibrated bin opening')
        current_width=float(art.get_dof_positions()[0,fi].sum())
        for width in np.linspace(current_width,empty_width,9):
            clear,why=checker.clear(measured_q,float(width))
            if not clear:raise RuntimeError(reason+'; bin recovery release would hit fixed structure: '+str(why))
        record(dict(event='uncertain_bin_hold_release_and_retry',reason=reason,attempt=attempt,encoder_TCP_world_m=measured_H[:3,3].tolist(),last_image_predicted_center_m=predicted_center.tolist(),release_width_m=empty_width,uses_free_body_poses=False,wall_s=time.perf_counter()-c['started']))
        if c.get('set_grip_hold'):c['set_grip_hold']('right',False)
        move([cmd[0,ids].copy(),cmd[0,ids].copy()],.3,'loose_ring_uncertain_hold_release_over_bin',empty_width);wait(.5,'loose_ring_uncertain_hold_stock_settle')
        retry=dict(c,config=dict(c['config'],pickup_retry_attempt=attempt+1,pickup_avoid_centers=list(avoid)+[p['center_m']]))
        retry.pop('prefetched_initial',None);result=execute(retry);result['events']=events+result['events'];return result
    def release_outer_before_lift(reason):
        # A failed contact test never licenses a loaded lift or a loaded
        # reverse. Open at this stationary, fixed-CAD checked stock pose,
        # then use the existing checked empty recovery and fresh reobserve.
        actual_q=art.get_dof_positions()[0,ids].copy();actual_width=float(art.get_dof_positions()[0,fi].sum())
        if c.get('set_grip_hold'):c['set_grip_hold']('right',False)
        for width in np.linspace(actual_width,empty_width,9):
            for q in [actual_q,cmd[0,ids]]:
                clear,why=checker.clear(q,float(width))
                if not clear:raise RuntimeError(reason+'; stationary outer release would hit fixed structure: '+str(why))
        record(dict(event='native_outer_failed_contact_release',reason=reason,actual_opening_m=actual_width,release_width_m=empty_width,uses_free_body_poses=False,wall_s=time.perf_counter()-c['started']))
        move([cmd[0,ids].copy(),cmd[0,ids].copy()],.25,'loose_ring_failed_contact_release',empty_width)
        wait(.15,'loose_ring_failed_contact_stock_settle')
        return retry_pick(reason)
    print('SCATTERED_GRASP_TYPE',part,grasp_type,p.get('planned_grip_width_m'),'offset',tcp_offset.tolist(),'tilt',p.get('grasp_tilt_rad'),'image_center',p['center_m'],flush=True)
    if c.get('bind_validation_target'):c['bind_validation_target'](part,p)
    P=np.array(p['center_m']);n=np.array(p['axis']);current_H=arm.fk(art.get_dof_positions()[0,ids]);pre_event=dict(mode='pregrasp_ring',part=part,tracked_grasp=(np.linalg.inv(current_H)@frame(P,axis_frame(n))).tolist(),camera_eye_offset=[.06,-.06,.30]);pre_future=vision.request(pre_event,current_H)
    if local_contact and abs(float(cmd[0,fi].sum())-empty_width)>.0001:move([cmd[0,ids],cmd[0,ids]],.2,'loose_ring_local_empty_preshape',empty_width)
    move(route,.8,'loose_ring_pregrasp',empty_width)
    if abs(float(cmd[0,fi].sum())-empty_width)>.0001:move([cmd[0,ids],cmd[0,ids]],.10,'loose_ring_empty_preshape',empty_width)
    while not pre_future.done():vision.poll();step()
    pre_value=pre_future.result();pre_event['delivered_wall_s']=time.perf_counter()-c['started']
    if pre_value['result'].get('error'):return retry_pick('Target ring not directly visible from the pregrasp confirmation camera',avoid_radius_m=.012)
    prior_max_age=c['config'].get('preclose_occlusion_max_age_s')
    if prior_max_age is not None:
        prior_max_age=float(prior_max_age)
        if not .25<=prior_max_age<=3.:raise ValueError('Occluded preclose prior age must be within .25--3 seconds')
    last_pregrasp_pose_captured_wall_s=pre_value.get('captured_wall_s');prior_pose_is_partial=bool(pre_value['result'].get('pose',{}).get('fresh_partial_tracking'))
    refined=pre_value['result']['pose'];newP=np.array(refined['center_m']);newN=np.array(refined['axis'])
    if c['config'].get('require_front_up_pickup') and newN@n<.5:return retry_pick('Pregrasp RGB-D selected the opposite signed CAD face')
    if np.linalg.norm(newP-P)>.008 or abs(newN[2])<.35:
        record(dict(event='pregrasp_camera_target_not_confirmed',selected_image_center_m=P.tolist(),new_image_center_m=newP.tolist(),new_image_axis=newN.tolist(),distance_m=float(np.linalg.norm(newP-P)),wall_s=time.perf_counter()-c['started']))
        # An exposed upper neighbour can hide the selected lower ring from
        # this close view. Keep its different 3-D centre available for retry.
        return retry_pick('Fresh pregrasp camera did not confirm the selected accessible ring',avoid_radius_m=.012)
    from isaac_kinematics import exp
    a=n;b=newN*(1 if n@newN>=0 else -1);cross=np.cross(a,b);change=exp(cross/max(np.linalg.norm(cross),1e-12)*np.arctan2(np.linalg.norm(cross),a@b));H[:3,:3]=change@H[:3,:3];tcp_offset=change@tcp_offset;P=newP;n=newN;H[:3,3]=P+n*np.sign(n[2])*contact_offset+tcp_offset
    if prior_max_age is not None:
        # Confirm from new pixels immediately before the final 15 mm. The
        # earlier image was captured before the empty approach and is old.
        via=H.copy();via[2,3]+=.015;line(via,.45,'loose_ring_descend_prior_free')
        current_H=arm.fk(art.get_dof_positions()[0,ids]);recent=observe('pregrasp_ring',part,np.linalg.inv(current_H)@frame(P,axis_frame(n)),allow_missing=True)
        fresh=recent['result'].get('pose',{})
        if not measured_pose_supported(recent['result'],'pregrasp_ring'):return retry_pick('No supported fresh RGB-D confirmation immediately above contact')
        if np.linalg.norm(np.asarray(fresh['center_m'])-P)>.003 or np.asarray(fresh['axis'])@n<np.cos(.06):return retry_pick('Target moved before the bounded occlusion descent')
        last_pregrasp_pose_captured_wall_s=recent['captured_wall_s'];prior_pose_is_partial=bool(fresh.get('fresh_partial_tracking'))
        P=np.asarray(fresh['center_m']);n=np.asarray(fresh['axis']);H[:3,3]=P+n*np.sign(n[2])*contact_offset+tcp_offset
        line(H,1.875*.015/float(c['config'].get('descent_contact_speed_m_s',.030)),'loose_ring_descend_prior_contact')
    else:line(H,.45,'loose_ring_descend')
    # Freeze/drain the receiver BEFORE the fresh preclose image and before
    # any native grip-hold force is enabled. No new payload exists yet.
    if c.get('_before_right_pick_contact'):c['_before_right_pick_contact']()
    # Loose neighbours can move the target during descent. Reobserve before
    # closing; only image measurements and the hand encoder enter this repair.
    # An ungrasped ring stays in the world, not on the commanded TCP. Encode
    # its last IMAGE pose relative to the measured hand at this exposure.
    measured_H=arm.fk(art.get_dof_positions()[0,ids]);settled_value=observe('pregrasp_ring',part,np.linalg.inv(measured_H)@frame(P,axis_frame(n)),allow_missing=True);settled_result=settled_value['result']
    visible_pose=settled_result.get('pose',{})
    directly_visible=bool(visible_pose.get('front_back_from_RGBD'));partial_visible=bool(c['config'].get('allow_partial_preclose_tracking') and visible_pose.get('fresh_partial_tracking'));fresh_pose_visible=directly_visible or partial_visible
    if not fresh_pose_visible and c['config'].get('require_fresh_preclose_rgbd'):return retry_pick('No fresh directly measured annular face before closing')
    prior_age=None if last_pregrasp_pose_captured_wall_s is None else time.perf_counter()-c['started']-last_pregrasp_pose_captured_wall_s
    if not fresh_pose_visible and prior_max_age is not None and (prior_age is None or prior_age>prior_max_age):return retry_pick('Occluded preclose direct-image prior exceeded its bounded age')
    settled=visible_pose if fresh_pose_visible else dict(center_m=P.tolist(),axis=n.tolist());settledP=np.array(settled['center_m']);settledN=np.array(settled['axis'])
    record(dict(event='preclose_reobservation',new_pose_visible=fresh_pose_visible,new_annular_face_visible=directly_visible,fresh_partial_pose=partial_visible,prior_pose_is_partial=prior_pose_is_partial,prior_size_and_signed_face_preserved=bool(partial_visible or prior_pose_is_partial),fallback='Explicit prior RGB-D estimate during hand occlusion; fresh afterlift evidence required' if not fresh_pose_visible else None,prior_image_age_s=prior_age,maximum_prior_age_s=prior_max_age,wall_s=time.perf_counter()-c['started']))
    if np.linalg.norm(settledP-P)>.025:raise RuntimeError('Ring moved outside local RGB-D grasp recovery range')
    if np.linalg.norm(settledP-P)>.0008 or np.arccos(np.clip(n@settledN,-1,1))>.035:
        aligned=settledN*(1 if n@settledN>=0 else -1);cross=np.cross(n,aligned);change=exp(cross/max(np.linalg.norm(cross),1e-12)*np.arctan2(np.linalg.norm(cross),n@aligned));H[:3,:3]=change@H[:3,:3];tcp_offset=change@tcp_offset
        P=settledP;n=settledN;H[:3,3]=P+n*np.sign(n[2])*contact_offset+tcp_offset
        if not checker.hand_clear(H,float(art.get_dof_positions()[0,fi].sum()))[0]:return retry_pick('Fresh RGB-D grasp correction would collide with fixed structure')
        line(H,.20,'loose_ring_observed_grasp_adjust')
    else:P=settledP;n=settledN
    if local_contact and c['config'].get('rim_encoder_recenter'):
        if grasp_type=='rim':from local_rim_grasp import candidate as local_candidate
        else:from inner_bore_grasp import candidate as local_candidate
        correction_points=stock_points(settled_value) if c.get('observed_grasp_clearance') and 'packet'in settled_value else observed_points
        for correction_index in range(2):
            actual_q=art.get_dof_positions()[0,ids].copy();actual_H=arm.fk(actual_q);distance=float(np.linalg.norm(H[:3,3]-actual_H[:3,3]));angle=float(np.linalg.norm(log(H[:3,:3]@actual_H[:3,:3].T)))
            if distance<.0006 and angle<.008:break
            desired_q=arm.solve(H[:3,3],H[:3,:3],actual_q)
            if desired_q is None or np.max(abs(desired_q-actual_q))>.03:return retry_pick('Rim encoder correction exceeds bounded local IK recovery')
            corrected=cmd[0,ids]+desired_q-actual_q
            if np.any(corrected<arm.bounds[:,0]) or np.any(corrected>arm.bounds[:,1]):return retry_pick('Rim encoder correction exceeds arm joint bounds')
            clear,why=checker.path([actual_q,cmd[0,ids],corrected],empty_width)
            if not clear:return retry_pick('Rim encoder correction would collide with fixed structure: '+str(why))
            if correction_points is not None:
                measured_pose=dict(part=part,center_m=P.tolist(),axis=n.tolist());proposed_H=arm.fk(corrected)
                if not c['observed_grasp_clearance'].clear(correction_points,measured_pose,actual_H,proposed_H,empty_width):return retry_pick('Rim encoder correction would collide with RGB-D observed stock')
            move([cmd[0,ids].copy(),corrected],.15,'loose_ring_observed_grasp_adjust_encoder');wait(.08,'loose_ring_encoder_correction_settle')
            actual_H=arm.fk(art.get_dof_positions()[0,ids]);fresh_value=observe('pregrasp_ring',part,np.linalg.inv(actual_H)@frame(P,axis_frame(n)),allow_missing=True);fresh=fresh_value['result']
            fresh_pose=fresh.get('pose',{})
            if not fresh_pose.get('front_back_from_RGBD'):return retry_pick('No directly measured annular face after rim encoder correction')
            P=np.asarray(fresh_pose['center_m']);n=np.asarray(fresh_pose['axis']);candidate_extra={'roll_rad':float(p.get('grasp_roll_rad',0.))} if grasp_type=='inner' else {};updated=local_candidate(c,part,fresh_pose,H[:3,1],axial_offset_m=contact_offset,tilt_rad=float(p['grasp_tilt_rad']),**candidate_extra)
            if updated is None:return retry_pick('Updated RGB-D rim pose has no opposing native pad contacts')
            H=np.asarray(updated['tcp_world']).copy();R=H[:3,:3];tcp_offset=H[:3,3]-P-n*np.sign(n[2])*contact_offset;p['planned_grip_width_m']=float(updated['contact_width_m']);p['command_width_m']=float(updated['command_width_m'])
            if c.get('observed_grasp_clearance') and 'packet'in fresh_value:correction_points=stock_points(fresh_value)
            record(dict(event='rim_encoder_recenter',iteration=correction_index,position_error_before_m=distance,angle_error_before_rad=angle,command_joint_correction_rad=(desired_q-actual_q).tolist(),wall_s=time.perf_counter()-c['started']))
        actual_H=arm.fk(art.get_dof_positions()[0,ids]);remaining=float(np.linalg.norm(H[:3,3]-actual_H[:3,3]))
        if remaining>.0015:return retry_pick('Rim encoder correction did not converge within 1.5 mm')
    # Native fingers close against the observed diameter with finite effort.
    c['stiffness'][0,fi]=0.;c['damping'][0,fi]=[4000.,.001];art.set_dof_stiffnesses(c['stiffness'],c['idx']);art.set_dof_dampings(c['damping'],c['idx']);D=.075 if part=='ring1'else .062
    close_width=float(p['command_width_m']) if local_contact else p['planned_grip_width_m'] if grasp_type=='inner' else max(.001,p['planned_grip_width_m']-.003)
    if part=='ring2' and grasp_type=='outer' and c['config'].get('verify_selected_outer_approach'):
        actual_q=art.get_dof_positions()[0,ids].copy();actual_H=arm.fk(actual_q);actual_gap=float(art.get_dof_positions()[0,fi].sum())
        fresh_points=stock_points(settled_value);close_pose=dict(part=part,center_m=P.tolist(),axis=n.tolist())
        # Recheck the SAME aperture domain with fresh pixels and actual
        # encoders. If stock moved, fingers have not yet closed: reverse the
        # checked empty approach and choose again from fresh RGB-D.
        closure_clear,closure_why=outer_closure_clear(c,close_pose,actual_H,fresh_points,max(empty_width,actual_gap),close_width,joint_positions=[actual_q,cmd[0,ids]],minimum_gap=actual_gap)
        if not closure_clear:return retry_pick('Fresh small outer closure aperture sweep blocked: '+str(closure_why),loaded=False)
    if c.get('set_grip_hold'):c['set_grip_hold']('right',True,direction=1. if grasp_type=='inner' else -1.)
    move([cmd[0,ids],cmd[0,ids]],.14,'loose_ring_close',close_width)
    if c.get('set_grip_hold'):c['set_grip_hold']('right',True,direction=1. if grasp_type=='inner' else -1.)
    wait(float(c['config'].get('grasp_settle_s',.12)),'loose_ring_grasp_settle')
    if local_contact or grasp_type=='outer':
        # A velocity/force ramp may still be traversing empty air after the
        # nominal close trajectory finishes. Do not lift on a timed timeout:
        # both pads must have stopped near their CAD contact separation.
        stable=0.;contact_confirmed=False;position_window=[]
        for _ in range(120):
            finger_q=art.get_dof_positions()[0,fi].copy();gap=float(finger_q.sum());speed=float(np.max(abs(art.get_dof_velocities()[0,fi])));position_window.append(finger_q)
            if len(position_window)>5:position_window.pop(0)
            empty=(gap>=close_width-.0005) if grasp_type=='inner' else (gap<=close_width+.0005)
            if empty:break
            near_contact=(gap>=p['planned_grip_width_m']-.001) if grasp_type=='inner' else (gap<=p['planned_grip_width_m']+.001)
            measured_window=np.asarray(position_window);stationary=len(position_window)==5 and np.max(np.ptp(measured_window,axis=0))<=.00015 and np.ptp(measured_window.sum(1))<=.00015
            stable=.1 if near_contact and stationary else 0.
            if stable>=.1:contact_confirmed=True;break
            wait(.025,'loose_ring_grasp_controller_completion')
        record(dict(event='native_outer_opposed_contact_plateau' if grasp_type=='outer' else 'native_local_opposed_contact_plateau',grasp_type=grasp_type,confirmed=contact_confirmed,aperture_m=float(art.get_dof_positions()[0,fi].sum()),expected_m=p['planned_grip_width_m'],reported_pad_velocity_m_s=float(np.max(abs(art.get_dof_velocities()[0,fi]))),per_finger_position_range_m=np.ptp(np.asarray(position_window),axis=0).tolist(),gap_position_range_m=float(np.ptp(np.asarray(position_window).sum(1))),stable_s=stable,hardware_velocity_guard_unchanged=True,wall_s=time.perf_counter()-c['started']))
        if not contact_confirmed:
            if grasp_type=='outer':return release_outer_before_lift('Native outer grasp did not establish a stationary opposing-pad contact before lift')
            if c.get('set_grip_hold'):c['set_grip_hold']('right',False)
            move([cmd[0,ids],cmd[0,ids]],.25,'loose_ring_failed_contact_release',empty_width)
            return retry_pick('Native '+grasp_type+' grasp did not establish a stationary opposing-pad contact before lift')
    else:
        for _ in range(12):
            gap=float(art.get_dof_positions()[0,fi].sum())
            if gap<=p['planned_grip_width_m']+.001 and np.max(abs(art.get_dof_velocities()[0,fi]))<.010:break
            if gap<=close_width+.001:break
            wait(.025,'loose_ring_grasp_controller_completion')
    aperture=float(art.get_dof_positions()[0,fi].sum())
    record(dict(event='encoder_grasp_aperture',aperture_m=aperture,expected_class_diameter_m=D,grasp_type=grasp_type))
    if grasp_type=='outer' and aperture>D+.0035:
        return release_outer_before_lift('Grasp aperture exceeds observed ring size; wrong class or neighbouring stock contact')
    if grasp_type=='outer' and aperture<close_width+.0005:
        return release_outer_before_lift('No opposing pad contact: fingers reached preload width without holding the rim')
    if grasp_type=='outer':
        # Encoder arrest alone could be contact with neighbouring stock.
        # Reobserve the selected ring from current RGB-D pixels before any
        # arm motion, then certify both native hulls at the actual aperture.
        from receiver_pad_contact import contact_state as outer_contact_state
        before_lift_H=arm.fk(art.get_dof_positions()[0,ids]);closed_value=observe('pregrasp_ring',part,np.linalg.inv(before_lift_H)@frame(P,axis_frame(n)),allow_missing=True)
        if not measured_pose_supported(closed_value['result'],'pregrasp_ring'):
            return release_outer_before_lift('No fresh RGB-D ring pose to verify the closed outer grasp')
        closed_pose=closed_value['result']['pose'];closed_P=np.asarray(closed_pose['center_m']);closed_n=np.asarray(closed_pose['axis'])
        if np.linalg.norm(closed_P-P)>.003 or closed_n@n<np.cos(.06):
            return release_outer_before_lift('Postclose RGB-D ring moved outside the bounded selected-target identity gate')
        measured_G=np.linalg.inv(closed_value['H'])@frame(closed_P,axis_frame(closed_n));contact_context=dict(c,config=dict(c['config'],receiver_axial_only=False))
        contact=outer_contact_state(contact_context,part,measured_G,float(art.get_dof_positions()[0,fi].sum()),contact_tolerance_m=.0008)
        record(dict(event='outer_fresh_opposed_contact_before_lift',contact=contact,measured_grasp=measured_G.tolist(),fresh_partial_pose=bool(closed_pose.get('fresh_partial_tracking')),uses_free_body_poses=False,wall_s=time.perf_counter()-c['started']))
        if not contact['opposed_contact_compatible']:
            return release_outer_before_lift('Fresh RGB-D and encoders do not certify both native outer pad contacts before lift')
        # Camera acquisition is asynchronous while finite-force holding
        # continues. Require the same near-CAD encoder plateau at delivery;
        # a formerly valid image cannot license an ongoing empty closure.
        position_window=[]
        for _ in range(5):
            position_window.append(art.get_dof_positions()[0,fi].copy());wait(.025,'loose_ring_outer_contact_final_encoder_window')
        measured_window=np.asarray(position_window);aperture=float(art.get_dof_positions()[0,fi].sum())
        stationary=np.max(np.ptp(measured_window,axis=0))<=.00015 and np.ptp(measured_window.sum(1))<=.00015
        if not stationary or not close_width+.0005<aperture<=p['planned_grip_width_m']+.001:
            return release_outer_before_lift('Native outer contact did not remain stationary through fresh RGB-D confirmation')
        # Re-evaluate the same fresh world pose with the latest encoders.
        measured_G=np.linalg.inv(arm.fk(art.get_dof_positions()[0,ids]))@frame(closed_P,axis_frame(closed_n))
        contact=outer_contact_state(contact_context,part,measured_G,aperture,contact_tolerance_m=.0008)
        record(dict(event='outer_final_contact_before_lift',contact=contact,measured_grasp=measured_G.tolist(),image_captured_wall_s=closed_value.get('captured_wall_s'),per_finger_position_range_m=np.ptp(measured_window,axis=0).tolist(),gap_position_range_m=float(np.ptp(measured_window.sum(1))),uses_free_body_poses=False,wall_s=time.perf_counter()-c['started']))
        if not contact['opposed_contact_compatible']:
            return release_outer_before_lift('Native outer aperture changed beyond the fresh RGB-D two-pad contact certificate')
        if part=='ring1' and c['config'].get('require_broad_large_donor_support'):
            from grasp_transfer_feasibility import large_donor_pad_support
            support=large_donor_pad_support(c,measured_G,aperture)
            record(dict(event='large_outer_native_pad_support_before_lift',support=support,measured_grasp=measured_G.tolist(),wall_s=time.perf_counter()-c['started']))
            if not support['broad_opposed_flat_pad_support']:
                return release_outer_before_lift('Fresh closed large ring lacks broad support on both native flat pads')
        prelift_measured_grasp=measured_G.copy()
        P=closed_P;n=closed_n
    if grasp_type=='rim' and (aperture<close_width+.0004 or aperture>p['planned_grip_width_m']+.0035):
        if c.get('set_grip_hold'):c['set_grip_hold']('right',False)
        move([cmd[0,ids],cmd[0,ids]],.25,'loose_ring_failed_contact_release',empty_width)
        return retry_pick('Local rim pinch aperture does not confirm finite opposing pad contact')
    if local_contact:
        if grasp_type=='rim':from local_rim_grasp import contact_state
        else:from inner_bore_grasp import contact_state
        before_lift_H=arm.fk(art.get_dof_positions()[0,ids]);closed_value=observe('pregrasp_ring',part,np.linalg.inv(before_lift_H)@frame(P,axis_frame(n)),allow_missing=True);closed_pose=closed_value['result'].get('pose',{})
        if not closed_pose.get('front_back_from_RGBD'):
            if c.get('set_grip_hold'):c['set_grip_hold']('right',False)
            move([cmd[0,ids],cmd[0,ids]],.25,'loose_ring_failed_contact_release',empty_width)
            return retry_pick('No directly measured annular face to verify the closed local rim pinch')
        measured_G=np.linalg.inv(closed_value['H'])@frame(closed_pose['center_m'],axis_frame(closed_pose['axis']));contact=contact_state(c,part,measured_G,float(art.get_dof_positions()[0,fi].sum()),contact_tolerance_m=.0008)
        record(dict(event='local_rim_fresh_opposed_contact_before_lift',contact=contact,measured_grasp=measured_G.tolist(),wall_s=time.perf_counter()-c['started']))
        if not contact['opposed_contact_compatible']:
            if c.get('set_grip_hold'):c['set_grip_hold']('right',False)
            move([cmd[0,ids],cmd[0,ids]],.25,'loose_ring_failed_contact_release',empty_width)
            return retry_pick('Fresh RGB-D and encoder aperture do not confirm contact on both sides of the local rim')
        P=np.asarray(closed_pose['center_m']);n=np.asarray(closed_pose['axis'])
        if c.get('set_grip_hold') and c['config'].get('native_loaded_preload_after_contact_m_s') is not None:
            target_preload=float(c['config']['native_loaded_preload_after_contact_m_s']);c['set_grip_hold']('right',True,direction=1. if grasp_type=='inner' else -1.,preload_m_s=target_preload)
            wait(float(c['config'].get('native_loaded_preload_ramp_s',.3))+.05,'loose_ring_confirmed_contact_force_ramp')
            actual_H=arm.fk(art.get_dof_positions()[0,ids]);force_value=observe('pregrasp_ring',part,np.linalg.inv(actual_H)@frame(P,axis_frame(n)),allow_missing=True);force_pose=force_value['result'].get('pose',{})
            if not force_pose.get('front_back_from_RGBD'):
                if c.get('set_grip_hold'):c['set_grip_hold']('right',False)
                move([cmd[0,ids],cmd[0,ids]],.25,'loose_ring_failed_contact_release',empty_width)
                return retry_pick('No fresh annular face after confirmed-contact force ramp')
            force_G=np.linalg.inv(force_value['H'])@frame(force_pose['center_m'],axis_frame(force_pose['axis']));contact=contact_state(c,part,force_G,float(art.get_dof_positions()[0,fi].sum()),contact_tolerance_m=.0008)
            record(dict(event='native_local_contact_after_force_ramp',grasp_type=grasp_type,preload_m_s=target_preload,contact=contact,measured_grasp=force_G.tolist(),wall_s=time.perf_counter()-c['started']))
            if not contact['opposed_contact_compatible']:
                if c.get('set_grip_hold'):c['set_grip_hold']('right',False)
                move([cmd[0,ids],cmd[0,ids]],.25,'loose_ring_failed_contact_release',empty_width)
                return retry_pick('Opposing native pad contact was lost during confirmed-contact force ramp')
            P=np.asarray(force_pose['center_m']);n=np.asarray(force_pose['axis'])
    record(dict(event='pickup_extraction_boundary',fixed_supply=fixed_supply,boundary_z_m=extraction_boundary_z,minimum_lift_m=.075,source='Calibrated original shelf support plane' if fixed_supply else 'Original basket lip',uses_free_body_poses=False,wall_s=time.perf_counter()-c['started']))
    G=np.linalg.inv(arm.fk(art.get_dof_positions()[0,ids]))@frame(P,axis_frame(n));lift=H.copy();half=.00775 if part=='ring1' else .007;support=D/2*np.sqrt(max(0,1-n[2]**2))+half*abs(n[2]);requested_lift=max(.075,extraction_boundary_z+support+.010-P[2]);lift_offset=np.asarray(p.get('pickup_lift_offset_world_m',[0.,0.,0.]));lift[:3,3]+=lift_offset+[0.,0.,requested_lift]
    if np.linalg.norm(lift_offset)>1e-8 or fixed_supply:
        breakaway=H.copy();breakaway[2,3]+=float(p['pickup_lift_breakaway_m']);line(breakaway,.05,'loose_ring_lift_breakaway')
        start_H=arm.fk(art.get_dof_positions()[0,ids])
        if observed_points is not None:
            distance,_=c['observed_grasp_clearance'].estimator.trees[part].query((observed_points-P)@axis_frame(n));lift_points=observed_points[distance>.0025];stock_pose=dict(p,center_m=P.tolist(),axis=n.tolist())
            if not c['observed_grasp_clearance'].palm_clear(lift_points,stock_pose,start_H,lift) or not c['observed_grasp_clearance'].clear(lift_points,stock_pose,start_H,lift,float(art.get_dof_positions()[0,fi].sum())):
                raise RuntimeError('Actual diagonal pickup lift intersects observed stock')
        with temporary_payload(G):line(lift,.6,'loose_ring_lift')
    else:line(lift,.6,'loose_ring_lift')
    wait(.10,'loose_ring_lift_settle')
    v=observe('held',part,G,allow_missing=bool(c['config'].get('allow_bin_release_retry')))
    if v['result'].get('error'):return retry_uncertain_hold(v['result']['error'],G)
    pose=v['result']['pose'];Gobs=np.linalg.inv(v['H'])@frame(pose['center_m'],axis_frame(pose['axis']));predicted=arm.fk(art.get_dof_positions()[0,ids])@Gobs;rise=float(predicted[2,3]-P[2]);wait(.02,'loose_ring_camera_confirmed_hold')
    observed_axis=np.array(pose['axis']);observed_bottom=pose['center_m'][2]-D/2*np.sqrt(max(0,1-observed_axis[2]**2))-half*abs(observed_axis[2]);
    if rise>.04 and -.025<observed_bottom-extraction_boundary_z<.003:
        # A small measured slip is recoverable by a short extra axial lift.
        # The same fresh image gate must confirm clearance after this move.
        delta=extraction_boundary_z+.012-observed_bottom;repair=arm.fk(cmd[0,ids]);repair[2,3]+=delta
        line(repair,.25,'loose_ring_RGBD_lift_clearance_repair');wait(.08,'loose_ring_repair_settle')
        v=observe('held',part,Gobs,allow_missing=bool(c['config'].get('allow_bin_release_retry')))
        if v['result'].get('error'):return retry_uncertain_hold(v['result']['error'],Gobs)
        pose=v['result']['pose'];Gobs=np.linalg.inv(v['H'])@frame(pose['center_m'],axis_frame(pose['axis']));predicted=arm.fk(art.get_dof_positions()[0,ids])@Gobs;rise=float(predicted[2,3]-P[2]);observed_axis=np.array(pose['axis']);observed_bottom=pose['center_m'][2]-D/2*np.sqrt(max(0,1-observed_axis[2]**2))-half*abs(observed_axis[2])
    if rise<requested_lift-.015 or observed_bottom<extraction_boundary_z+.003:
        if c['config'].get('allow_bin_release_retry'):return retry_uncertain_hold('RGB-D did not confirm selected ring fully above rim',Gobs)
        raise RuntimeError('RGB-D did not confirm selected ring fully above rim')
    if c['config'].get('require_front_up_pickup') and np.asarray(pose['axis'])@np.asarray(p['axis'])<.5:
        if c['config'].get('allow_bin_release_retry'):return retry_uncertain_hold('Lifted RGB-D signed face disagrees with the selected front-up image',Gobs)
        return retry_pick('Lifted RGB-D signed face disagrees with the selected front-up image',loaded=True)
    if part=='ring1' and grasp_type=='outer' and c['config'].get('verify_large_lift_grasp_retention'):
        # A static two-pad patch can remain geometrically valid while the
        # part rolls toward the finger tips. Compare TWO fresh RGB-D poses
        # in the actual encoder hand frame before loaded transport.
        translation_limit=float(c['config'].get('large_lift_grasp_translation_limit_m',.002))
        axis_limit=float(c['config'].get('large_lift_grasp_axis_limit_rad',.05))
        if not .0005<=translation_limit<=.005 or not .01<=axis_limit<=.1:
            raise ValueError('Large lift retention bounds must remain within local camera-tracking limits')
        translation=float(np.linalg.norm(Gobs[:3,3]-prelift_measured_grasp[:3,3]))
        axis_change=float(np.arccos(np.clip(Gobs[:3,2]@prelift_measured_grasp[:3,2],-1,1)))
        retained=bool(translation<=translation_limit and axis_change<=axis_limit)
        record(dict(event='large_outer_lift_grasp_retention',retained=retained,translation_change_m=translation,axis_change_rad=axis_change,translation_limit_m=translation_limit,axis_limit_rad=axis_limit,prelift_measured_grasp=prelift_measured_grasp.tolist(),postlift_measured_grasp=Gobs.tolist(),uses_free_body_poses=False,wall_s=time.perf_counter()-c['started']))
        if not retained:
            reason='Large ring rolled or slipped in the native pads during bin lift'
            if c['config'].get('allow_bin_release_retry'):return retry_uncertain_hold(reason,Gobs)
            return retry_pick(reason,loaded=True)
    if part=='ring2' and grasp_type=='outer' and c['config'].get('small_ring_repick_on_unstable_hold'):
        limit=float(c['config'].get('small_ring_com_x_limit_m',.0025));valid_width=c['grasp_pad_aperture'](pose,arm.fk(art.get_dof_positions()[0,ids])) if c.get('grasp_pad_aperture') else D
        center_offset=float(abs(Gobs[0,3]));record(dict(event='donor_lifted_grasp_geometry',pad_width_center_offset_m=center_offset,limit_m=limit,opposing_pad_geometry=valid_width is not None,wall_s=time.perf_counter()-c['started']))
        if valid_width is None or center_offset>limit:
            reason='Lifted small ring is not centered in a stable two-pad hold'
            if c['config'].get('allow_bin_release_retry'):return retry_uncertain_hold(reason,Gobs)
            return retry_pick(reason,loaded=True)
    if grasp_type=='outer' and c['config'].get('verify_lifted_pad_contact'):
        from receiver_pad_contact import contact_state as outer_contact_state
        contact_context=dict(c,config=dict(c['config'],receiver_axial_only=False))
        lifted_contact=outer_contact_state(contact_context,part,Gobs,float(art.get_dof_positions()[0,fi].sum()),contact_tolerance_m=.0008)
        record(dict(event='outer_lifted_fresh_opposed_contact',contact=lifted_contact,measured_grasp=Gobs.tolist(),wall_s=time.perf_counter()-c['started']))
        if not lifted_contact['opposed_contact_compatible']:
            if c['config'].get('allow_bin_release_retry'):return retry_uncertain_hold('Fresh lifted RGB-D does not confirm opposing native outer pad contacts',Gobs)
            return retry_pick('Fresh lifted RGB-D does not confirm opposing native outer pad contacts',loaded=True)
    if part=='ring1' and grasp_type=='outer' and c['config'].get('require_broad_large_donor_support'):
        from grasp_transfer_feasibility import large_donor_pad_support
        support=large_donor_pad_support(c,Gobs,float(art.get_dof_positions()[0,fi].sum()))
        record(dict(event='large_outer_native_pad_support_after_lift',support=support,measured_grasp=Gobs.tolist(),wall_s=time.perf_counter()-c['started']))
        if not support['broad_opposed_flat_pad_support']:
            if c['config'].get('allow_bin_release_retry'):return retry_uncertain_hold('Fresh lifted large ring lacks broad support on both native flat pads',Gobs)
            return retry_pick('Fresh lifted large ring lacks broad support on both native flat pads',loaded=True)
    if local_contact:
        if grasp_type=='rim':from local_rim_grasp import contact_state as lifted_contact_state
        else:from inner_bore_grasp import contact_state as lifted_contact_state
        lifted_contact=lifted_contact_state(c,part,Gobs,float(art.get_dof_positions()[0,fi].sum()),contact_tolerance_m=.0008)
        record(dict(event='native_local_lifted_geometry',contact=lifted_contact,measured_grasp=Gobs.tolist(),wall_s=time.perf_counter()-c['started']))
        if not lifted_contact['opposed_contact_compatible']:
            if c['config'].get('allow_bin_release_retry'):return retry_uncertain_hold('RGB-D does not confirm two native pad contacts on the lifted ring',Gobs)
            return retry_pick('RGB-D does not confirm two native pad contacts on the lifted ring',loaded=True)
    if c.get('bind_validation_target'):c['bind_validation_target'](part,pose)
    return dict(state='RGBD_grasp_and_lift_confirmed',grasp_type=grasp_type,release_width_m=float(p.get('release_width_m',.001 if grasp_type=='inner' else .08)),selected_instance=p,measured_grasp=Gobs.tolist(),observed_lift_m=rise,events=events,elapsed_s=time.perf_counter()-c['started'],full_task_pass=False,uses_simulator_body_poses=False)
