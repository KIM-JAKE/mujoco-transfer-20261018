"""Static 15-degree single-finger bore hook from saved RGB-D and encoders.

No simulation robot commands or free-body transforms are read or written.
The proposed route must later be tested through real physical contact.
"""
from pathlib import Path
import json,time
import numpy as np
from scipy.spatial import ConvexHull
from isaac_kinematics import frame,log
from rgbd_geometry import axis_frame


def bore_proof(c,pose,H,opening):
    cache=c.setdefault('_static_hook_native_geometry',{})
    if not cache:
        v=np.load(Path(__file__).resolve().parent/'native-official-finger.npz')['vertices'];hull=ConvexHull(v)
        edges=np.unique(np.sort(np.concatenate([hull.simplices[:,[0,1]],hull.simplices[:,[1,2]],hull.simplices[:,[2,0]]]),axis=1),axis=0)
        cache.update(vertices=v,edges=edges,planes=hull.equations)
    v,edges,planes=[cache[k]for k in ['vertices','edges','planes']];P=np.asarray(pose['center_m']);n=np.asarray(pose['axis']);n/=np.linalg.norm(n);part=pose['part'];R=H[:3,:3]
    model=c['observed_grasp_clearance'].estimator.models[part];world=model@axis_frame(n).T+P
    bore_radius=float(np.linalg.norm(model[:,:2],axis=1).min());proofs=[]
    for sign in [-1.,1.]:
        flip=np.array([sign,sign,1.]);origin=H[:3,3]+R@np.array([0.,sign*opening/2,-.045]);W=(v*flip)@R.T+origin;local=(world-origin)@R*flip
        distance=(local@planes[:,:3].T+planes[:,3]).max(1);d=(W-P)@n
        A=W[edges[:,0]];B=W[edges[:,1]];da=d[edges[:,0]];db=d[edges[:,1]];mask=(da*db<=0)&(abs(da-db)>1e-12);u=da[mask]/(da[mask]-db[mask]);Q=A[mask]+(B[mask]-A[mask])*u[:,None]
        radial=Q-P-np.outer((Q-P)@n,n);maxrad=float(np.linalg.norm(radial,axis=1).max())if len(Q)else None
        inside=bool(len(Q)>=3 and maxrad<=bore_radius-.001 and distance.min()>=-.0001)
        proofs.append(dict(jaw_sign=sign,midplane_intersection_points=len(Q),maximum_midplane_radius_m=maxrad,minimum_class_bore_radius_m=bore_radius,minimum_native_hull_to_selected_CAD_m=float(distance.min()),inside_bore=inside))
    return dict(inside_bore=any(p['inside_bore']for p in proofs)and all(p['minimum_native_hull_to_selected_CAD_m']>=-.0001 for p in proofs),jaws=proofs,source='RGB-D signed ring plane, class CAD and actual unmodified native finger hull')



def first_contact_proof(c,pose,H,direction,maximum_stroke_m,opening=.070):
    """First selected-CAD/native-hull touch along a measured-plane sweep.

    This is a geometric preflight only. It does not assume the stock follows
    the finger or infer force/retention from the planned contact.
    """
    bore_proof(c,pose,H,opening)
    planes=c['_static_hook_native_geometry']['planes'];estimator=c['observed_grasp_clearance'].estimator
    P=np.asarray(pose['center_m'],float);n=np.asarray(pose['axis'],float);n/=np.linalg.norm(n)
    direction=np.asarray(direction,float);direction/=np.linalg.norm(direction)
    part=pose['part'];model=estimator.models[part];normals=estimator.normals[part]
    R=axis_frame(n);world=model@R.T+P;world_normals=normals@R.T
    # The small ring has a tapered bore. Negative radial normal identifies
    # its true inside wall without calling a cap or an outside wall a bore.
    inner=(np.sum(model[:,:2]*normals[:,:2],axis=1)<0)&(abs(normals[:,2])<.9)
    cap=abs(normals[:,2])>.9
    normal_component=float(n@direction)
    if abs(normal_component)>1e-6:
        return dict(supported=False,reason='pull_leaves_observed_ring_plane',normal_direction_component=normal_component)
    count=max(2,int(np.ceil(maximum_stroke_m/.0005)))
    for stroke in np.linspace(0,maximum_stroke_m,count+1):
        distances=[];facet_normals=[]
        for sign in [-1.,1.]:
            flip=np.array([sign,sign,1.]);origin=H[:3,3]+direction*stroke+H[:3,:3]@np.array([0.,sign*opening/2,-.045])
            local=(world-origin)@H[:3,:3]*flip
            halfspace=local@planes[:,:3].T+planes[:,3]
            distances.append(halfspace.max(1))
            facets=halfspace.argmax(1);facet_normals.append((planes[facets,:3]*flip)@H[:3,:3].T)
        minimum=min(float(value.min())for value in distances)
        if minimum>.0005:continue
        patches=[distance<=minimum+.0001 for distance in distances]
        total=sum(int(mask.sum())for mask in patches)
        inner_count=sum(int((mask&inner).sum())for mask in patches)
        cap_count=sum(int((mask&cap).sum())for mask in patches)
        resultant=sum(world_normals[mask].sum(0)for mask in patches);resultant/=max(np.linalg.norm(resultant),1e-12)
        alignment=sum(float((-np.sum(facets[mask]*world_normals[mask],axis=1)).sum())for mask,facets in zip(patches,facet_normals))/max(total,1)
        supporting_cosine=float(-resultant@direction)
        supported=bool(total>=3 and inner_count>=3 and inner_count/max(total,1)>=.8 and cap_count/max(total,1)<=.2 and supporting_cosine>=1/np.sqrt(1+.4**2))
        return dict(supported=supported,reason='inner_wall_supports_pull'if supported else'first_contact_not_supported_inner_wall',first_contact_stroke_m=float(stroke),minimum_hull_clearance_m=minimum,first_patch_count=total,inner_wall_samples=inner_count,cap_samples=cap_count,opposing_normal_to_pull_cosine=supporting_cosine,normal_direction_component=normal_component,mean_native_facet_normal_opposition=alignment,first_patch_world_normal=resultant.tolist(),force_contact_measured=False,uses_free_body_poses=False)
    return dict(supported=False,reason='no_selected_inner_wall_contact_in_sweep',normal_direction_component=normal_component)


def _plan(c,measurements,broader=False):
    arm,cmd,ids,vision,checker=[c[k]for k in ['arm','cmd','ids','vision','checker']]
    part=c['config'].get('pick_part','ring2')
    if part not in ['ring1','ring2']:raise ValueError('Tilted bore hook supports ring classes only')
    deadline=time.perf_counter()+float(c['config'].get('axial_receiver_fixture_budget_s',90.))
    poses=[]
    for value in measurements:
        for pose in value['result'].get('instances',[]):
            if pose['part']!=part or abs(pose['axis'][2])<.65:continue
            if c['config'].get('rearrange_front_up_only',c['config'].get('require_front_up_pickup',False)) and pose['axis'][2]<.35:continue
            if not any(np.linalg.norm(np.asarray(pose['center_m'])-old['center_m'])<.012 for old in poses):poses.append(pose)
    # High exposed stock leaves clearance for the native 205 mm palm.
    poses.sort(key=lambda p:-p['center_m'][2])
    points=np.concatenate([c['observed_grasp_clearance'].points(value['packet'],c['calibration']) for value in measurements])
    floor=np.asarray(c['calibration']['floor_origin_world_m']);B=np.asarray(c['calibration']['basket_to_world_R']);normal=B[:,2]
    y=B[:,1].copy();y[2]=0;y/=np.linalg.norm(y);z=np.array([np.sin(np.radians(15.)),0.,-np.cos(np.radians(15.))]);z-=y*(z@y);z/=np.linalg.norm(z);R=np.column_stack([np.cross(y,z),y,z])
    rotations=[R,R@np.diag([-1.,-1.,1.])];current_R=arm.fk(cmd[0,ids])[:3,:3];rotations.sort(key=lambda r:np.linalg.norm(log(r@current_R.T)))
    transverse=np.cross(y,np.array([0.,0.,1.]))
    postures=[(rotation,15.,.006,0.)for rotation in rotations]
    if broader:
        rotations=[]
        for tilt_deg in [15.,-15.,30.,-30.]:
            angle=np.radians(tilt_deg);z=transverse*np.sin(angle)-np.array([0.,0.,1.])*np.cos(angle)
            R=np.column_stack([np.cross(y,z),y,z])
            rotations.extend([(R,tilt_deg),(R@np.diag([-1.,-1.,1.]),tilt_deg)])
        rotations.sort(key=lambda pair:np.linalg.norm(log(pair[0]@current_R.T)))
        # The two default +15deg/depth6/offset0 postures have already been
        # exhausted over ALL image poses by plan(). Try only alternatives.
        postures=[(rotation,tilt,depth,offset)for offset in [0.,-.004,.004]for depth in [.006,.008]for rotation,tilt in rotations if not(tilt==15. and depth==.006 and offset==0.)]
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
        if time.perf_counter()>deadline:reject('probe_budget');break
        P=np.asarray(pose['center_m']);ring_normal=np.asarray(pose['axis'],float);ring_normal/=np.linalg.norm(ring_normal)
        toward=floor-P;toward-=ring_normal*(toward@ring_normal);distance=np.linalg.norm(toward)
        if distance<.015:continue
        direction=toward/distance;sign=1 if (floor-P)@y>=0 else -1
        for rotation,tilt_deg,depth,offset in postures:
            if time.perf_counter()>deadline:reject('probe_budget');break
            position=P+sign*y*tcp_offset+normal*depth
            if offset:position+=transverse*offset
            H=frame(position,rotation);pre=H.copy();pre[:3,3]-=rotation[:,2]*.105
            # The first 18 mm only takes up bore clearance. Up to another
            # 52 mm physically translates the stock toward the bin centre.
            pull=H.copy();pull[:3,3]+=direction*min(.070,distance+.015)
            trial=dict(tool_tilt_about_bin_Y_deg=tilt_deg,TCP_basket_normal_offset_m=depth,TCP_transverse_offset_m=offset,TCP_transverse_axis=transverse.tolist(),image_center_m=P.tolist(),image_axis=pose['axis'],contact_TCP_world_m=H[:3,3].tolist(),pregrasp_TCP_world_m=pre[:3,3].tolist(),pull_TCP_world_m=pull[:3,3].tolist(),rotation=rotation.tolist(),pregrasp_ik_attempts=0);pose_trials.append(trial)
            bore=bore_proof(c,pose,H,.070)
            trial['bore_proof']=bore
            if not bore['inside_bore']:reject('native_bore_geometry',bore);continue
            clear,why=checker.hand_clear(H,.070)
            if not clear:reject('contact_hand_fixed',dict(trial=trial,collision=why));continue
            if not c['observed_grasp_clearance'].palm_clear(points,pose,pre,H):reject('approach_palm_observed_stock',trial);continue
            if not c['observed_grasp_clearance'].clear(points,pose,pre,H,.070):reject('approach_fingers_observed_other_stock',trial);continue
            seeds=[cmd[0,ids],c['pickup_seed']]+list(np.random.default_rng(9127).uniform(arm.bounds[:,0],arm.bounds[:,1],size=(32,7)));seen=[]
            for seed_index,seed in enumerate(seeds):
                if time.perf_counter()>deadline:reject('probe_budget');break
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
                    trial_direction=direction*np.cos(angle)+np.cross(ring_normal,direction)*np.sin(angle)
                    contact_proof=first_contact_proof(c,pose,H,trial_direction,.070)
                    if not contact_proof['supported']:reject('planned_inner_wall_contact',dict(image_center_m=P.tolist(),direction=trial_direction.tolist(),proof=contact_proof));continue
                    for stroke in [min(.070,distance+.015),.055,.045,.035,.025,.020,.018]:
                        if stroke<.018:continue
                        if contact_proof['first_contact_stroke_m']>stroke-.001:reject('planned_stroke_before_hook_engagement');continue
                        takeup=H.copy();takeup[:3,3]+=trial_direction*.018
                        endpoint=H.copy();endpoint[:3,3]+=trial_direction*stroke
                        unload=endpoint.copy();unload[:3,3]-=trial_direction*.012
                        withdrawn=unload.copy();withdrawn[2,3]=max(1.235,withdrawn[2,3]+.040)
                        if not c['observed_grasp_clearance'].palm_clear(points,pose,H,endpoint):reject('planned_pull_palm_observed_stock');continue
                        if not c['observed_grasp_clearance'].clear(points,pose,H,endpoint,.070):reject('planned_pull_fingers_observed_other_stock');continue
                        stages=[];last_H=H;last_q=qs[-1];blocked=False
                        for label,target in [('takeup',takeup),('pull',endpoint),('unload',unload),('withdraw',withdrawn)]:
                            if not c['observed_grasp_clearance'].palm_clear(points,pose,last_H,target) or not c['observed_grasp_clearance'].clear(points,pose,last_H,target,.070):reject('planned_'+label+'_observed_other_stock');blocked=True;break
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
                        if not blocked:contact_plan=stages;direction=trial_direction;pull=endpoint;trial['planned_stroke_m']=stroke;trial['planned_pull_TCP_world_m']=endpoint[:3,3].tolist();trial['selected_inner_wall_contact_proof']=contact_proof;break
                    if contact_plan is not None:break
                if contact_plan is None:continue
                route=[cmd[0,ids].copy(),preq] if checker.path([cmd[0,ids],preq],.08)[0] else None
                if route is None:
                    for via in c.get('empty_return_waypoints',[]):
                        if checker.path([cmd[0,ids],via,preq],.08)[0]:route=[cmd[0,ids].copy(),via,preq];break
                if route is None:route=checker.route(cmd[0,ids],preq,.08,seconds=.35)
                if route is not None and not all(checker.path(route,float(w))[0] for w in [.070,.075,.080]):reject('empty_approach_aperture_sweep');route=None
                if route is not None:solution=(pose,H,pre,qs,route,direction,pull,contact_plan);break
                reject('empty_robot_approach_route',dict(image_center_m=P.tolist(),q=preq.tolist()))
            if solution:break
        if solution:break
    diagnostics()
    if solution is None:raise RuntimeError('No RGB-D single-finger rearrangement route')
    pose,H,pre,descent,route,direction,pull,contact_plan=solution
    return dict(approved=True,selected_inner_wall_contact_proof=trial['selected_inner_wall_contact_proof'],pose=pose,contact_TCP=H.tolist(),pre_TCP=pre.tolist(),pre_route=np.asarray(route).tolist(),descent=np.asarray(descent).tolist(),pull_direction=direction.tolist(),pull_TCP=pull.tolist(),contact_stage_routes=[np.asarray(v).tolist()for v in contact_plan],counts=counts,examples=examples,pose_trials=pose_trials,robot_motion_executed=False,physical_hook_validated=False)



def plan(c,measurements):
    """Preserve the full proven proposal pass before opt-in alternatives."""
    began=time.perf_counter();budget=float(c['config'].get('axial_receiver_fixture_budget_s',90.))
    try:return _plan(c,measurements)
    except RuntimeError as error:
        if str(error)!='No RGB-D single-finger rearrangement route' or not c['config'].get('broaden_tilted_bore_hook_proposals'):raise
        remaining=budget-(time.perf_counter()-began)
        if remaining<=0:raise
        path=c['vision'].out/'rearrange-plan-diagnostics.json'
        if path.exists():(c['vision'].out/'rearrange-tilted-default-plan-diagnostics.json').write_text(path.read_text())
        alternate=dict(c,config=dict(c['config'],axial_receiver_fixture_budget_s=remaining))
        result=_plan(alternate,measurements,broader=True)
        result['broader_proposal_fallback']=True
        return result


def execute(c):
    began=time.perf_counter();here=Path(__file__).resolve().parent;name=c['config'].get('insertion_probe_source_run','native-roas-table-scatter-stable-v206-policy205-all3-446');source=here.parents[1]/'runs'/name
    observations=json.loads((source/'rgb-observations.json').read_text());initial=[(i,v)for i,v in enumerate(observations,1)if v['mode']=='initial'][-3:];measurements=[]
    for i,row in initial:
        packet_path=source/'rgb-perception'/('observation-%03d-initial'%i)/'rgbd.npz';measurements.append(dict(result=row['result'],packet=dict(np.load(packet_path))))
    if not measurements:raise RuntimeError('No saved initial RGB-D views for static hook')
    trace=np.load(source/'physx-sequence-trace.npz');report=json.loads((source/'pipeline-results.json').read_text());index=int(np.argmin(abs(trace['time_s']-initial[-1][1]['captured_wall_s'])))
    def joints(side):return [report['dof_names'].index(side+'_arm_fr3v2_joint'+str(i))for i in range(1,8)]
    def fingers(side):return [i for i,n in enumerate(report['dof_names'])if n.startswith(side+'_native_')]
    right=trace['q'][index,joints('right')];left=trace['q'][index,joints('left')];width=float(trace['q'][index,fingers('left')].sum());ctx=dict(c,cmd=c['cmd'].copy());ctx['cmd'][0,c['ids']]=right;ctx['cmd'][0,c['left_ids']]=left
    try:
        with c['checker'].other_arm_at(c['checkers']['left'],left,width):result=plan(ctx,measurements)
    except RuntimeError as error:
        if str(error)!='No RGB-D single-finger rearrangement route':raise
        diagnostics_path=c['vision'].out/'rearrange-plan-diagnostics.json';result=dict(approved=False,error=str(error),diagnostics=json.loads(diagnostics_path.read_text())if diagnostics_path.exists()else None)
    result.update(state='static_tilted_single_finger_hook_probe_only',source_run=name,saved_RGBD_camera_times_s=[v['captured_wall_s']for _,v in initial],right_encoder_q=right.tolist(),left_encoder_q=left.tolist(),encoder_sample_wall_s=float(trace['time_s'][index]),elapsed_s=time.perf_counter()-began,full_task_pass=False,uses_free_body_poses=False,robot_motion_executed=False,physical_hook_validated=False)
    print('STATIC_TILTED_BORE_HOOK',json.dumps(result),flush=True);return result
