"""Test setup only. Seeded perturbations never enter perception or control.

Rigid bodies may be initialized before the first camera snapshot. Subsequent
poses are recorded strictly for test evidence, never returned to the controller.
"""
import json,numpy as np
from scipy.spatial.transform import Rotation

def apply(view,model,calibration,config,settle_step=None):
    rng=np.random.default_rng(int(config['seed']));R=np.array(calibration['basket_to_world_R']);floor=np.array(calibration['floor_origin_world_m']);normal=R[:,2];records=[];views=[]
    scatter=config.get('randomization_mode')=='full_bin_drop'
    with np.load(model)as archive:
        parked={}
        if scatter:
            # Initialize outside the working scene, then release one at a time
            # just above the rim. This avoids a metre-high rain striking CAD
            # above the basket; all initialization precedes the task clock.
            for path in archive['paths'].tolist():
                if path.endswith(('workpiece_ring1','workpiece_ring2')) or path.startswith('/World/BinStock/'):
                    b=view.create_rigid_body_view(path);old=b.get_transforms().copy();parked[path]=(b,old);park=old.copy();park[0,:3]=[4.+len(parked)*.2,4.,3.];b.set_transforms(park,np.array([0],np.uint32));b.set_velocities(np.zeros((1,6),np.float32),np.array([0],np.uint32))
        paths=archive['paths'].tolist()
        order=[i for i,path in enumerate(paths) if path.endswith(('workpiece_ring1','workpiece_ring2')) or path.startswith('/World/BinStock/')]
        if scatter and config.get('shuffle_drop_order',True):rng.shuffle(order)
        for i in order:
            path=paths[i]
            body,old=parked[path] if scatter else (view.create_rigid_body_view(path),None);old=body.get_transforms().copy() if old is None else old;tr=old.copy();primary='/BinStock/'not in path
            if scatter:
                # All ten bodies use the same sampling rule. There are no
                # privileged target locations or collision-free layout searches.
                vertices=archive[f'v{i}'];radius=float(np.max(np.linalg.norm(vertices,axis=1)))
                half=np.array(calibration['outer_dimensions_in_basket_frame_m'][:2])/2-.012-radius
                local_xy=rng.uniform(-half,half);Q=Rotation.from_matrix(R)*Rotation.random(random_state=rng)
                drop_height=.15
                tr[0,:3]=floor+R@np.r_[local_xy,drop_height];tr[0,3:]=Q.as_quat()
                body.set_transforms(tr,np.array([0],np.uint32));body.set_velocities(np.zeros((1,6),np.float32),np.array([0],np.uint32))
                views.append((path,body,old.copy()));records.append(dict(path=path,primary=primary,distribution='uniform bin XY, uniform SO(3), randomized body drop order',drop_rank=len(records),requested_bin_xy_m=local_xy.tolist(),drop_height_above_floor_m=drop_height,initialized_transform=tr[0].tolist(),original_transform=old[0].tolist()))
                if settle_step is not None:
                    for _ in range(240):settle_step()
                continue
            xy=float(config.get('primary_xy_mm',8.)if primary else config.get('clutter_xy_mm',20.))/1000;tilt=float(config.get('primary_tilt_deg',12.)if primary else config.get('clutter_tilt_deg',20.));dx,dy=rng.uniform(-xy,xy,2);rx,ry=rng.uniform(-tilt,tilt,2);yaw=rng.uniform(-180,180);Q=(Rotation.from_matrix(R)*Rotation.from_euler('xyz',[rx,ry,yaw],degrees=True))if primary else(Rotation.from_rotvec(R@np.radians([rx,ry,yaw*.05]))*Rotation.from_quat(old[0,3:]));tr[0,3:]=Q.as_quat();tr[0,:3]+=R@np.array([dx,dy,0.]);vertices=archive[f'v{i}'];support=-float(np.min((vertices@Q.as_matrix().T)@normal));height=float((tr[0,:3]-floor)@normal);target=max(support+.001,height)if not primary else support+.0005;tr[0,:3]+=(target-height)*normal
            body.set_transforms(tr,np.array([0],np.uint32));body.set_velocities(np.zeros((1,6),np.float32),np.array([0],np.uint32));views.append((path,body,old.copy()));records.append(dict(path=path,primary=primary,requested_local_xy_m=[dx,dy],requested_local_tilt_deg=[rx,ry],requested_local_yaw_deg=yaw,initialized_transform=tr[0].tolist(),original_transform=old[0].tolist()))
    assert len(records)==10
    return records,views

def save(out,config,records,views,calibration):
    R=np.array(calibration['basket_to_world_R']);normal=R[:,2]
    for rec,(path,body,old)in zip(records,views):
        tr=body.get_transforms()[0];rec['settled_transform']=tr.tolist();rec['settled_local_translation_mm']=((tr[:3]-old[0,:3])@R*1000).tolist();axis=Rotation.from_quat(tr[3:]).as_matrix()[:,2];rec['settled_axis_tilt_from_floor_deg']=float(np.degrees(np.arccos(np.clip(abs(axis@normal),-1,1))))
    (out/'randomization-evidence.json').write_text(json.dumps(dict(config=config,setup_before_first_capture=True,controller_has_random_seed_or_true_poses=False,ring_count=10,target_body_privileged=False if config.get('randomization_mode')=='full_bin_drop' else True,records=records),indent=2))
