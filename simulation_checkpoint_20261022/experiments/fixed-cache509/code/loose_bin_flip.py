"""Image-guided flip-and-drop into the EXISTING basket for regrasp.

All workpiece motion is PhysX contact/gravity. No body pose writes, extra
support, identity or scene seed is used. A fresh image must authorize release.
"""
from pathlib import Path
import types,time,numpy as np
from isaac_kinematics import frame
from rgbd_geometry import axis_frame
from fr3_path_timing import retime
from grasp_transfer_feasibility import gripper_boxes
from predictive_pair_clearance import proxy,shape_overlap

def execute(c):
    h=Path(__file__).resolve().parent;arm=c['arm'];art=c['art'];cmd=c['cmd'];ids=c['ids'];fi=c['fi'];checker=c['checker'];dt=c['dt'];vision=c['vision'];events=[]
    module=types.ModuleType('flip_pick');module.__file__=str(h/'loose_ring_pick.py');exec(compile((h/'loose_ring_pick.py').read_text(),module.__file__,'exec'),module.__dict__)
    context=dict(c,config=dict(c['config'],allow_internal_grasp=False,prefer_internal_small_grasp=False,prefer_transfer_compatible=False,flip_pick=True,stable_small_grasp=False),handover_feasible_grasp=None,grasp_pad_aperture=None)
    context.pop('prefetched_initial',None);pickup=module.execute(context);G=np.asarray(pickup['measured_grasp']);axis=G[:3,2];radial=axis.copy();radial[2]=0.;radial/=max(np.linalg.norm(radial),1e-12)
    if np.linalg.norm(axis[:2])<.35:raise RuntimeError('Flip recovery requires an oblique physical grasp')
    local_basis=np.column_stack([radial,np.cross([0,0,1.],radial),[0,0,1.]])
    floor=np.asarray(c['calibration']['floor_origin_world_m']);release_part=np.r_[floor[:2],1.310918206667893]
    cad=c['observed_grasp_clearance'].estimator.models[c['config']['pick_part']]
    solution=None
    checker.attach_payload(c['part_collision_templates'][c['config']['pick_part']],frame([0,0,.1034],np.eye(3))@G)
    try:
        for approach in [[1.,0,0],[-1.,0,0],[0,1.,0],[0,-1.,0]]:
            z=np.asarray(approach);R=np.column_stack([[0,0,1.],np.cross(z,[0,0,1.]),z])@local_basis.T
            H=frame(release_part-R@G[:3,3],R);part=H@G;v=cad@part[:3,:3].T+part[:3,3]
            fall=proxy(np.r_[v,v-[0,0,.24]])
            if any(shape_overlap(fall,box,margin=.0005)for box in gripper_boxes(checker,H,.08)):continue
            for seed in [cmd[0,ids],c['pickup_seed']]+list(c.get('empty_return_waypoints',[])):
                q=arm.solve(H[:3,3],H[:3,:3],seed)
                if q is None:continue
                route=checker.route(cmd[0,ids],q,float(art.get_dof_positions()[0,fi].sum()),seconds=1.)
                if route is not None:solution=route;break
            if solution is not None:break
    finally:checker.detach_payload()
    if solution is None:raise RuntimeError('No collision-free flip pose over existing basket')
    def move(qs,label,width=None):
        qs=np.asarray(qs);qs[0]=cmd[0,ids];start=cmd[0,fi].copy();end=start if width is None else np.full(2,width/2)
        q,w,timing=retime(qs,.3,dt,start,end,optimize=False)
        scale=max(1.,float(c['config'].get('stability_time_scale',1.)))
        q,w,timing=retime(qs,timing['duration_s']*scale,dt,start,end,optimize=False)
        for row,u in zip(q[1:],w[1:]):cmd[0,ids]=row;cmd[0,fi]=start+(end-start)*u;c['step'](label)
        events.append(dict(event=label,wall_s=time.perf_counter()-c['started']));print('BIN_FLIP',label,events[-1]['wall_s'],flush=True)
    def wait(seconds,label):
        for _ in range(int(np.ceil(seconds/dt))):c['step'](label)
    move(solution,'ring2_flip_above_existing_bin');wait(.25,'ring2_flip_hold')
    measured=None
    for offset in [[.12,-.08,.30],[-.12,.08,.30],[.15,-.25,.15]]:
        event=dict(mode='held',part='ring2',tracked_grasp=G.tolist(),camera_eye_offset=offset,label='flip_before_release_RGBD',require_annular_face=True);future=vision.request(event,arm.fk(art.get_dof_positions()[0,ids]));deadline=time.perf_counter()+3
        while not future.done():
            vision.poll();c['step']('ring2_flip_RGBD')
            if time.perf_counter()>deadline:raise RuntimeError('Flip RGB-D deadline')
        value=future.result();event['delivered_wall_s']=time.perf_counter()-c['started']
        if not value['result'].get('error'):measured=value['result']['pose'];break
    if measured is None:raise RuntimeError('Flip release blocked: ring not visibly held')
    P=np.asarray(measured['center_m']);n=np.asarray(measured['axis'])
    if n[2]<.35 or np.linalg.norm(P[:2]-floor[:2])>.045 or P[2]<1.26:raise RuntimeError('Flip release blocked: observed pose outside basket drop region')
    c['set_grip_hold']('right',False);move([cmd[0,ids],cmd[0,ids]],'ring2_flip_release',.08);wait(.7,'ring2_flip_gravity_settle')
    H=arm.fk(cmd[0,ids]);H[2,3]+=.08;q=arm.solve(H[:3,3],H[:3,:3],cmd[0,ids]);route=checker.route(cmd[0,ids],q,.08,seconds=.3) if q is not None else None
    if route is not None:move(route,'ring2_flip_empty_withdraw',.08)
    wait(.7,'ring2_flip_settle_before_new_RGBD')
    return dict(state='physical_flip_drop_requires_new_pick',events=events,release_observed_pose=measured,uses_free_body_poses=False,workpiece_teleports=0,extra_supports=False,full_task_pass=False)
