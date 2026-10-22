"""Reproduce an experimental original-CAD spatial candidate from the bundled seed.

This creates motor targets and observed-part pose references only. It does not
move a PhysX body, certify contact, or export a real-robot playback program.
"""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import numpy as np
import mujoco
from scipy.interpolate import CubicSpline
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp
from source_model import load_model
from task_timing_fast import source_time
from task_path_offsets import pin_clearance_z
from fr3_motion_control import LOWER, UPPER
from optimization_clock import require_optimization_window, install_cutoff_alarm


def ease(u):
    u = np.clip(u, 0., 1.)
    return u**3 * (10.-15.*u+6.*u*u)


def prepare(output, respect_cutoff=False, pin_angle=-60., analytic_ik=False, pin_blend=.07):
    if not np.isfinite(pin_angle) or abs(pin_angle)>180:
        raise ValueError('Pin grasp angle must be finite and within +/-180 degrees')
    if not np.isfinite(pin_blend) or not .01<=pin_blend<=.5:
        raise ValueError('Pin shortcut blend must be between 0.01 and 0.5 reference seconds')
    if respect_cutoff:
        require_optimization_window(120)
        install_cutoff_alarm()
    root = Path(__file__).resolve().parent
    recording = root/'generated/contact_task/fast'
    old = np.load(root/'generated/contact_task/arm-posture-fr3.npz')
    z = np.load(recording/'trajectory.npz')
    ts = z['time_s']
    model = load_model()
    data = mujoco.MjData(model)
    addresses = {s:np.array([int(model.joint(f'{s}_arm_fr3v2_joint{j}').qposadr[0]) for j in range(1,8)]) for s in ['left','right']}
    dofs = np.array([int(model.joint(f'left_arm_fr3v2_joint{j}').dofadr[0]) for j in range(1,8)])
    hands = {s:int(model.body(s+'_grip_base').id) for s in addresses}
    rows = {s:old[s+'_qpos'].copy() for s in addresses}
    pin_rotation = Slerp(ts,Rotation.from_quat(z['mocap_quat'][:,2][:,[1,2,3,0]]))
    previous = rows['left'][0].copy()
    bounds = (LOWER+.03, UPPER-.03)
    errors = []
    # Rotate about the cylindrical neck without changing the part's goal pose.
    for k,t in enumerate(ts):
        if respect_cutoff:require_optimization_window(30)
        if t<2.0277:
            previous=rows['left'][k]
            continue
        data.qpos[:]=z['qpos'][k]
        for s in addresses:data.qpos[addresses[s]]=old[s+'_qpos'][k]
        mujoco.mj_kinematics(model,data)
        p=data.xpos[hands['left']].copy()
        r=Rotation.from_matrix(data.xmat[hands['left']].reshape(3,3))
        part=z['mocap_pos'][k,2].copy()
        part[2]+=pin_clearance_z(source_time('left',float(t)))
        pr=pin_rotation(float(t))
        delta=pr*Rotation.from_rotvec([0,0,np.deg2rad(pin_angle)*ease((t-2.0277)/.85)])*pr.inv()
        goal_p=part+delta.apply(p-part)
        goal_r=delta*r
        def residual(q):
            data.qpos[addresses['left']]=q
            mujoco.mj_kinematics(model,data)
            if analytic_ik:mujoco.mj_comPos(model,data)
            return np.r_[10*(data.xpos[hands['left']]-goal_p),(Rotation.from_matrix(data.xmat[hands['left']].reshape(3,3))*goal_r.inv()).as_rotvec()]
        seed=previous+(old['left_qpos'][k]-old['left_qpos'][k-1])
        solution=np.clip(seed,*bounds)
        for _ in range(60):
            error=residual(solution)
            if np.linalg.norm(error)<1e-10:break
            jp=np.zeros((3,model.nv));jr=np.zeros_like(jp)
            mujoco.mj_jacBody(model,data,jp,jr,hands['left'])
            jac=np.r_[10*jp[:,dofs],jr[:,dofs]]
            change=-jac.T@np.linalg.solve(jac@jac.T+1e-10*np.eye(6),error)
            change*=min(1.,.15/max(float(np.max(abs(change))),1e-15))
            solution=np.clip(solution+change,*bounds)
        if np.linalg.norm(residual(solution))>1e-8:
            solution=least_squares(residual,solution,bounds=bounds,max_nfev=1000,ftol=1e-13,xtol=1e-13,gtol=1e-13).x
        error=residual(solution)
        if np.linalg.norm(error[:3])>1e-5 or np.linalg.norm(error[3:])>1e-5:
            raise RuntimeError(f'Pin grasp IK failed at {t}: {error}')
        rows['left'][k]=solution
        previous=solution.copy()
        errors.append(float(np.linalg.norm(error)))
    original={s:CubicSpline(ts,rows[s],axis=0) for s in rows}
    segments=[('left','ring2',.5797916666666667,1.538125,[.819375,1.2985416666666667],1),
              ('left','empty',2.121458333333333,2.65,[2.3857291666666667],None),
              ('left','pin',3.357916666666667,5.107916666666667,[],2),
              ('right','ring1',2.3,3.408333333333333,[],0),
              ('right','empty',5.1,6.447975,[],None)]
    mp=z['mocap_pos'].copy();mq=z['mocap_quat'].copy()
    old_data=mujoco.MjData(model)
    rotations=[Slerp(ts,Rotation.from_quat(mq[:,i][:,[1,2,3,0]])) for i in range(3)]
    for side,label,begin,end,mids,index in segments:
        points=[begin,*mids,end]
        curve=CubicSpline(points,original[side](points),axis=0,bc_type=((1,np.zeros(7)),(1,np.zeros(7))))
        active=(ts>=begin)&(ts<=end)
        ticks=ts[active]
        width=pin_blend if label=='pin'else .07
        blend=ease((ticks-begin)/width)*ease((end-ticks)/width)
        rows[side][active]=original[side](ticks)+(curve(ticks)-original[side](ticks))*blend[:,None]
        if index is None:continue
        for k in np.flatnonzero(active):
            t=float(ts[k])
            old_data.qpos[:]=z['qpos'][k]
            for s in addresses:old_data.qpos[addresses[s]]=original[s](t)
            mujoco.mj_kinematics(model,old_data)
            data.qpos[:]=old_data.qpos
            data.qpos[addresses[side]]=rows[side][k]
            mujoco.mj_kinematics(model,data)
            hand=hands[side]
            delta=data.xmat[hand].reshape(3,3)@old_data.xmat[hand].reshape(3,3).T
            p=z['mocap_pos'][k,index].copy()
            if index==2:p[2]+=pin_clearance_z(source_time('left',t))
            mp[k,index]=data.xpos[hand]+delta@(p-old_data.xpos[hand])
            if index==2:mp[k,index,2]-=pin_clearance_z(source_time('left',t))
            mq[k,index]=Rotation.from_matrix(delta@rotations[index](t).as_matrix()).as_quat()[[3,0,1,2]]
    output.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(output/'arm-posture-fr3.npz',time_s=ts,**{s+'_qpos':rows[s] for s in rows})
    np.savez_compressed(output/'trajectory.npz',time_s=ts,qpos=z['qpos'],mocap_pos=mp,mocap_quat=mq)
    for f in ['config.json','independent_schedule.json','retiming.json']:shutil.copy2(recording/f,output/f)
    report=dict(state='experimental_spatial_candidate_unvalidated',hardware_ready=False,pin_grasp_angle_deg=pin_angle,maximum_weighted_ik_error=max(errors),segments=[dict(side=s,part=n,begin=b,end=e,midpoints=m)for s,n,b,e,m,i in segments],requires=['robot/static/carried-part geometry screen','stock-pin preclosure screen','new simultaneous-path collision audit','native PhysX contact and FR3 acceptance'],archive_sha256=hashlib.sha256((output/'arm-posture-fr3.npz').read_bytes()).hexdigest())
    report.update(analytic_ik=bool(analytic_ik),pin_shortcut_blend_reference_s=float(pin_blend))
    (output/'arm-posture-fr3.json').write_text(json.dumps(report,indent=2)+'\n')
    base=output/'planning-base';(base/'recording').mkdir(parents=True,exist_ok=True)
    for f in ['arm-posture-fr3.npz','arm-posture-fr3.json']:shutil.copy2(output/f,base/f)
    for f in ['trajectory.npz','config.json','independent_schedule.json','retiming.json']:shutil.copy2(output/f,base/'recording'/f)
    shutil.copy2(root/'assets/reference-dof-names.json',base/'dof-names.json')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--respect-cutoff',action='store_true',help='Enforce the optimization deadline; omit for later reproduction')
    parser.add_argument('--pin-angle',type=float,default=-60.)
    parser.add_argument('--analytic-ik',action='store_true')
    parser.add_argument('--pin-blend',type=float,default=.07)
    args=parser.parse_args()
    prepare(args.output,args.respect_cutoff,args.pin_angle,args.analytic_ik,args.pin_blend)
