"""Change a circular ring grasp azimuth, preserving every part/fixture goal."""
from pathlib import Path
import argparse,json,shutil,time
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from source_model import load_model
from fr3_motion_control import LOWER,UPPER
from optimization_clock import require_optimization_window,install_cutoff_alarm


def ease(x):x=np.clip(x,0,1);return x**3*(10-15*x+6*x*x)


def prepare(base,output,angle,respect_cutoff=False):
    if respect_cutoff:require_optimization_window(120);install_cutoff_alarm()
    if output.exists():raise FileExistsError('Use a fresh candidate directory')
    if not np.isfinite(angle)or abs(angle)>120:raise ValueError('Finite ring grasp azimuth within +/-120 degrees required')
    started=time.monotonic();root=Path(__file__).parent;recording=base if(base/'trajectory.npz').exists()else base/'recording';z=np.load(recording/'trajectory.npz');plan=np.load(base/'arm-posture-fr3.npz');ts=z['time_s'];rows={s:plan[s+'_qpos'].copy()for s in ['left','right']}
    model=load_model();old=mujoco.MjData(model);data=mujoco.MjData(model);addresses={s:np.array([int(model.joint(f'{s}_arm_fr3v2_joint{j}').qposadr[0])for j in range(1,8)])for s in rows};dofs=np.array([int(model.joint(f'left_arm_fr3v2_joint{j}').dofadr[0])for j in range(1,8)]);hand=int(model.body('left_grip_base').id);previous=rows['left'][0].copy();max_error=0.;maximum_delta=0.
    fall,end=2.0277083333333334,2.982916666666666
    for k,t in enumerate(ts):
        if respect_cutoff:require_optimization_window()
        weight=ease(t/.36)*ease((end-t)/(end-fall))
        if weight<1e-14:previous=rows['left'][k].copy();continue
        old.qpos[:]=z['qpos'][k]
        for s in rows:old.qpos[addresses[s]]=plan[s+'_qpos'][k]
        mujoco.mj_kinematics(model,old);part=z['mocap_pos'][k,1];part_r=Rotation.from_quat(z['mocap_quat'][k,1,[1,2,3,0]])
        delta=part_r*Rotation.from_rotvec([0,0,np.deg2rad(angle)*weight])*part_r.inv();goal=part+delta.apply(old.xpos[hand]-part);goal_r=delta*Rotation.from_matrix(old.xmat[hand].reshape(3,3))
        data.qpos[:]=old.qpos;q=previous+plan['left_qpos'][k]-plan['left_qpos'][k-1]
        if t>=fall:q=plan['left_qpos'][k]+weight*(q-plan['left_qpos'][k])
        q=np.clip(q,LOWER+.02,UPPER-.02)
        for _ in range(100):
            data.qpos[addresses['left']]=q;mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
            residual=np.r_[10*(data.xpos[hand]-goal),(Rotation.from_matrix(data.xmat[hand].reshape(3,3))*goal_r.inv()).as_rotvec()]
            if np.linalg.norm(residual)<1e-10:break
            jp=np.zeros((3,model.nv));jr=np.zeros_like(jp);mujoco.mj_jacBody(model,data,jp,jr,hand);jac=np.r_[10*jp[:,dofs],jr[:,dofs]];change=-jac.T@np.linalg.solve(jac@jac.T+1e-10*np.eye(6),residual);change*=min(1.,.12/max(float(abs(change).max()),1e-15));q=np.clip(q+change,LOWER+.02,UPPER-.02)
        error=float(np.linalg.norm(residual));max_error=max(max_error,error)
        if error>1e-7:raise RuntimeError(f'Circular grasp IK failed at {t}: {residual.tolist()}')
        rows['left'][k]=q;previous=q.copy();maximum_delta=max(maximum_delta,float(abs(q-plan['left_qpos'][k]).max()))
    output.mkdir(parents=True);np.savez_compressed(output/'arm-posture-fr3.npz',time_s=ts,**{s+'_qpos':q for s,q in rows.items()})
    for n in ['trajectory.npz','config.json','independent_schedule.json','retiming.json']:shutil.copy2(recording/n,output/n)
    report=dict(state='circular_ring_grasp_candidate_unvalidated',hardware_ready=False,part='ring2',angle_deg=angle,part_and_fixture_targets_unchanged=True,maximum_weighted_ik_error=max_error,maximum_joint_delta_rad=maximum_delta,seconds=time.monotonic()-started,scope='Circular ring azimuth symmetry assumed; grip geometry/contact, original CAD and simultaneous paths require new acceptance')
    (output/'arm-posture-fr3.json').write_text(json.dumps(report,indent=2)+'\n');stage=output/'planning-base';(stage/'recording').mkdir(parents=True)
    for n in ['trajectory.npz','config.json','independent_schedule.json','retiming.json']:shutil.copy2(output/n,stage/'recording'/n)
    for n in ['arm-posture-fr3.npz','arm-posture-fr3.json']:shutil.copy2(output/n,stage/n)
    shutil.copy2(root/'assets/reference-dof-names.json',stage/'dof-names.json');print(json.dumps(report,indent=2));return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--angle',type=float,required=True);p.add_argument('--respect-cutoff',action='store_true');a=p.parse_args();prepare(a.base,a.output,a.angle,a.respect_cutoff)
