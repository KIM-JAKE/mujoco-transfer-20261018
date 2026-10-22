"""Move a redundant arm posture while preserving its entire hand-pose path.

No part, fixture or hand target is moved. New robot collisions, dynamics and
measured grasp acceptance must be tested before promoting this experiment.
"""
from pathlib import Path
import argparse,json,shutil,time
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from source_model import load_model
from fr3_motion_control import LOWER,UPPER
from optimization_clock import require_optimization_window,install_cutoff_alarm


def ease(x):x=np.clip(x,0,1);return x**3*(10-15*x+6*x*x)


def prepare(base,output,side,amplitude,window,respect_cutoff=False):
    if respect_cutoff:require_optimization_window(120);install_cutoff_alarm()
    if output.exists():raise FileExistsError('Use a fresh candidate directory')
    begin,rise,fall,end=window
    if not 0<=begin<rise<=fall<end or not np.isfinite(amplitude)or abs(amplitude)>1.5:raise ValueError('Invalid posture perturbation')
    started=time.monotonic();root=Path(__file__).parent;recording=base if(base/'trajectory.npz').exists()else base/'recording'
    z=np.load(recording/'trajectory.npz');plan=np.load(base/'arm-posture-fr3.npz');ts=z['time_s'];rows={s:plan[s+'_qpos'].copy()for s in ['left','right']}
    model=load_model();old=mujoco.MjData(model);data=mujoco.MjData(model);addresses={s:np.array([int(model.joint(f'{s}_arm_fr3v2_joint{j}').qposadr[0])for j in range(1,8)])for s in rows}
    dofs=np.array([int(model.joint(f'{side}_arm_fr3v2_joint{j}').dofadr[0])for j in range(1,8)]);hand=int(model.body(side+'_grip_base').id);previous=rows[side][0].copy();previous_null=None;max_error=0.;max_delta=0.
    for k,t in enumerate(ts):
        if respect_cutoff:require_optimization_window()
        weight=ease((t-begin)/(rise-begin))*ease((end-t)/(end-fall))
        if weight<1e-14:previous=rows[side][k].copy();continue
        old.qpos[:]=z['qpos'][k]
        for s in rows:old.qpos[addresses[s]]=plan[s+'_qpos'][k]
        mujoco.mj_kinematics(model,old);mujoco.mj_comPos(model,old)
        goal=old.xpos[hand].copy();goal_r=Rotation.from_matrix(old.xmat[hand].reshape(3,3));jp=np.zeros((3,model.nv));jr=np.zeros_like(jp);mujoco.mj_jacBody(model,old,jp,jr,hand)
        _,_,vt=np.linalg.svd(np.r_[10*jp[:,dofs],jr[:,dofs]],full_matrices=True);null=vt[-1]
        if previous_null is None:
            if null[np.argmax(abs(null))]<0:null=-null
        elif previous_null@null<0:null=-null
        previous_null=null;data.qpos[:]=old.qpos
        q=np.clip(previous+plan[side+'_qpos'][k]-plan[side+'_qpos'][k-1],LOWER+.015,UPPER-.015)
        for _ in range(80):
            data.qpos[addresses[side]]=q;mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
            residual=np.r_[10*(data.xpos[hand]-goal),(Rotation.from_matrix(data.xmat[hand].reshape(3,3))*goal_r.inv()).as_rotvec(),.3*(null@(q-plan[side+'_qpos'][k])-weight*amplitude)]
            if np.linalg.norm(residual)<1e-10:break
            mujoco.mj_jacBody(model,data,jp,jr,hand);jac=np.r_[10*jp[:,dofs],jr[:,dofs],.3*null[None,:]]
            delta=-jac.T@np.linalg.solve(jac@jac.T+1e-10*np.eye(7),residual);delta*=min(1.,.1/max(float(abs(delta).max()),1e-15));q=np.clip(q+delta,LOWER+.015,UPPER-.015)
        error=float(np.linalg.norm(residual));max_error=max(max_error,error)
        if error>1e-7:raise RuntimeError(f'Posture IK failed at {t}, residual {residual.tolist()}')
        rows[side][k]=q;previous=q.copy();max_delta=max(max_delta,float(abs(q-plan[side+'_qpos'][k]).max()))
    output.mkdir(parents=True);np.savez_compressed(output/'arm-posture-fr3.npz',time_s=ts,**{s+'_qpos':q for s,q in rows.items()})
    for n in ['trajectory.npz','config.json','independent_schedule.json','retiming.json']:shutil.copy2(recording/n,output/n)
    report=dict(state='nullspace_posture_candidate_unvalidated',hardware_ready=False,side=side,amplitude_rad=amplitude,window=list(window),hand_targets_unchanged=True,part_and_fixture_targets_unchanged=True,maximum_weighted_FK_residual=max_error,maximum_joint_delta_rad=max_delta,seconds=time.monotonic()-started)
    (output/'arm-posture-fr3.json').write_text(json.dumps(report,indent=2)+'\n');stage=output/'planning-base';(stage/'recording').mkdir(parents=True)
    for n in ['trajectory.npz','config.json','independent_schedule.json','retiming.json']:shutil.copy2(output/n,stage/'recording'/n)
    for n in ['arm-posture-fr3.npz','arm-posture-fr3.json']:shutil.copy2(output/n,stage/n)
    shutil.copy2(root/'assets/reference-dof-names.json',stage/'dof-names.json');print(json.dumps(report,indent=2));return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--side',choices=['left','right'],default='left');p.add_argument('--amplitude',type=float,required=True);p.add_argument('--window',nargs=4,type=float,default=[.1,.8,2.2,2.983]);p.add_argument('--respect-cutoff',action='store_true');a=p.parse_args();prepare(a.base,a.output,a.side,a.amplitude,a.window,a.respect_cutoff)
