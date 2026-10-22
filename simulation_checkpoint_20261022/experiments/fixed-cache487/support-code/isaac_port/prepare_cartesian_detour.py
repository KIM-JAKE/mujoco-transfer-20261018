"""Translate a carried-ring hand path; preserve pickup, insertion and fixed CAD.

Only motor targets and pose references change. No workpiece is teleported.
Every candidate requires independent geometry, timing and native acceptance.
"""
from pathlib import Path
import argparse,json,shutil,time
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from source_model import load_model
from fr3_motion_control import LOWER,UPPER
from optimization_clock import require_optimization_window,install_cutoff_alarm


def ease(x):x=np.clip(x,0,1);return x**3*(10-15*x+6*x*x)


def prepare(base,output,translation,window,respect_cutoff=False):
    if respect_cutoff:require_optimization_window(120);install_cutoff_alarm()
    if output.exists():raise FileExistsError('Fresh candidate directory required')
    begin,rise,fall,end=window;translation=np.asarray(translation,dtype=float)
    if not 1.71875<=begin<rise<=fall<end<=3.408333333333333 or not np.isfinite(translation).all()or np.linalg.norm(translation)>.3:raise ValueError('Detour must stay inside ring1 transfer; finite translation <=0.3m required')
    started=time.monotonic();root=Path(__file__).parent;recording=base if(base/'trajectory.npz').exists()else base/'recording';z=np.load(recording/'trajectory.npz');plan=np.load(base/'arm-posture-fr3.npz');ts=z['time_s'];rows={s:plan[s+'_qpos'].copy()for s in ['left','right']};positions=z['mocap_pos'].copy()
    model=load_model();old=mujoco.MjData(model);data=mujoco.MjData(model);addresses={s:np.array([int(model.joint(f'{s}_arm_fr3v2_joint{j}').qposadr[0])for j in range(1,8)])for s in rows};dofs=np.array([int(model.joint(f'right_arm_fr3v2_joint{j}').dofadr[0])for j in range(1,8)]);hand=int(model.body('right_grip_base').id);previous=rows['right'][0].copy();max_error=0.;max_delta=0.
    for k,t in enumerate(ts):
        if respect_cutoff:require_optimization_window()
        weight=ease((t-begin)/(rise-begin))*ease((end-t)/(end-fall))
        if weight<1e-14:previous=rows['right'][k].copy();continue
        old.qpos[:]=z['qpos'][k]
        for s in rows:old.qpos[addresses[s]]=plan[s+'_qpos'][k]
        mujoco.mj_kinematics(model,old);goal=old.xpos[hand]+weight*translation;goal_r=Rotation.from_matrix(old.xmat[hand].reshape(3,3));data.qpos[:]=old.qpos
        q=previous+plan['right_qpos'][k]-plan['right_qpos'][k-1]
        if t>=fall:q=plan['right_qpos'][k]+weight*(q-plan['right_qpos'][k])
        q=np.clip(q,LOWER+.02,UPPER-.02)
        for _ in range(100):
            data.qpos[addresses['right']]=q;mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
            residual=np.r_[10*(data.xpos[hand]-goal),(Rotation.from_matrix(data.xmat[hand].reshape(3,3))*goal_r.inv()).as_rotvec()]
            if np.linalg.norm(residual)<1e-10:break
            jp=np.zeros((3,model.nv));jr=np.zeros_like(jp);mujoco.mj_jacBody(model,data,jp,jr,hand);jac=np.r_[10*jp[:,dofs],jr[:,dofs]];change=-jac.T@np.linalg.solve(jac@jac.T+1e-10*np.eye(6),residual);change*=min(1.,.12/max(float(abs(change).max()),1e-15));q=np.clip(q+change,LOWER+.02,UPPER-.02)
        error=float(np.linalg.norm(residual));max_error=max(max_error,error)
        if error>1e-7:raise RuntimeError(f'Detour IK failed at {t}: {residual.tolist()}')
        rows['right'][k]=q;previous=q.copy();positions[k,0]+=weight*translation;max_delta=max(max_delta,float(abs(q-plan['right_qpos'][k]).max()))
    output.mkdir(parents=True);np.savez_compressed(output/'arm-posture-fr3.npz',time_s=ts,**{s+'_qpos':q for s,q in rows.items()})
    np.savez_compressed(output/'trajectory.npz',**{n:(positions if n=='mocap_pos'else z[n])for n in z.files})
    for n in ['config.json','independent_schedule.json','retiming.json']:shutil.copy2(recording/n,output/n)
    report=dict(state='cartesian_ring_detour_candidate_unvalidated',hardware_ready=False,side='right',part='ring1',translation_world_m=translation.tolist(),window=list(window),pickup_insertion_and_original_fixed_CAD_unchanged=True,maximum_weighted_ik_error=max_error,maximum_joint_delta_rad=max_delta,seconds=time.monotonic()-started)
    (output/'arm-posture-fr3.json').write_text(json.dumps(report,indent=2)+'\n');stage=output/'planning-base';(stage/'recording').mkdir(parents=True)
    for n in ['trajectory.npz','config.json','independent_schedule.json','retiming.json']:shutil.copy2(output/n,stage/'recording'/n)
    for n in ['arm-posture-fr3.npz','arm-posture-fr3.json']:shutil.copy2(output/n,stage/n)
    shutil.copy2(root/'assets/reference-dof-names.json',stage/'dof-names.json');print(json.dumps(report,indent=2));return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--translation',nargs=3,type=float,required=True);p.add_argument('--window',nargs=4,type=float,default=[1.72,2.15,2.65,3.35]);p.add_argument('--respect-cutoff',action='store_true');a=p.parse_args();prepare(a.base,a.output,a.translation,a.window,a.respect_cutoff)
