"""Adapt pickup/transport poses to observations while fixing insertion goals.

The resulting artifact is a new unvalidated spatial plan. It must be collision
screened, retimed and tested; an optimized joint tape is never globally shifted.
"""
from pathlib import Path
import argparse,hashlib,json,shutil,time
import numpy as np,mujoco
from scipy.interpolate import CubicSpline
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation,Slerp
from source_model import load_model
from fr3_motion_control import LOWER,UPPER
from task_timing_fast import source_time
from task_path_offsets import pin_clearance_z
from fieldkit.perception import PARTS,pose,transform
from optimization_clock import require_optimization_window,install_cutoff_alarm


def ease(x):x=np.clip(x,0,1);return x**3*(10-15*x+6*x*x)


def prepare(base,observations,output,respect_cutoff=False,pin_holder_tracks_pin=False):
    if respect_cutoff:require_optimization_window(120);install_cutoff_alarm()
    started=time.monotonic();root=Path(__file__).resolve().parent;recording=base if(base/'trajectory.npz').exists()else base/'recording'
    observation=json.loads(observations.read_text())
    if observation.get('output_frame')!='cell' or observation.get('units')!='m' or set(observation['parts'])!=set(PARTS):raise ValueError('Expected all three metric poses in calibrated cell frame')
    source=json.loads((root/'generated/contact_task/contact-source.json').read_text());z=np.load(recording/'trajectory.npz');plan=np.load(base/'arm-posture-fr3.npz');ts=z['time_s'];schedule=json.loads((recording/'independent_schedule.json').read_text())
    model=load_model();data=mujoco.MjData(model);nominal_data=mujoco.MjData(model)
    addresses={s:np.array([int(model.joint(f'{s}_arm_fr3v2_joint{j}').qposadr[0])for j in range(1,8)])for s in ['left','right']};dofs={s:np.array([int(model.joint(f'{s}_arm_fr3v2_joint{j}').dofadr[0])for j in range(1,8)])for s in addresses};hands={s:int(model.body(s+'_grip_base').id)for s in addresses}
    old={s:plan[s+'_qpos'].copy()for s in addresses};rows={s:q.copy()for s,q in old.items()};positions=z['mocap_pos'].copy();quaternions=z['mocap_quat'].copy();windows={};deltas={}
    for name,side,approach,grasp,lift,transfer in [('ring2','left','approach_two_parts','grasp_two_parts','lift_two_parts_30mm','transfer_two_parts'),('ring1','right','approach_two_parts','grasp_two_parts','lift_two_parts_30mm','transfer_two_parts'),('pin','left','approach_last_pin','grasp_last_pin','lift_last_pin_30mm','transfer_last_pin')]:
        row=observation['parts'][name]
        quality=np.asarray([row['rms_m'],row['inlier_fraction']],float)
        if not np.isfinite(quality).all() or not 0<=quality[0]<=.0015 or not .6<=quality[1]<=1 or row.get('inliers',0)<150:
            raise RuntimeError('Pickup observation quality is insufficient: '+name)
        observed=transform(row['T_cell_part']);prior=pose(source['parts'][name]['position'],source['parts'][name]['quaternion']);delta_p=observed[:3,3]-prior[:3,3];delta_r=Rotation.from_matrix(observed[:3,:3]@prior[:3,:3].T)
        if np.linalg.norm(delta_p)>.035 or np.max(abs(delta_p))>.035 or delta_r.magnitude()>.2:raise RuntimeError('Observed part outside current adaptation envelope: '+name)
        deltas[name]=(delta_p,delta_r.as_rotvec());lane=schedule['lanes'][side]
        at=lambda label,key:next(r[key]for r in lane if r['stage']['label']==label)
        lift_end=at(lift,'end_s');lift_begin=at(grasp,'end_s')
        # Extract first, then join the checked 30mm-lift endpoint. The pin
        # remains over its relocated sleeve until most extraction is complete.
        fade_start=lift_begin+(.6 if name=='pin'else .2)*(lift_end-lift_begin)
        windows[name]=(side,at(approach,'start_s'),at(grasp,'start_s'),fade_start,lift_end)
    max_error=0.;maximum_joint_delta=0.;previous={s:old[s][0].copy()for s in old}
    for k,t in enumerate(ts):
        if respect_cutoff:require_optimization_window()
        nominal_data.qpos[:]=z['qpos'][k]
        for s in old:nominal_data.qpos[addresses[s]]=old[s][k]
        mujoco.mj_kinematics(model,nominal_data)
        for name in PARTS:
            side,begin,grasp,lift,end=windows[name];delta_p,rv=deltas[name];index=PARTS.index(name)
            part_weight=1.-ease((t-lift)/(end-lift)) if t>=lift else 1.
            delta=Rotation.from_rotvec(rv*part_weight);positions[k,index]=z['mocap_pos'][k,index]+part_weight*delta_p;quaternions[k,index]=(delta*Rotation.from_quat(z['mocap_quat'][k,index,[1,2,3,0]])).as_quat()[[3,0,1,2]]
        for side in old:
            name=next((n for n,(s,b,g,l,e)in windows.items()if s==side and b<t<e),None)
            if name is None:previous[side]=old[side][k].copy();continue
            _,begin,grasp,lift,end=windows[name]
            weight=ease((t-begin)/(grasp-begin)) if t<grasp else 1.-ease((t-lift)/(end-lift))if t>=lift else 1.
            delta_p,rv=deltas[name];delta=Rotation.from_rotvec(rv*weight);part=z['mocap_pos'][k,PARTS.index(name)].copy()
            if name=='pin':part[2]+=pin_clearance_z(source_time('left',float(t)))
            hand=hands[side];goal_p=part+weight*delta_p+delta.apply(nominal_data.xpos[hand]-part);goal_r=delta*Rotation.from_matrix(nominal_data.xmat[hand].reshape(3,3))
            data.qpos[:]=nominal_data.qpos
            def residual(q):
                data.qpos[addresses[side]]=q;mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data)
                return np.r_[10*(data.xpos[hand]-goal_p),(Rotation.from_matrix(data.xmat[hand].reshape(3,3))*goal_r.inv()).as_rotvec()]
            lower=np.maximum(LOWER+.005,old[side][k]-.25);upper=np.minimum(UPPER-.005,old[side][k]+.25)
            seed=previous[side]+old[side][k]-old[side][k-1]
            # Fade the warm-start redundancy together with the Cartesian
            # pickup offset. A fixed extra joint constraint can become poorly
            # conditioned during approach; free accumulated redundancy would
            # otherwise leave a joint jump when the baseline resumes.
            if t>=lift:seed=old[side][k]+weight*(seed-old[side][k])
            solution=np.clip(seed,lower,upper)
            for _ in range(40):
                error=residual(solution)
                if np.linalg.norm(error)<1e-10:break
                jp=np.zeros((3,model.nv));jr=np.zeros_like(jp);mujoco.mj_jacBody(model,data,jp,jr,hand);jac=np.r_[10*jp[:,dofs[side]],jr[:,dofs[side]]];change=-jac.T@np.linalg.solve(jac@jac.T+1e-10*np.eye(6),error);change*=min(1.,.1/max(float(abs(change).max()),1e-15));solution=np.clip(solution+change,lower,upper)
            if np.linalg.norm(residual(solution))>1e-8:solution=least_squares(residual,solution,bounds=(lower,upper),max_nfev=200,ftol=1e-12,xtol=1e-12,gtol=1e-12).x
            error=residual(solution);max_error=max(max_error,float(np.linalg.norm(error)))
            if np.linalg.norm(error[:3])>1e-5 or np.linalg.norm(error[3:])>1e-5:raise RuntimeError(f'Observed-pose IK failed: {side} at {t}: error={error}, q={solution}, old={old[side][k]}, bounds=({lower},{upper})')
            rows[side][k]=solution;previous[side]=solution.copy();maximum_joint_delta=max(maximum_joint_delta,float(abs(solution-old[side][k]).max()))
    output.mkdir(parents=True,exist_ok=True);np.savez_compressed(output/'arm-posture-fr3.npz',time_s=ts,**{s+'_qpos':rows[s]for s in rows});np.savez_compressed(output/'trajectory.npz',time_s=ts,qpos=z['qpos'],mocap_pos=positions,mocap_quat=quaternions)
    for f in ['config.json','independent_schedule.json','retiming.json']:shutil.copy2(recording/f,output/f)
    if observations.resolve()!=(output/'observations.json').resolve():
        shutil.copy2(observations,output/'observations.json')
    (output/'estimated-scene.json').write_text(json.dumps(dict(scope='Estimated pickup poses; no simulation ground truth used for planning',parts={n:dict(T_cell_part=observation['parts'][n]['T_cell_part'])for n in PARTS},move_pin_holder_with_pin=bool(pin_holder_tracks_pin),m80_machine_fixed=True),indent=2)+'\n')
    report=dict(state='adapted_spatial_plan_unvalidated',hardware_ready=False,planning_frame='CAD cell; field base/tool calibration not certified',source_base=str(base),observation_sha256=hashlib.sha256(observations.read_bytes()).hexdigest(),reference_plan_sha256=hashlib.sha256((base/'arm-posture-fr3.npz').read_bytes()).hexdigest(),fixture_goals_unchanged=True,adaptation_handoffs={n:float(w[-1])for n,w in windows.items()},initial_robot_q_unchanged=all(np.array_equal(rows[s][0],old[s][0])for s in rows),maximum_joint_delta_rad=maximum_joint_delta,maximum_weighted_ik_error=max_error,pickup_deltas={n:dict(translation_m=p.tolist(),rotation_vector_rad=r.tolist())for n,(p,r)in deltas.items()},requires=['updated stock geometry collision checks','new simultaneous-path and contact/FR3 validation','fresh scene confirmation before cached plan execution'],planning_seconds=time.monotonic()-started)
    (output/'arm-posture-fr3.json').write_text(json.dumps(report,indent=2)+'\n');stage=output/'planning-base';(stage/'recording').mkdir(parents=True,exist_ok=True)
    for f in ['trajectory.npz','config.json','independent_schedule.json','retiming.json']:shutil.copy2(output/f,stage/'recording'/f)
    for f in ['arm-posture-fr3.npz','arm-posture-fr3.json']:shutil.copy2(output/f,stage/f)
    shutil.copy2(root/'assets/reference-dof-names.json',stage/'dof-names.json');print(json.dumps(report,indent=2));return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,required=True);p.add_argument('--observations',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--respect-cutoff',action='store_true');p.add_argument('--pin-holder-tracks-pin',action='store_true');a=p.parse_args();prepare(a.base,a.observations,a.output,a.respect_cutoff,a.pin_holder_tracks_pin)
