"""Keep the tested hand path while moving the button posture off joint 2's stop."""
from pathlib import Path
import sys, json, hashlib

root = Path(__file__).resolve().parent
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from plan_right_posture import Checker
from prepare import SOURCES
from task_timing_fast import source_time
base = root / 'runs/contact-fast-30'
old = np.load(base / 'arm-posture.npz')
record = np.load(base / 'recording/trajectory.npz')
model = mujoco.MjModel.from_xml_path(str(SOURCES['cap_button']))
data = mujoco.MjData(model)
checker = Checker(root/'generated/contact_task/fast',source_clock=source_time); checker.side = 'both'
ids = {s: np.array([int(model.joint(f'{s}_arm_fr3v2_joint{j}').qposadr[0]) for j in range(1,8)]) for s in ['left','right']}
bounds = model.jnt_range[[model.joint(f'right_arm_fr3v2_joint{j}').id for j in range(1,8)]]
rows = []; previous = None; max_error = 0.
for f,t in enumerate(old['time_s']):
    data.qpos[:] = record['qpos'][f]
    for side in ids: data.qpos[ids[side]] = old[side+'_qpos'][f]
    mujoco.mj_kinematics(model,data)
    hand = model.body('right_grip_base').id
    P = data.xpos[hand].copy(); R = Rotation.from_matrix(data.xmat[hand].reshape(3,3))
    u = float(np.clip((t-4.35)/.65,0,1)); weight = u*u*u*(10+u*(-15+6*u))
    q = old['right_qpos'][f].copy(); q[0] -= .30*weight
    def residual(x):
        q[1:] = x; data.qpos[ids['right']] = q; mujoco.mj_kinematics(model,data)
        return np.r_[10*(data.xpos[hand]-P),(Rotation.from_matrix(data.xmat[hand].reshape(3,3))*R.inv()).as_rotvec()]
    if weight:
        seed = old['right_qpos'][f,1:] if previous is None else previous[1:]
        fit = least_squares(residual,np.clip(seed,bounds[1:,0]+.025,bounds[1:,1]-.025),bounds=(bounds[1:,0]+.025,bounds[1:,1]-.025),max_nfev=160,ftol=1e-11,xtol=1e-11,gtol=1e-11)
        error = np.linalg.norm(residual(fit.x)); assert error<1e-5,(f,t,error)
        max_error = max(max_error,float(error))
    data.qpos[ids['right']] = q; mujoco.mj_kinematics(model,data); checker.current_t = float(t)
    ok,why = checker.check(model,data); assert ok,(f,t,why)
    if previous is not None: assert np.max(abs(q-previous))<.18,(f,t,q-previous)
    rows.append(q.copy()); previous = q.copy()
    if f%60==0: print('FR3_POSTURE',f,len(old['time_s']),flush=True)
out=root/'generated/contact_task'; np.savez_compressed(out/'arm-posture-fr3.npz',time_s=old['time_s'],left_qpos=old['left_qpos'],right_qpos=np.array(rows))
report=dict(state='passed',source_run='contact-fast-30',source_plan_sha256=hashlib.sha256((base/'arm-posture.npz').read_bytes()).hexdigest(),right_joint1_shift_rad=-.30,blend_reference_interval_s=[4.35,5.0],max_weighted_hand_pose_error=max_error,right_joint2_min_rad=float(np.array(rows)[:,1].min()),dense_collision_samples=len(rows),note='Hand poses preserved; native robot vs original M80 physical sensor geometry, installed rings, passive supply guide, self and cross-arm collision checks')
(out/'arm-posture-fr3.json').write_text(json.dumps(report,indent=2)+'\n'); print(json.dumps(report,indent=2))
