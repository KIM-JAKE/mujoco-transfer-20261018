"""Offline screening for FR3 transfer. Never certifies or connects to hardware.

The inverse dynamics screen uses the imported model, excludes contact/friction,
and holds finger velocities/accelerations at zero. Its result is a model-based
warning, not measured motor torque or a proof of hardware feasibility.
"""
from pathlib import Path
import argparse,json,hashlib
import numpy as np
import mujoco
from scipy.interpolate import BPoly
from fr3_motion_control import LOWER,UPPER,position_velocity_bounds
from source_model import load_model,model_xml

def reconstruct_commands(z):
    t=z['time_s'];values=np.stack([z['command_position'],z['command_velocity'],z['command_acceleration']],axis=1)
    return BPoly.from_derivatives(t,values)

def audit(run,output=None):
    run=Path(run);out=Path(output)if output else run/'hardware-readiness.json';z=np.load(run/'joint-motion.npz');t=np.arange(np.ceil(z['time_s'][0]*1000),np.floor(z['time_s'][-1]*1000)+1)/1000;clock=reconstruct_commands(z)
    q,v,a,j=[clock(t,k)for k in range(4)];lo,hi=position_velocity_bounds(q.reshape(-1,2,7));lo=lo.reshape(-1,14);hi=hi.reshape(-1,14)
    command=dict(sample_rate_hz=1000,samples=len(t),position_pass=bool(np.all(q>=np.tile(LOWER,2))and np.all(q<=np.tile(UPPER,2))),velocity_pass=bool(np.all(v>=lo-1e-6)and np.all(v<=hi+1e-6)),acceleration_max_rad_s2=float(abs(a).max()),jerk_max_rad_s3=float(abs(j).max()),acceleration_pass=bool(abs(a).max()<=10+1e-6),jerk_pass=bool(abs(j).max()<=5000+1e-3),interpolation='C2 quintic Hermite reconstruction from recorded command q/dq/ddq; not a tested FCI execution loop')
    model=load_model();model.body_gravcomp[:]=0;model.opt.gravity[:]=[0,0,-9.81];data=mujoco.MjData(model)
    names=json.loads((run/'dof-names.json').read_text());arms=[names[int(i)]for i in z['dof_indices']];qa=[int(model.joint(n).qposadr[0])for n in arms];va=[int(model.joint(n).dofadr[0])for n in arms];allqa=[int(model.joint(n).qposadr[0])for n in names];tape=np.load(run/'physics-tape.npz');fullq=np.array([np.interp(t,tape['time_s'],tape['joint_positions'][:,i])for i in range(len(names))]).T
    torque=np.empty((len(t),14));gravity=np.empty_like(torque);tau=np.zeros(model.nv)
    for k in range(len(t)):
        data.qpos[allqa]=fullq[k];data.qpos[qa]=q[k];data.qvel[:]=0;data.qacc[:]=0
        mujoco.mj_kinematics(model,data);mujoco.mj_comPos(model,data);mujoco.mj_comVel(model,data);mujoco.mj_rne(model,data,1,tau);gravity[k]=tau[va]
        data.qvel[va]=v[k];data.qacc[va]=a[k];mujoco.mj_comVel(model,data);mujoco.mj_rne(model,data,1,tau);torque[k]=tau[va]
    dynamic=torque-gravity;rate=np.diff(dynamic,axis=0)*1000;limit=np.tile([87,87,87,87,12,12,12],2)
    rows=[]
    for i,n in enumerate(arms):
        rows.append(dict(joint=n,max_total_inverse_dynamics_nm=float(abs(torque[:,i]).max()),max_gravity_nm=float(abs(gravity[:,i]).max()),max_without_gravity_nm=float(abs(dynamic[:,i]).max()),max_without_gravity_rate_nm_s=float(abs(rate[:,i]).max()),torque_limit_nm=float(limit[i]),rate_limit_nm_s=1000.,model_total_torque_within_limit=bool(abs(torque[:,i]).max()<=limit[i]),model_without_gravity_rate_within_limit=bool(abs(rate[:,i]).max()<=1000)))
    m=json.loads((run/'run-manifest.json').read_text());trial=m.get('layout')=='trial_rail_offsets'or(m.get('layout')is None and m['arguments'].get('clamps'))
    blockers=[dict(id='layout',status='failed'if trial else'unverified',detail='Shifted sensor experiment differs from user-confirmed original CAD; separate optical-axis CAD solids were incorrectly treated as rigid rails.'if trial else'Original sensor mounts retained; verify the optical-axis semantic correction and physical head/tail clearance. No hardware assembly pass implied.'),dict(id='controller',status='unverified',detail='480Hz implicit PhysX position/velocity drives with K=12000, D=600/1800; internal FR3 damping/controller response and FCI filtering not replicated.'),dict(id='sensing',status='unverified',detail='120Hz exact zero-latency workpiece pose, individual pad contacts and fixture impulses; no real sensor/calibration/latency model. Alignment gate is 25um.'),dict(id='gravity_payload',status='unverified',detail='Robot gravity disabled in PhysX; real FR3 gravity compensation exists but motor torque/load calibration and contact torque budget remain unverified.'),dict(id='gripper',status='unverified',detail='Robotiq 2F-85 direct joint motor/mimic model is not the real position/speed/force register interface; command latency, force/position resolution and compliance not calibrated.'),dict(id='fixture',status='unverified',detail='Added clamp jaws, 80ms closure ramp, pin supply sleeve and spring button are assumed; no measured mechanical stroke, response, placement or PLC I/O.'),dict(id='fci',status='unverified',detail='No executed 1kHz libfranka loop, network jitter test, robot-specific limits, collision thresholds or measured torque-rate trace.'),dict(id='repeatability',status='unverified',detail='One deterministic episode; no tolerance, friction, perception-noise or delay robustness distribution; cycle omits ready-pose initialization and return to start.')]
    report=dict(state='not_hardware_ready',hardware_execution_certified=False,run=str(run),joint_command_1khz_screen=command,inverse_dynamics_screen=dict(scope='Model-only arm + imported gripper, no contacts, carried workpiece or motor friction; frozen finger velocity/acceleration; torque rate uses gravity-excluded torque as FCI convention',model_scene_sha256=hashlib.sha256(model_xml()[0].read_bytes()).hexdigest(),all_total_torque_within_limits=all(x['model_total_torque_within_limit']for x in rows),all_without_gravity_rate_within_limits=all(x['model_without_gravity_rate_within_limit']for x in rows),joints=rows),blockers=blockers,sources=dict(fr3_limits='https://frankarobotics.github.io/docs/robot_specifications.html',fci='https://frankarobotics.github.io/docs/overview.html',robot_state='https://frankarobotics.github.io/libfranka/latest/structfranka_1_1RobotState.html',impedance='https://frankarobotics.github.io/libfranka/latest/classfranka_1_1Robot.html',gripper='https://robotiq.com/products/adaptive-grippers'))
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2)+'\n');np.savez_compressed(out.with_suffix('.npz'),time_s=t,position=q,velocity=v,acceleration=a,jerk=j,model_total_torque=torque,model_gravity=gravity,model_torque_without_gravity=dynamic)
    print(json.dumps(dict(state=report['state'],command_1khz=command,model_max_total_torque_nm=float(abs(torque).max()),model_max_without_gravity_rate_nm_s=float(abs(rate).max()),model_total_torque_pass=report['inverse_dynamics_screen']['all_total_torque_within_limits'],model_torque_rate_pass=report['inverse_dynamics_screen']['all_without_gravity_rate_within_limits'],output=str(out)),indent=2));return report
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path);a=p.parse_args();audit(a.run,a.output)
