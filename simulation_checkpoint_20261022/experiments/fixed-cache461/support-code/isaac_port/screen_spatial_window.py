"""Diagnostic robot-only static screen for a selected source-time interval.

This is not a full spatial or assembly pass. It omits carried-part checks and
other-arm timing. Surviving candidates still need audit_spatial_candidate.py.
"""
from pathlib import Path
import argparse,json
import numpy as np,mujoco
from scipy.interpolate import CubicSpline
from source_model import load_model
from plan_right_posture import Checker
from task_timing_fast import source_time
from optimization_clock import require_optimization_window,install_cutoff_alarm


def screen(candidate,side,interval,output,respect_cutoff=False):
    if respect_cutoff:require_optimization_window(120);install_cutoff_alarm()
    model=load_model();data=mujoco.MjData(model);z=np.load(candidate/'trajectory.npz');p=np.load(candidate/'arm-posture-fr3.npz');ts=z['time_s']
    addresses={s:np.array([int(model.joint(f'{s}_arm_fr3v2_joint{j}').qposadr[0])for j in range(1,8)])for s in ['left','right']};curves={s:CubicSpline(ts,p[s+'_qpos'],axis=0)for s in addresses}
    ticks=ts[(ts>=interval[0])&(ts<=interval[1])]
    if not len(ticks):raise ValueError('Empty source-time diagnostic interval')
    checker=Checker(candidate,source_clock=source_time,include_cross_arm=False);checker.side=side;failures=[];count=0
    for t in ticks:
        if respect_cutoff:require_optimization_window()
        data.qpos[:]=[np.interp(t,ts,z['qpos'][:,j])for j in range(model.nq)]
        for arm in addresses:data.qpos[addresses[arm]]=curves[arm](t)
        mujoco.mj_kinematics(model,data);checker.current_t={side:float(t),('right'if side=='left'else'left'):float(ts[-1])};ok,reason=checker.check(model,data);count+=1
        if not ok:failures.append(dict(reference=float(t),reason=reason));break
    report=dict(state='diagnostic_only',diagnostic_static_clear=not failures,full_spatial_pass=False,hardware_ready=False,side=side,interval=list(interval),samples=count,failures=failures,scope=__doc__)
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True);return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--candidate',type=Path,required=True);p.add_argument('--side',choices=['left','right'],required=True);p.add_argument('--interval',nargs=2,type=float,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--respect-cutoff',action='store_true');a=p.parse_args();screen(a.candidate,a.side,a.interval,a.output,a.respect_cutoff)
