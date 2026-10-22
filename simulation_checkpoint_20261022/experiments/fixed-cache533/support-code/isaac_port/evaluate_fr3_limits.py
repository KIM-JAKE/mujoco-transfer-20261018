"""Independent 480 Hz FR3 command and measured-joint kinematic acceptance."""
from pathlib import Path
import argparse,json,hashlib
import numpy as np
from fr3_motion_control import LOWER,UPPER,VMAX,position_velocity_bounds

def evaluate(run):
    run=Path(run);path=run/'joint-motion.npz';z=np.load(path);t=z['time_s'];dt=np.diff(t)
    assert len(t)>100 and np.allclose(dt,1/480,atol=1e-9)
    reports={}
    for mode in ['command','actual']:
        q=z[mode+'_position'].astype(float);v=z[mode+'_velocity'].astype(float)
        a=z['command_acceleration'].astype(float)if mode=='command'else np.diff(v,axis=0)/dt[:,None]
        j=np.diff(a,axis=0)/dt[-(len(a)-1):,None]
        assert all(np.isfinite(x).all() for x in [q,v,a,j])
        low,high=position_velocity_bounds(q.reshape(-1,2,7));low=low.reshape(-1,14);high=high.reshape(-1,14)
        rows=[]
        for k in range(14):
            peak_v=float(abs(v[:,k]).max());peak_a=float(abs(a[:,k]).max());peak_j=float(abs(j[:,k]).max())
            pos_pass=bool(np.all(q[:,k]>=LOWER[k%7]-1e-6)and np.all(q[:,k]<=UPPER[k%7]+1e-6))
            vel_pass=bool(np.all(v[:,k]>=low[:,k]-1e-4)and np.all(v[:,k]<=high[:,k]+1e-4))
            rows.append(dict(side='left'if k<7 else'right',joint=k%7+1,peak_velocity_rad_s=peak_v,nominal_velocity_limit_rad_s=float(VMAX[k%7]),peak_acceleration_rad_s2=peak_a,acceleration_limit_rad_s2=10.,peak_jerk_rad_s3=peak_j,jerk_limit_rad_s3=5000.,maximum_velocity_envelope_excess_rad_s=float(max(np.max(low[:,k]-v[:,k]),np.max(v[:,k]-high[:,k]),0)),position_pass=pos_pass,position_dependent_velocity_pass=vel_pass,acceleration_pass=peak_a<=10.0001,jerk_pass=peak_j<=5000.01))
        for k,row in enumerate(rows):
            row.update(peak_velocity_time_s=float(t[np.argmax(abs(v[:,k]))]),peak_acceleration_time_s=float(t[-len(a):][np.argmax(abs(a[:,k]))]),peak_jerk_time_s=float(t[-len(j):][np.argmax(abs(j[:,k]))]))
        passed=all(all(row[key]for key in ['position_pass','position_dependent_velocity_pass','acceleration_pass','jerk_pass'])for row in rows)
        reports[mode]=dict(passed=bool(passed),joints=rows)
    passed=all(r['passed']for r in reports.values())
    manifest=json.loads((run/'run-manifest.json').read_text())if(run/'run-manifest.json').exists()else {}
    planning_budget=None
    if manifest.get('arguments',{}).get('efficient_timing',False):
        profile=json.loads((run/'reference-profile.json').read_text())
        fraction=float(profile['planning_fraction']);assert fraction==.95
        q=z['command_position'].astype(float);v=z['command_velocity'].astype(float);a=z['command_acceleration'].astype(float)
        j=np.diff(a,axis=0)/dt[:,None];lo,hi=position_velocity_bounds(q.reshape(-1,2,7));lo=lo.reshape(-1,14);hi=hi.reshape(-1,14)
        envelope=np.where(v>=0,hi,-lo);velocity_fraction=float(np.max(abs(v)/np.maximum(envelope,1e-8)))
        acceleration_fraction=float(np.max(abs(a))/10);jerk_fraction=float(np.max(abs(j))/5000)
        budget_pass=max(velocity_fraction,acceleration_fraction,jerk_fraction)<=fraction+1e-6
        planning_budget=dict(required=True,fraction=fraction,passed=bool(budget_pass),command_velocity_fraction=velocity_fraction,command_acceleration_fraction=acceleration_fraction,command_jerk_fraction=jerk_fraction,semantics='95% commanded-motion ceiling; measured PhysX joints independently satisfy the official envelope')
        passed=passed and budget_pass
    result=dict(state='passed'if passed else'failed',fr3_kinematic_limits_pass=passed,sample_rate_hz=480,samples=len(t),source='https://frankarobotics.github.io/docs/robot_specifications.html',source_checked_date='2026-10-18',joint_motion_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),command=reports['command'],actual=reports['actual'],hardware_execution_certified=False,limitations=['Commands and measured simulation states checked at 480 Hz, not a 1 kHz FCI hardware execution','Torque-rate compliance, calibrated payload and real contact behavior are not certified'])
    result['planning_budget']=planning_budget
    (run/'fr3-limits.json').write_text(json.dumps(result,indent=2)+'\n');return result

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path);r=evaluate(parser.parse_args().run)
    print(json.dumps(r,indent=2));raise SystemExit(0 if r['fr3_kinematic_limits_pass']else 1)
