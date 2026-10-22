"""Execute the complete prepared command clock, including an interlock pause."""
from pathlib import Path
from tempfile import TemporaryDirectory
import json,shutil,argparse
import numpy as np
from fr3_motion_control import PathMotionLimiter,position_velocity_bounds
from fr3_reference_profile import PreparedReferenceProfile

parser=argparse.ArgumentParser();parser.add_argument('--target-15',action='store_true');args=parser.parse_args()
root=Path(__file__).resolve().parent;base=root/'runs/contact-fr3-09'
names=json.loads((base/'dof-names.json').read_text());source=json.loads((root/'generated/contact_task/contact-source.json').read_text());lookup={j['name']:j['qpos_address']for j in source['joints']}
z=np.load(base/'recording/trajectory.npz');q=z['qpos'][:,[lookup[n]for n in names]].copy();plan=np.load(base/'arm-posture-fr3.npz')
for i,name in enumerate(names):
    if '_arm_'in name:q[:,i]=plan[name.split('_')[0]+'_qpos'][:,int(name[-1])-1]
profile=PreparedReferenceProfile(z['time_s'],q,names,json.loads((base/'recording/independent_schedule.json').read_text()),root/'generated/contact_task'/('fr3-under15-profile.npz'if args.target_15 else'fr3-efficient-profile.npz'))
reference=0.;count=0;gate=profile.bounds[1]-1e-8;values=[]
with TemporaryDirectory()as directory:
    impl=Path(directory)/'implementation';impl.mkdir()
    for name in ['fr3_motion_control.py','fr3_motion_worker.py']:shutil.copy2(root/name,impl/name)
    limiter=PathMotionLimiter(names,q[0],impl/'fr3_motion_worker.py',profile)
    try:
        for step in range(480*45):
            if step%4==0:
                proposed=limiter.peek_reference(4/480);hold=None
                if reference<=gate and proposed>gate and count<24:reference=gate;hold='test_interlock';count+=1
                else:reference=proposed
                limiter.set_reference(reference,hold,True)
                target=np.array([np.interp(reference,z['time_s'],q[:,i])for i in range(len(names))]);limiter.set_target(target,np.zeros(len(names)))
            limiter.step();values.append([limiter.p.copy(),limiter.v.copy(),limiter.a.copy()])
            if profile.elapsed>=profile.ends[-1] and np.max(abs(limiter.v))<1e-6 and np.max(abs(limiter.a))<1e-5:break
        else:raise AssertionError('Prepared profile did not finish')
        assert count==24
    finally:limiter.close()
values=np.array(values);pos,vel,acc=values[:,0],values[:,1],values[:,2]
lo,hi=position_velocity_bounds(pos.reshape(-1,2,7));lo=lo.reshape(-1,14);hi=hi.reshape(-1,14)
vratio=np.max(abs(vel)/np.maximum(np.where(vel>=0,hi,-lo),1e-6));amax=float(np.max(abs(acc)));jmax=float(np.max(abs(np.diff(acc,axis=0)))*480)
assert vratio<=.95+1e-6 and amax<=9.5+1e-6 and jmax<=4750+1e-4,(vratio,amax,jmax)
assert np.max(abs(np.diff(vel,axis=0)))*480<=9.5+1e-3
assert np.max(abs(pos[-1]-q[-1,limiter.indices]))<1e-4
print(json.dumps(dict(state='passed',nominal_s=float(profile.ends[-1]),interlock_pause_s=.2,command_velocity_fraction=float(vratio),command_acceleration_max_rad_s2=amax,command_jerk_max_rad_s3=jmax),indent=2))
