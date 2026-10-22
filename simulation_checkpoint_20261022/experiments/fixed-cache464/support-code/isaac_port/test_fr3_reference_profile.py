"""Check shared path timing, an interlock pause, and isolated offset smoothing."""
from pathlib import Path
from tempfile import TemporaryDirectory
import shutil,json
import numpy as np
from fr3_reference_profile import ReferenceProfile
from fr3_motion_control import PathMotionLimiter

root=Path(__file__).resolve().parent;base=root/'runs/contact-fr3-03'
names=json.loads((base/'dof-names.json').read_text());source=json.loads((root/'generated/contact_task/contact-source.json').read_text());lookup={j['name']:j['qpos_address']for j in source['joints']}
z=np.load(base/'recording/trajectory.npz');q=z['qpos'][:,[lookup[n]for n in names]].copy();plan=np.load(base/'arm-posture-fr3.npz')
for i,n in enumerate(names):
    if '_arm_'in n:q[:,i]=plan[n.split('_')[0]+'_qpos'][:,int(n[-1])-1]
profile=ReferenceProfile(z['time_s'],q,names,json.loads((base/'recording/independent_schedule.json').read_text()))
max_deviation=0.;count=0;reference=0.;gate=profile.bounds[1]-1e-8
with TemporaryDirectory()as directory:
    implementation=Path(directory)/'implementation';implementation.mkdir()
    for name in ['fr3_motion_worker.py','fr3_motion_control.py']:shutil.copy2(root/name,implementation/name)
    limiter=PathMotionLimiter(names,q[0],implementation/'fr3_motion_worker.py',profile)
    try:
        for step in range(480*60):
            if step%4==0:
                proposed=limiter.peek_reference(4/480);hold=None
                if reference<=gate and proposed>gate and count<24:reference=gate;hold='test_interlock';count+=1
                else:reference=proposed
                limiter.set_reference(reference,hold,True)
                target=np.array([np.interp(reference,z['time_s'],q[:,i])for i in range(len(names))]);limiter.set_target(target,np.zeros(len(names)))
            limiter.step();nominal=profile.path(profile.reference_at(profile.elapsed)[0]);max_deviation=max(max_deviation,float(np.max(abs(limiter.p-nominal))))
            if profile.elapsed>=profile.ends[-1] and np.max(abs(limiter.v))<1e-5 and np.max(abs(limiter.a))<1e-4:break
        else:raise AssertionError('Profile did not converge')
        assert count==24 and max_deviation<.005,(count,max_deviation)
        assert np.max(abs(limiter.p-q[-1,limiter.indices]))<1e-4
    finally:limiter.close()
report=dict(state='passed',shared_profile_nominal_s=float(profile.ends[-1]),test_duration_s=(step+1)/480,interlock_hold_control_ticks=count,max_offset_from_spatial_path_rad=max_deviation)
(root/'generated/contact_task/fr3-reference-test.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
