"""Exercise reversal, stopping, and full-path tracking before a PhysX run."""
from pathlib import Path
import json
import numpy as np
from fr3_motion_control import MotionLimiter

root=Path(__file__).resolve().parent
names=[f'{side}_arm_fr3v2_joint{j}'for side in ['left','right']for j in range(1,8)]
initial=np.tile([0.,0.,0.,-1.5,0.,1.5,0.],2)
limiter=MotionLimiter(names,initial)
for step in range(3600):
    target=initial.copy();target[0]=.5 if step<900 else -.4 if step<2100 else 0.
    limiter.set_target(target,np.zeros(14));limiter.step()
assert np.max(abs(limiter.p-initial))<1e-4
plan=np.load(root/'generated/contact_task/arm-posture-fr3.npz');ts=plan['time_s'];q=np.c_[plan['left_qpos'],plan['right_qpos']];v=np.gradient(q,ts,axis=0)/3
limiter=MotionLimiter(names,q[0]);ref=0.;wait=0
for step in range(480*60):
    if step%4==0:
        rate=limiter.progress_scale(limiter.p);hold=rate==0
        if not hold:ref=min(float(ts[-1]),ref+rate*4/(480*3))
        else:wait+=1
        target=np.array([np.interp(ref,ts,q[:,k])for k in range(14)])
        feed=np.zeros(14)if hold or ref>=ts[-1]else rate*np.array([np.interp(ref,ts,v[:,k])for k in range(14)])
        limiter.set_target(target,feed)
    limiter.step()
    if ref>=ts[-1]and np.max(abs(limiter.p-q[-1]))<1e-4:break
else:raise AssertionError('Full reference failed to converge within 60s')
report=dict(state='passed',abrupt_reversal_and_stop=True,full_path_converged=True,full_path_command_time_s=(step+1)/480,following_wait_control_ticks=wait,final_position_error_rad=float(np.max(abs(limiter.p-q[-1]))))
(root/'generated/contact_task/fr3-controller-test.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
