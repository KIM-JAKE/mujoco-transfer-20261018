"""A smooth known signal passes; a measured spike cannot hide behind good commands."""
from pathlib import Path
from tempfile import TemporaryDirectory
import json
import numpy as np
from evaluate_fr3_limits import evaluate

with TemporaryDirectory()as directory:
    run=Path(directory);t=np.arange(0,4+1/480,1/480);u=t/4
    q=np.tile([0.,0.,0.,-1.5,0.,1.5,0.],(len(t),2));v=np.zeros_like(q);a=np.zeros_like(q)
    q[:,0]=.05*(10*u**3-15*u**4+6*u**5)
    v[:,0]=.05*(30*u**2-60*u**3+30*u**4)/4
    a[:,0]=.05*(60*u-180*u**2+120*u**3)/16
    def save(actual_v):
        np.savez_compressed(run/'joint-motion.npz',time_s=t,command_position=q,command_velocity=v,command_acceleration=a,actual_position=q,actual_velocity=actual_v)
    save(v);report=evaluate(run);assert report['fr3_kinematic_limits_pass']
    bad=v.copy();bad[900,7]=10.;save(bad);report=evaluate(run)
    assert report['command']['passed'] and not report['actual']['passed'] and not report['fr3_kinematic_limits_pass']
    # A smooth move can satisfy the official limits but exceed the requested
    # 95% command budget. This must fail the new mode independently.
    u=np.clip(t/.54,0,1);q[:,0]=-.25+.485*(10*u**3-15*u**4+6*u**5)
    v[:,0]=.485*(30*u**2-60*u**3+30*u**4)/.54
    a[:,0]=.485*(60*u-180*u**2+120*u**3)/.54**2
    save(v);report=evaluate(run);assert report['fr3_kinematic_limits_pass']
    (run/'run-manifest.json').write_text(json.dumps({'arguments':{'efficient_timing':True}}))
    (run/'reference-profile.json').write_text(json.dumps({'planning_fraction':.95}))
    report=evaluate(run);assert report['command']['passed']and report['actual']['passed']and not report['planning_budget']['passed']and not report['fr3_kinematic_limits_pass']
print('PASS: smooth motion accepted; measured spike and commands above the 95% budget rejected independently')
