"""Parallel calibrated RGB pair. No depth or semantic labels are generated."""
from pathlib import Path
import numpy as np
from rgb_sensor import capture,look_at

def stereo(stage,eye,target,span=.16,width=512,baseline=.025,out=None):
    eye=np.array(eye,float);target=np.array(target,float);R=look_at(eye,target)[:3,:3];out=Path(out);out.mkdir(exist_ok=True,parents=True)
    result=[]
    for i,sign in enumerate([-1,1]):
        shift=R[:,0]*baseline*.5*sign
        result.append(capture(stage,eye+shift,target+shift,span,width,width,out=out/('left.npz'if i==0 else'right.npz')))
    return result
