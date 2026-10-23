"""Bounded read-only FR3 IK seed alternatives, never commanded joint targets."""
import numpy as np

# Pinned FR3-v2 limits, same source values as fieldkit/servo_core.hpp. The
# native read-only backend independently validates every optional seed.
LOWER=np.array([-2.9007,-1.8361,-2.9007,-3.0770,-2.8763,.4398,-3.0508])
UPPER=np.array([2.9007,1.8361,2.9007,-.1169,2.8763,4.6216,3.0508])


def candidates(measured_q_d, offsets):
    q=np.asarray(measured_q_d,float);rows=np.asarray(offsets,float)
    if q.shape!=(7,)or rows.ndim!=2 or rows.shape[1]!=7 or not 1<=len(rows)<=4 or not np.isfinite(q).all()or not np.isfinite(rows).all()or np.max(abs(rows))>1.:
        raise ValueError('One to four bounded7-joint IK offsets and actual desired q required')
    result=[]
    for row in rows:
        seed=np.clip(q+row,LOWER+.001,UPPER-.001)
        if not any(np.max(abs(seed-old))<1e-9 for old in result):result.append(seed)
    return [row.tolist()for row in result]
