"""Keep NEW IK endpoints inside unchanged bounds after command quantization.

This never accepts or edits a current robot encoder/command state. It handles
only a newly solved endpoint that is already strictly inside the original
float64 model limits. Every route/issued-row/contact gate remains mandatory.
"""
import numpy as np
from isaac_kinematics import log


def receiver_candidate(arm,q,target_H):
    original=np.asarray(q,float);bounds=np.asarray(arm.bounds,float);target=np.asarray(target_H,float)
    if original.shape!=(7,)or bounds.shape!=(7,2)or target.shape!=(4,4)or not np.isfinite(np.r_[original,bounds.ravel(),target.ravel()]).all():
        raise ValueError('Finite new IK endpoint, bounds and requested pose required')
    if np.any(original<bounds[:,0])or np.any(original>bounds[:,1]):
        raise ValueError('New IK endpoint already outside original joint bounds')
    encoded=original.astype(np.float32).astype(float)
    if np.all(encoded>=bounds[:,0])and np.all(encoded<=bounds[:,1]):return original,None
    low=bounds[:,0].astype(np.float32);high=bounds[:,1].astype(np.float32)
    low=np.where(low.astype(float)<bounds[:,0],np.nextafter(low,np.float32(np.inf)),low)
    high=np.where(high.astype(float)>bounds[:,1],np.nextafter(high,np.float32(-np.inf)),high)
    safe=np.clip(encoded,low.astype(float),high.astype(float))
    if np.any(safe<bounds[:,0])or np.any(safe>bounds[:,1]):raise ValueError('No float32 endpoint inside original bounds')
    # More than one adjacent float32 step is not a quantization-only repair.
    ulp=np.maximum(abs(np.spacing(encoded.astype(np.float32)).astype(float)),np.finfo(np.float32).tiny)
    if np.any(abs(safe-encoded)>ulp):raise ValueError('Endpoint repair exceeds one float32 step')
    achieved=arm.fk(safe)
    position_error=float(np.linalg.norm(achieved[:3,3]-target[:3,3]))
    rotation_error=float(np.linalg.norm(log(achieved[:3,:3]@target[:3,:3].T)))
    if position_error>1e-5 or rotation_error>1e-4:
        raise ValueError('Interior endpoint does not satisfy original IK pose tolerances')
    return safe,dict(original_IK_q=original.tolist(),encoded_q=encoded.tolist(),interior_q=safe.tolist(),maximum_joint_change_rad=float(abs(safe-original).max()),position_error_m=position_error,rotation_error_rad=rotation_error,new_endpoint_only=True,current_robot_state_modified=False,original_bounds_and_IK_tolerances_unchanged=True)
