"""Bounded physical EMPTY Hand pickup proposal, never object/force mutation.

The same physical pin tail point, classz=-15.5mm, is mapped to Hand[0,0,3mm].
Original public target part pose is preserved by changing the REAL pickupHand.
All original current full-scene/holder/contact/retention gates remain mandatory.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def retarget(candidate):
    G=np.asarray(candidate['grasp_transform'],float);H=np.asarray(candidate['pickup_H'],float)
    if any(T.shape!=(4,4)or not np.isfinite(T).all()for T in[G,H]):raise ValueError('Finite original public PIN catalogue poses required')
    for T in[G,H]:
        if np.max(abs(T[3]-[0,0,0,1]))>1e-8 or np.max(abs(T[:3,:3].T@T[:3,:3]-np.eye(3)))>1e-5 or np.linalg.det(T[:3,:3])<.99999:raise ValueError('Original rigid public PIN catalogue poses required')
    adjusted=np.eye(4);adjusted[:3,3]=[0.,0.,.0185]
    part=H@G;target=part@np.linalg.inv(adjusted)
    distance=float(np.linalg.norm(target[:3,3]-H[:3,3]));angle=float(Rotation.from_matrix(target[:3,:3]@H[:3,:3].T).magnitude())
    if distance>.015 or angle>np.pi/4:raise ValueError('Parallel PIN emptyHand target exceeds15mm/45degree envelope')
    result=dict(candidate);result['grasp_transform']=adjusted.tolist();result['pickup_H']=target.tolist();result.pop('native_pad_support',None)
    return result,dict(kind='parallel_tail_physical_Hand_target',source='Original public class catalogue and original existing initialRGBD correction',original_grasp=G.tolist(),proposed_grasp=adjusted.tolist(),physical_tail_point_class_m=[0.,0.,-.0155],physical_tail_point_Hand_m=[0.,0.,.003],original_part_target_world_unchanged=True,physical_empty_Hand_target=True,physical_pickup_translation_m=distance,physical_pickup_rotation_rad=angle,maximum_translation_m=.015,maximum_rotation_rad=float(np.pi/4),all_original_fullscene_holder_motion_contact_retention_gates_required=True,original_physics_force_friction_mass_caps_unchanged=True,uses_free_body_poses=False,physical_grasp_tested=False)
