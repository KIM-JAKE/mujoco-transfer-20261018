"""Bounded EMPTY receiver target from current public RGB-D and robot FK.

This proposes a REAL Hand pose. It never changes a measured grasp or physical
object. All original contact, dual-arm, world, issued-motion and final fixture
checks remain the caller's responsibility before actuation.
"""
import numpy as np
from isaac_kinematics import exp,log


def propose(receiver_H,observed_part_H):
    H=np.asarray(receiver_H,float);part=np.asarray(observed_part_H,float)
    if any(T.shape!=(4,4)or not np.isfinite(T).all()for T in [H,part]):raise ValueError('Finite public receiver/observed-part pose required')
    for T in [H,part]:
        if np.max(abs(T[:3,:3].T@T[:3,:3]-np.eye(3)))>1e-5 or np.linalg.det(T[:3,:3])<.99999:raise ValueError('Original rigid public pose required')
    original=np.linalg.inv(H)@part;n=original[:3,2]
    if not(n[2]>.75 and .35<n[0]<.65):raise ValueError('Balanced SMALL target requires the original positive30degree family')
    # A real rotation about localY retains30degree tilt; the small localX
    # rotation makes actual jaw-closingY perpendicular to the observed axis.
    alpha=float(np.arctan2(n[0],n[2])-np.pi/6)
    Q=exp(np.array([0.,alpha,0.]));intermediate=Q.T@original[:3,:3]
    beta=float(-np.arctan2(intermediate[1,2],intermediate[2,2]));Q=Q@exp(np.array([beta,0.,0.]))
    grasp=original.copy();grasp[:3,:3]=Q.T@original[:3,:3]
    # TCP lies4.5mm along the measured symmetry axis below COM. This removes
    # the lateral moment arm without pushing the ring toward the proximal pad.
    grasp[:3,3]=.0045*grasp[:3,2]
    target=part@np.linalg.inv(grasp)
    translation=float(np.linalg.norm(target[:3,3]-H[:3,3]));rotation=float(np.linalg.norm(log(target[:3,:3]@H[:3,:3].T)))
    if translation>.006 or rotation>.060:raise ValueError('Balanced SMALL correction exceeds6mm/.060rad bounded empty-Hand proposal')
    return target,dict(source='Current direct RGB-D part pose and original SDK/model receiver target',empty_Hand_physical_target=True,original_part_pose_unchanged=True,original_G=original.tolist(),proposed_G=grasp.tolist(),target_H=target.tolist(),translation_m=translation,rotation_rad=rotation,closing_axis_dot_part_axis=float(grasp[1,2]),preserved_part_axis_toolZ_tilt_rad=float(np.arccos(np.clip(grasp[2,2],-1,1))),TCP_axis_offset_m=.0045,maximum_translation_m=.006,maximum_rotation_rad=.060,uses_free_body_poses=False,all_original_preflights_still_required=True)
