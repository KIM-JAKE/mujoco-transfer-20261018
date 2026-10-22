"""A bounded physical empty-Hand pickup target, never a changed object pose.

The original catalogue's −40 degree family is preserved. Moving the actual
Hand3mm back along its localZ excludes the13mm collar from the fingertip while
putting the10.5mm tail on the original flat native pads. Every original route,
preclose image/SDK, contact, transport and final fixture gate remains mandatory.
"""
import numpy as np


def retarget(candidate):
    result=dict(candidate)
    G=np.asarray(candidate['grasp_transform'],float);H=np.asarray(candidate['pickup_H'],float)
    if any(T.shape!=(4,4)or not np.isfinite(T).all()for T in[G,H]):raise ValueError('Finite original PIN class grasp and pickup target required')
    for T in[G,H]:
        if np.max(abs(T[:3,:3].T@T[:3,:3]-np.eye(3)))>1e-5 or np.linalg.det(T[:3,:3])<.99999:raise ValueError('Rigid original PIN pickup target required')
    if not(-.8<G[0,2]<-.35 and abs(G[1,2])<.05 and .6<G[2,2]<.95):raise ValueError('PIN flat-tail offset requires original negative tilted family')
    adjusted=G.copy();adjusted[2,3]+=.003
    original_part=H@G;target=original_part@np.linalg.inv(adjusted)
    if np.linalg.norm(target[:3,3]-H[:3,3])>.003000001 or np.max(abs(target[:3,:3]-H[:3,:3]))>1e-8:raise ValueError('PIN tail target exceeds3mm fixed-orientation displacement')
    result['grasp_transform']=adjusted.tolist();result['pickup_H']=target.tolist()
    # An old catalogue contact certificate describes the ORIGINAL target only.
    result.pop('native_pad_support',None)
    return result,dict(source='Original public class-CAD/catalogue grasp; existing fresh initial RGB-D retarget still mandatory',physical_empty_Hand_target=True,original_grasp=G.tolist(),proposed_grasp=adjusted.tolist(),positive_Hand_contact_z_offset_m=.003,physical_pickup_translation_m=float(np.linalg.norm(target[:3,3]-H[:3,3])),original_part_pose_unchanged=True,original_orientation_family_unchanged=True,all_original_guards_required_before_motion=True,original_actual_RGBD_SDK_grasp_measured_after_lift=True,uses_free_body_poses=False)
