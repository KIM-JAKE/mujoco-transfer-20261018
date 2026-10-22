"""Physical empty-Hand target for the broad cylindrical PIN tail.

Move the REAL empty Hand, preserving the original public class-part worldpose.
This does not alter a part pose, collider, force, friction, mass or speed. A fresh
RGB-D/encoder lift grasp and every original geometry/retention gate are required.
The pickup has not yet been qualified by a native whole-task physics run.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def retarget(candidate,tilt_deg=-30.):
    tilt_deg=float(tilt_deg)
    if tilt_deg not in (-40.,-30.):
        raise ValueError('Broad tail tilt must be exactly negative30 or40degrees')
    G=np.asarray(candidate['grasp_transform'],float)
    H=np.asarray(candidate['pickup_H'],float)
    for T in [G,H]:
        if T.shape!=(4,4) or not np.isfinite(T).all():
            raise ValueError('Finite public PIN class and pickup transforms required')
        if (np.max(abs(T[3]-[0,0,0,1]))>1e-8 or
            np.max(abs(T[:3,:3].T@T[:3,:3]-np.eye(3)))>1e-5 or
            np.linalg.det(T[:3,:3])<.99999):
            raise ValueError('Rigid public PIN class and pickup transforms required')
    if not (-.8<G[0,2]<-.35 and abs(G[1,2])<.05 and .6<G[2,2]<.95):
        raise ValueError('Broad tail proposal requires original negative tilted PIN family')
    adjusted=np.eye(4)
    adjusted[:3,:3]=Rotation.from_euler('y',tilt_deg,degrees=True).as_matrix()
    adjusted[:3,3]=-adjusted[:3,:3]@np.array([0.,0.,-.0155])+[0.,0.,.002]
    part=H@G
    target=part@np.linalg.inv(adjusted)
    distance=float(np.linalg.norm(target[:3,3]-H[:3,3]))
    angle=float(Rotation.from_matrix(target[:3,:3]@H[:3,:3].T).magnitude())
    if distance>.006 or angle>np.deg2rad(15):
        raise ValueError('Broad tail physical pickup exceeds6mm/15degree proposal envelope')
    result=dict(candidate)
    result['grasp_transform']=adjusted.tolist()
    result['pickup_H']=target.tolist()
    result.pop('native_pad_support',None)
    return result,dict(kind='broad_tail_physical_Hand_target',
        source='Original public PIN class geometry/catalogue; fresh initial RGB-D retarget remains mandatory',
        original_grasp=G.tolist(),proposed_grasp=adjusted.tolist(),
        physical_tail_point_class_m=[0.,0.,-.0155],
        physical_tail_point_Hand_m=[0.,0.,.002],
        proposed_PIN_axis_tilt_in_Hand_deg=tilt_deg,
        physical_empty_Hand_target=True,
        original_part_world_target_unchanged=True,
        physical_pickup_translation_m=distance,physical_pickup_rotation_rad=angle,
        maximum_translation_m=.006,maximum_rotation_rad=float(np.deg2rad(15)),
        all_original_holder_native_route_issued_retention_gates_required=True,
        original_physics_force_friction_mass_speed_caps_unchanged=True,
        uses_free_body_poses=False,physical_grasp_tested=False)
