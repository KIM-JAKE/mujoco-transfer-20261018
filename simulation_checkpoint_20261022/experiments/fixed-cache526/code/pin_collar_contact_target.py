"""Bounded REAL empty-Hand collar-flat pickup proposal; object pose unchanged.

Negative1--2mm original Hand-local contactZ seats more of the ORIGINAL13mm
collar on the flat nativepads. This is a target change, NEVER a collider/force/
held-measurement change. Original full holder/route/retention gates mandatory.
"""
import numpy as np


def retarget(candidate,offset_m=-.002):
    offset_m=float(offset_m)
    if offset_m not in(-.001,-.002):raise ValueError('Collar-flat offset must be exactly negative1mm or2mm')
    G=np.asarray(candidate['grasp_transform'],float);H=np.asarray(candidate['pickup_H'],float)
    if any(T.shape!=(4,4)or not np.isfinite(T).all()for T in[G,H]):raise ValueError('Finite original PIN catalogue transforms required')
    for T in[G,H]:
        if np.max(abs(T[3]-[0,0,0,1]))>1e-8 or np.max(abs(T[:3,:3].T@T[:3,:3]-np.eye(3)))>1e-5 or np.linalg.det(T[:3,:3])<.99999:raise ValueError('Original rigid PIN class/pickup pose required')
    if not(-.8<G[0,2]<-.35 and abs(G[1,2])<.05 and .6<G[2,2]<.95):raise ValueError('Collar-flat offset requires original negative tilted PIN family')
    adjusted=G.copy();adjusted[2,3]+=offset_m;part=H@G;target=part@np.linalg.inv(adjusted)
    delta=target[:3,3]-H[:3,3]
    if np.linalg.norm(delta)>.002000001 or np.max(abs(target[:3,:3]-H[:3,:3]))>1e-8:raise ValueError('Collar-flat target exceeds2mm fixed-orientation displacement')
    result=dict(candidate);result['grasp_transform']=adjusted.tolist();result['pickup_H']=target.tolist();result.pop('native_pad_support',None)
    return result,dict(kind='collar_flat_physical_Hand_target',source='Original public class catalogue; original freshinitialRGBD retarget remains mandatory',original_grasp=G.tolist(),proposed_grasp=adjusted.tolist(),Hand_contact_z_offset_m=offset_m,physical_empty_Hand_target=True,physical_pickup_translation_m=float(np.linalg.norm(delta)),physical_pickup_world_delta_m=delta.tolist(),original_part_world_target_unchanged=True,original_Hand_orientation_unchanged=True,all_original_holder_native_route_issued_retention_gates_required=True,original_force_physics_friction_mass_caps_unchanged=True,uses_free_body_poses=False,physical_grasp_tested=False)
