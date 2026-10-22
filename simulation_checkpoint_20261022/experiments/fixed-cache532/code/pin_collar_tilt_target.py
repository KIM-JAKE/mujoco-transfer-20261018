"""Experimental real empty-Hand−30degree collar pickup; original CAD unchanged.

Input must be the existing original−40degree / negative1mm collar proposal.
The SAME physical class tailpoint maps to the SAME Hand-local contactpoint.
This helper proposes only a target; every original motion/grip gate remains.
"""
import numpy as np


def retarget(candidate):
    G=np.asarray(candidate['grasp_transform'],float);H=np.asarray(candidate['pickup_H'],float)
    if any(T.shape!=(4,4)or not np.isfinite(T).all()for T in(G,H)):raise ValueError('Finite original PIN class/pickup transforms required')
    for T in(G,H):
        if np.max(abs(T[3]-[0,0,0,1]))>1e-8 or np.max(abs(T[:3,:3].T@T[:3,:3]-np.eye(3)))>1e-5 or np.linalg.det(T[:3,:3])<.99999:raise ValueError('Rigid original PIN transforms required')
    def rotation(deg):
        c,s=np.cos(np.deg2rad(deg)),np.sin(np.deg2rad(deg))
        return np.array([[c,0,s],[0,1,0],[-s,0,c]])
    if np.max(abs(G[:3,:3]-rotation(-40)))>1e-6:raise ValueError('Tilt proposal requires exact original negative40degree family')
    tail_z=float(candidate.get('grip_local_pin_z_m',float('nan')))
    if not np.isfinite(tail_z)or abs(tail_z+.0155)>1e-9:raise ValueError('Original class tailcontact point negative15.5mm required')
    point=np.array([0.,0.,tail_z]);contact=G[:3,:3]@point+G[:3,3]
    if np.max(abs(contact-[0.,0.,-.001]))>1e-8:raise ValueError('Tilt proposal requires existing negative1mm collarcontact target')
    adjusted=G.copy();adjusted[:3,:3]=rotation(-30);adjusted[:3,3]=contact-adjusted[:3,:3]@point
    part=H@G;target=part@np.linalg.inv(adjusted);delta=target[:3,3]-H[:3,3]
    angle=float(np.arccos(np.clip((np.trace(target[:3,:3]@H[:3,:3].T)-1)/2,-1,1)))
    if np.linalg.norm(delta)>.001 or angle>np.deg2rad(11):raise ValueError('Tilt proposal exceeds bounded1mm/11degree physical target change')
    result=dict(candidate);result['grasp_transform']=adjusted.tolist();result['pickup_H']=target.tolist();result.pop('native_pad_support',None)
    return result,dict(kind='experimental_negative30degree_collar_real_Hand_target',original_grasp=G.tolist(),proposed_grasp=adjusted.tolist(),original_class_tail_contact_point_m=point.tolist(),same_Hand_contact_point_m=contact.tolist(),physical_empty_Hand_target=True,physical_pickup_translation_m=float(np.linalg.norm(delta)),physical_pickup_rotation_rad=angle,physical_pickup_world_delta_m=delta.tolist(),original_world_part_target_unchanged=True,physical_grasp_tested=False,all_original_holder_native_route_issued_retention_gates_required=True,original_force_physics_friction_mass_caps_unchanged=True,uses_free_body_poses=False)
