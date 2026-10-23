"""Explicit short upward support-contact departure; NEVER free-space clearance.

Only selected image/SDK-associated payload and named fingers may contact loose
stock. Measured walls, palm, arm and other arm stay strict. Issued FK is checked
for upward progress; actual native tracking/wrench/heartbeat guards remain.
"""
import copy
import numpy as np
from .collision import finite,transform,inside_box,segment_box_distance
from .stock_contact import validate_stock_contact_policy

def validate_departure_policy(phase,envelope):
    spec=phase.get('bounded_loose_stock_departure')
    if not isinstance(spec,dict)or spec.get('schema_version')!=1:raise ValueError('Explicit support-contact departure contract required')
    if phase.get('_phase_name')!='stock_departure'or phase.get('_contact')!='depart'or phase.get('_active_part')not in ('ring1','ring2'):raise ValueError('Contact departure applies only to an actual selected ring short lift')
    up=finite(spec.get('up_axis_cell'),(3,))
    if abs(np.linalg.norm(up)-1)>1e-6:raise ValueError('Measured unit up axis in cell frame required')
    limit=spec.get('maximum_lift_m')
    if type(limit)not in (float,int)or not 0<limit<=.07:raise ValueError('Stock departure length must be at most70mm')
    normalized=copy.deepcopy(phase);normalized.update(_phase_name='pickup',_contact='grasp',bounded_loose_stock_contact=spec['contact_policy'])
    contract=validate_stock_contact_policy(normalized,envelope)
    support=spec.get('support_floor')
    if support is not None:
        origin=finite(support['origin_cell_m'],(3,));normal=finite(support['normal_cell'],(3,))
        if abs(np.linalg.norm(normal)-1)>1e-6 or normal@up<.9:raise ValueError('Measured floor normal must face the calibrated upward departure')
        if not support.get('obstacle_name'):raise ValueError('Named measured support floor required')
        for key in ('maximum_inference_correction_m','reported_fit_depth_bound_m'):
            value=support.get(key)
            if type(value)not in (int,float)or not np.isfinite(value)or not 0<value<=.005:raise ValueError('Explicit reported floor-fit correction/depth bounds<=5mm required')
        tolerance=support.get('plane_alignment_tolerance_m')
        if type(tolerance)not in (float,int)or not 0<tolerance<=.001:raise ValueError('Explicit support-plane model alignment bound<=1mm required')
    contract.update(up_axis_cell=up,maximum_lift_m=float(limit),up_axis_measurement=spec['up_axis_measurement'],support_floor=support)
    return contract

def certify_issued_departure(tcps,phase,envelope,*,selected_payload):
    contract=validate_departure_policy(phase,envelope);poses=np.asarray(tcps,float)
    if poses.ndim!=3 or poses.shape[1:]!=(4,4)or len(poses)<2:raise ValueError('Actual issued FK sample curve required')
    for T in poses:transform(T)
    up=contract['up_axis_cell'];delta=poses[:,:3,3]-poses[0,:3,3];progress=delta@up
    lateral=np.linalg.norm(delta-progress[:,None]*up,axis=1)
    rotation=np.arccos(np.clip((np.trace(poses[:,:3,:3]@poses[0,:3,:3].T,axis1=1,axis2=2)-1)/2,-1,1))
    if (progress[-1]<=0 or progress.max()>contract['maximum_lift_m']+1e-6 or progress.min()< -1e-6 or np.any(np.diff(progress)< -1e-6)
            or lateral.max()>.0005 or rotation.max()>.002):raise ValueError('Issued stock departure must progress upward without downward/lateral/rotation carry')
    if selected_payload.get('owner')!='active'or selected_payload.get('part')!=phase['_active_part']or not selected_payload.get('instance_id')or selected_payload['instance_id']!=phase.get('_target_instance_id'):raise ValueError('Departure requires current selected RGBD/SDK held-payload association')
    support=contract['support_floor'];floor_proof={}
    if support is not None:
        from .floor_constraint import support_vertices
        G=transform(selected_payload['T_tcp_part']);v=support_vertices(selected_payload['part']);n=np.asarray(support['normal_cell']);o=np.asarray(support['origin_cell_m']);minimum=[]
        for tcp in poses:
            P=tcp@G;minimum.append(float(np.min(v@(P[:3,:3].T@n)+(P[:3,3]-o)@n)))
        if min(minimum)<-1e-7 or np.any(np.diff(minimum)<-1e-6):raise ValueError('Selected actual CAD must depart nondeepening from measured support floor')
        floor_proof=dict(selected_CAD_above_measured_support_floor=True,minimum_selected_CAD_floor_height_m=min(minimum),support_floor_obstacle_name=support['obstacle_name'])
    return dict(**floor_proof,kind='bounded_support_contact_not_free_space',selected_instance_id=selected_payload['instance_id'],maximum_commanded_lift_m=float(progress.max()),maximum_commanded_lateral_m=float(lateral.max()),maximum_commanded_rotation_rad=float(rotation.max()),issued_samples=len(poses),physical_monotonic_motion_certified=False,actual_SDK_wrench_and_tracking_guard_required=True)

def departure_contact_mask(obstacle,*,phase,capsule_index,capsule,envelope,active_robot=False,is_payload=False):
    a,b=finite(capsule['a']),finite(capsule['b']);denied=np.zeros(a.shape[:-1],bool)
    if obstacle.get('source')!='conservative_registered_RGBD_rays'or obstacle.get('occupancy_kind')not in ('unknown','occupied'):return denied
    proof=phase.get('_departure_certificate')
    if not isinstance(proof,dict)or proof.get('kind')!='bounded_support_contact_not_free_space':return denied
    try:contract=validate_departure_policy(phase,envelope)
    except (ValueError,KeyError,TypeError):return denied
    if is_payload:
        if capsule.get('owner')!='active'or capsule.get('part')!=phase['_active_part']or capsule.get('instance_id')!=proof['selected_instance_id']:return denied
    elif not active_robot or capsule_index not in contract['finger_indices']or capsule.get('name')not in contract['finger_names']:return denied
    radius=finite(capsule['r'])
    # Contacts only where the entire conservative controlled shape lies inside
    # the measured bin. Exterior walls remain independent rigid obstacles.
    inside=_controlled_inside(a,b,radius,contract,proof,is_payload);hit=segment_box_distance(a,b,obstacle)<=radius
    return np.asarray(inside&hit,bool)

def loose_stock_pair_mask(controlled,obstacle,*,phase,envelope,capsule_index=None,is_payload=False):
    """The same intentional contact against recognized loose-ring geometry."""
    denied=np.zeros(np.asarray(controlled['a']).shape[:-1],bool)
    if obstacle.get('source')!='current_RGBD_loose_ring'or obstacle.get('part')not in ('ring1','ring2'):return denied
    if phase.get('_phase_name')=='stock_departure':
        proof=phase.get('_departure_certificate')
        if not isinstance(proof,dict)or proof.get('kind')!='bounded_support_contact_not_free_space':return denied
        try:contract=validate_departure_policy(phase,envelope)
        except (ValueError,KeyError,TypeError):return denied
        if is_payload:
            if controlled.get('owner')!='active'or controlled.get('part')!=phase['_active_part']or controlled.get('instance_id')!=proof['selected_instance_id']:return denied
        elif capsule_index not in contract['finger_indices']or controlled.get('name')not in contract['finger_names']:return denied
    elif not is_payload:
        try:contract=validate_stock_contact_policy(phase,envelope)
        except (ValueError,KeyError,TypeError):return denied
        if capsule_index not in contract['finger_indices']or controlled.get('name')not in contract['finger_names']:return denied
    else:return denied
    return np.asarray(_controlled_inside(controlled['a'],controlled['b'],controlled['r'],contract,phase.get('_departure_certificate',{}),is_payload),bool)


def _controlled_inside(a,b,radius,contract,proof,is_payload):
    from .collision import box
    if not is_payload or not proof.get('selected_CAD_above_measured_support_floor'):return inside_box(a,b,radius,contract['bin_interior'])
    # Preserve both lateral walls and upper bound. The support-floor proof uses
    # ALL actual selected CAD vertices, allowing its conservative capsule skin
    # to overlap the support plane without inventing a1mm hover or moving CAD.
    T,half=box(contract['bin_interior']);la=(np.asarray(a)-T[:3,3])@T[:3,:3];lb=(np.asarray(b)-T[:3,3])@T[:3,:3];r=np.asarray(radius)
    lateral=np.all(np.maximum(abs(la[...,:2]),abs(lb[...,:2]))+r[...,None]<=half[:2],axis=-1)
    upper=np.maximum(la[...,2],lb[...,2])+r<=half[2]
    return lateral&upper


def departure_support_floor_mask(obstacle,*,phase,capsule,envelope,is_payload=False):
    a,b=finite(capsule['a']),finite(capsule['b']);denied=np.zeros(a.shape[:-1],bool)
    proof=phase.get('_departure_certificate',{})
    if not is_payload or not proof.get('selected_CAD_above_measured_support_floor'):return denied
    try:contract=validate_departure_policy(phase,envelope)
    except (ValueError,KeyError,TypeError):return denied
    support=contract['support_floor']
    if (not support or obstacle.get('name')!=support['obstacle_name']or obstacle.get('source')in ('conservative_registered_RGBD_rays','measured_unknown_scene_enclosure')
       or capsule.get('owner')!='active'or capsule.get('part')!=phase['_active_part']or capsule.get('instance_id')!=proof['selected_instance_id']):return denied
    from .collision import box
    T,half=box(obstacle);n=np.asarray(support['normal_cell']);alignment=float(T[:3,2]@n)
    if abs(alignment)<1-1e-6:raise ValueError('Named support obstacle must be the measured floor top plane, never a wall')
    top=T[:3,3]+np.sign(alignment)*T[:3,2]*half[2]
    if abs((top-np.asarray(support['origin_cell_m']))@n)>support['plane_alignment_tolerance_m']:raise ValueError('Named floor OBB top differs from measured support plane')
    return np.asarray(_controlled_inside(a,b,capsule['r'],contract,proof,True),bool)
