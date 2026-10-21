"""Optional PIN pickup feedback using fresh RGB-D, actual SDK and normal planners.

This module never creates a scene/SDK observation or actuator proof. Every
correction/park is planned, reobserved and dispatched through Runtime.execute.
The opposite arm must remain empty in the tracked payload state and actual SDK.
"""
import copy
import json
import numpy as np
from scipy.spatial.transform import Rotation
from .calibration import rigid
from .pin_precision import rotation_error,sdk_sample


def options(value):
    if value is None:return None
    if not isinstance(value,dict)or type(value.get('enabled'))is not bool:raise ValueError('Explicit PIN pickup enabled boolean required')
    if not value['enabled']:return None
    iterations=value.get('max_corrections',3);gain=value.get('gain',.5)
    if type(iterations)is not int or not 1<=iterations<=3 or type(gain)not in(int,float)or not .3<=gain<=.5:raise ValueError('PIN pickup requires1--3 corrections and gain0.3--0.5')
    park=value.get('other_empty_park')
    if park is not None:
        if not isinstance(park,dict)or type(park.get('enabled'))is not bool:raise ValueError('Explicit optional empty-arm park enabled flag required')
        if park['enabled']:
            delta=np.asarray(park['target_offset_base_m'],float)
            if delta.shape!=(3,)or not np.isfinite(delta).all()or np.linalg.norm(delta)>.8:raise ValueError('One bounded calibrated base-relative park target required')
            if park.get('phase')!='pin_empty_park':raise ValueError('Dedicated measured pin_empty_park planner phase required')
            if not isinstance(park.get('measurement'),dict):raise ValueError('Actual calibrated empty-arm park evidence required')
    return dict(max_corrections=iterations,gain=float(gain),other_empty_park=park)


def preserve_axial_gauge(observed,previous):
    """Use measured signed axis; retain the previous unobservable axial yaw."""
    P,O=map(rigid,(observed,previous));a,b=O[:3,2],P[:3,2];cross=np.cross(a,b);n=float(np.linalg.norm(cross));dot=float(np.clip(a@b,-1,1))
    if dot<=0:raise RuntimeError('Fresh PIN signed axis changed hemisphere')
    Q=P.copy();Q[:3,:3]=Rotation.from_rotvec(cross/max(n,1e-12)*np.arctan2(n,dot)).as_matrix()@O[:3,:3]
    return Q


def preclose_target(command,actual,part,family,*,gain=.5):
    C,A,P,F=map(rigid,(command,actual,part,family));desired=P@F
    error=dict(translation_m=float(np.linalg.norm(desired[:3,3]-A[:3,3])),rotation_rad=rotation_error(desired[:3,:3],A[:3,:3]),translation_limit_m=.0003,rotation_limit_rad=.01,uses_free_body_poses=False)
    error['aligned']=error['translation_m']<=.0003 and error['rotation_rad']<=.01
    if np.linalg.norm(C[:3,3]-A[:3,3])>.002 or rotation_error(C[:3,:3],A[:3,:3])>.01:raise RuntimeError('Actual SDK pickup TCP bias exceeds2mm/.01rad')
    target=C@np.linalg.inv(A)@desired
    target[:3,3]=C[:3,3]+gain*(target[:3,3]-C[:3,3]);target[:3,:3]=Rotation.from_rotvec(gain*Rotation.from_matrix(target[:3,:3]@C[:3,:3].T).as_rotvec()).as_matrix()@C[:3,:3]
    if np.linalg.norm(target[:3,3]-C[:3,3])>.002 or rotation_error(target[:3,:3],C[:3,:3])>.05:raise RuntimeError('PIN pickup correction exceeds2mm/.05rad step bound')
    return target,error


def _sdk(runtime,side):
    settings=json.loads((runtime.root/runtime.recipe['planner_config']).read_text());tracking=settings.get('joint_tracking_error_limit_rad',.02)
    state=runtime._pin_precision_sdk(side);A,C=sdk_sample(state,tracking);base=rigid(runtime.site.config['robots'][side]['T_cell_base'])
    return base@A,base@C,state


def preclose(runtime,entry,family,previous):
    from .random_cycle import check_observation,instances,select_observation
    cfg=options(entry.get('pin_pickup_feedback'))
    if cfg is None:return previous
    if entry['part']!='pin':raise ValueError('PIN pickup feedback applies only to PIN')
    side=entry['pickup_side'];anchor=rigid(previous['T_cell_part']);latest=previous
    for iteration in range(cfg['max_corrections']+1):
        runtime.ready();hand=runtime.hand_status(side)
        if hand['is_grasped']or hand['width_m']<.024-.0003:raise RuntimeError('PIN preclose requires an actually empty Hand opening>=24mm')
        raw=check_observation(runtime,runtime.observe('pin_holder','pin',side=side,previous=latest));candidates=instances(raw,'pin')
        if len(candidates)!=1:raise RuntimeError('Fresh preclose PIN is ambiguous; no correction/close')
        measured_cap(candidates[0])
        P=preserve_axial_gauge(candidates[0]['T_cell_part'],anchor)
        if np.linalg.norm(P[:3,3]-anchor[:3,3])>.004 or rotation_error(P[:3,:3],anchor[:3,:3])>.1:raise RuntimeError('Fresh PIN changed beyond bounded4mm/.1rad association; no correction/close')
        candidate=copy.deepcopy(candidates[0]);candidate['T_cell_part']=P.tolist();candidate['center_m']=P[:3,3].tolist();raw=copy.deepcopy(raw);raw['instances']=[candidate];latest=select_observation(raw,candidate)
        # This yaw is a representation gauge; measured centre/signed axis stay unchanged.
        latest['pin_axial_yaw_gauge_source']='previous image, not an observed rotation about the symmetric PIN axis'
        A,C,_=_sdk(runtime,side);target,evidence=preclose_target(C,A,P,family['T_part_tcp'],gain=cfg['gain'])
        runtime.event('pin_fresh_preclose_feedback',iteration=iteration,capture_id=latest['capture_id'],**evidence)
        if evidence['aligned']:
            runtime._pin_pickup_guard=dict(side=side,T_part_tcp=rigid(family['T_part_tcp']).tolist(),previous_part=P.tolist())
            return runtime._selected_record(latest)
        if iteration==cfg['max_corrections']:raise RuntimeError('PIN preclose did not converge within bounded corrections; Hand remains open')
        # Normal whole-arm/current-other/payload/UNKNOWN/aperture checks and
        # fresh predispatch reobservation remain authoritative.
        plan=runtime.plan('pickup','pin',side,[target.tolist()],latest,[],contact='grasp')
        runtime.execute(plan,latest)
    raise AssertionError('Unreachable bounded preclose loop')


def park_other(runtime,entry,observed,carry):
    from .random_cycle import held_measurement,measured,offset
    cfg=options(entry.get('pin_pickup_feedback'));spec=(cfg or {}).get('other_empty_park')
    if not spec or not spec['enabled']:return observed,carry
    if entry['part']!='pin'or entry['pickup_side']!=entry['insert_side']:raise ValueError('PIN empty-other parking requires direct same-arm insertion')
    owner=entry['pickup_side'];other='left'if owner=='right'else'right';measured(spec['measurement'],runtime.root)
    state=runtime.hand_status(other)
    if state['is_grasped']or state['width_m']<.078 or other in runtime.site.expected_held or runtime.payload.owner[other]is not None:raise RuntimeError('Opposite Hand is not tracked-empty, SDK-open>=78mm and unloaded; no park')
    settings=json.loads((runtime.root/runtime.recipe['planner_config']).read_text())
    if spec['phase']not in settings['phases']:raise ValueError('Actual measured pin_empty_park corridor missing')
    A,_,_=_sdk(runtime,other);base=rigid(runtime.site.config['robots'][other]['T_cell_base']);target=A.copy();target[:3,3]=(base@np.r_[np.asarray(spec['target_offset_base_m'],float),1])[:3]
    plan=runtime.plan(spec['phase'],'pin',other,[target.tolist()],observed,[carry])
    runtime.execute(plan,observed)
    attained,_,_=_sdk(runtime,other)
    if np.linalg.norm(attained[:3,3]-target[:3,3])>.002 or rotation_error(attained[:3,:3],target[:3,:3])>.01:raise RuntimeError('Actual empty-arm park did not reach2mm/.01rad; retain PIN')
    if runtime.hand_status(other)['is_grasped']or other in runtime.site.expected_held or runtime.payload.owner[other]is not None:raise RuntimeError('Opposite payload ownership changed during park')
    observed,carry=held_measurement(runtime,entry,owner,observed)
    # Admit an ORIGINAL complete current-state preinsert plan only after actual
    # parking. This is not a fabricated future SDK state or a terminal-seat proof.
    goal=rigid(runtime.recipe['insertion_targets']['pin']);pre=offset(goal,entry['insert_entry_offset_part_m'],local=True)@np.linalg.inv(rigid(carry['T_tcp_part']))
    preview=runtime.plan('preinsert','pin',owner,[pre.tolist()],observed,[carry])
    runtime.event('pin_goal_aware_other_park',empty_side=other,actual_target_cell_tcp=attained.tolist(),held_PIN_reobserved=True,current_state_preinsert_admitted=True,terminal_stroke_proof=False,preview_csv_sha256=preview.get('csv_sha256'),uses_free_body_poses=False,hardware_precision_validated=False)
    return observed,carry


def measured_cap(value):
    if value.get('robust_cap_circle_from_pixels')is not True or value.get('lateral_from_previous_RGBD_grasp_and_encoders')is not False or value.get('fallback_used',False)is not False:
        raise RuntimeError('PIN pickup feedback requires a directly measured class cap circle; no predicted-XY/ICP fallback')


def dispatch_guard(runtime,side,part,mode,fresh):
    if part!='pin'or mode!='grasp':return
    entry=next(e for e in runtime.recipe['parts']if e['part']=='pin')
    if options(entry.get('pin_pickup_feedback'))is None:return
    guard=getattr(runtime,'_pin_pickup_guard',None)
    if not guard or guard['side']!=side:raise RuntimeError('PIN grasp lacks its bounded fresh preclose admission')
    value=fresh['selected_instance'];measured_cap(value)
    P=preserve_axial_gauge(value['T_cell_part'],guard['previous_part']);A,C,_=_sdk(runtime,side)
    _,proof=preclose_target(C,A,P,guard['T_part_tcp'])
    if not proof['aligned']:raise RuntimeError('Fresh PIN moved outside0.3mm/.01rad before actual Hand sweep; remains open')
    runtime.event('pin_actual_predispatch_preclose',capture_id=fresh['capture_id'],**proof)
