"""Seed-free RGB-D -> dynamically screened FR3 motion -> measured three-part cycle.

Every motion is generated from current images and measured SDK state. No stored
simulation joint trace or image actor identity is reused. Dry replay executes
this same state machine against explicit fake devices, never real actuators.
"""
import argparse
import copy
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import time
import uuid

from isaac_port.fieldkit.hand import FrankaHand,bounded
from isaac_port.fieldkit.site import Site
from .calibration import rigid,utc_stamp
from .execution import StopListener
from .collision import CollisionError

PARTS=('ring1','ring2','pin')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def offset(transform,translation,*,local=False):
    import numpy as np
    t=rigid(transform).copy();delta=np.asarray(translation,float)
    if delta.shape!=(3,) or not np.isfinite(delta).all() or np.linalg.norm(delta)>.5:
        raise ValueError('Explicit bounded metric approach/retreat offsets required')
    t[:3,3]+=t[:3,:3]@delta if local else delta
    return t


def measured(record,root,subject=None):
    if isinstance(record,dict)and record.get('kind')=='model_derived':
        from .model_records import validate
        return validate(record,root,subject=subject)
    if record.get('verified') is not True or not record.get('by') or utc_stamp(record['at_utc'])>datetime.now(timezone.utc):
        raise ValueError('Verified actual field measurement provenance required')
    evidence=Path(root)/record['evidence_file']
    if not evidence.is_file() or not evidence.read_bytes().strip():raise ValueError('Nonempty measurement evidence required')


def load_payload_profile(recipe,root):
    """Explicit measured profile, immutable bytes; null/missing inputs reject."""
    name=recipe.get('payload_profile')
    if not isinstance(name,str) or not name:raise ValueError('Measured payload_profile required for live execution')
    path=(Path(root)/name).resolve()
    if not path.is_file() or recipe.get('payload_profile_sha256')!=sha(path):raise ValueError('Exact measured payload profile SHA256 required')
    profile=json.loads(path.read_text())
    from .payload_boundary import PayloadBoundaries
    # Full profile evidence/parameters validation without SDK or device I/O.
    from types import SimpleNamespace
    PayloadBoundaries(SimpleNamespace(),profile,root,apply=lambda *a:None)
    binary=(Path(root)/profile['driver_binary']).resolve()
    if not binary.is_file():raise ValueError('Built stopped-boundary payload driver required')
    return profile


def validate_payload_sdk_state(manager,side,state):
    """Read-only initial actual tool/load check; Hand is never counted twice."""
    import numpy as np
    from .payload_boundary import parameters
    if state.get('robot_mode_name')!='idle' or any(abs(v)>.001 for v in state['dq']+state['dq_d']):raise RuntimeError('Measured idle stopped payload startup required')
    for key,mass,center,inertia in [('tool','m_ee_kg','F_x_Cee_m','I_ee_column_major_kg_m2'),('empty_payload','m_load_kg','F_x_Cload_m','I_load_column_major_kg_m2')]:
        wanted=manager.profile['robots'][side][key];actual=dict(wanted,mass_kg=state[mass],center_m=state[center],inertia_column_major_kg_m2=state[inertia])
        a=parameters(wanted);b=parameters(actual)
        if not np.allclose([a[0],*a[1],*a[2].ravel()],[b[0],*b[1],*b[2].ravel()],atol=1e-9,rtol=0):raise RuntimeError('Actual SDK initial tool/load differs from measured profile; no overwrite')


def validate_recipe(path,site_config=None):
    path=Path(path).resolve();c=json.loads(path.read_text());root=path.parent
    if c.get('schema_version')!=2 or c.get('recognition')!='seed_free_rgbd':raise ValueError('Random cycle schema 2 / seed_free_rgbd required')
    if len(c['parts'])!=3 or {e['part'] for e in c['parts']}!=set(PARTS):raise ValueError('Exactly ring1, ring2 and pin required')
    for key in ('site_config','camera_config','planner_config'):
        if not (root/c[key]).is_file():raise ValueError('Missing actual configuration: '+key)
    measured(c['commissioning'],root)
    bounded(c['feedback_timeout_s'],.1,5,'feedback_timeout_s');bounded(c['feedback_dwell_s'],.1,1,'feedback_dwell_s')
    if c['feedback_timeout_s']<c['feedback_dwell_s']:raise ValueError('Feedback timeout must cover dwell')
    bounded(c['max_task_s'],60,1800,'max_task_s')
    bounded(c['max_observation_age_s'],.1,2,'max_observation_age_s')
    bounded(c['held_roi_radius_m'],.01,.048,'held_roi_radius_m')
    bounded(c['target_position_tolerance_m'],0,.001,'target_position_tolerance_m',positive=True)
    bounded(c['target_angle_tolerance_rad'],0,.01,'target_angle_tolerance_rad',positive=True)
    bounded(c['release_width_tolerance_m'],0,.002,'release_width_tolerance_m',positive=True)
    bounded(c['insert_pose_confirmation_tolerance_m'],0,.003,'insert_pose_confirmation_tolerance_m',positive=True)
    for e in c['parts']:
        if e['pickup_side'] not in ('left','right') or e['insert_side'] not in ('left','right'):raise ValueError('Explicit arm assignment required')
        rigid(c['insertion_targets'][e['part']]);bounded(e['min_tcp_clearance_m'],.04,1,'min_tcp_clearance_m')
        if not e['grasp_families']:raise ValueError('Measured grasp families required')
        for family in e['grasp_families']:
            if not family['name']:raise ValueError('Named measured grasp family required')
            rigid(family['T_part_tcp']);measured(family['measurement'],root,subject=family)
            if family.get('pregrasp_offset_part_m')is not None:offset(family['T_part_tcp'],family['pregrasp_offset_part_m'])
            actions=family.get('actions') or (site_config or {})['gripper_actions'][e['part']]
            for mode in ('open','release'):FrankaHand.validate_action(actions[mode])
            FrankaHand.validate_action(actions['grasp'],grasp=True)
            if actions['open']['width_m']<=actions['grasp']['width_m']+actions['grasp']['epsilon_outer_m']:raise ValueError('Opening must clear actual grasp width')
        dimensions=e['payload_dimensions'];measured(dimensions['measurement'],root,subject=dimensions)
        if e.get('shape_model_measurement'):
            measured(e['shape_model_measurement'],root,subject=dict(capsules_part_m=e['capsules_part_m'],payload_dimensions=e['payload_dimensions']))
        for capsule in e.get('capsules_part_m') or []:
            for endpoint in ('a','b'):
                offset(e['grasp_families'][0]['T_part_tcp'],capsule[endpoint])
            bounded(capsule['radius_m'],.00001,.2,'measured part shell capsule radius')
        bounded(dimensions['radius_m'],.001,.2,'payload radius');bounded(dimensions['length_m'],.001,.5,'payload length')
        offset(c['insertion_targets'][e['part']],e['insert_entry_offset_part_m'],local=True)
        offset(c['insertion_targets'][e['part']],e['pregrasp_offset_part_m'],local=True)
        if not e['lift_offsets_cell_m']:raise ValueError('Measured above-basket lift corridor required')
        for delta in e['lift_offsets_cell_m']:offset(c['insertion_targets'][e['part']],delta)
        if e['pickup_side']!=e['insert_side']:
            h=e['handover'];rigid(h['donor_T_cell_tcp']);rigid(h['receiver_T_part_tcp'])
            for delta in (h['receiver_approach_offset_part_m'],h['donor_retreat_offset_cell_m']):offset(h['donor_T_cell_tcp'],delta)
            for mode in ('receiver_open','donor_release','receiver_release'):FrankaHand.validate_action(h[mode])
            FrankaHand.validate_action(h['receiver_grasp'],grasp=True)
            if h['donor_grasped_signal']==h['receiver_grasped_signal']:raise ValueError('Distinct donor/receiver evidence names required')
            for waypoint in h['donor_waypoints_cell_tcp']:rigid(waypoint)
    c['_path']=str(path);c['_root']=str(root);c['_sha256']=sha(path)
    if c.get('stabilization_profile') is not None:
        from .stabilization import bind_recipe
        c['_stabilization_binding']=bind_recipe(c,c['stabilization_profile'])
    return c


def validate_live_inputs(recipe,site_config):
    """All physical inputs checked before constructing device adapters."""
    root=Path(recipe['_root']);load_payload_profile(recipe,root)
    if recipe.get('replay_only')is True:raise ValueError('Synthetic replay recipe cannot enable real hardware')
    if recipe.get('clamp_control',{}).get('mode')not in ('actual_PLC','operator_manual'):raise ValueError('Explicit actual_PLC or operator_manual clamp control required')
    if site_config.get('part_evidence_source')!='RGBD_SDK'or site_config.get('abort_hand_policy')!='preserve_completed_grasp':raise ValueError('Live random policy requires RGBD_SDK and preserve_completed_grasp abort')
    config=json.loads((root/recipe['planner_config']).read_text())
    if config['scene'].get('require_residual_occupancy')is not True:raise ValueError('Conservative residual RGB-D occupancy required')
    departure=config['phases'].get('stock_departure',{}).get('bounded_loose_stock_departure')
    policy=config['phases']['pickup'].get('bounded_loose_stock_contact')or (departure or {}).get('contact_policy')
    if policy is not None:
        from .stock_contact import validate_stock_contact_policy
        for side in {e['pickup_side']for e in recipe['parts']if e['part'].startswith('ring')}:
            validate_stock_contact_policy(dict(config['phases']['pickup'],bounded_loose_stock_contact=policy,_phase_name='pickup',_contact='grasp',_active_part='ring1'),config['robots'][side])
            if departure is not None:
                from .stock_departure import validate_departure_policy
                validate_departure_policy(dict(config['phases']['stock_departure'],_phase_name='stock_departure',_contact='depart',_active_part='ring1'),config['robots'][side])
        inspection=recipe.get('inventory_inspection')
        if not isinstance(inspection,dict):raise ValueError('Actual two-view inspection and measured bin return required')
        for name in ('inventory_inspection','inventory_return','empty_hand_inspection'):
            if name not in config['phases']:raise ValueError('Measured planner corridor missing: '+name)
        envelope=inspection['uncertain_payload_dimensions'];measured(envelope['measurement'],root)
        bounded(envelope['radius_m'],.06,.3,'uncertain inventory radius');bounded(envelope['length_m'],.01,.6,'uncertain inventory length')
        for side in ('left','right'):
            mount=inspection['native_hand_mount'][side];rigid(mount['T_flange_hand']);measured(mount['measurement'],root,subject=mount)
            views=inspection['relative_tcp_views'][side]
            if not 1<=len(views)<=3:raise ValueError('One to three actual checked inspection rotations required')
            for step in views:rigid(step['T_tcp_inspection']);measured(step['measurement'],root)
            back=inspection['return_over_bin'][side];rigid(back['T_cell_tcp']);measured(back['measurement'],root)
            from .collision import box
            box(back['release_region']);measured(back['release_region']['measurement'],root)
            FrankaHand.validate_action(back['release_action'])
            if back['release_action']['width_m']<.078:raise ValueError('Bin return open width must be at least78mm')
            if not back.get('empty_hand_views'):raise ValueError('Actual complementary empty-Hand inspection required')
            for step in back['empty_hand_views']:rigid(step['T_tcp_inspection']);measured(step['measurement'],root)
    return dict(physical_inputs_checked=True,hardware_tested=False)


def check_observation(runtime,observation):
    if observation.get('state')!='random_ring_observation' or observation.get('replay') is not runtime.replay:
        raise ValueError('Expected explicit live or replay random image observation')
    metadata=observation['source_metadata']
    for key in ('capture_id','captured_at_utc','calibration_sha256'):
        if not observation.get(key) or observation[key]!=metadata.get(key):raise ValueError('Image source identity mismatch')
    if not runtime.replay:
        age=(datetime.now(timezone.utc)-utc_stamp(observation['captured_at_utc'])).total_seconds()
        if metadata.get('source')!='realsense_live' or metadata.get('replay') is not False or not 0<=age<=runtime.recipe['max_observation_age_s']:
            raise RuntimeError('Live image unavailable, stale or from replay')
    if observation.get('_scope')=='basket' and observation.get('ring_pose_seed_used') is not False:
        raise RuntimeError('Random basket image must not use ring pose seeds')
    return observation


def instances(observation,part):
    values=[v for v in observation.get('instances',[]) if v.get('part')==part]
    if not values:raise RuntimeError('No image-derived accessible candidate: '+part)
    for value in values:
        if not value.get('instance_id'):raise ValueError('Image-derived instance ID required')
        rigid(value['T_cell_part'])
    return values


def select_observation(observation,instance):
    result=copy.deepcopy(observation);result['selected_instance']=copy.deepcopy(instance)
    result['selected_part']=instance['part'];result['T_cell_part']=instance['T_cell_part']
    return result


def held_measurement(runtime,entry,side,previous=None):
    import numpy as np
    observed=check_observation(runtime,runtime.observe('held',entry['part'],side=side,previous=previous))
    values=instances(observed,entry['part'])
    if len(values)!=1:raise RuntimeError('Held part image is ambiguous; retain and stop')
    observed=select_observation(observed,values[0])
    tcp=rigid(runtime.tcp(side))
    g=np.linalg.inv(tcp)@rigid(values[0]['T_cell_part'])
    observed['measured_tcp_pose']=tcp.tolist();observed['T_tcp_part']=g.tolist()
    facts=getattr(runtime,'facts',None)
    if facts is not None:facts.confirm_held(entry['part'],side,observed,g,runtime.hand_status(side))
    payload=getattr(runtime,'payload',None)
    if payload is not None and payload.owner[side]==entry['part']:
        payload.refresh_held(side,entry['part'],runtime.flange_frame(side),g)
        runtime.event('payload_refresh_after_fresh_RGBD',side=side,part=entry['part'],capture_id=observed['capture_id'])
    carry=dict(part=entry['part'],robot=side,T_tcp_part=g.tolist(),payload_dimensions=entry['payload_dimensions'],
        capsules_part_m=copy.deepcopy(entry.get('capsules_part_m')),
        measurement=dict(verified=True,by='live_rgbd_registration' if not runtime.replay else 'replay_contract_only',
            at_utc=observed['captured_at_utc'],evidence_file=observed['_path']),observation_capture_id=observed['capture_id'])
    return observed,carry


class PreMotionPickupChanged(RuntimeError):
    """Only a fresh image/scene rejection before the arm process is launched."""


def screen_pickup(runtime,entry,observation,carried,*,rejected_targets=None):
    """Bounded pixel-only search: alternate instances before repeating families.

    No Hand/arm action occurs here. Capture-scoped IDs are never remembered as
    object identity; failed world TCP/pregrasp targets are compared geometrically.
    """
    import numpy as np
    rejected_targets=rejected_targets if rejected_targets is not None else []
    part=entry['part'];side=entry['pickup_side'];failures=[];plans=0;captures=1
    maximum=int(runtime.recipe.get('pickup_search_max_plans',16));maxcaptures=int(runtime.recipe.get('pickup_search_max_captures',2))
    if not 1<=maximum<=40 or not 1<=maxcaptures<=4:raise ValueError('Bounded pickup search plan/capture budgets required')
    def equivalent(a,b):
        return (np.linalg.norm(a[:3,3]-b[:3,3])<=runtime.recipe['target_position_tolerance_m']and
            np.arccos(np.clip((np.trace(a[:3,:3]@b[:3,:3].T)-1)/2,-1,1))<=runtime.recipe['target_angle_tolerance_rad'])
    def old():
        return not runtime.replay and (datetime.now(timezone.utc)-utc_stamp(observation['captured_at_utc'])).total_seconds()>runtime.recipe['max_observation_age_s']
    while True:
        candidates=instances(observation,part)
        if part.startswith('ring'):candidates=[v for v in candidates if v.get('geometrically_accessible')is True]
        if not candidates:raise RuntimeError('No screened pickup candidate: no unambiguous image-derived accessible '+part)
        expired=False
        # Breadth first: a blocked wall-side ring does not consume every yaw
        # before another exposed ring of the SAME required class is considered.
        for family in entry['grasp_families']:
            for instance in candidates:
                if old():expired=True;break
                tcp=rigid(instance['T_cell_part'])@rigid(family['T_part_tcp'])
                pre=rigid(instance['T_cell_part'])@offset(family['T_part_tcp'],family.get('pregrasp_offset_part_m',entry['pregrasp_offset_part_m']))
                if any(equivalent(pre,p)and equivalent(tcp,t)for p,t in rejected_targets):continue
                if plans>=maximum:raise RuntimeError('No screened pickup candidate: bounded planning budget exhausted '+repr(failures))
                observed=select_observation(observation,instance);plans+=1
                try:plan=runtime.plan('pickup',part,side,[pre.tolist(),tcp.tolist()],observed,carried,contact='grasp')
                except (RuntimeError,ValueError)as error:
                    if old()and not isinstance(error,CollisionError)and not str(error).startswith('Read-only candidate unavailable:'):
                        expired=True;break  # stale information is not a failed physical target
                    failures.append(str(error));rejected_targets.append((pre.copy(),tcp.copy()));continue
                runtime.event('pickup_candidate_selected',part=part,plans=plans,captures=captures,failed_targets=len(rejected_targets),capture_id=observation['capture_id'],family=family['name'])
                return observed,family,plan,pre,tcp
            if expired:break
        if captures>=maxcaptures:raise RuntimeError('No screened pickup candidate: '+repr(failures))
        observation=check_observation(runtime,runtime.observe('pin_holder'if part=='pin'else'basket',part,side=side));captures+=1


def run(runtime,recipe,*,single_pick=None):
    """Same state transitions in live/replay; failures stop without retries/opening."""
    import numpy as np
    completed=[];installed=set();held_signals=set();carried={};started=time.monotonic()
    payload=getattr(runtime,'payload',None);facts=getattr(runtime,'facts',None)
    def ready():
        if payload is not None and not payload.valid:raise RuntimeError('Payload state unknown; all motion inhibited')
        if time.monotonic()-started>recipe['max_task_s']:raise TimeoutError('Random cycle time budget exceeded')
        for signal in installed|held_signals:
            if getattr(runtime,'stock_return_release_part',None)and signal.startswith(runtime.stock_return_release_part+'_'):continue
            if not runtime.read(signal):raise RuntimeError('Measured part feedback lost: '+signal)
    runtime.monitor=ready
    runtime.carried_state=lambda:copy.deepcopy(list(carried.values()))
    def wait(signal):runtime.wait(signal,recipe['feedback_timeout_s'],recipe['feedback_dwell_s'])
    def move(phase,part,side,targets,observation,contact=None,plan=None):
        ready()
        if payload is not None:
            for held in carried.values():
                if payload.owner[held['robot']]!=held['part']:raise RuntimeError('Measured payload ownership differs from carried geometry; inhibit motion')
        plan=plan or runtime.plan(phase,part,side,targets,observation,list(carried.values()),contact=contact)
        runtime.execute(plan,observation);ready()
    try:
        if not runtime.replay and payload is None:raise RuntimeError('Live cycle requires measured automatic payload boundaries')
        runtime.ready()
        for part in PARTS:
            for suffix in ('grasped','seated','clamped','retained'):
                if runtime.read(part+'_'+suffix):raise RuntimeError('Fresh cycle requires cleared '+part+'_'+suffix)
        # Observe before choosing any trajectory. Every subsequent basket call is
        # another real capture; IDs from an earlier image are never carried over.
        for entry in recipe['parts']:
            if single_pick is not None and entry['part']!=single_pick:continue
            part=entry['part'];donor,receiver=entry['pickup_side'],entry['insert_side']
            observation=check_observation(runtime,runtime.observe('pin_holder' if part=='pin' else 'basket',part,side=donor))
            if runtime.hand_status(donor)['is_grasped']:raise RuntimeError('Pickup hand is not empty')
            prepared_open=False
            if hasattr(runtime,'prepare_pickup_hand'):
                prepared_open=runtime.prepare_pickup_hand(entry,observation)
                if prepared_open:observation=check_observation(runtime,runtime.observe('pin_holder'if part=='pin'else'basket',part,side=donor))
            rejected=[];preflight_retries=0
            while True:
                observed,family,plan,pre,tcp=screen_pickup(runtime,entry,observation,list(carried.values()),rejected_targets=rejected)
                actions=family.get('actions')or runtime.site.config['gripper_actions'][part]
                if not prepared_open:runtime.hand_move(donor,part,'open',actions['open']);prepared_open=True
                try:move('pickup',part,donor,[],observed,plan=plan);break
                except PreMotionPickupChanged:
                    if preflight_retries>=1 or runtime.hand_status(donor)['is_grasped']:raise
                    rejected.append((pre.copy(),tcp.copy()));preflight_retries+=1
                    observation=check_observation(runtime,runtime.observe('pin_holder'if part=='pin'else'basket',part,side=donor))
                    runtime.event('pickup_pre_motion_reselection',part=part,retry=preflight_retries)
            if runtime.read(part+'_grasped'):raise RuntimeError('Grasp feedback high before actual command')
            runtime.hand_move(donor,part,'grasp',actions['grasp'])
            if facts is None:wait(part+'_grasped')
            if not runtime.hand_status(donor)['is_grasped']:raise RuntimeError('Actual pickup was not grasped')
            if facts is None:held_signals.add(part+'_grasped')
            runtime.site.expected_held[donor]=part
            observed,carry=held_measurement(runtime,entry,donor,observed)
            if hasattr(runtime,'constrain_initial_held_floor'):observed,carry=runtime.constrain_initial_held_floor(entry,donor,observed,carry)
            carried[part]=carry
            held_signals.add(part+'_grasped')
            if payload is not None:
                payload.after_grasp(donor,part,runtime.flange_frame(donor),carry['T_tcp_part'])
                runtime.event('payload_grasp_confirmed_before_lift',side=donor,part=part)
            current=rigid(runtime.tcp(donor));targets=[];remaining=[];lift_phase='lift'
            if hasattr(runtime,'initial_stock_lift'):
                lift_phase,targets,remaining=runtime.initial_stock_lift(entry,current)
            else:
                for delta in entry['lift_offsets_cell_m']:
                    current=offset(current,delta);targets.append(current.tolist())
            move(lift_phase,part,donor,targets,observed,contact='depart'if lift_phase=='stock_departure'else None)
            observed,carried[part]=held_measurement(runtime,entry,donor,observed)
            if hasattr(runtime,'verify_stock_inventory'):
                observed,carried[part]=runtime.verify_stock_inventory(entry,donor,observed,carried[part],move)
            # Further transport uses strict free-space checks, only AFTER the
            # bounded contact lift has a fresh complementary-view item proof.
            if remaining:
                current=rigid(runtime.tcp(donor));targets=[]
                for delta in remaining:current=offset(current,delta);targets.append(current.tolist())
                move('lift',part,donor,targets,observed)
                observed,carried[part]=held_measurement(runtime,entry,donor,observed)
            if single_pick is not None:
                runtime.ready()
                return dict(state='single_pick_held_for_commissioning',part=part,side=donor,
                    measured_T_tcp_part=carried[part]['T_tcp_part'],actual_hand=runtime.hand_status(donor),
                    single_item_inventory_proof=getattr(runtime,'latest_inventory_proof',None),
                    motion_executed=not runtime.replay,hardware_execution_verified=False,
                    instruction='Arms are stopped; completed native grasp is retained. No automatic release or resume.')
            if donor!=receiver:
                h=entry['handover']
                for name in (h['donor_grasped_signal'],):
                    if not runtime.read(name):raise RuntimeError('Independent donor feedback missing')
                if runtime.read(h['receiver_grasped_signal']) or runtime.hand_status(receiver)['is_grasped']:
                    raise RuntimeError('Receiver feedback must start clear')
                held_signals.add(h['donor_grasped_signal'])
                runtime.hand_move(receiver,part,'open',h['receiver_open'])
                move('donor_carry',part,donor,[*h['donor_waypoints_cell_tcp'],h['donor_T_cell_tcp']],observed)
                observed,carried[part]=held_measurement(runtime,entry,donor,observed)
                part_pose=rigid(observed['T_cell_part'])
                receiver_tcp=part_pose@rigid(h['receiver_T_part_tcp'])
                approach=part_pose@offset(h['receiver_T_part_tcp'],h['receiver_approach_offset_part_m'])
                move('receiver_approach',part,receiver,[approach.tolist(),receiver_tcp.tolist()],observed,contact='handover')
                if runtime.read(h['receiver_grasped_signal']):raise RuntimeError('Receiver feedback became high before command')
                runtime.hand_move(receiver,part,'grasp',h['receiver_grasp'])
                if facts is None:wait(h['receiver_grasped_signal'])
                else:
                    # Receiving part must be optically confirmed while donor still holds.
                    receiver_image,receiver_carry=held_measurement(runtime,entry,receiver,observed)
                    facts.confirm_held(part,receiver,receiver_image,receiver_carry['T_tcp_part'],runtime.hand_status(receiver))
                if not runtime.hand_status(receiver)['is_grasped'] or not runtime.hand_status(donor)['is_grasped']:
                    raise RuntimeError('Both actual Hands must confirm before donor opens')
                held_signals.add(h['receiver_grasped_signal']);ready()
                runtime.site.expected_held.pop(donor,None);runtime.site.expected_held[receiver]=part
                held_signals.remove(h['donor_grasped_signal'])
                opened=runtime.hand_move(donor,part,'release',h['donor_release'])
                if opened['is_grasped'] or opened['width_m']<h['donor_release']['width_m']-recipe['release_width_tolerance_m']:
                    raise RuntimeError('Donor opening was not measured')
                if facts is not None:facts.confirm_release(part,donor,opened,h['donor_release']['width_m']-recipe['release_width_tolerance_m'])
                observed,carried[part]=held_measurement(runtime,entry,receiver,observed)
                if facts is not None:facts.confirm_transfer(part,donor,receiver,observed,carried[part]['T_tcp_part'],runtime.hand_status(donor),runtime.hand_status(receiver),h['donor_release']['width_m']-recipe['release_width_tolerance_m'])
                if payload is not None:
                    if facts is None:runtime.wait_signal(h['donor_grasped_signal'],False)
                    frames={side:runtime.flange_frame(side)for side in ('left','right')}
                    payload.after_transfer_release(donor,receiver,part,frames,carried[part]['T_tcp_part'],h['donor_grasped_signal'],h['receiver_grasped_signal'])
                    runtime.event('payload_transfer_confirmed_before_retreat',donor=donor,receiver=receiver,part=part)
                retreat=offset(runtime.tcp(donor),h['donor_retreat_offset_cell_m'])
                move('donor_retreat',part,donor,[retreat.tolist()],observed)
                observed,carried[part]=held_measurement(runtime,entry,receiver,observed)
            # G is remeasured from actual held images, rather than a nominal
            # grasp transform or a remembered object pose in the basket.
            observed,carried[part]=held_measurement(runtime,entry,receiver,observed)
            goal=rigid(recipe['insertion_targets'][part])
            entry_part=offset(goal,entry['insert_entry_offset_part_m'],local=True)
            desired=entry_part@np.linalg.inv(rigid(carried[part]['T_tcp_part']))
            move('preinsert',part,receiver,[desired.tolist()],observed)
            observed,carried[part]=held_measurement(runtime,entry,receiver,observed)
            desired=goal@np.linalg.inv(rigid(carried[part]['T_tcp_part']))
            if runtime.read(part+'_seated'):raise RuntimeError('Seat feedback high before insertion')
            move('insert',part,receiver,[desired.tolist()],observed,contact='insert')
            if facts is not None:
                observed,carried[part]=held_measurement(runtime,entry,receiver,observed)
                facts.confirm_seated(part,observed)
            wait(part+'_seated');installed.add(part+'_seated')
            if runtime.read(part+'_clamped'):raise RuntimeError('Clamp feedback high before command')
            runtime.clamp(part);wait(part+'_clamped');installed.add(part+'_clamped')
            if runtime.read(part+'_retained'):raise RuntimeError('Retention must confirm after commanded opening')
            if not runtime.hand_status(receiver)['is_grasped']:raise RuntimeError('Insertion Hand lost the part')
            runtime.site.expected_held.pop(receiver,None);held_signals.discard(part+'_grasped')
            if donor!=receiver:held_signals.discard(entry['handover']['receiver_grasped_signal'])
            release=entry['handover']['receiver_release'] if donor!=receiver else actions['release']
            opened=runtime.hand_move(receiver,part,'release',release)
            if opened['is_grasped'] or opened['width_m']<release['width_m']-recipe['release_width_tolerance_m']:
                raise RuntimeError('Actual receiver opening not confirmed')
            if facts is not None:facts.confirm_release(part,receiver,opened,release['width_m']-recipe['release_width_tolerance_m'])
            wait(part+'_retained');installed.add(part+'_retained')
            if payload is not None:
                payload.after_retained_release(receiver,part,runtime.flange_frame(receiver))
                runtime.event('payload_empty_confirmed_before_withdraw',side=receiver,part=part)
            carried.pop(part)
            observed=check_observation(runtime,runtime.observe('held',part,side=receiver,previous=observed))
            visible=instances(observed,part)
            if len(visible)!=1:raise RuntimeError('Released fixture part is not uniquely visible')
            observed=select_observation(observed,visible[0])
            if np.linalg.norm(rigid(observed['T_cell_part'])[:3,3]-goal[:3,3])>recipe['insert_pose_confirmation_tolerance_m']:
                raise RuntimeError('Image of installed part disagrees with measured fixture')
            runtime.mark_installed(part,observed['T_cell_part'],entry['payload_dimensions'],observed,capsules_part_m=entry.get('capsules_part_m'))
            move('withdraw',part,receiver,[offset(runtime.tcp(receiver),entry['withdraw_offset_cell_m']).tolist()],observed)
            runtime.verify_clearance(recipe);completed.append(part)
            runtime.event('part_completed',part=part)
        final=time.monotonic()
        while time.monotonic()-final<.5:runtime.ready();time.sleep(.02)
        runtime.verify_clearance(recipe)
        if any(runtime.hand_status(s)['is_grasped'] for s in ('left','right')):raise RuntimeError('Held object at cycle end')
        runtime.ready()
        return dict(state='random_cycle_replay_contract_completed' if runtime.replay else 'random_cycle_completed',
            completed_parts=completed,all_parts_retained=True,replay=runtime.replay,
            hardware_execution_verified=False,motion_executed=not runtime.replay,
            stabilization_profile=recipe.get('_stabilization_binding',{}).get('profile_id'),
            automatic_payload_boundaries_exercised=payload is not None,
            payload_write_scope='fake_offline' if runtime.replay else 'SDK_stopped_boundary',
            experimental_not_field_ready=True)
    except BaseException:
        runtime.stop();raise
    finally:runtime.monitor=lambda:None


class LiveRuntime:
    """Concrete persistent RGB-D, native Hand/PLC and dynamic planner adapter."""
    replay=False
    def __init__(self,site,recipe,output_dir,*,planner=None):
        from .random_planner import DynamicPlanner
        self.site,self.recipe,self.root=site,recipe,Path(recipe['_root'])
        self.output=Path(output_dir).resolve();self.output.mkdir(parents=True,exist_ok=True)
        self.planner=planner or DynamicPlanner(site,self.root/recipe['planner_config'],self.output/'plans')
        planner_settings=json.loads((self.root/recipe['planner_config']).read_text())
        if planner_settings.get('scene',{}).get('require_residual_occupancy')is not True:raise ValueError('Live RGB-D/SDK cycle requires conservative residual occupancy in every basket plan')
        self.require_basket_observation=planner_settings.get('scene',{}).get('require_basket_observation',True)
        self.width_aware=any(v.get('native_hand_aperture_geometry')for v in planner_settings['robots'].values())
        self.known_hand_surface=planner_settings['scene'].get('known_hand_surface')
        if self.known_hand_surface is not None:
            from .known_hand_capture import settings
            settings(self.known_hand_surface)
        self._active_aperture_binding=None;self._latest_plan_context=None;self._stopped_arm_monitors={}
        self.monitor=lambda:None;self.installed_parts=[];self.streams={};self.camera_configs={};self.scene_epoch=uuid.uuid4().hex
        from .sensor_facts import SensorFacts
        self.facts=SensorFacts(recipe)
        self.payload=None
        profile=load_payload_profile(recipe,self.root)
        from .payload_boundary import PayloadBoundaries
        self.payload=PayloadBoundaries(site,profile,self.root,facts=self.facts)
        payload_path=(self.root/recipe['payload_profile']).resolve();driver=(self.root/profile['driver_binary']).resolve()
        self._payload_bound_files={payload_path:sha(payload_path),driver:sha(driver)}
        from .hand_monitor import HandMonitor
        self.hand_monitors={};self._raw_hand_status={}
        for side,hand in site.grippers.items():
            raw=hand.status;self._raw_hand_status[side]=raw
            monitor=HandMonitor(raw);self.hand_monitors[side]=monitor
            # One actual synchronous startup sample, before motion/heartbeat.
            monitor.publish(raw());hand.status=monitor.status
        self._base_ready=site.require_ready;site.require_ready=self.ready
        # Read-only startup comparison; no payload overwrite before the cycle.
        for side in ('left','right'):
            validate_payload_sdk_state(self.payload,side,self.sdk_state(side))
    def ready(self):
        if hasattr(self,'_payload_bound_files') and any(sha(path)!=digest for path,digest in self._payload_bound_files.items()):
            self.payload.valid=False;self.site.cancelled.set();raise RuntimeError('Payload profile or driver changed; inhibit motion')
        if self.payload is None or not self.payload.valid:raise RuntimeError('Known measured payload state required')
        self._base_ready();self.monitor()
        for monitor in getattr(self,'_stopped_arm_monitors',{}).values():monitor.status()
        if getattr(self,'_active_aperture_binding',None)is not None:
            from .hand_aperture import current_within
            current_within(self._active_aperture_binding,{s:self.hand_status(s)for s in ['left','right']})
    def wait_signal(self,name,expected):
        return self.site.guarded(lambda cancel:self.site.plc.wait(name,timeout_s=self.recipe['feedback_timeout_s'],dwell_s=self.recipe['feedback_dwell_s'],cancelled=cancel,expected=expected))
    def read(self,name):
        for entry in self.recipe['parts']:
            part=entry['part']
            if name==part+'_grasped':return any(self.facts.grasped(part,s,self.hand_status(s))for s in ('left','right'))
            for suffix in ('seated','clamped','retained'):
                if name==part+'_'+suffix:return self.facts.fixture_fact(part,suffix)
            if entry['pickup_side']!=entry['insert_side']:
                h=entry['handover']
                if name==h['donor_grasped_signal']:return self.facts.grasped(part,entry['pickup_side'],self.hand_status(entry['pickup_side']))
                if name==h['receiver_grasped_signal']:return self.facts.grasped(part,entry['insert_side'],self.hand_status(entry['insert_side']))
        raise ValueError('Unknown part evidence request: '+name)
    def wait(self,name,timeout,dwell):
        for entry in self.recipe['parts']:
            part=entry['part']
            if name==part+'_retained':
                side=entry['insert_side'];images=[]
                for index in range(2):
                    if index:time.sleep(float(self.recipe.get('retention_observation_dwell_s',.3)))
                    image=check_observation(self,self.observe_fixture(part));values=instances(image,part)
                    if len(values)!=1:raise RuntimeError('Released part not uniquely visible for persistence')
                    images.append(select_observation(image,values[0]))
                deadline=time.monotonic()+timeout
                required=float(self.recipe.get('retention_observation_dwell_s',.3))
                while (utc_stamp(images[-1]['captured_at_utc'])-utc_stamp(images[0]['captured_at_utc'])).total_seconds()<required:
                    if time.monotonic()>deadline:raise RuntimeError('Actual postrelease exposures did not cover retention dwell')
                    image=check_observation(self,self.observe_fixture(part));values=instances(image,part)
                    if len(values)!=1:raise RuntimeError('Fixture persistence became ambiguous')
                    images.append(select_observation(image,values[0]))
                self.facts.confirm_retained(part,side,images,self.hand_status(side));self.event('RGBD_postrelease_retention',part=part,captures=[i['capture_id']for i in images]);return
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.ready()
            if self.read(name):return
            time.sleep(.02)
        raise TimeoutError('RGB-D/SDK fact not confirmed: '+name)
    def clamp(self,part):
        control=self.recipe.get('clamp_control',{});mode=control.get('mode')
        if mode=='actual_PLC':
            if self.site.plc is None:raise ValueError('Actual mapped clamp PLC unavailable')
            self.site.guarded(lambda cancel:self.site.plc.write(part+'_clamp_command',True))
            self.site.guarded(lambda cancel:self.site.plc.wait(part+'_clamped',timeout_s=self.recipe['feedback_timeout_s'],dwell_s=self.recipe['feedback_dwell_s'],cancelled=cancel))
            evidence=dict(signal=part+'_clamped',source='mapped_PLC_readback')
        elif mode=='operator_manual':
            expected='CLAMP '+part
            answer=input('Physically clamp '+part+' and enter '+expected+': ').strip()
            if answer!=expected:raise RuntimeError('Manual clamp confirmation absent; no automatic clamp claimed')
            evidence=dict(operator_confirmation=answer,at_utc=datetime.now(timezone.utc).isoformat())
        else:raise ValueError('Explicit actual_PLC or operator_manual clamp mode required')
        self.facts.confirm_clamp(part,source='actual_PLC'if mode=='actual_PLC'else'operator_manual_clamp',evidence=evidence)
        self.event('clamp_confirmation',part=part,mode=mode,evidence=evidence)
    def hand_status(self,side):return self.site.grippers[side].status()
    def hand_move(self,side,part,mode,parameters):
        # One cell lock covers fresh stopped SDK proof through Hand completion.
        with self.site.control_lock():return self._hand_move_locked(side,part,mode,parameters)
    def _hand_move_locked(self,side,part,mode,parameters):
        g=self.site.grippers[side]
        if mode=='grasp' and g.status()['width_m']<=parameters['width_m']+parameters['epsilon_outer_m']:
            raise ValueError('Field native grasp must close from a measured larger opening; simulated outward expansion is not enabled')
        if mode=='release' and parameters['width_m']<=g.status()['width_m']+self.recipe['release_width_tolerance_m']:
            raise ValueError('Release must open beyond the measured held width')
        proof=None
        if getattr(self,'width_aware',False):
            if self._active_aperture_binding is not None:raise RuntimeError('Hand commands cannot overlap checked arm motion or another Hand sweep')
            proof=self._check_hand_sweep(side,part,mode,parameters)
            self._active_aperture_binding=proof['aperture_binding']
        try:
            if proof is not None:
                from .stopped_arm_monitor import StoppedArmMonitor
                settings=json.loads((self.root/self.recipe['planner_config']).read_text())
                for arm in ['left','right']:
                    sdk=proof['active_SDK']if arm==side else proof['other_SDK']
                    self._stopped_arm_monitors[arm]=StoppedArmMonitor(lambda arm=arm:self.sdk_state(arm),sdk['state']['q_d'],settings.get('joint_tracking_error_limit_rad',.02))

            with self.site.control_lock():
                result=self.site.guarded(lambda cancel:g.move(**parameters,grasp=mode=='grasp',cancelled=cancel))
                self.hand_monitors[side].publish(result)
                if proof is not None:
                    from .hand_aperture import current_within
                    current_within(proof['aperture_binding'],{s:self.hand_status(s)for s in ['left','right']})
        finally:
            for monitor in getattr(self,'_stopped_arm_monitors',{}).values():monitor.close()
            self._stopped_arm_monitors={}
            if proof is not None:self._active_aperture_binding=None

        if mode!='grasp' and (result['is_grasped'] or result['width_m']<parameters['width_m']-self.recipe['release_width_tolerance_m']):
            raise RuntimeError('Native Hand opening was not measured')
        return result
    def prepare_pickup_hand(self,entry,observation):
        if not getattr(self,'width_aware',False):return False
        side=entry['pickup_side'];part=entry['part']
        actions=[family.get('actions')or self.site.config['gripper_actions'][part]for family in entry['grasp_families']]
        if any(action['open']!=actions[0]['open']for action in actions[1:]):raise ValueError('Width-aware candidate search requires one common actual pickup opening per part')
        current=self.hand_status(side)
        if current['is_grasped']:raise RuntimeError('Initial native pickup Hand is not empty')
        from .hand_aperture import TOLERANCE_M
        if abs(current['width_m']-actions[0]['open']['width_m'])<=TOLERANCE_M:return True
        choices=instances(observation,part)
        if part.startswith('ring'):choices=[v for v in choices if v.get('geometrically_accessible')is True]
        if not choices:raise RuntimeError('No screened pickup candidate before empty Hand opening')
        self._latest_plan_context=dict(part=part,phase='pickup',kind='actual_image_context_for_stopped_Hand_sweep',
            _runtime_observation=select_observation(observation,choices[0]))
        self.hand_move(side,part,'open',actions[0]['open'])
        return True

    def _check_hand_sweep(self,side,part,mode,parameters):
        import numpy as np
        from .hand_aperture import binding
        from .random_planner import check_stopped_aperture
        plan=self._latest_plan_context
        if not isinstance(plan,dict)or plan.get('part')!=part:raise RuntimeError('Current same-part screened context required before Hand actuation')
        held=getattr(self,'carried_state',lambda:[])();target=next((v for v in held if v['part']==part),None)
        if target is not None:
            owner=target['robot'];previous=json.loads(Path(target['measurement']['evidence_file']).read_text())
            previous['T_tcp_part']=target['T_tcp_part']
            fresh=check_observation(self,self.observe('held',part,side=owner,previous=previous));values=instances(fresh,part)
            if len(values)!=1:raise RuntimeError('Fresh held target ambiguous before Hand aperture sweep')
            fresh=select_observation(fresh,values[0])
            current_tcp=rigid(self.tcp(owner));G=np.linalg.inv(current_tcp)@rigid(fresh['T_cell_part'])
            fresh.update(measured_tcp_pose=current_tcp.tolist(),T_tcp_part=G.tolist())
            fresh=self._record(fresh)
            target['T_tcp_part']=G.tolist();target['observation_capture_id']=fresh['capture_id']
            target['measurement']=dict(verified=True,by='live_RGBD_SDK_stopped_Hand_sweep',at_utc=fresh['captured_at_utc'],evidence_file=fresh['_path'])
            self.facts.confirm_held(part,owner,fresh,G,self.hand_status(owner))
            if self.payload.owner[owner]==part:self.payload.refresh_held(owner,part,self.flange_frame(owner),G)
        else:
            original=plan['_runtime_observation'];fresh=check_observation(self,self.observe(original['_scope'],part,side=side,previous=original))
            old=rigid(original['selected_instance']['T_cell_part']);matches=[]
            for value in instances(fresh,part):
                current=rigid(value['T_cell_part']);angle=np.arccos(np.clip((np.trace(current[:3,:3]@old[:3,:3].T)-1)/2,-1,1))
                if np.linalg.norm(current[:3,3]-old[:3,3])<=self.recipe['target_position_tolerance_m']and angle<=self.recipe['target_angle_tolerance_rad']:matches.append(value)
            if len(matches)!=1:raise RuntimeError('Fresh target changed before Hand aperture sweep')
            fresh=select_observation(fresh,matches[0])
        fresh=self._selected_record(fresh)
        samples={s:self.hand_status(s)for s in ['left','right']};aperture=binding(samples,sweep_side=side,target_width_m=parameters['width_m'],epsilon_inner_m=parameters.get('epsilon_inner_m',0.),epsilon_outer_m=parameters.get('epsilon_outer_m',0.))
        phase=plan['phase'];contact=None
        if mode=='grasp':contact='handover'if target is not None and target['robot']!=side else'grasp'
        elif target is not None and target['robot']!=side:contact='handover'
        elif phase=='insert':contact='insert'
        proof=check_stopped_aperture(self.root/self.recipe['planner_config'],self.site,phase,part,side,fresh,held,aperture,contact=contact)
        path=self.output/('aperture-proof-'+uuid.uuid4().hex+'.json');path.write_text(json.dumps(proof,indent=2,allow_nan=False)+'\n');path.chmod(0o444)
        self.event('actual_stopped_Hand_full_aperture_sweep',part=part,side=side,mode=mode,proof_file=str(path),sha256=sha(path))
        return proof

    def sdk_state(self,side):
        import subprocess
        import numpy as np
        robot=self.site.config['robots'][side]
        p=subprocess.run([str(self.site.path(self.site.config['state_binary'])),robot['ip']],check=True,capture_output=True,text=True,timeout=10)
        state=json.loads(p.stdout)
        if state.get('read_only') is not True or state.get('server_protocol_version')!=10:raise ValueError('Measured protocol-10 robot state required')
        if any(abs(v)>.01 for v in state['dq']+state['dq_d']):raise RuntimeError('Stationary measured TCP required')
        return state
    def flange_frame(self,side):
        import numpy as np
        return rigid(np.asarray(self.sdk_state(side)['F_T_EE_column_major']).reshape(4,4,order='F'))
    def tcp(self,side):
        import numpy as np
        robot=self.site.config['robots'][side];state=self.sdk_state(side)
        t=rigid(np.asarray(state['O_T_EE_column_major']).reshape(4,4,order='F'))
        return (rigid(robot['T_cell_base'])@t).tolist()
    def _camera(self,scope,side):
        from .random_perception import load_random_config
        from .camera import RealSenseCapture
        import numpy as np
        role=('held_'+side) if scope=='held' else scope
        path=self.root/(self.recipe.get('camera_configs',{}).get(role) or self.recipe['camera_config'])
        key=str(path.resolve())
        if key not in self.camera_configs:self.camera_configs[key]=load_random_config(path)
        config=self.camera_configs[key];serial=config['camera_serial']
        if serial not in self.streams:self.streams[serial]=RealSenseCapture(config).start()
        stream=self.streams[serial]
        if (stream.config['camera_frame']!=config['camera_frame'] or
            not np.allclose(rigid(stream.config['T_cell_camera']),rigid(config['T_cell_camera']),atol=1e-9)):
            raise RuntimeError('One physical camera cannot assume different unimplemented moving extrinsics')
        stream.config=config
        return stream,config,path
    def _profile_check(self):
        binding=self.recipe.get('_stabilization_binding')
        if binding is not None:
            from .stabilization import revalidate_binding
            revalidate_binding(binding)
        return binding
    def _record(self,report):
        report=copy.deepcopy(report);report.pop('_sha256',None);report['installed_parts']=copy.deepcopy(self.installed_parts)
        binding=self.recipe.get('_stabilization_binding')
        if binding is not None:
            report['stabilization_profile']={key:binding[key] for key in ('profile_id','sha256','source_run','simulation_wall_elapsed_s','physical_speed_claimed')}
            report['stabilization_dependency_files']=copy.deepcopy(binding['dependency_files'])
        path=self.output/('observation-'+uuid.uuid4().hex+'.json');report['_path']=str(path)
        path.write_text(json.dumps(report,allow_nan=False,indent=2)+'\n');path.chmod(0o444);report['_sha256']=sha(path)
        return report
    def observe(self,scope,part,*,side=None,previous=None):
        from .random_perception import detect_packet
        from .camera import save_capture
        stream,config,camera_path=self._camera(scope,side)
        sdk_before=None
        if scope=='basket'and self.known_hand_surface is not None:
            from .known_hand_capture import read_sample
            sdk_before={arm:read_sample(self,arm)for arm in ['left','right']}
        packet,metadata=stream.read()
        if sdk_before is not None:
            # Only an exposure actually newer than both SDK reads is eligible.
            cutoff=max(utc_stamp(v['read_returned_utc'])for v in sdk_before.values())
            deadline=time.monotonic()+self.known_hand_surface['max_sdk_capture_gap_s']
            while utc_stamp(metadata['captured_at_utc'])<cutoff:
                if time.monotonic()>deadline:raise RuntimeError('No actual RGB-D exposure within stopped SDK synchronization budget')
                packet,metadata=stream.read()
        releases=[utc_stamp(v['released_at_utc'])for v in self.facts.releases.values()if v['part']==part] if part is not None else []
        if releases:
            # A buffered camera frame may predate SDK completion. Require an
            # actual newer exposure rather than relabeling its timestamp.
            required=max(releases);deadline=time.monotonic()+self.recipe['feedback_timeout_s']
            while utc_stamp(metadata['captured_at_utc'])<=required:
                if time.monotonic()>deadline:raise RuntimeError('No actual postrelease RGB-D exposure')
                packet,metadata=stream.read()
        sdk_bracket=None
        if sdk_before is not None:
            from .known_hand_capture import read_sample,bind_states
            after={arm:read_sample(self,arm)for arm in ['left','right']}
            mounts=self.recipe['inventory_inspection']['native_hand_mount']
            for arm,mount in mounts.items():measured(mount['measurement'],self.root,subject=mount)
            sdk_bracket=bind_states(sdk_before,after,metadata,self.site.config['robots'],mounts,self.known_hand_surface)
        path=self.output/('capture-'+uuid.uuid4().hex+'.npz');save_capture(path,packet,metadata)
        result=detect_packet(packet,config,metadata,replay=False,scope=scope,part=part,
            held_tcp_pose=self.tcp(side) if scope=='held' else None,
            held_roi_radius_m=self.recipe['held_roi_radius_m'],previous_observation=previous,
            held_robot=side if scope=='held' else None,held_grasp_transform=(previous or {}).get('T_tcp_part') if scope=='held' else None,
            allow_unknown_basket=scope=='basket' and part is None)
        if scope=='basket':
            from .residual_occupancy import build_occupancy
            settings=config.get('residual_occupancy')
            if not isinstance(settings,dict):raise ValueError('Actual basket residual occupancy settings required')
            camera_packet=dict(packet,T_cell_camera=config['T_cell_camera'])
            occupancy_metadata=dict(metadata,source_packet_sha256=sha(path),scene_epoch=self.scene_epoch,cell_frame=config['cell_frame'],depth_uncertainty_m=settings['depth_uncertainty_m'])
            snapshot=build_occupancy(camera_packet,occupancy_metadata,config['basket']['outer_bounds_cell_m'],voxel_size_m=settings['voxel_size_m'],depth_margin_m=settings['depth_margin_m'],max_age_s=self.recipe['max_observation_age_s'])
            result['residual_occupancy']=snapshot.to_record();result['basket_scene_status']='conservative_RGBD_residual_occupancy';result['unobserved_geometry_enclosed']=True;result['conservative_scene_obstacles']=[]
            result['residual_occupancy_metadata']=occupancy_metadata
            if sdk_bracket is not None:
                from .known_hand_capture import associate
                _,surface_proof=associate(snapshot,camera_packet,occupancy_metadata,sdk_bracket)
                result['known_hand_surface']=dict(sdk_bracket,association_proof=surface_proof)
        result.update(_scope=scope,_part=part,_side=side,source_packet=str(path),source_packet_sha256=sha(path),source_metadata_file=str(path.with_suffix('.json')),
            source_metadata_sha256=sha(path.with_suffix('.json')),_camera_config_path=str(camera_path.resolve()))
        result=self._record(result)
        if scope!='basket' and self.require_basket_observation:
            scene=self.observe('basket',None)
            # Only an independently matched currently-held image may identify a
            # body already represented by its measured carried geometry.
            if scope=='held' and self.site.expected_held.get(side)==part:
                import numpy as np
                current=[v for v in result['instances'] if v['part']==part]
                if len(current)==1:
                    position=rigid(current[0]['T_cell_part'])[:3,3]
                    candidates=[v for v in scene['instances'] if v['part']==part and
                        np.linalg.norm(rigid(v['T_cell_part'])[:3,3]-position)<=self.recipe['target_position_tolerance_m']]
                    if len(candidates)==1:
                        v=candidates[0];scene['excluded_instance_ids']=[v['instance_id']]
                        scene['excluded_measurements']=[dict(instance_id=v['instance_id'],part=part,T_cell_part=v['T_cell_part'],
                            reason='current_held_image_association',source_capture_id=result['capture_id'])]
                        # Tag observed selected surface with this exact current
                        # held-image association. UNKNOWN is never reassigned.
                        scene=select_observation(scene,v)
                        scene=self._selected_occupancy(scene)
                        scene=self._record(scene)
            result['scene_observations']=[scene]
            result=self._record(result)
        return result
    def observe_fixture(self,part):
        """Measured jig ROI only; actual pixels determine pose, never a fake TCP."""
        from .random_perception import detect_packet
        from .camera import save_capture
        stream,config,camera_path=self._camera('fixture',None)
        deadline=time.monotonic()+self.recipe['feedback_timeout_s'];last_error=None
        while time.monotonic()<deadline:
            packet,metadata=stream.read()
            releases=[utc_stamp(v['released_at_utc'])for v in self.facts.releases.values()if v['part']==part]
            if releases and utc_stamp(metadata['captured_at_utc'])<=max(releases):continue
            path=self.output/('fixture-capture-'+uuid.uuid4().hex+'.npz');save_capture(path,packet,metadata)
            try:
                result=detect_packet(packet,config,metadata,replay=False,scope='fixture',part=part,fixture_T_cell_target=self.recipe['insertion_targets'][part])
                result.update(_scope='fixture',_part=part,_side=None,fixture_search_prior_source='measured_fixture_target',source_packet=str(path),source_packet_sha256=sha(path),source_metadata_file=str(path.with_suffix('.json')),source_metadata_sha256=sha(path.with_suffix('.json')),_camera_config_path=str(camera_path.resolve()))
                return self._record(result)
            except RuntimeError as error:last_error=error
        raise RuntimeError('Fixture not freshly observable; retain/stop: '+str(last_error))

    def _selected_occupancy(self,observation):
        import numpy as np
        from scipy.spatial import cKDTree
        from .residual_occupancy import build_occupancy
        from .perception_models import model_points
        if observation.get('scope')!='basket' or not observation.get('selected_instance'):return observation
        source=Path(observation['source_packet'])
        if sha(source)!=observation['source_packet_sha256']:raise RuntimeError('Actual occupancy source packet changed')
        config_path=Path(observation['_camera_config_path']);config=self.camera_configs[str(config_path.resolve())]
        with np.load(source,allow_pickle=False)as z:packet={key:z[key]for key in z.files}
        packet['T_cell_camera']=config['T_cell_camera'];depth=np.asarray(packet['depth_m']);K=np.asarray(packet['K']);v,u=np.indices(depth.shape)
        # Occupancy operates only on root rectification's actual pinhole output.
        local=np.asarray(packet['points_camera_image_m']) if 'points_camera_image_m'in packet else np.stack([(u-K[0,2])/K[0,0]*depth,(v-K[1,2])/K[1,1]*depth,depth],-1);T=rigid(config['T_cell_camera']);world=local@T[:3,:3].T+T[:3,3];finite=np.isfinite(world).all(2)&(depth>0)
        selected=observation['selected_instance'];S=rigid(selected['T_cell_part']);surface=model_points(selected['part'])@S[:3,:3].T+S[:3,3]
        roi=finite&np.all(world>=surface.min(0)-.003,axis=2)&np.all(world<=surface.max(0)+.003,axis=2)
        distances=np.full(depth.shape,np.inf);distances[roi]=cKDTree(surface).query(world[roi])[0];tolerance=.002
        mask=finite&(distances<=tolerance)
        for other in observation['instances']:
            if other['instance_id']==selected['instance_id']:continue
            P=rigid(other['T_cell_part']);cloud=model_points(other['part'])@P[:3,:3].T+P[:3,3];competitor=np.full(depth.shape,np.inf);competitor[mask]=cKDTree(cloud).query(world[mask])[0]
            mask&=distances+.0005<competitor
        bounds=np.array([surface.min(0)-.002,surface.max(0)+.002]);settings=config['residual_occupancy']
        contact=dict(capture_id=observation['capture_id'],instance_id=selected['instance_id'],surface_mask=mask,cad_surface_points_cell_m=surface,contact_bounds_cell_m=bounds,shell_tolerance_m=tolerance)
        snapshot=build_occupancy(packet,observation['residual_occupancy_metadata'],config['basket']['outer_bounds_cell_m'],voxel_size_m=settings['voxel_size_m'],depth_margin_m=settings['depth_margin_m'],selected_contact=contact,max_age_s=self.recipe['max_observation_age_s'])
        result=copy.deepcopy(observation);result['residual_occupancy']=snapshot.to_record()
        if result.get('known_hand_surface')is not None:
            from .known_hand_capture import associate
            _,proof=associate(snapshot,packet,result['residual_occupancy_metadata'],result['known_hand_surface']);result['known_hand_surface']['association_proof']=proof
        return result

    def _selected_record(self,observation):
        # Bind planner target selection to immutable bytes, not just memory.
        return self._record(self._selected_occupancy(observation))
    def constrain_initial_held_floor(self,entry,side,observed,carry):
        import numpy as np
        settings=json.loads((self.root/self.recipe['planner_config']).read_text())
        spec=settings['phases'].get('stock_departure',{}).get('bounded_loose_stock_departure')
        support=(spec or {}).get('support_floor')
        if not entry['part'].startswith('ring')or support is None:return observed,carry
        from .floor_constraint import project_inferred_ring_above_floor
        corrected,proof=project_inferred_ring_above_floor(observed['T_cell_part'],entry['part'],support['origin_cell_m'],support['normal_cell'],maximum_correction_m=support['maximum_inference_correction_m'],reported_uncertainty_m=support['reported_fit_depth_bound_m'])
        result=copy.deepcopy(observed);identity=result['selected_instance']['instance_id']
        for item in [result['selected_instance'],*result['instances']]:
            if item['instance_id']==identity:item.update(T_cell_part=corrected.tolist(),center_m=corrected[:3,3].tolist(),floor_fit_constraint=proof)
        G=np.linalg.inv(rigid(result['measured_tcp_pose']))@corrected
        result.update(T_cell_part=corrected.tolist(),T_tcp_part=G.tolist(),floor_fit_constraint=proof);result=self._record(result)
        carry=copy.deepcopy(carry);carry['T_tcp_part']=G.tolist();carry['measurement']['evidence_file']=result['_path']
        self.facts.confirm_held(entry['part'],side,result,G,self.hand_status(side))
        self.event('initial_RGBD_support_floor_inference',part=entry['part'],proof=proof)
        return result,carry

    def initial_stock_lift(self,entry,current):
        import numpy as np
        settings=json.loads((self.root/self.recipe['planner_config']).read_text())
        spec=settings['phases'].get('stock_departure',{}).get('bounded_loose_stock_departure')
        if not entry['part'].startswith('ring')or spec is None:
            targets=[]
            for delta in entry['lift_offsets_cell_m']:current=offset(current,delta);targets.append(current.tolist())
            return 'lift',targets,[]
        from .stock_departure import validate_departure_policy
        phase=dict(settings['phases']['stock_departure'],_phase_name='stock_departure',_contact='depart',_active_part=entry['part'])
        contract=validate_departure_policy(phase,settings['robots'][entry['pickup_side']]);up=contract['up_axis_cell']
        requested=np.asarray(entry['lift_offsets_cell_m'][0],float);height=float(requested@up)
        if height<=0 or np.linalg.norm(requested-height*up)>1e-6:raise ValueError('First stock lift must follow measured calibrated up')
        departure=up*min(height,contract['maximum_lift_m']);remaining=[(requested-departure).tolist(),*entry['lift_offsets_cell_m'][1:]]
        remaining=[v for v in remaining if np.linalg.norm(v)>1e-9]
        return 'stock_departure',[offset(current,departure).tolist()],remaining

    def _inventory_frame(self,observation,side,holding_epoch):
        import numpy as np
        from .native_hand_geometry import actual_hand_points
        source=Path(observation['source_packet'])
        if sha(source)!=observation['source_packet_sha256']:raise RuntimeError('Actual inventory image bytes changed')
        config=self.camera_configs[str(Path(observation['_camera_config_path']).resolve())]
        with np.load(source,allow_pickle=False)as z:packet={k:z[k]for k in z.files}
        packet['T_cell_camera']=config['T_cell_camera']
        # Capture and these actual SDK reads occur while the arm is stopped,
        # under the same cell control lock; exposure timestamps stay unchanged.
        before=self.sdk_state(side);tcp=rigid(self.tcp(side));hand=self.hand_status(side)
        mount=self.recipe['inventory_inspection']['native_hand_mount'][side]
        measured(mount['measurement'],self.root,subject=mount)
        F=rigid(np.asarray(before['F_T_EE_column_major']).reshape(4,4,order='F'))
        robot=actual_hand_points(tcp,F,mount['T_flange_hand'],hand['width_m'])
        metadata=dict(observation['source_metadata'],source_packet_sha256=sha(source),holding_epoch=holding_epoch)
        return dict(packet=packet,metadata=metadata,T_cell_tcp=tcp.tolist(),T_cell_part=observation.get('T_cell_part'),
            known_robot_points_cell_m=robot,robot_geometry_source='actual_sdk_fk',
            robot_geometry_capture_id=observation['capture_id'],self_surface_tolerance_m=.002)

    def verify_stock_inventory(self,entry,side,observed,carry,move):
        import numpy as np
        settings=json.loads((self.root/self.recipe['planner_config']).read_text())
        policy=settings['phases']['pickup'].get('bounded_loose_stock_contact')or settings['phases'].get('stock_departure',{}).get('bounded_loose_stock_departure')
        if not entry['part'].startswith('ring')or policy is None:return observed,carry
        from .selected_inventory_guard import verify_single_inventory
        from .perception_models import model_points
        inspection=self.recipe.get('inventory_inspection')
        if not isinstance(inspection,dict):raise ValueError('Bounded stock contact requires configured actual two-view inventory inspection')
        epoch=uuid.uuid4().hex;frames=[self._inventory_frame(observed,side,epoch)]
        initial=rigid(self.tcp(side));safe_carry=copy.deepcopy(carry)
        envelope=inspection['uncertain_payload_dimensions'];measured(envelope['measurement'],self.root)
        safe_carry['payload_dimensions']=envelope;safe_carry['capsules_part_m']=None
        for step in inspection['relative_tcp_views'][side]:
            measured(step['measurement'],self.root);delta=rigid(step['T_tcp_inspection'])
            if np.linalg.norm(delta[:3,3])>.1:raise ValueError('Inspection translation exceeds bounded local100mm')
            target=initial@delta
            plan=self.plan('inventory_inspection',entry['part'],side,[target.tolist()],observed,[safe_carry])
            move('inventory_inspection',entry['part'],side,[],observed,plan=plan)
            observed,carry=held_measurement(self,entry,side,observed)
            safe_carry.update(T_tcp_part=carry['T_tcp_part'],measurement=carry['measurement'],observation_capture_id=carry['observation_capture_id'])
            frames.append(self._inventory_frame(observed,side,epoch))
        proof=verify_single_inventory(entry['part'],frames,{entry['part']:model_points(entry['part'])},self.hand_status(side),max_age_s=self.recipe['max_observation_age_s'])
        self.latest_inventory_proof=proof
        self.event('postlift_single_inventory_RGBD',part=entry['part'],proof=proof)
        if not proof['verified']:
            self.return_stock_to_bin(entry,side,observed,safe_carry,initial,epoch)
            raise RuntimeError('Inventory not certified; physically returned to bin: '+proof['reason'])
        return observed,carry

    def return_stock_to_bin(self,entry,side,observed,carry,inspection_origin,epoch):
        """Conservative full-envelope return; any missing image/route aborts HOLD."""
        import numpy as np
        from .selected_inventory_guard import verify_empty_hand
        spec=self.recipe['inventory_inspection']['return_over_bin'][side];measured(spec['measurement'],self.root)
        # This measured pose is above the existing bin, not a new fixture.
        target=rigid(spec['T_cell_tcp']);plan=self.plan('inventory_return',entry['part'],side,[target.tolist()],observed,[carry])
        self.execute(plan,observed)
        measured(spec['release_region']['measurement'],self.root)
        from .collision import inside_box
        state=self.tcp(side)
        minimum_extent=float(np.linalg.norm(rigid(carry['T_tcp_part'])[:3,3]))+float(carry['payload_dimensions']['radius_m'])+float(carry['payload_dimensions']['length_m'])/2
        if float(spec['held_extent_radius_m'])<minimum_extent:raise ValueError('Release-region extent must enclose the entire conservative carried inventory')
        if not bool(inside_box(rigid(state)[:3,3],rigid(state)[:3,3],float(spec['held_extent_radius_m']),spec['release_region'])):
            raise RuntimeError('Actual held envelope is not inside measured above-bin release region; retain')
        release=spec['release_action'];FrankaHand.validate_action(release)
        if release['width_m']<.078:raise ValueError('Bin return requires actual open Hand>=78mm')
        self.site.expected_held.pop(side,None)
        self.stock_return_release_part=entry['part']  # Intentional release is now inside measured above-bin ROI.
        opened=self.hand_move(side,entry['part'],'release',release)
        self.facts.confirm_release(entry['part'],side,opened,.078)
        self.stock_return_release_part=entry['part']
        # Keep old object load until a fresh broad, complementary empty-Hand
        # observation proves release. No guessed zero load from command alone.
        views=[]
        for index,step in enumerate([None,*spec['empty_hand_views']]):
            if step is not None:
                measured(step['measurement'],self.root);destination=rigid(state)@rigid(step['T_tcp_inspection'])
                # Until optical empty proof, conservatively enclose possible
                # snagged inventory even with an SDK-open Hand. Bin rays remain.
                current=self.observe('basket',None)
                current=select_observation(current,instances(current,entry['part'])[0])
                route=self.plan('empty_hand_inspection',entry['part'],side,[destination.tolist()],current,[carry])
                self.execute(route,current)
            # No nearest-item fitter: take all current pixels for empty proof.
            stream,config,path=self._camera('held',side);packet,meta=stream.read()
            from .camera import save_capture
            file=self.output/('empty-hand-'+uuid.uuid4().hex+'.npz');save_capture(file,packet,meta)
            raw=dict(source_packet=str(file),source_packet_sha256=sha(file),source_metadata=meta,
                capture_id=meta['capture_id'],_camera_config_path=str(path.resolve()))
            views.append(self._inventory_frame(raw,side,epoch))
        proof=verify_empty_hand(views,self.hand_status(side),max_age_s=self.recipe['max_observation_age_s'])
        self.event('post_bin_return_empty_hand_RGBD',part=entry['part'],proof=proof)
        if not proof['verified']:raise RuntimeError('Bin return release not optically confirmed; load remains conservative: '+proof['reason'])
        self.payload.after_bin_return_release(side,entry['part'],self.flange_frame(side),proof)

    def plan(self,phase,part,side,targets,observation,held_parts,*,contact=None):
        self.ready()
        self._profile_check()
        observation=self._selected_record(observation)
        options={}
        if getattr(self,'width_aware',False):
            from .hand_aperture import binding
            options['aperture_binding']=binding({s:self.hand_status(s)for s in ['left','right']})
        plan=self.planner.plan(phase,part,side,targets,observation,held_parts=held_parts,contact=contact,**options)
        self._latest_plan_context=plan

        plan['_runtime_observation']=observation;plan['_runtime_side']=side
        return plan
    def execute(self,plan,observation):
        import numpy as np
        self.ready()
        original=plan['_runtime_observation']
        fresh=check_observation(self,self.observe(original['_scope'],original['_part'],side=original['_side'],previous=original))
        old=rigid(original['selected_instance']['T_cell_part'])
        eligible=[]
        for value in instances(fresh,original['selected_part']):
            current=rigid(value['T_cell_part'])
            position=np.linalg.norm(current[:3,3]-old[:3,3])
            angle=np.arccos(np.clip((np.trace(current[:3,:3]@old[:3,:3].T)-1)/2,-1,1))
            if position<=self.recipe['target_position_tolerance_m'] and angle<=self.recipe['target_angle_tolerance_rad']:
                eligible.append(value)
        if len(eligible)!=1:raise PreMotionPickupChanged('Fresh image no longer uniquely agrees with the planned target; no arm motion issued')
        fresh=self._selected_record(select_observation(fresh,eligible[0]))
        with self.site.control_lock():
            self._profile_check()
            self.ready()
            try:self.planner.revalidate(plan,observation=fresh)
            except CollisionError as error:
                raise PreMotionPickupChanged('Fresh scene blocks pre-motion pickup: '+str(error))from error
            row=copy.deepcopy(plan['stage_row'])
            if row.get('planning_source')!='dynamic_field_envelope' or row.get('field_path_reviewed') is True:
                raise ValueError('Dynamic stage requires explicit scene-envelope proof, not a fabricated taught-path approval')
            if sha(plan['csv'])!=plan['csv_sha256'] or row.get('sha256')!=plan['csv_sha256']:
                raise ValueError('Exact screened dynamic CSV hash required')
            self.site.config.setdefault('stages',{})['dynamic.'+uuid.uuid4().hex]=row
            aperture=plan.get('proof',{}).get('aperture_binding')
            if getattr(self,'width_aware',False)and aperture is None:raise RuntimeError('Width-aware arm motion lacks actual Hand geometry binding')
            self._active_aperture_binding=aperture
            try:self.site.motion(plan['_runtime_side'],'csv',[Path(plan['csv'])]);self.ready()
            finally:self._active_aperture_binding=None
    def verify_clearance(self,recipe):
        import numpy as np
        positions={side:rigid(self.tcp(side))[:3,3] for side in ('left','right')}
        for side,point in positions.items():
            for entry in recipe['parts']:
                target=rigid(recipe['insertion_targets'][entry['part']])[:3,3]
                if np.linalg.norm(point-target)<entry['min_tcp_clearance_m']:
                    raise RuntimeError('Current '+side+' TCP is too close to fixture '+entry['part'])
        # Camera-only fixture evidence is reacquired after hand withdrawal.
        # Cached seating/clamp facts cannot stand in for final persistence.
        for installed in self.installed_parts:
            self.wait(installed['part']+'_retained',recipe['feedback_timeout_s'],recipe['feedback_dwell_s'])
        self.ready()
    def mark_installed(self,part,pose,dimensions,observation,*,capsules_part_m=None):
        self.installed_parts.append(dict(part=part,T_cell_part=pose,payload_dimensions=dimensions,capsules_part_m=copy.deepcopy(capsules_part_m),
            measurement=dict(verified=True,by='live_rgbd_and_fixture_feedback',at_utc=observation['captured_at_utc'],evidence_file=observation['_path']),
            verified_by=['seated','clamped','retained']))
    def event(self,name,**values):self.site.event(name,**values)
    def stop(self):return self.site.stop()
    def close(self):
        self.site.require_ready=self._base_ready
        for side,monitor in getattr(self,'hand_monitors',{}).items():
            self.site.grippers[side].status=self._raw_hand_status[side];monitor.close()
        for stream in self.streams.values():stream.close()


class ReplayRuntime(LiveRuntime):
    """Actual packet recognition plus explicit fake robot/Hands/PLC; no SDK calls.

    A manifest supplies captured RGB-D frames in phase order and initial fake
    TCPs. Planning here is a policy contract only, never collision certification.
    """
    replay=True
    def __init__(self,recipe,manifest_path,output_dir):
        from types import SimpleNamespace
        self.payload=None
        self.recipe=recipe;self.root=Path(recipe['_root']);self.output=Path(output_dir).resolve();self.output.mkdir(parents=True,exist_ok=True)
        self.manifest_path=Path(manifest_path).resolve();m=json.loads(self.manifest_path.read_text())
        if m.get('schema_version')!=1 or m.get('replay') is not True or not m.get('frames'):raise ValueError('Explicit replay RGB-D manifest required')
        self.frames=iter(m['frames']);self.tcps={s:rigid(m['initial_tcp'][s]).tolist() for s in ('left','right')}
        self.hands={s:dict(part=None,width_m=bounded(m['initial_hand_width_m'][s],0,.08,'fake initial width')) for s in ('left','right')}
        self.site=SimpleNamespace(config=json.loads((self.root/recipe['site_config']).read_text()),expected_held={})
        self.width_aware=any(v.get('native_hand_aperture_geometry')for v in planner_settings['robots'].values())
        self.known_hand_surface=planner_settings['scene'].get('known_hand_surface')
        if self.known_hand_surface is not None:
            from .known_hand_capture import settings
            settings(self.known_hand_surface)
        self._active_aperture_binding=None;self._latest_plan_context=None;self._stopped_arm_monitors={}
        self.monitor=lambda:None;self.installed_parts=[];self.camera_configs={};self.stopped=False;self.events=[]
        self.signals={part+'_'+suffix:False for part in PARTS for suffix in ('grasped','seated','clamped','retained')}
        for e in recipe['parts']:
            if e['pickup_side']!=e['insert_side']:
                for key in ('donor_grasped_signal','receiver_grasped_signal'):self.signals[e['handover'][key]]=False
    def ready(self):
        if self.stopped:raise RuntimeError('Replay controller stopped')
        self.monitor()
    def read(self,name):return self.signals[name]
    def wait(self,name,timeout,dwell):
        self.ready()
        if not self.read(name):raise RuntimeError('Replay feedback transition missing: '+name)
    def clamp(self,part):self.ready();self.signals[part+'_clamped']=True
    def hand_status(self,side):return dict(is_grasped=self.hands[side]['part'] is not None,width_m=self.hands[side]['width_m'])
    def hand_move(self,side,part,mode,parameters):
        self.ready();hand=self.hands[side];entry=next(e for e in self.recipe['parts'] if e['part']==part)
        if mode=='grasp':hand['part']=part
        elif hand['part']==part:hand['part']=None
        hand['width_m']=parameters['width_m']
        self.signals[part+'_grasped']=any(h['part']==part for h in self.hands.values())
        if entry['pickup_side']!=entry['insert_side']:
            key='donor_grasped_signal' if side==entry['pickup_side'] else 'receiver_grasped_signal'
            self.signals[entry['handover'][key]]=hand['part']==part
        if mode=='release' and self.signals[part+'_clamped']:self.signals[part+'_retained']=True
        self.event('fake_hand',side=side,part=part,mode=mode);self.ready();return self.hand_status(side)
    def tcp(self,side):return self.tcps[side]
    def observe(self,scope,part,*,side=None,previous=None):
        import numpy as np
        from .random_perception import load_random_config,detect_packet
        self.ready()
        try:item=next(self.frames)
        except StopIteration as error:raise RuntimeError('Replay frames exhausted; no invented camera observations') from error
        if isinstance(item,str):item=dict(snapshot=item)
        if item.get('scope',scope)!=scope or item.get('part',part)!=part:raise ValueError('Replay frame phase mismatch')
        snapshot=self.manifest_path.parent/item['snapshot']
        metadata=json.loads(snapshot.with_suffix('.json').read_text())
        role='held_'+side if scope=='held' else scope
        camera_path=self.root/(self.recipe.get('camera_configs',{}).get(role) or self.recipe['camera_config'])
        config=load_random_config(camera_path)
        with np.load(snapshot,allow_pickle=False) as z:packet={name:z[name].copy() for name in z.files}
        result=detect_packet(packet,config,metadata,replay=True,scope=scope,part=part,
            held_tcp_pose=self.tcp(side) if scope=='held' else None,held_roi_radius_m=self.recipe['held_roi_radius_m'],
            previous_observation=previous,held_robot=side if scope=='held' else None,
            held_grasp_transform=(previous or {}).get('T_tcp_part') if scope=='held' else None)
        result.update(_scope=scope,_part=part,_side=side,source_packet=str(snapshot),source_packet_sha256=sha(snapshot),source_metadata_file=str(snapshot.with_suffix('.json')),
            source_metadata_sha256=sha(snapshot.with_suffix('.json')),_camera_config_path=str(camera_path.resolve()))
        return self._record(result)
    def plan(self,phase,part,side,targets,observation,held_parts,*,contact=None):
        for t in targets:rigid(t)
        return dict(replay_contract_only=True,phase=phase,part=part,side=side,targets=targets,
                    collision_certified=False,hardware_command_allowed=False)
    def execute(self,plan,observation):
        self.ready()
        if plan.get('replay_contract_only') is not True or plan.get('hardware_command_allowed') is not False:
            raise ValueError('Replay must not accept a real actuator plan')
        self.tcps[plan['side']]=plan['targets'][-1]
        if plan['phase']=='insert':self.signals[plan['part']+'_seated']=True
        self.event('fake_motion',phase=plan['phase'],part=plan['part'],side=plan['side']);self.ready()
    def event(self,name,**values):self.events.append(dict(event=name,**values))
    def stop(self):self.stopped=True;self.event('fake_stop')
    def close(self):pass


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--recipe',type=Path,required=True);p.add_argument('--output-dir',type=Path,default=Path('random-cycle-logs'))
    p.add_argument('--replay-manifest',type=Path)
    p.add_argument('--single-pick',choices=['ring1','ring2'],help='Stop stationary holding one inspected ring; no transfer/insertion/release')
    modes=p.add_mutually_exclusive_group(required=True)
    modes.add_argument('--validate',action='store_true');modes.add_argument('--execute',action='store_true');modes.add_argument('--dry-run',action='store_true')
    a=p.parse_args(argv)
    if a.dry_run and not a.replay_manifest:p.error('--dry-run requires --replay-manifest with actual recorded RGB-D packets')
    raw=json.loads(a.recipe.read_text());root=a.recipe.resolve().parent
    site_config=json.loads((root/raw['site_config']).read_text())
    recipe=validate_recipe(a.recipe,site_config)
    if a.execute or (a.validate and recipe.get('replay_only')is not True):validate_live_inputs(recipe,site_config)
    if a.execute and recipe.get('replay_only') is True:raise ValueError('Synthetic replay recipe cannot enable real hardware')
    if a.validate:
        from .random_perception import load_random_config
        configs=set(recipe.get('camera_configs',{}).values())|{recipe['camera_config']}
        for path in configs:
            if path:load_random_config(root/path)
        from .random_planner import load,validate_config
        planner_path=root/recipe['planner_config'];dependencies={}
        validate_config(load(planner_path),planner_path.parent,dependencies,datetime.now(timezone.utc))
        print(json.dumps(dict(state='random_cycle_configuration_checked',hardware_ready=False,robot_connections=0)));return 0
    if a.dry_run:
        runtime=ReplayRuntime(recipe,a.replay_manifest,a.output_dir)
        try:result=run(runtime,recipe,single_pick=a.single_pick)
        finally:runtime.close()
    else:
        site=Site(root/recipe['site_config'],a.output_dir/'hardware')
        runtime=LiveRuntime(site,recipe,a.output_dir)
        try:
            with site.control_lock(),StopListener(site):result=run(runtime,recipe,single_pick=a.single_pick)
        finally:runtime.close()
    print(json.dumps(result,indent=2));return 0


if __name__=='__main__':raise SystemExit(main())
