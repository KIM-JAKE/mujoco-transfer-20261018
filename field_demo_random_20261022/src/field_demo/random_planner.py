"""Live SDK phase planning with explicitly commissioned conservative collision envelopes.

No motion command is issued here. Empty geometry, unmeasured envelopes, stale
images, unbounded sample gaps, changed dependencies and collisions all fail.
"""
import copy
import csv
from datetime import datetime, timezone
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import subprocess
import tempfile
import uuid
import numpy as np

from . import collision
from .calibration import utc_stamp


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def canonical(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def load(path):
    result = json.loads(Path(path).read_text())
    if not isinstance(result, dict):
        raise ValueError('JSON object required: ' + str(path))
    return result


def positive(value, name, upper=None):
    if type(value) not in (int, float) or not np.isfinite(value) or value <= 0 or (upper is not None and value > upper):
        raise ValueError('Explicit positive bounded field value required: ' + name)
    return float(value)


def measured(record, root, dependencies, name, now, subject=None):
    if isinstance(record,dict)and record.get('kind')=='model_derived':
        from .model_records import validate
        return validate(record,root,subject=subject,dependencies=dependencies,name=name,now=now)
    if (not isinstance(record, dict) or record.get('verified') is not True or
            not isinstance(record.get('by'), str) or not record['by'].strip() or
            not isinstance(record.get('at_utc'), str) or utc_stamp(record['at_utc']) > now):
        raise ValueError('Verified actual-cell measurement required: ' + name)
    evidence = record.get('evidence_file')
    if not isinstance(evidence, str) or not evidence:
        raise ValueError('Measurement evidence file required: ' + name)
    path = (root / evidence).resolve()
    if not path.is_file() or not path.read_bytes().strip():
        raise ValueError('Nonempty measured evidence required: ' + name)
    dependencies[name] = {'path': str(path), 'sha256': digest(path)}


def validate_config(config, root, dependencies, now):
    if config.get('schema_version') != 2 or config.get('units') != 'm':
        raise ValueError('Random motion schema 2 with explicit metre units required')
    measured(config.get('measurement'), root, dependencies, 'dynamic_workspace_review', now)
    tracking = float(config.get('joint_tracking_error_limit_rad', .02))
    if not np.isfinite(tracking) or not .0001 <= tracking <= .02:
        raise ValueError('Actual native tracking guard must be within [0.0001,0.02]rad')
    if tracking < .02:
        measured(config.get('tracking_guard_measurement'), root, dependencies, 'tight_tracking_stop_test', now)
    scene = config['scene']
    if scene.get('known_hand_surface')is not None:
        from .known_hand_capture import settings
        settings(scene['known_hand_surface'])
    measured(scene.get('measurement'), root, dependencies, 'scene_measurement', now)
    if scene.get('complete_within_workspace') is not True or scene.get('unobserved_geometry_enclosed') is not True:
        raise ValueError('Complete conservative scene geometry, including unobserved geometry, must be field-verified')
    collision.box(scene['workspace'])
    if not scene.get('obstacles'):
        raise ValueError('Unknown or empty scene cannot enable planning')
    for row in scene['obstacles']:
        collision.box(row)
    for side in ('left', 'right'):
        envelope = config['robots'][side]
        measured(envelope.get('measurement'), root, dependencies, side + '_robot_envelope', now, subject=envelope)
        if envelope.get('whole_robot_hand_tool_cables_enclosed') is not True:
            raise ValueError('Actual whole robot, Hand, tool and cables need enclosing geometry: ' + side)
        radii = collision.finite(envelope['link_radii_m'], (8,))
        if np.any(radii <= 0):
            raise ValueError('Eight positive actual enclosing link radii required')
        if not envelope.get('tool_capsules_tcp_m') and envelope.get('tool_in_link_envelopes') is not True:
            raise ValueError('Actual Hand/tool enclosure is missing')
    for part,row in config.get('loose_part_envelopes',{}).items():
        if row.get('measurement',{}).get('kind')=='model_derived':measured(row['measurement'],root,dependencies,'model_part_'+part,now,subject=row)
    for field, limit in [('collision_margin_m', .1), ('max_observation_age_s', 2),
                          ('max_state_read_s', 30), ('other_arm_q_tolerance_rad', .005),
                          ('target_position_tolerance_m', .001), ('target_angle_tolerance_rad', .01)]:
        positive(config.get(field), field, limit)


def native_output(value):
    if (value.get('read_only') is not True or value.get('motion_executed') is not False or
            value.get('server_protocol_version') != 10):
        raise ValueError('Read-only actual SDK protocol-10 output required')
    state = value['state']
    for key in ('q', 'q_d', 'dq', 'dq_d'):
        collision.finite(state[key], (7,))
    if np.max(np.abs(np.asarray(state['q']) - state['q_d'])) > .005 or np.max(np.abs(state['dq'])) > .01 or np.max(np.abs(state['dq_d'])) > .01:
        raise ValueError('Stationary actual robot matching its desired state required')
    if len(state['F_T_EE_column_major']) != 16:
        raise ValueError('Measured actual TCP configuration required')
    return state


def validate_model(model):
    """Independently bind FK samples to the actual stationary-quintic CSV curve."""
    state = native_output(model)
    if model.get('continuous_10percent_checked') is not True:
        raise ValueError('Native continuous ten-percent trajectory screen required')
    knots = model['knots']
    if not 2 <= len(knots) <= 33:
        raise ValueError('Bounded stationary phase knots required')
    times = collision.finite([k['time_s'] for k in knots])
    q = collision.finite([k['q'] for k in knots], (len(knots), 7))
    if times[0] != 0 or np.any(np.diff(times) <= 0) or times[-1] > 60 or abs(times[-1]-float(model['duration_s'])) > 1e-9:
        raise ValueError('Monotone bounded native phase duration required')
    for knot in knots:
        if np.max(np.abs(collision.finite(knot['dq'], (7,)))) > 1e-12 or np.max(np.abs(collision.finite(knot['ddq'], (7,)))) > 1e-12:
            raise ValueError('This backend requires zero-velocity/acceleration stationary C2 knots')
    if not np.allclose(q[0], state['q_d'], atol=1e-9) or not np.allclose(q[0], model['q_start'], atol=1e-9):
        raise ValueError('Native path does not start at measured desired robot state')
    samples = model['samples']
    if not 2 <= len(samples) <= 60002:
        raise ValueError('Actual per-millisecond SDK FK samples required')
    ts = collision.finite([s['time_s'] for s in samples])
    gaps = np.diff(ts)
    if ts[0] != 0 or abs(ts[-1]-times[-1]) > 1e-9 or np.any(gaps <= 0) or np.max(gaps) > .001000001:
        raise ValueError('SDK samples must cover the entire C2 curve at no more than one millisecond')
    index = np.clip(np.searchsorted(times, ts, side='right')-1, 0, len(times)-2)
    duration = times[index+1]-times[index]
    u = (ts-times[index])/duration
    blend = 10*u**3-15*u**4+6*u**5
    expected_q = q[index]+blend[:, None]*(q[index+1]-q[index])
    expected_dq = ((30*u**2-60*u**3+30*u**4)/duration)[:, None]*(q[index+1]-q[index])
    actual_q = collision.finite([s['q'] for s in samples], expected_q.shape)
    actual_dq = collision.finite([s['dq'] for s in samples], expected_dq.shape)
    if np.max(np.abs(actual_q-expected_q)) > 1e-8 or np.max(np.abs(actual_dq-expected_dq)) > 1e-8:
        raise ValueError('SDK FK samples do not belong to the exact exported C2 knot curve')
    velocities = np.max(1.875*np.abs(np.diff(q, axis=0))/np.diff(times)[:, None], axis=0)
    if not np.allclose(velocities, collision.finite(model['joint_velocity_bound_rad_s'], (7,)), atol=1e-9, rtol=1e-8):
        raise ValueError('Native joint velocity bound does not match the actual C2 curve')
    links = collision.finite([s['link_points_base_m'] for s in samples], (len(samples), 9, 3))
    lengths = collision.finite(model['link_segment_lengths_m'], (8,))
    if np.any(lengths < 0) or np.max(lengths) > 2 or np.sum(lengths) <= 0:
        raise ValueError('Measured SDK rigid-chain length bounds required')
    actual_lengths = np.linalg.norm(np.diff(links, axis=1), axis=-1)
    if np.any(actual_lengths > lengths+1e-7) or np.any(np.max(actual_lengths, axis=0)-np.min(actual_lengths, axis=0) > 2e-6):
        raise ValueError('SDK joint-frame chain is not rigid within its measured length bounds')
    speeds = np.zeros(9)
    for point in range(2, 9):
        for joint in range(7):
            if joint+1 < point:
                speeds[point] += np.sum(lengths[joint+1:point])*velocities[joint]
    if not np.allclose(speeds, collision.finite(model['link_point_speed_bounds_m_s'], (9,)), atol=1e-9, rtol=1e-8):
        raise ValueError('Native point speed bounds do not match rigid serial-chain geometry')
    if np.any(np.linalg.norm(np.diff(links, axis=0), axis=-1) > gaps[:, None]*speeds[None, :]+2e-7):
        raise ValueError('SDK link samples exceed independently derived continuous motion bounds')
    tcps = collision.finite([s['tcp_base'] for s in samples], (len(samples), 4, 4))
    rotations = tcps[:, :3, :3]
    if (not np.allclose(tcps[:, 3], [0, 0, 0, 1], atol=1e-9) or
            not np.allclose(np.einsum('nji,njk->nik', rotations, rotations), np.eye(3), atol=1e-7) or
            np.max(np.abs(np.linalg.det(rotations)-1)) > 1e-7):
        raise ValueError('All sampled SDK TCP frames must be proper rigid transforms')
    if not np.allclose(tcps[:, :3, 3], links[:, -1], atol=1e-8):
        raise ValueError('TCP frames and actual SDK link endpoints differ')
    return {'links': links, 'tcps': tcps, 'speeds': speeds, 'joint_velocities': velocities,
            'gap': float(np.max(gaps)), 'sample_count': len(samples), 'duration_s': float(times[-1])}


def run_native(binary, mode, address, targets=None, *, ik_seed=None):
    args = [str(binary), mode, str(ipaddress.ip_address(address))]
    with tempfile.TemporaryDirectory(prefix='random-field-plan-') as directory:
        if targets is not None:
            path = Path(directory) / 'targets.txt'
            path.write_text(''.join(' '.join(format(v, '.17g') for v in matrix.ravel())+'\n' for matrix in targets))
            args.append(str(path))
        if ik_seed is not None:
            seed=collision.finite(ik_seed,(7,))
            if mode!='--read-model' or targets is None:raise ValueError('IK seed is read-model candidate input only')
            args += ['--ik-seed',*[format(v,'.17g')for v in seed]]
        try:
            result = subprocess.run(args, capture_output=True, text=True, check=True, timeout=90 if targets is not None else 10)
        except subprocess.CalledProcessError as error:
            # These two pinned read-only backend failures concern this target,
            # not SDK communication/state. All other native failures propagate.
            target_failures=['No measured-model bounded IK for this phase','Phase exceeds 60s continuous 10-percent envelope']
            if mode=='--read-model'and any((error.stderr or '').strip()=='Read-only dynamic model failed: '+reason for reason in target_failures):
                raise RuntimeError('Read-only candidate unavailable: '+(error.stderr or '').strip())from error
            raise

    return json.loads(result.stdout)


def dependencies_valid(dependencies):
    for name, row in dependencies.items():
        if digest(row['path']) != row['sha256']:
            raise ValueError('Planning dependency changed: ' + name)


def observation_valid(observation, config, root, dependencies, now, *, require_fresh=True,static_scene_lease=None):
    if observation.get('replay') is not False or observation.get('units') != 'm' or not observation.get('capture_id'):
        raise ValueError('A real current metric RGB-D capture is required; replay cannot authorize live motion')
    if observation.get('static_scene_lease')is not None:
        from .static_scene_lease import validate
        validate(observation['static_scene_lease'],now=now)
    age = (now-utc_stamp(observation['captured_at_utc'])).total_seconds()
    if age < 0 or (require_fresh and age > config['max_observation_age_s']):
        raise ValueError('Stale/future camera observation; capture again before moving')
    name = observation.get('_path')
    if not isinstance(name, str) or not name or digest(name) != observation.get('_sha256'):
        raise ValueError('An immutable actual observation file and its exact hash are required')
    raw = load(name)
    for key in ('capture_id', 'captured_at_utc', 'calibration_sha256', 'calibration_id', 'units', 'replay'):
        if raw.get(key) != observation.get(key):
            raise ValueError('Immutable sensor packet metadata differs: '+key)
    if raw.get('residual_occupancy')!=observation.get('residual_occupancy'):raise ValueError('Immutable residual occupancy record differs from saved observation')
    if raw.get('known_hand_surface')!=observation.get('known_hand_surface'):raise ValueError('Immutable actual SDK Hand surface record differs from saved observation')
    if raw.get('static_scene_lease')!=observation.get('static_scene_lease'):raise ValueError('Immutable scene lease differs from actual observation')
    selected = observation.get('selected_instance')
    if selected is not None:
        if not any(value.get('instance_id') == selected.get('instance_id') and value.get('part') == selected.get('part') and
                   value.get('T_cell_part') == selected.get('T_cell_part') for value in raw.get('instances', [])):
            raise ValueError('Selected pose must be an actual instance in the immutable camera packet')
        if observation.get('T_cell_part', selected['T_cell_part']) != selected['T_cell_part']:
            raise ValueError('Selected pose and explicit target pose differ')
    dependencies['observation'] = {'path': str(Path(name).resolve()), 'sha256': observation['_sha256']}
    # The explicitly selected stabilization strategy and the exact field
    # engine travel with every immutable camera report into the stage proof.
    profile = observation.get('stabilization_profile')
    profile_dependencies = observation.get('stabilization_dependency_files', {})
    if raw.get('stabilization_profile') != profile or raw.get('stabilization_dependency_files', {}) != profile_dependencies:
        raise ValueError('Immutable observation stabilization binding differs')
    if profile is not None or profile_dependencies:
        if not isinstance(profile, dict) or not profile or not isinstance(profile_dependencies, dict) or not profile_dependencies:
            raise ValueError('Selected stabilization profile needs its complete exact dependency files')
        for label, binding in profile_dependencies.items():
            if not isinstance(label, str) or not label or not isinstance(binding, dict):
                raise ValueError('Named exact stabilization dependency required')
            dependency = binding.get('path')
            if not isinstance(dependency, str) or not dependency:
                raise ValueError('Stabilization dependency path required: '+label)
            dependency = (Path(name).resolve().parent/dependency).resolve()
            if digest(dependency) != binding.get('sha256'):
                raise ValueError('Stabilization dependency changed: '+label)
            dependencies['stabilization_'+label] = {'path': str(dependency), 'sha256': binding['sha256']}
    for label, path_key, sha_key in [('raw_rgbd_packet', 'source_packet', 'source_packet_sha256'),
                                     ('raw_capture_metadata', 'source_metadata_file', 'source_metadata_sha256')]:
        source = observation.get(path_key)
        if not isinstance(source, str) or not source or digest(source) != observation.get(sha_key):
            raise ValueError('Actual RGB-D evidence file and exact hash required: '+label)
        dependencies[label] = {'path': str(Path(source).resolve()), 'sha256': observation[sha_key]}
    source_metadata = load(observation['source_metadata_file'])
    if source_metadata != observation.get('source_metadata'):
        raise ValueError('Observation source metadata differs from the immutable actual capture record')
    allowed = config.get('camera_configs') or {'default': config.get('camera_config')}
    matching = []
    for role, camera in allowed.items():
        if not isinstance(camera, str) or not camera:
            continue
        path = (root/camera).resolve()
        if digest(path) == observation.get('calibration_sha256'):
            calibrated = load(path)
            if calibrated.get('calibration_id') != observation.get('calibration_id'):
                raise ValueError('Observed calibration identity differs from its exact commissioned file')
            measured(calibrated.get('measured'), path.parent, dependencies, 'camera_measurement', now)
            dependencies['camera_config'] = {'path': str(path), 'sha256': digest(path)}
            from .observations import validate_observation
            calibrated['_calibration_sha256'] = digest(path)
            validate_observation(observation['source_metadata'], calibrated, replay=False, now=now,
                                 max_age_s=config['max_observation_age_s'],static_scene_lease=static_scene_lease)
            if observation.get('output_frame') != calibrated.get('cell_frame'):
                raise ValueError('Observation and commissioned cell frame differ')
            matching.append(role)
    if not matching:
        raise ValueError('Observation calibration is not one of the commissioned role camera configurations')
    return matching


def scene_packets(observation, config, root, dependencies, now):
    lease=observation.get('static_scene_lease')
    if lease is not None:
        from .static_scene_lease import validate,target_agrees,options
        cfg=options(config['scene']['static_scene_lease']);validate(lease,now=now,maximum_age_s=cfg['maximum_scene_age_s']);target_agrees(lease,observation,config)
        attached=list(observation.get('scene_observations',[]))
        if observation.get('scope')!='contact_target'or len(attached)!=1 or attached[0].get('_path')!=lease['source_observation_file']or attached[0].get('_sha256')!=lease['source_observation_sha256']:raise ValueError('Contact target requires its exact original untouched map')
        packets=[dict(attached[0],_static_scene_lease=lease)]
        bound={};observation_valid(packets[0],config,root,bound,now,require_fresh=False,static_scene_lease=lease);dependencies.update({'leased_scene_'+k:v for k,v in bound.items()})
    else:
        packets = [observation] + list(observation.get('scene_observations', []))
        for index, packet in enumerate(packets[1:]):
            bound = {}
            observation_valid(packet, config, root, bound, now)
            dependencies.update({'scene_packet_'+str(index)+'_'+key: value for key, value in bound.items()})
    scene = config['scene']
    for packet in packets:
        if packet.get('scope')=='basket' and scene.get('require_residual_occupancy') is True:
            from .residual_occupancy import OccupancySnapshot
            record=packet.get('residual_occupancy')
            if not record:raise ValueError('Fresh conservative residual occupancy required; partial ring detection is not empty space')
            snapshot=OccupancySnapshot.from_record(record,expected_capture_id=packet['capture_id'],expected_calibration_sha256=packet['calibration_sha256'],max_age_s=config['max_observation_age_s'],now_utc=now,static_scene_lease=packet.get('_static_scene_lease'))
            if not any(src['source_packet_sha256']==packet.get('source_packet_sha256')for src in snapshot.sources):raise ValueError('Residual occupancy is not bound to actual saved camera bytes')
        if packet.get('scope') == 'basket' and not packet.get('instances') and scene.get('require_residual_occupancy') is not True:
            if packet.get('basket_scene_status') != 'unknown_treated_fully_occupied' or not packet.get('conservative_scene_obstacles'):
                raise ValueError('No detected basket objects cannot be treated as free space without its measured occupied enclosure')
    if scene.get('require_basket_observation') is True:
        if not any(packet.get('scope') == 'basket' for packet in packets):
            raise ValueError('Held-only perception cannot establish empty clutter; capture and fuse a fresh complete basket observation')
    else:
        names = scene.get('unseen_loose_objects_enclosure_names')
        known = {row['name'] for row in scene['obstacles']}
        if not isinstance(names, list) or not names or not set(names) <= known:
            raise ValueError('Unseen loose objects need explicit measured static enclosure boxes or fresh basket fusion')
    return packets


def payload_capsules(entry):
    capsules = entry.get('capsules_part_m')
    if capsules:
        return capsules
    dimensions = entry.get('payload_dimensions', {})
    radius = positive(dimensions.get('radius_m', dimensions.get('outer_radius_m')), 'actual payload radius', .3)
    length = positive(dimensions.get('length_m', dimensions.get('thickness_m')), 'actual payload length', .6)
    return [{'a': [0, 0, -length/2], 'b': [0, 0, length/2], 'radius_m': radius}]


def geometry_check(model, other_state, config, site_config, phase_name, side, part, observation, held, root, dependencies, now, contact=None, *, aperture_binding=None, stopped_aperture=False):
    if stopped_aperture:
        native_output(model)
        points=collision.finite(model['link_points_base_m'],(9,3))
        trace=dict(links=points[None],tcps=collision.transform(model['tcp_base'])[None],speeds=np.zeros(9),joint_velocities=np.zeros(7),gap=0.,sample_count=1)
        model=dict(model,link_segment_lengths_m=(np.linalg.norm(np.diff(points,axis=0),axis=1)+1e-6).tolist())
    else:trace=validate_model(model)
    guard = float(config.get('joint_tracking_error_limit_rad', .02))
    if np.max(np.abs(np.asarray(model['state']['q'])-model['state']['q_d'])) > min(.005, guard):
        raise ValueError('Actual start tracking error already exceeds the requested native guard')
    native_output(other_state)
    phase = copy.deepcopy(config['phases'][phase_name])
    measured(phase.get('measurement'), root, dependencies, 'phase_corridor_review', now)
    force = phase.get('force_monitor', {})
    measured(force.get('measurement'), root, dependencies, 'actual_force_stop_test', now)
    if force.get('enabled') is not True or force.get('force_limit_n') != 5 or force.get('moment_limit_nm') != 1:
        raise ValueError('Actual tested native 5N/1Nm stop monitoring must remain enabled')
    if phase.get('allow_payload_contact_fixture'):
        if contact != 'insert' or phase_name != 'insert' or phase.get('payload_contact_approved') is not True:
            raise ValueError('Fixture contact may only exempt active held payload during explicitly approved insertion')
        measured(phase.get('contact_measurement'), root, dependencies, 'payload_fixture_contact_review', now)
    phase['_phase_name']=phase_name
    phase['_contact'] = contact
    if contact=='strict_empty_approach':
        if phase_name!='pickup'or held:raise ValueError('Strict empty approach requires pickup with no held stock')
        phase.pop('bounded_loose_stock_contact',None);phase['target_contact_tool_capsules']=[]
    if observation.get('static_scene_lease')is not None:
        if not stopped_aperture or phase_name!='pickup'or contact!='grasp':raise ValueError('Static scene lease authorizes only stopped pickup closure, never arm transport')
    phase['_active_part'] = part
    phase['_target_instance_id'] = observation.get('selected_instance', {}).get('instance_id')
    if observation.get('static_scene_lease')is not None:
        from .static_scene_lease import target_agrees
        phase['_target_instance_id']=target_agrees(observation['static_scene_lease'],observation,config)
    phase['_tcp_sweep_m'] = trace['speeds'][-1]*trace['gap']/2
    from .common_duo_mount import binding as common_duo_binding,KEY as common_duo_key
    common_mount=common_duo_binding(config,site_config,root,dependencies,now)
    from .convex_self import validate_optin as validate_convex_self,KEY as convex_self_key
    convex_self_enabled=validate_convex_self(config,root,dependencies,now)
    base = collision.transform(site_config['robots'][side]['T_cell_base'])
    other_side = 'right' if side == 'left' else 'left'
    other_base = collision.transform(site_config['robots'][other_side]['T_cell_base'])
    links = np.einsum('ij,nkj->nki', base[:3, :3], trace['links'])+base[:3, 3]
    tcps = base[None, :, :]@trace['tcps']
    other_links = collision.finite(other_state['link_points_base_m'], (9, 3))@other_base[:3, :3].T+other_base[:3, 3]
    other_tcp = other_base@collision.transform(other_state['tcp_base'])
    envelope = copy.deepcopy(config['robots'][side]); envelope['_angular_speed_bound_rad_s'] = float(np.sum(trace['joint_velocities']))
    envelope.pop(common_duo_key,None)
    envelope.pop(convex_self_key,None)
    if common_mount is not None:envelope[common_duo_key]=common_mount
    if any(config['robots'][s].get('native_hand_aperture_geometry')for s in ['left','right'])and aperture_binding is None:raise ValueError('Actual both-Hand SDK aperture binding required for width-aware native geometry')
    if phase.get('bounded_loose_stock_contact')is not None:
        from .stock_contact import validate_stock_contact_policy
        contract=validate_stock_contact_policy(phase,envelope);policy=phase['bounded_loose_stock_contact'];monitor=policy['monitoring']
        records=[('inventory',policy['inventory_contract']['measurement']),('bin_interior',policy['bin_interior']['measurement']),('monitoring',monitor['measurement'])]+[('finger_'+str(i),f['measurement'])for i,f in enumerate(policy['finger_capsules'])]
        for label,record in records:measured(record,root,dependencies,'stock_contact_'+label,now)
        for i,entry in enumerate(policy['finger_capsules']):
            if entry.get('measurement',{}).get('kind')=='model_derived':measured(entry['measurement'],root,dependencies,'stock_contact_finger_model_'+str(i),now,subject=entry)
        binary=(root/site_config['motion_binary']).resolve();stop_evidence=(root/force['measurement']['evidence_file']).resolve()
        if digest(binary)!=contract['native_driver_sha256']or digest(stop_evidence)!=contract['stop_test_sha256']or monitor['heartbeat_timeout_s']!=.25:
            raise ValueError('Loose-stock contact must bind the actual native5N/1Nm executor and its250ms stop/heartbeat test')
        dependencies['stock_contact_native_executor']={'path':str(binary),'sha256':digest(binary)}
        dependencies['stock_contact_stop_test']={'path':str(stop_evidence),'sha256':digest(stop_evidence)}
    if phase.get('bounded_loose_stock_departure')is not None:
        from .stock_departure import validate_departure_policy
        contract=validate_departure_policy(phase,envelope);policy=phase['bounded_loose_stock_departure']['contact_policy'];monitor=policy['monitoring']
        records=[('departure_up',contract['up_axis_measurement']),('inventory',policy['inventory_contract']['measurement']),('bin_interior',policy['bin_interior']['measurement']),('monitoring',monitor['measurement'])]+[('finger_'+str(i),f['measurement'])for i,f in enumerate(policy['finger_capsules'])]
        if contract['support_floor']is not None:records.append(('support_floor',contract['support_floor']['measurement']))
        for label,record in records:measured(record,root,dependencies,'stock_departure_'+label,now)
        for i,entry in enumerate(policy['finger_capsules']):
            if entry.get('measurement',{}).get('kind')=='model_derived':measured(entry['measurement'],root,dependencies,'departure_finger_model_'+str(i),now,subject=entry)
        binary=(root/site_config['motion_binary']).resolve();stop_evidence=(root/force['measurement']['evidence_file']).resolve()
        if digest(binary)!=contract['native_driver_sha256']or digest(stop_evidence)!=contract['stop_test_sha256']or monitor['heartbeat_timeout_s']!=.25:raise ValueError('Departure must bind actual native5N/1Nm executor and250ms stop test')
        dependencies['stock_departure_native_executor']={'path':str(binary),'sha256':digest(binary)}
        dependencies['stock_departure_stop_test']={'path':str(stop_evidence),'sha256':digest(stop_evidence)}
    other_envelope = copy.deepcopy(config['robots'][other_side]); other_envelope['_angular_speed_bound_rad_s'] = 0.
    if aperture_binding is not None:
        from .hand_aperture import validate
        intervals=validate(aperture_binding)
        envelope['_aperture_interval_m']=intervals[side];other_envelope['_aperture_interval_m']=intervals[other_side]
    # Enclose the actual native stop tracking budget, including local geometry
    # rotating about each SDK joint origin between millisecond FK samples.
    lengths=np.asarray(model['link_segment_lengths_m'])
    tracking_limit=float(config.get('joint_tracking_error_limit_rad',.02))
    tracking_bounds=np.zeros(9)
    for point in range(2,9):
        tracking_bounds[point]=tracking_limit*sum(np.sum(lengths[j+1:point])for j in range(7)if j+1<point)
    other_lengths=np.linalg.norm(np.diff(other_links,axis=0),axis=-1)+1e-6
    other_joint_error=np.abs(np.asarray(other_state['state']['q'])-other_state['state']['q_d'])+config['other_arm_q_tolerance_rad']
    other_tracking=np.zeros(9)
    for point in range(2,9):
        other_tracking[point]=sum(np.sum(other_lengths[j+1:point])*other_joint_error[j]for j in range(7)if j+1<point)
    def geometry(env,raw,pts,poses,speeds,velocities,gap,tracking,error,prefix,base_pose):
        if env.get('geometry_backend')=='sdk_link_columns':
            from .model_geometry import frames,robot_capsules
            F=collision.transform(np.asarray(raw['state']['F_T_EE_column_major']).reshape(4,4,order='F'))
            if not np.allclose(F.ravel(order='F'),env['F_T_EE_column_major'],atol=1e-10,rtol=0):raise ValueError('Actual configured TCP changed since model authoring; regenerate from fresh SDK state')
            source=raw.get('samples')
            fs=[row['link_frames_base']for row in source]if source else [raw['link_frames_base']]
            native_tcp=np.linalg.inv(base_pose)[None]@poses
            native_points=(pts-base_pose[:3,3])@base_pose[:3,:3]
            fs=frames(fs,native_points,native_tcp,F)
            fs=base_pose[None,None]@fs
            rows=robot_capsules(fs,poses,env,speeds,velocities,gap,tracking,error,prefix)
            for row in rows:row['robot_side']=side if prefix==side+'_'else other_side
            if convex_self_enabled and prefix==side+'_':
                from .convex_self import context as convex_self_context
                env[convex_self_key]=convex_self_context(env,rows,fs,side)
            support=config.get('robot_mounting_supports',{}).get(side if prefix==side+'_'else other_side)
            if isinstance(support,dict)and support.get('obstacle_name'):
                measured(support.get('measurement'),root,dependencies,prefix+'actual_mount_support',now)
                from .mount_support import certify
                from .model_preset import ASSETS
                with np.load(ASSETS/'collision-models.npz',allow_pickle=False)as z:vertices=z['link0_vertices']
                obstacles=[o for o in config['scene']['obstacles']if o.get('name')==support['obstacle_name']]
                if len(obstacles)!=1:raise ValueError('Unique measured fixed-base support obstacle required')
                proof=certify(support,obstacles[0],base_pose,vertices)
                for row in rows:
                    if row.get('link_index')==0:row['fixed_mount_support']=proof
            return rows
        rows=collision.robot_capsules(pts,env['link_radii_m'],speeds,gap,poses,env,prefix)
        for i,row in enumerate(rows):
            extra=max(tracking[i],tracking[i+1])if i<8 else tracking[-1]+np.sum(error)*max(np.linalg.norm(env['tool_capsules_tcp_m'][i-8]['a']),np.linalg.norm(env['tool_capsules_tcp_m'][i-8]['b']))
            row['r']+=extra
        return rows
    active=geometry(envelope,model,links,tcps,trace['speeds'],trace['joint_velocities'],trace['gap'],tracking_bounds,np.full(7,tracking_limit),side+'_',base)
    other=geometry(other_envelope,other_state,other_links[None],other_tcp[None],np.zeros(9),np.zeros(7),0,other_tracking,other_joint_error,other_side+'_',other_base)
    payloads = []
    for index, entry in enumerate(held):
        measured(entry.get('measurement'), root, dependencies, 'held_pose_'+str(index), now)
        dimensions = entry.get('payload_dimensions', {})
        if dimensions.get('measurement'):
            measured(dimensions['measurement'], root, dependencies, 'payload_shape_'+str(index), now)
        elif not entry.get('capsules_part_m'):
            raise ValueError('Actual payload enclosing geometry needs measurement evidence')
        g = collision.transform(entry['T_tcp_part'])
        moving = entry['robot'] == side
        if entry['robot'] not in (side, other_side):
            raise ValueError('Measured payload owner must be an actual arm')
        owner_tcp = tcps if moving else other_tcp[None]
        local = []
        for item in payload_capsules(entry):
            converted = dict(item)
            converted['a'] = collision.points(g, collision.finite(item['a'], (3,))).tolist()
            converted['b'] = collision.points(g, collision.finite(item['b'], (3,))).tolist()
            local.append(converted)
        rows = collision.local_capsules(local, owner_tcp, trace['speeds'][-1] if moving else 0.,
                                       envelope['_angular_speed_bound_rad_s'] if moving else 0., trace['gap'] if moving else 0., 'held_'+entry['part'])
        for row, shape in zip(rows, local):
            extent = max(np.linalg.norm(shape['a']), np.linalg.norm(shape['b']))
            row['r'] += (tracking_bounds[-1] + tracking_limit*7*extent if moving else other_tracking[-1] + np.sum(other_joint_error)*extent)
            row['r'] += config['target_position_tolerance_m'] + 2*extent*np.sin(config['target_angle_tolerance_rad']/2)
            row.update(part=entry['part'],T_tcp_part=entry['T_tcp_part'], owner='active' if moving else 'other',
                       robot_ignore_capsules=other_envelope.get('payload_robot_ignore_capsules', []),
                       instance_id=entry.get('instance_id', phase['_target_instance_id'] if entry['part']==part else None))
        payloads.extend(rows)
    if phase.get('bounded_loose_stock_departure')is not None:
        from .stock_departure import certify_issued_departure
        association=[entry for entry in held if entry['robot']==side and entry['part']==part and entry.get('observation_capture_id')==observation['capture_id']]
        selected=[row for row in payloads if row.get('owner')=='active'and row.get('part')==part]
        if len(association)!=1 or not selected or observation.get('scope')!='held':raise ValueError('Departure requires this fresh held-image/SDK G association')
        phase['_departure_certificate']=certify_issued_departure(tcps,phase,envelope,selected_payload=selected[0])
    dynamic = []
    scene = copy.deepcopy(config['scene'])
    # All observed clutter is retained, including fresh basket packets when
    # the primary view is a local held-object camera.
    for packet_index, packet in enumerate(scene_packets(observation, config, root, dependencies, now)):
        packet_residual_boxes=[]
        if packet.get('scope')=='basket' and config['scene'].get('require_residual_occupancy') is True:
            from .residual_occupancy import OccupancySnapshot
            snapshot=OccupancySnapshot.from_record(packet['residual_occupancy'],expected_capture_id=packet['capture_id'],expected_calibration_sha256=packet['calibration_sha256'],max_age_s=config['max_observation_age_s'],now_utc=now,static_scene_lease=packet.get('_static_scene_lease'))
            from .known_hand_capture import planner_boxes
            packet_residual_boxes,association_hash=planner_boxes(snapshot,packet,config,site_config,root,dependencies)
            if association_hash is not None:phase.setdefault('_known_hand_association_hashes',[]).append(association_hash)
            scene['obstacles'].extend(packet_residual_boxes)
        for box_index, obstacle in enumerate(packet.get('conservative_scene_obstacles', [])):
            if obstacle.get('source') != 'measured_unknown_scene_enclosure' or packet.get('basket_scene_status') != 'unknown_treated_fully_occupied':
                raise ValueError('Unknown-space enclosure needs actual measured basket provenance')
            role_paths = config.get('camera_configs') or {'default': config.get('camera_config')}
            camera_paths = [(root/name).resolve() for name in role_paths.values() if isinstance(name, str) and name]
            matching = [path for path in camera_paths if digest(path) == packet['calibration_sha256']]
            if not matching:
                raise ValueError('Unknown basket enclosure camera calibration missing')
            measured(obstacle.get('measurement'), matching[0].parent, dependencies,
                     'unknown_scene_'+str(packet_index)+'_'+str(box_index), now)
            if obstacle.get('includes_possible_protrusions') is not True:
                raise ValueError('Unknown basket enclosure must bound possible protrusions')
            collision.box(obstacle)
            occupied = copy.deepcopy(obstacle)
            occupied['name'] = 'unknown_'+str(packet_index)+'_'+str(box_index)+'_'+obstacle['name']
            scene['obstacles'].append(occupied)
        exclusions = packet.get('excluded_instance_ids', [])
        if exclusions:
            candidates = {value['instance_id']: value for value in packet.get('instances', [])}
            for identity in exclusions:
                instance = candidates.get(identity)
                association = [value for value in packet.get('excluded_measurements', []) if value.get('instance_id') == identity]
                if instance is None or len(association) != 1:
                    raise ValueError('Every excluded scene instance needs exact measured held-image association')
                association = association[0]
                if (association.get('source_capture_id') != observation['capture_id'] or
                        association.get('reason') != 'current_held_image_association' or
                        association.get('T_cell_part') != instance['T_cell_part'] or association.get('part') != instance['part']):
                    raise ValueError('Scene exclusion is not bound to this actual held-image capture')
                actual = collision.transform(instance['T_cell_part'])
                matched = []
                for entry in held:
                    if entry['part'] != instance['part']:
                        continue
                    tcp = tcps[0] if entry['robot'] == side else other_tcp
                    expected = tcp@collision.transform(entry['T_tcp_part'])
                    axis_error=float(np.arccos(np.clip(actual[:3,2]@expected[:3,2],-1,1)))
                    if np.linalg.norm(actual[:3, 3]-expected[:3, 3]) <= config['target_position_tolerance_m']and axis_error<=config['target_angle_tolerance_rad']:
                        matched.append(entry)
                close = [value for value in packet.get('instances', []) if value['part'] == instance['part'] and
                         np.linalg.norm(collision.transform(value['T_cell_part'])[:3, 3]-actual[:3, 3]) <= config['target_position_tolerance_m']]
                if len(matched) != 1 or len(close) != 1:
                    raise ValueError('Excluded basket instance is ambiguous or does not match the actual measured held transform')
                owner=matched[0];owned_payloads=[row for row in payloads if row.get('part')==owner['part']and row.get('owner')==('active'if owner['robot']==side else'other')]
                for occupied in packet_residual_boxes:
                    if occupied.get('occupancy_kind')=='occupied'and occupied.get('selected_contact_instance_id')==identity:
                        occupied['owned_payload_part']=owner['part'];occupied['owned_payload_owner']='active'if owner['robot']==side else'other'
                        occupied['owned_payload_instance_ids']=sorted({row['instance_id']for row in owned_payloads if row.get('instance_id')})
                        occupied['owned_surface_source_capture_id']=observation['capture_id']
        for instance in packet.get('instances', []):
            if instance['instance_id'] in exclusions:
                continue
            if (packet.get('scope') == 'held' and instance.get('instance_id') == observation.get('selected_instance', {}).get('instance_id') and
                    any(entry['part'] == instance['part'] for entry in held)):
                continue
            definition = config.get('loose_part_envelopes', {}).get(instance.get('part'))
            if not definition:
                raise ValueError('Actual enclosing geometry missing for observed loose part: '+str(instance.get('part')))
            measured(definition.get('measurement'), root, dependencies, 'loose_shape_'+instance['part'], now)
            object_pose = collision.transform(instance['T_cell_part'])[None]
            for row in collision.local_capsules(payload_capsules(definition), object_pose, 0, 0, 0, 'observed_'+instance['instance_id']):
                row.update(instance_id=instance['instance_id'], part=instance['part'],source='current_RGBD_loose_ring'if instance['part']in ('ring1','ring2')else'current_RGBD_pin')
                dynamic.append(row)
    for index, installed in enumerate(observation.get('installed_parts', [])):
        measured(installed.get('measurement'), root, dependencies, 'installed_pose_'+str(index), now)
        dimensions = installed.get('payload_dimensions', {})
        measured(dimensions.get('measurement'), root, dependencies, 'installed_shape_'+str(index), now)
        for row in collision.local_capsules(payload_capsules(installed), collision.transform(installed['T_cell_part'])[None], 0, 0, 0, 'installed_'+installed['part']):
            row.update(instance_id='installed:'+installed['part'], part=installed['part'])
            dynamic.append(row)
    result = collision.check_envelopes(active, other, scene, phase, envelope,
                                       payloads=payloads, dynamic_obstacles=dynamic, margin=config['collision_margin_m'])
    result['convex_self_narrowphase']=envelope[convex_self_key].record()if convex_self_key in envelope else None
    result['common_duo_mount_proof']=dict(proof_sha256=common_mount.proof_sha256,description_sha256=common_mount.description_sha256,scope='Only installed static opposite link0 pairs; moving and obstacle guards unchanged')if common_mount is not None else None
    result.update(sample_count=trace['sample_count'], maximum_sample_gap_s=trace['gap'],
                  independent_c2_curve_bound=not stopped_aperture, unsampled_motion_inflation=not stopped_aperture,
                  stopped_SDK_aperture_sweep=stopped_aperture,checked_Hand_aperture_binding=aperture_binding,
                  point_speed_bounds_m_s=trace['speeds'].tolist(),
                  enforced_joint_tracking_error_bound_rad=tracking_limit,
                  tracking_envelope_inflation_m=tracking_bounds.tolist(),
                  tight_clearance_rejection_possible=True,stock_departure_certificate=phase.get('_departure_certificate'))
    return result, trace


def write_new(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False); stream.write('\n')
    path.chmod(0o444)


def plan_phase(request, config, site_dir, *, native_plan=None, other_state=None, now=None):
    now = now or datetime.now(timezone.utc)
    site_dir = Path(site_dir).resolve()
    config_path = Path(config).resolve() if isinstance(config, (str, Path)) else None
    config = load(config_path) if config_path else copy.deepcopy(config)
    root = config_path.parent if config_path else site_dir
    dependencies = {}
    if config_path:
        dependencies['planner_config'] = {'path': str(config_path), 'sha256': digest(config_path)}
    validate_config(config, root, dependencies, now)
    side, part, phase = request['robot'], request['part'], request['phase']
    if side not in ('left', 'right') or part not in ('ring1', 'ring2', 'pin') or phase not in config['phases']:
        raise ValueError('Known actual robot/part and reviewed phase policy required')
    observation = request['observation']
    observation_valid(observation, config, root, dependencies, now)
    for key, value in request.get('dependency_files', {}).items():
        path = (site_dir/value['path']).resolve()
        if digest(path) != value['sha256']:
            raise ValueError('Request dependency hash changed: '+key)
        dependencies['request_'+key] = {'path': str(path), 'sha256': value['sha256']}
    site_path = (root/config['site_config']).resolve()
    site = load(site_path)
    dependencies['site_config'] = {'path': str(site_path), 'sha256': digest(site_path)}
    calibrated = load(dependencies['camera_config']['path'])
    for robot in ('left', 'right'):
        if not np.allclose(collision.transform(site['robots'][robot]['T_cell_base']),
                           collision.transform(calibrated['robots'][robot]['T_cell_base']), atol=1e-9):
            raise ValueError('Camera and site robot-base calibration differ')
    binary = (root/config['planner_binary']).resolve()
    dependencies['native_planner'] = {'path': str(binary), 'sha256': digest(binary)}
    targets = [collision.transform(target) for target in request.get('T_cell_tcp_targets', [])]
    if not targets and request.get('T_cell_part_target') is not None and request.get('T_tcp_part') is not None:
        targets = [collision.transform(request['T_cell_part_target'])@np.linalg.inv(collision.transform(request['T_tcp_part']))]
    if not 1 <= len(targets) <= 32:
        raise ValueError('One to 32 explicit measured/observed target transforms required')
    base = collision.transform(site['robots'][side]['T_cell_base'])
    base_targets = [np.linalg.inv(base)@target for target in targets]
    before = datetime.now(timezone.utc)
    if native_plan:
        seed_options={}if request.get('ik_seed')is None else dict(ik_seed=request['ik_seed'])
        model=native_plan(site['robots'][side]['ip'],base_targets,**seed_options)
    else:model=run_native(binary,'--read-model',site['robots'][side]['ip'],base_targets,ik_seed=request.get('ik_seed'))
    received = datetime.now(timezone.utc)
    opposite = 'right' if side == 'left' else 'left'
    other = other_state(site['robots'][opposite]['ip']) if callable(other_state) else other_state
    if other is None:
        other = run_native(binary, '--read-state', site['robots'][opposite]['ip'])
    if (datetime.now(timezone.utc)-received).total_seconds() > config['max_state_read_s']:
        raise ValueError('Other-arm state read exceeded its commissioned freshness limit')
    geometry, trace = geometry_check(model, other, config, site, phase, side, part, observation,
                                     request.get('held_parts', []), root, dependencies, now, request.get('contact'),aperture_binding=request.get('aperture_binding'))
    for actual, expected in zip((base@collision.transform(row['tcp_base']) for row in [model['samples'][-1]]), targets[-1:]):
        if np.linalg.norm(actual[:3, 3]-expected[:3, 3]) > 1e-4 or np.linalg.norm(actual[:3, :3]-expected[:3, :3]) > 1e-3:
            raise ValueError('Native phase endpoint differs from the measured requested target')
    dependencies_valid(dependencies)
    directory = Path(request.get('output_dir', site_dir/'dynamic-candidates')).resolve()/str(uuid.uuid4())
    directory.mkdir(parents=True, exist_ok=False)
    trace_path, csv_path, proof_path = directory/'native-model.json', directory/'phase.csv', directory/'proof.json'
    write_new(trace_path, model)
    with csv_path.open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['time_s']+[key+str(index) for key in ('q', 'dq', 'ddq') for index in range(1, 8)])
        for knot in model['knots']:
            writer.writerow([knot['time_s'], *knot['q'], *knot['dq'], *knot['ddq']])
    csv_path.chmod(0o444)
    motion_binary = (site_path.parent/site['motion_binary']).resolve()
    dependencies['native_motion'] = {'path': str(motion_binary), 'sha256': digest(motion_binary)}
    test_only = native_plan is not None or other_state is not None
    if not test_only:
        subprocess.run([str(motion_binary), '--validate-csv', str(csv_path)], capture_output=True, text=True, check=True, timeout=15)
    proof = {'schema_version': 2, 'planning_source': 'dynamic_field_envelope', 'test_only': test_only,
             'hardware_ready': False, 'motion_executed': False, 'phase': phase, 'part': part, 'robot': side,
             'robot_ip': site['robots'][side]['ip'], 'config': config, 'config_root': str(root),
             'dependencies': dependencies, 'native_model': {'path': str(trace_path), 'sha256': digest(trace_path)},
             'csv': {'path': str(csv_path), 'sha256': digest(csv_path)}, 'collision': geometry,
             'measured_state_started_at_utc': before.isoformat(), 'model_received_at_utc': received.isoformat(),
             'other_state': other, 'observation': observation, 'held_parts': request.get('held_parts', []),
             'aperture_binding':request.get('aperture_binding'),
             'contact': request.get('contact'), 'T_cell_tcp_targets': [target.tolist() for target in targets]}
    write_new(proof_path, proof)
    row = {'csv': str(csv_path), 'sha256': digest(csv_path), 'planning_source': 'dynamic_field_envelope',
           'field_path_reviewed': False, 'tool_type': 'franka_hand', 'robot_ip': site['robots'][side]['ip'],
           'F_T_EE_column_major': model['state']['F_T_EE_column_major'],
           'joint_tracking_error_limit_rad': float(config.get('joint_tracking_error_limit_rad', .02)),
           'dynamic_proof_file': str(proof_path), 'dynamic_proof_sha256': digest(proof_path),
           'dynamic_observation_file': observation['_path'], 'dynamic_observation_sha256': observation['_sha256']}
    return {'csv': str(csv_path), 'csv_sha256': row['sha256'], 'sha256': row['sha256'], 'stage_row': row,
            'proof': proof, 'proof_path': str(proof_path), 'phase': phase, 'part': part, 'robot': side,
            'model_start': model['state'], 'test_only': test_only, 'hardware_ready': False}


def validate_runtime_stage(row, side, *, site):
    if row.get('planning_source') != 'dynamic_field_envelope' or row.get('field_path_reviewed') is not False:
        raise ValueError('A generated conservative-envelope stage is required; it cannot claim an individual taught-path review')
    proof_path = Path(row['dynamic_proof_file'])
    if digest(proof_path) != row['dynamic_proof_sha256']:
        raise ValueError('Dynamic collision proof changed')
    proof = load(proof_path)
    if proof.get('test_only') is not False or proof['robot'] != side or proof['robot_ip'] != site.config['robots'][side]['ip']:
        raise ValueError('Synthetic backend output or different robot cannot execute')
    dependencies_valid(proof['dependencies'])
    if digest(row['csv']) != row['sha256'] or row['sha256'] != proof['csv']['sha256'] or str(Path(row['csv']).resolve()) != proof['csv']['path']:
        raise ValueError('Dynamic CSV differs from the actual checked C2 curve')
    if digest(proof['native_model']['path']) != proof['native_model']['sha256']:
        raise ValueError('Actual SDK trace changed')
    model = load(proof['native_model']['path'])
    with Path(row['csv']).open() as stream:
        reader = csv.DictReader(stream)
        csv_points = list(reader)
    if len(csv_points) != len(model['knots']):
        raise ValueError('Dynamic CSV knot count differs from the actual SDK trajectory')
    for point, knot in zip(csv_points, model['knots']):
        actual = [float(point['time_s'])]+[float(point[key+str(index)]) for key in ('q','dq','ddq') for index in range(1,8)]
        expected = [knot['time_s'], *knot['q'], *knot['dq'], *knot['ddq']]
        if not np.allclose(collision.finite(actual, (22,)), expected, atol=1e-12, rtol=1e-12):
            raise ValueError('Dynamic CSV differs from the independently checked SDK C2 knots')
    config, root, dependencies = proof['config'], Path(proof['config_root']), copy.deepcopy(proof['dependencies'])
    if row.get('joint_tracking_error_limit_rad') != float(config.get('joint_tracking_error_limit_rad', .02)):
        raise ValueError('Runtime tracking guard differs from the exact collision-envelope guard')
    now = datetime.now(timezone.utc)
    validate_config(config, root, dependencies, now)
    observation_path = row['dynamic_observation_file']
    if digest(observation_path) != row['dynamic_observation_sha256']:
        raise ValueError('Fresh execution observation changed')
    observation = load(observation_path)
    observation.update(_path=observation_path, _sha256=row['dynamic_observation_sha256'])
    observation_valid(observation, config, root, dependencies, now)
    compare_observation(proof['observation'], observation, config)
    fresh_dependencies = copy.deepcopy(dependencies)
    scene_packets(observation, config, root, fresh_dependencies, now)
    native = Path(proof['dependencies']['native_planner']['path'])
    active = run_native(native, '--read-state', proof['robot_ip'])
    native_output(active)
    if np.max(np.abs(np.asarray(active['state']['q_d'])-model['q_start'])) > 1e-6:
        raise ValueError('Active robot start changed since measured dynamic planning')
    if active['state']['F_T_EE_column_major'] != row['F_T_EE_column_major']:
        raise ValueError('Measured active TCP changed since planning')
    opposite = 'right' if side == 'left' else 'left'
    other = run_native(native, '--read-state', site.config['robots'][opposite]['ip'])
    native_output(other)
    if np.max(np.abs(np.asarray(other['state']['q_d'])-proof['other_state']['state']['q_d'])) > config['other_arm_q_tolerance_rad']:
        raise ValueError('Other arm posture changed; replan before moving')
    if proof.get('aperture_binding')is not None:
        if proof['aperture_binding'].get('actual_sweep_side')is not None:raise ValueError('A Hand opening/closing sweep cannot authorize arm motion')
        from .hand_aperture import current_within
        current_within(proof['aperture_binding'],{s:site.grippers[s].status()for s in ['left','right']})
    # Recompute the checked C2 curve and whole conservative geometry with current other-arm FK.
    geometry_check(model, other, config, load(proof['dependencies']['site_config']['path']), proof['phase'], side,
                   proof['part'], observation, proof['held_parts'], root, dependencies, now, proof.get('contact'),aperture_binding=proof.get('aperture_binding'))
    # Native reads and geometric validation consume real time. Check the real
    # sensor exposure timestamps again at the end; never restamp old images.
    final_now = datetime.now(timezone.utc)
    observation_valid(observation, config, root, fresh_dependencies, final_now)
    scene_packets(observation, config, root, fresh_dependencies, final_now)
    if proof.get('aperture_binding')is not None:
        current_within(proof['aperture_binding'],{s:site.grippers[s].status()for s in ['left','right']})
    dependencies_valid(fresh_dependencies)
    dependencies_valid(proof['dependencies'])
    return {'state': 'dynamic_envelope_revalidated', 'hardware_ready': False}


def compare_observation(previous, current, config):
    if (previous.get('stabilization_profile') != current.get('stabilization_profile') or
            previous.get('stabilization_dependency_files', {}) != current.get('stabilization_dependency_files', {})):
        raise ValueError('Selected stabilization profile/engine changed between planning and execution')
    if previous.get('calibration_sha256') != current.get('calibration_sha256'):
        raise ValueError('Calibration changed between planning and execution')
    def target(value):
        return value.get('T_cell_part', value.get('selected_instance', {}).get('T_cell_part'))
    a, b = target(previous), target(current)
    if a is None or b is None:
        raise ValueError('Exact selected image-derived part pose required for motion revalidation')
    a, b = collision.transform(a), collision.transform(b)
    angle = np.arccos(np.clip((np.trace(a[:3, :3].T@b[:3, :3])-1)/2, -1, 1))
    if np.linalg.norm(a[:3, 3]-b[:3, 3]) > config['target_position_tolerance_m'] or angle > config['target_angle_tolerance_rad']:
        raise ValueError('Fresh image-derived target moved after planning; replan')


class DynamicPlanner:
    def __init__(self, site, config, output_dir):
        self.site, self.config, self.output_dir = site, Path(config).resolve(), Path(output_dir).resolve()

    def plan(self, phase, part, side, T_cell_tcp_targets, observation, *, held_parts=None, contact=None,aperture_binding=None,ik_seed=None):
        return plan_phase({'phase': phase, 'part': part, 'robot': side,
                           'T_cell_tcp_targets': T_cell_tcp_targets, 'observation': observation,
                           'held_parts': held_parts or [], 'contact': contact,'aperture_binding':aperture_binding,'ik_seed':ik_seed, 'output_dir': str(self.output_dir)},
                          self.config, self.site.filename.parent)

    def revalidate(self, plan, observation=None):
        row = plan['stage_row']
        if observation is not None:
            config = load(self.config)
            compare_observation(plan['proof']['observation'], observation, config)
            row['dynamic_observation_file'] = observation['_path']
            row['dynamic_observation_sha256'] = observation['_sha256']
        return validate_runtime_stage(row, plan['robot'], site=self.site)


def check_stopped_aperture(config_path,site,phase,part,side,observation,held_parts,aperture_binding,*,contact=None):
    """Full geometry sweep at actual stopped SDK FK; never an issued arm curve."""
    config_path=Path(config_path).resolve();root=config_path.parent;config=load(config_path);dependencies={};now=datetime.now(timezone.utc)
    validate_config(config,root,dependencies,now);observation_valid(observation,config,root,dependencies,now)
    site_config=load(site.filename);binary=(root/config['planner_binary']).resolve()
    from .hand_aperture import current_within
    current_within(aperture_binding,{s:site.grippers[s].status()for s in ['left','right']})
    active=run_native(binary,'--read-state',site_config['robots'][side]['ip']);opposite='right'if side=='left'else'left'
    other=run_native(binary,'--read-state',site_config['robots'][opposite]['ip'])
    result,_=geometry_check(active,other,config,site_config,phase,side,part,observation,held_parts,root,dependencies,now,contact,
                            aperture_binding=aperture_binding,stopped_aperture=True)
    current_within(aperture_binding,{s:site.grippers[s].status()for s in ['left','right']})
    final=datetime.now(timezone.utc);observation_valid(observation,config,root,dependencies,final);scene_packets(observation,config,root,dependencies,final);dependencies_valid(dependencies)
    return dict(scope='actual_stopped_SDK_pose_full_commanded_Hand_aperture_sweep',arm_motion_issued=False,physical_execution_certified=False,
                config_file=str(config_path),config_sha256=digest(config_path),leased_scene_closure=observation.get('static_scene_lease')is not None,
                geometry=result,aperture_binding=aperture_binding,active_SDK=active,other_SDK=other,
                observation_file=observation['_path'],observation_sha256=observation['_sha256'],dependencies=dependencies)
