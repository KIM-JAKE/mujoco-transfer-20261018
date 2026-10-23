"""Synthetic geometry/native contracts only; never connects or authorizes hardware."""
import copy
from datetime import datetime, timezone, timedelta
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch, Mock
from contextlib import contextmanager
import numpy as np

from capx_autoring.field_demo import collision, random_planner as planner


def pose(x=0., y=0., z=0.):
    result = np.eye(4); result[:3, 3] = [x, y, z]; return result.tolist()


def obb(x=0., y=0., z=0., half=(2, 2, 2), name=None):
    result = {'T_cell_box': pose(x, y, z), 'half_extents_m': list(half)}
    if name:
        result['name'] = name
    return result


def model(delta=0., radius=.0001):
    q0, q1 = np.zeros(7), np.array([delta, 0, 0, 0, 0, 0, 0])
    samples = []
    for t, q in [(0., q0), (.001, q1)]:
        angle = q[0]; r = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        links = np.zeros((9, 3)); links[1] = [.1, 0, 0]
        for point in range(2, 9):
            links[point] = [.1, 0, 0]+r@np.array([.1*(point-1), 0, 0])
        tcp = np.eye(4); tcp[:3, :3] = r; tcp[:3, 3] = links[-1]
        samples.append({'time_s': t, 'q': q.tolist(), 'dq': [0.]*7,
                        'tcp_base': tcp.tolist(), 'link_points_base_m': links.tolist()})
    lengths = np.array([.1+1e-6]*8)
    velocities = np.abs(q1-q0)*1.875/.001
    speeds = np.zeros(9)
    for k in range(2, 9):
        for j in range(7):
            if j+1 < k:
                speeds[k] += sum(lengths[j+1:k])*velocities[j]
    state = {'q': q0.tolist(), 'q_d': q0.tolist(), 'dq': [0.]*7, 'dq_d': [0.]*7,
             'F_T_EE_column_major': np.eye(4).ravel(order='F').tolist(), 'm_ee_kg': 1., 'm_load_kg': 0.}
    return {'read_only': True, 'motion_executed': False, 'server_protocol_version': 10,
            'continuous_10percent_checked': True, 'q_start': q0.tolist(), 'state': state,
            'duration_s': .001, 'knots': [{'time_s': 0., 'q': q0.tolist(), 'dq': [0.]*7, 'ddq': [0.]*7},
                                        {'time_s': .001, 'q': q1.tolist(), 'dq': [0.]*7, 'ddq': [0.]*7}],
            'samples': samples, 'joint_velocity_bound_rad_s': velocities.tolist(),
            'link_segment_lengths_m': lengths.tolist(), 'link_point_speed_bounds_m_s': speeds.tolist()}


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name); self.now = datetime.now(timezone.utc)
        (self.root/'evidence.txt').write_text('synthetic unit test fixture; never hardware evidence')
        self.measurement = {'verified': True, 'by': 'offline test fixture', 'at_utc': self.now.isoformat(), 'evidence_file': 'evidence.txt'}
        for name in ('planner', 'motion'):
            (self.root/name).write_text('fake unit test backend')
        self.site = {'robots': {'left': {'ip': '192.0.2.1', 'T_cell_base': pose()},
                                 'right': {'ip': '192.0.2.2', 'T_cell_base': pose(y=1.)}}, 'motion_binary': 'motion'}
        self.write('site.json', self.site)
        self.camera = {'calibration_id': 'test', 'measured': self.measurement, 'cell_frame': 'cell',
                       'camera_frame': 'camera', 'camera_serial': 'unit-test-no-device', 'robots': self.site['robots']}
        self.write('camera.json', self.camera)
        self.instance = {'instance_id': 'test-image:0', 'part': 'ring1', 'T_cell_part': pose(.8, .5)}
        meta = {'units': 'm', 'camera_frame': 'camera', 'point_cloud_frame': 'camera', 'calibration_id': 'test',
                'calibration_sha256': planner.digest(self.root/'camera.json'), 'capture_id': 'test-image',
                'source': 'realsense_live', 'replay': False, 'frame_number': 1, 'camera_serial': 'unit-test-no-device',
                'captured_at_utc': self.now.isoformat(), 'exposure_s': {'rgb': .001, 'depth': .001}}
        (self.root/'capture.npz').write_bytes(b'offline fake RGB-D evidence, no hardware')
        self.write('capture.json', meta)
        self.observation = {'scope': 'basket', 'units': 'm', 'replay': False, 'capture_id': 'test-image', 'captured_at_utc': self.now.isoformat(),
                            'calibration_id': 'test', 'calibration_sha256': planner.digest(self.root/'camera.json'),
                            'source_metadata': meta, 'output_frame': 'cell', 'instances': [self.instance],
                            'selected_instance': self.instance, 'T_cell_part': self.instance['T_cell_part'],
                            'source_packet': str(self.root/'capture.npz'), 'source_packet_sha256': planner.digest(self.root/'capture.npz'),
                            'source_metadata_file': str(self.root/'capture.json'), 'source_metadata_sha256': planner.digest(self.root/'capture.json')}

        self.write_observation(self.observation)
        envelope = {'measurement': self.measurement, 'whole_robot_hand_tool_cables_enclosed': True,
                    'link_radii_m': [.0001]*8, 'tool_in_link_envelopes': True, 'tool_capsules_tcp_m': [],
                    'self_ignore_pairs': [[i, i+1] for i in range(7)], 'payload_robot_ignore_capsules': []}
        phase = {'measurement': self.measurement, 'corridors': [obb()], 'tcp_corridor_radius_m': .01,
                 'force_monitor': {'measurement': self.measurement, 'enabled': True, 'force_limit_n': 5, 'moment_limit_nm': 1}}
        self.config = {'schema_version': 2, 'units': 'm', 'measurement': self.measurement,
                       'site_config': 'site.json', 'planner_binary': 'planner', 'camera_configs': {'basket': 'camera.json'},
                       'collision_margin_m': .001, 'max_observation_age_s': 1., 'max_state_read_s': 10.,
                       'other_arm_q_tolerance_rad': .001, 'target_position_tolerance_m': .001,
                       'target_angle_tolerance_rad': .01,
                       'scene': {'measurement': self.measurement, 'complete_within_workspace': True, 'unobserved_geometry_enclosed': True,
                                 'require_basket_observation': True, 'workspace': obb(), 'obstacles': [obb(1.5, 1.5, 1.5, half=(.05, .05, .05), name='far fixture')]},
                       'robots': {'left': copy.deepcopy(envelope), 'right': copy.deepcopy(envelope)},
                       'phases': {'pickup': phase, 'insert': copy.deepcopy(phase)},
                       'loose_part_envelopes': {'ring1': {'measurement': self.measurement, 'payload_dimensions': {'radius_m': .01, 'length_m': .01}}}}
        self.write('random-motion.json', self.config)
        self.native = model()
        self.other = {'read_only': True, 'motion_executed': False, 'server_protocol_version': 10,
                      'state': copy.deepcopy(self.native['state']), 'link_points_base_m': self.native['samples'][0]['link_points_base_m'],
                      'tcp_base': self.native['samples'][0]['tcp_base']}

    def write(self, name, value):
        path = self.root/name; path.write_text(json.dumps(value, allow_nan=False)+'\n'); return path

    def write_observation(self, value):
        value.pop('_path', None); value.pop('_sha256', None)
        self.write('observation.json', value)
        value['_path'] = str(self.root/'observation.json'); value['_sha256'] = planner.digest(self.root/'observation.json')

    def plan(self, **kwargs):
        request = {'phase': 'pickup', 'robot': 'left', 'part': 'ring1', 'observation': self.observation,
                   'T_cell_tcp_targets': [self.native['samples'][-1]['tcp_base']]}
        request.update(kwargs)
        return planner.plan_phase(request, self.root/'random-motion.json', self.root,
                                  native_plan=lambda *_: copy.deepcopy(self.native), other_state=copy.deepcopy(self.other), now=self.now)

    def test_safe_synthetic_candidate_is_immutable_and_cannot_authorize_live_execution(self):
        with patch('subprocess.run', side_effect=AssertionError('No subprocess in synthetic fixture')):
            result = self.plan()
        self.assertTrue(result['test_only']); self.assertFalse(result['hardware_ready'])
        self.assertFalse(result['stage_row']['field_path_reviewed'])
        self.assertEqual(result['proof']['collision']['sample_count'], 2)
        self.assertTrue(result['proof']['collision']['independent_c2_curve_bound'])
        self.assertEqual(planner.digest(result['csv']), result['csv_sha256'])
        site = type('FakeSite', (), {'config': self.site})()
        with self.assertRaisesRegex(ValueError, 'Synthetic backend'):
            planner.validate_runtime_stage(result['stage_row'], 'left', site=site)

    def test_actual_geometric_obstacle_collision_stops(self):
        self.config['scene']['obstacles'] = [obb(.4, 0, 0, half=(.01, .01, .01), name='basket wall')]
        self.write('random-motion.json', self.config)
        with self.assertRaises(collision.CollisionError): self.plan()

    def test_other_arm_collision_stops(self):
        self.site['robots']['right']['T_cell_base'] = pose()
        self.camera['robots'] = self.site['robots']
        self.write('site.json', self.site); self.write('camera.json', self.camera)
        self.observation['calibration_sha256'] = planner.digest(self.root/'camera.json')
        self.observation['source_metadata']['calibration_sha256'] = self.observation['calibration_sha256']
        self.write('capture.json', self.observation['source_metadata'])
        self.observation['source_metadata_sha256'] = planner.digest(self.root/'capture.json')
        self.write_observation(self.observation)
        with self.assertRaisesRegex(collision.CollisionError, 'Other-arm'): self.plan()

    def test_unknown_or_empty_field_geometry_never_passes(self):
        for mutate in [lambda c: c['scene'].update(obstacles=[]),
                       lambda c: c['scene'].update(complete_within_workspace=None),
                       lambda c: c['robots']['left'].update(whole_robot_hand_tool_cables_enclosed=None),
                       lambda c: c['robots']['left'].update(link_radii_m=[None]*8)]:
            original = copy.deepcopy(self.config); mutate(self.config); self.write('random-motion.json', self.config)
            with self.assertRaises((ValueError, TypeError)): self.plan()
            self.config = original

    def test_stale_or_replay_observation_stops(self):
        for patch_values in [{'captured_at_utc': (self.now-timedelta(seconds=3)).isoformat()}, {'replay': True}]:
            original = copy.deepcopy(self.observation); self.observation.update(patch_values); self.write_observation(self.observation)
            with self.assertRaises(ValueError): self.plan()
            self.observation = original

    def test_cached_trace_cannot_replace_the_actual_c2_curve(self):
        self.native['samples'][-1]['q'][0] = .01
        with self.assertRaisesRegex(ValueError, 'exact exported C2'): self.plan()

    def test_sample_gaps_and_forged_velocity_bounds_stop(self):
        changed = model(); changed['samples'][-1]['time_s'] = .002
        with self.assertRaises(ValueError): planner.validate_model(changed)
        changed = model(.0001); changed['link_point_speed_bounds_m_s'] = [0]*9
        with self.assertRaisesRegex(ValueError, 'point speed bounds'): planner.validate_model(changed)

    def test_continuous_intersample_inflation_catches_a_collision_between_samples(self):
        moving = model(.0001); trace = planner.validate_model(moving)
        links = trace['links']; tcps = trace['tcps']; envelope = copy.deepcopy(self.config['robots']['left'])
        envelope['link_radii_m'] = [1e-6]*8; envelope['_angular_speed_bound_rad_s'] = sum(trace['joint_velocities'])
        center = (links[0, -1]+links[1, -1])/2
        obstacle = obb(*center, half=(1e-7, 1e-7, 1e-7), name='thin obstruction')
        nominal = collision.robot_capsules(links, envelope['link_radii_m'], np.zeros(9), 0, tcps, envelope, 'active')
        inflated = collision.robot_capsules(links, envelope['link_radii_m'], trace['speeds'], trace['gap'], tcps, envelope, 'active')
        self.assertTrue(np.all(collision.segment_box_distance(nominal[-1]['a'], nominal[-1]['b'], obstacle)>nominal[-1]['r']))
        self.assertTrue(np.any(collision.segment_box_distance(inflated[-1]['a'], inflated[-1]['b'], obstacle)<=inflated[-1]['r']))

    def test_changed_scene_and_stale_other_arm_are_runtime_rejections(self):
        result = self.plan(); proof_path = Path(result['proof_path']); proof_path.chmod(0o644)
        proof = planner.load(proof_path); proof['test_only'] = False
        proof_path.write_text(json.dumps(proof)); result['stage_row']['dynamic_proof_sha256'] = planner.digest(proof_path)
        self.config['scene']['obstacles'][0]['half_extents_m'][0] += .01
        self.write('random-motion.json', self.config)
        site = type('FakeSite', (), {'config': self.site})()
        with self.assertRaisesRegex(ValueError, 'dependency changed'):
            planner.validate_runtime_stage(result['stage_row'], 'left', site=site)

    def test_selected_pose_must_refer_to_actual_saved_image_instance(self):
        self.observation['selected_instance'] = dict(self.instance, T_cell_part=pose(0, 0, 0))
        with self.assertRaisesRegex(ValueError, 'actual instance'): self.plan()

    def production_fixture(self):
        result = self.plan()
        path = Path(result['proof_path']); path.chmod(0o644)
        proof = planner.load(path); proof['test_only'] = False
        path.write_text(json.dumps(proof)); result['stage_row']['dynamic_proof_sha256'] = planner.digest(path)
        return result

    def fake_site(self, result):
        from capx_autoring.isaac_port.fieldkit.site import Site
        from capx_autoring.isaac_port.fieldkit.hand import FrankaHand
        site = object.__new__(Site)
        site.filename = self.root/'site.json'; site.config = copy.deepcopy(self.site)
        site.config.update(stages={'left.dynamic.test': result['stage_row']}, state_binary='state',
                           calibration_record='evidence.txt', commissioning_record='evidence.txt')
        site.config['robots']['left'].update(tool_mass_kg=1., load_mass_kg=0.)
        site.grippers = {'left': object.__new__(FrankaHand)}
        site.log_dir = self.root
        site.validate_motion_config = Mock(); site.require_ready = Mock(); site.event = Mock()
        return site

    def test_site_rejects_test_backend_before_state_probe_or_motion_process(self):
        result = self.plan(); site = self.fake_site(result)
        with patch('subprocess.run', side_effect=AssertionError('Must reject before state probe')), \
                patch('subprocess.Popen', side_effect=AssertionError('No motion process')):
            with self.assertRaisesRegex(ValueError, 'Synthetic backend'):
                site.motion('left', 'csv', [result['csv']])

    def test_site_revalidates_again_under_lock_before_motion_process(self):
        result = self.plan(); site = self.fake_site(result); states = []; locked = False
        @contextmanager
        def lock():
            nonlocal locked
            locked = True
            try: yield
            finally: locked = False
        site.control_lock = lock
        def verify(*args, **kwargs):
            states.append(locked)
            if locked: raise ValueError('Under-lock revalidation refused')
        probe = Mock(stdout=json.dumps({'F_T_EE_column_major': result['stage_row']['F_T_EE_column_major']}))
        with patch.object(planner, 'validate_runtime_stage', side_effect=verify), \
                patch('subprocess.run', return_value=probe), \
                patch('subprocess.Popen', side_effect=AssertionError('No motion process')):
            with self.assertRaisesRegex(ValueError, 'Under-lock'):
                site.motion('left', 'csv', [result['csv']])
        self.assertEqual(states, [False, True])

    def test_runtime_rejects_changed_other_arm(self):
        result = self.production_fixture()
        site = self.fake_site(result)
        active = copy.deepcopy(self.other)
        other = copy.deepcopy(self.other); other['state']['q_d'][0] = .002; other['state']['q'][0] = .002
        with patch.object(planner, 'run_native', side_effect=[active, other]):
            with self.assertRaisesRegex(ValueError, 'Other arm posture changed'):
                planner.validate_runtime_stage(result['stage_row'], 'left', site=site)

    def test_runtime_rechecks_freshness_after_reads_and_geometry(self):
        result = self.production_fixture(); site = self.fake_site(result)
        with patch.object(planner, 'run_native', side_effect=[copy.deepcopy(self.other), copy.deepcopy(self.other)]), \
                patch.object(planner, 'datetime') as clock:
            clock.now.side_effect = [self.now, self.now+timedelta(seconds=3)]
            with self.assertRaisesRegex(ValueError, 'Stale/future'):
                planner.validate_runtime_stage(result['stage_row'], 'left', site=site)

    def test_held_only_view_requires_full_basket_or_verified_static_clutter_enclosure(self):
        self.observation['scope'] = 'held'; self.write_observation(self.observation)
        with self.assertRaisesRegex(ValueError, 'Held-only'):
            self.plan()

    def test_raw_depth_evidence_mutation_is_rejected(self):
        (self.root/'capture.npz').write_bytes(b'changed packet')
        with self.assertRaisesRegex(ValueError, 'RGB-D evidence'):
            self.plan()

    def test_tighter_guard_requires_actual_evidence_and_matches_runtime(self):
        self.config['joint_tracking_error_limit_rad'] = .001
        self.write('random-motion.json', self.config)
        with self.assertRaisesRegex(ValueError, 'Verified actual-cell'):
            self.plan()
        self.config['tracking_guard_measurement'] = self.measurement
        self.write('random-motion.json', self.config)
        result = self.plan()
        self.assertEqual(result['stage_row']['joint_tracking_error_limit_rad'], .001)
        self.assertEqual(result['proof']['collision']['enforced_joint_tracking_error_bound_rad'], .001)
        self.native['state']['q'][0] = .002
        with self.assertRaisesRegex(ValueError, 'start tracking error'):
            self.plan()

    def test_runtime_guard_cannot_differ_from_collision_bound(self):
        result = self.production_fixture(); result['stage_row']['joint_tracking_error_limit_rad'] = .001
        with patch.object(planner, 'run_native', side_effect=AssertionError('Must reject before reads')):
            with self.assertRaisesRegex(ValueError, 'tracking guard differs'):
                planner.validate_runtime_stage(result['stage_row'], 'left', site=self.fake_site(result))

    def test_runtime_checks_fresh_other_objects_even_when_selected_target_does_not_move(self):
        result = self.production_fixture(); fresh = copy.deepcopy(self.observation)
        fresh.pop('_path'); fresh.pop('_sha256')
        fresh['capture_id'] = 'fresh-image'; fresh['source_metadata']['capture_id'] = 'fresh-image'
        fresh['source_metadata']['frame_number'] = 2
        fresh['instances'].append({'instance_id': 'fresh-image:other', 'part': 'ring1', 'T_cell_part': pose(.4)})
        self.write('fresh-capture.json', fresh['source_metadata'])
        fresh['source_metadata_file'] = str(self.root/'fresh-capture.json')
        fresh['source_metadata_sha256'] = planner.digest(fresh['source_metadata_file'])
        path = self.write('fresh.json', fresh)
        result['stage_row']['dynamic_observation_file'] = str(path)
        result['stage_row']['dynamic_observation_sha256'] = planner.digest(path)
        with patch.object(planner, 'run_native', side_effect=[copy.deepcopy(self.other), copy.deepcopy(self.other)]):
            with self.assertRaisesRegex(collision.CollisionError, 'Observed loose-part'):
                planner.validate_runtime_stage(result['stage_row'], 'left', site=self.fake_site(result))

    def test_unsupported_global_self_masks_are_rejected(self):
        self.config['robots']['left']['self_ignore_pairs'].append([0, 7])
        self.write('random-motion.json', self.config)
        with self.assertRaisesRegex(ValueError, 'adjacent joints'):
            self.plan()

    def test_measured_ring_shell_capsules_preserve_the_real_hole(self):
        body = {'capsules_part_m': [{'a': [.02, 0, -.001], 'b': [.02, 0, .001], 'radius_m': .001}],
                'payload_dimensions': {'radius_m': .03, 'length_m': .01}}
        self.assertEqual(planner.payload_capsules(body), body['capsules_part_m'])

    def profile_observation(self):
        source = self.root/'selected-profile.json'
        source.write_text('{"profile_id":"synthetic-unit-test-only"}\n')
        self.observation['stabilization_profile'] = {'profile_id': 'synthetic-unit-test-only',
            'sha256': planner.digest(source), 'physical_speed_claimed': False}
        self.observation['stabilization_dependency_files'] = {'profile': {'path': str(source), 'sha256': planner.digest(source)}}
        self.write_observation(self.observation)
        return source

    def test_selected_profile_dependency_is_sealed_into_plan_and_mutation_rejected(self):
        source = self.profile_observation()
        result = self.plan()
        self.assertEqual(result['proof']['dependencies']['stabilization_profile']['sha256'], planner.digest(source))
        self.assertIn('site_config', result['proof']['dependencies'])
        source.write_text('{"profile_id":"changed"}\n')
        with self.assertRaisesRegex(ValueError, 'Stabilization dependency changed'):
            self.plan()

    def test_fresh_capture_cannot_silently_switch_stabilization_profile(self):
        self.profile_observation()
        fresh = copy.deepcopy(self.observation)
        fresh['stabilization_profile']['profile_id'] = 'different-profile'
        with self.assertRaisesRegex(ValueError, 'profile/engine changed'):
            planner.compare_observation(self.observation, fresh, self.config)

    def test_exact_segment_obb_distance_detects_interior_crossing(self):
        value = obb(half=(.1, .1, .1))
        self.assertEqual(float(collision.segment_box_distance(np.array([-1., 0, 0]), np.array([1., 0, 0]), value)), 0.)
        self.assertAlmostEqual(float(collision.segment_box_distance(np.array([-1., .2, 0]), np.array([1., .2, 0]), value)), .1)
        self.assertEqual(float(collision.segment_distance([-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0])), 0.)


if __name__ == '__main__': unittest.main()
