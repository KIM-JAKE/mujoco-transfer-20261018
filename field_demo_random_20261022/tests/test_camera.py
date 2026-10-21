import json
from datetime import datetime, timedelta, timezone
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from field_demo.calibration import load_config, fit_correspondences
from field_demo.camera import finite_sdk_cloud, save_capture, RealSenseCapture, run_observer, list_devices
from field_demo.observations import validate_observation, target_proposals, estimate_snapshot


def translated(x=0, y=0, z=0):
    a = np.eye(4)
    a[:3, 3] = [x, y, z]
    return a.tolist()


class CameraContracts(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        (self.root/'evidence.txt').write_text('Synthetic test correspondences, not deployment calibration')
        self.config_path = self.root/'camera.json'
        self.c = dict(schema_version=1, units='m', calibration_id='test-only', camera_frame='color_optical', camera_serial='TEST_ONLY', cell_frame='cell',
                      measured=dict(verified=True, by='unit test fixture', at_utc='2020-01-01T00:00:00Z',
                                    method='synthetic unit test fixture only', evidence_file='evidence.txt', correspondences=4, residual_m=.001),
                      T_cell_camera=translated(.2, 0, 1),
                      robots=dict(left=dict(base_frame='left_base', T_cell_base=translated(1, 0, 0)),
                                  right=dict(base_frame='right_base', T_cell_base=translated(-1, 0, 0))),
                      nominal_stock_poses={n:translated(.1, .2, .3) for n in ('ring1', 'ring2', 'pin')},
                      part_robot=dict(ring1='right', ring2='left', pin='left'), pin_holder_tracks_pin=False)
        self.write_config()
        self.loaded = load_config(self.config_path)
        self.now = datetime.now(timezone.utc)
        self.meta = dict(capture_id='capture-a', camera_frame='color_optical', point_cloud_frame='color_optical',
                         calibration_id='test-only', calibration_sha256=self.loaded['_calibration_sha256'], units='m',
                         source='realsense_live', replay=False, camera_serial='TEST_ONLY', frame_number=10,
                         hardware_timestamp_ms=1000., exposure_s=dict(rgb=.005, depth=.005),
                         captured_at_utc=(self.now-timedelta(seconds=.05)).isoformat())

    def write_config(self):
        self.config_path.write_text(json.dumps(self.c))

    def test_sdk_structured_and_image_arrays_are_finite_flat_clouds(self):
        vertices = np.zeros((1, 4), dtype=[('f0','f4'),('f1','f4'),('f2','f4')])
        vertices['f2'] = [1, 0, np.nan, 2]
        points = finite_sdk_cloud(vertices)
        self.assertEqual(points.shape, (2, 3))
        np.testing.assert_array_equal(points[:,2], [1,2])
        self.assertEqual(finite_sdk_cloud(points.reshape(1,2,3)).shape, (2,3))

    def test_missing_optional_sdk_is_actionable_and_wrong_camera_rejected(self):
        with patch.dict('sys.modules', {'pyrealsense2': None}):
            with self.assertRaisesRegex(RuntimeError, 'requirements-camera.txt'):
                RealSenseCapture(self.loaded).start()
        with self.assertRaisesRegex(ValueError, 'serial'):
            RealSenseCapture(self.loaded, serial='ANOTHER_CAMERA')

    def test_device_listing_reads_descriptors_without_starting_stream(self):
        values = dict(name='TEST CAMERA',serial_number='TEST_SERIAL',firmware_version='TEST_FIRMWARE',usb_type_descriptor='3.2')
        class Device:
            def supports(self,info):return info in values
            def get_info(self,info):return values[info]
        sdk = SimpleNamespace(camera_info=SimpleNamespace(**{k:k for k in values}),
                              context=lambda:SimpleNamespace(query_devices=lambda:[Device()]))
        report = list_devices(sdk)
        self.assertEqual(report['devices'],[dict(model='TEST CAMERA',serial='TEST_SERIAL',firmware='TEST_FIRMWARE',usb='3.2')])
        self.assertFalse(report['streams_started'])
        self.assertFalse(report['robot_command_sent'])

    def test_calibration_rejects_mm_reflection_and_unmeasured_defaults(self):
        for kind in ('millimetres', 'reflection', 'unmeasured'):
            with self.subTest(kind=kind):
                c = json.loads(json.dumps(self.c))
                if kind == 'millimetres': c['T_cell_camera'][0][3] = 600
                elif kind == 'reflection': c['T_cell_camera'][0][0] = -1
                else: c['measured']['verified'] = False
                self.config_path.write_text(json.dumps(c))
                with self.assertRaises(ValueError): load_config(self.config_path)

    def test_calibration_requires_real_evidence_file(self):
        (self.root/'evidence.txt').unlink()
        with self.assertRaisesRegex(ValueError, 'evidence'): load_config(self.config_path)

    def test_calibration_fit_known_transform_and_reject_degenerate_mm_reflected(self):
        source = np.array([[0,0,0],[.1,0,0],[0,.1,0],[0,0,.1]])
        rotation = np.array([[0,-1,0],[1,0,0],[0,0,1]])
        target = source@rotation.T+[.3,.4,.5]
        d = dict(units='m',points_camera_m=source.tolist(),points_cell_m=target.tolist())
        r = fit_correspondences(d)
        np.testing.assert_allclose(np.array(r['T_cell_camera'])[:3,:3],rotation,atol=1e-12)
        np.testing.assert_allclose(np.array(r['T_cell_camera'])[:3,3],[.3,.4,.5])
        self.assertFalse(r['measured_verified'])
        for bad in (dict(d,units='mm'), dict(d,points_camera_m=(source*1000).tolist()),
                    dict(d,points_camera_m=[[0,0,0],[.1,0,0],[.2,0,0],[.3,0,0]]),
                    dict(d,points_cell_m=(source*np.array([-1,1,1])).tolist())):
            with self.assertRaises(ValueError): fit_correspondences(bad)

    def test_freshness_utc_does_not_trust_cross_machine_monotonic(self):
        m = dict(self.meta, received_monotonic_s=1e99, monotonic_clock_id='other-host')
        self.assertFalse(validate_observation(m, self.loaded, now=self.now)['execution_allowed'])
        for offset in (-5, .1):
            with self.subTest(offset=offset):
                m['captured_at_utc'] = (self.now+timedelta(seconds=offset)).isoformat()
                with self.assertRaises(RuntimeError): validate_observation(m, self.loaded, now=self.now)

    def test_frames_units_and_calibration_identity_rejected(self):
        for key, value in (('camera_frame','wrong'), ('point_cloud_frame','cell'), ('units','mm'), ('calibration_sha256','changed')):
            with self.subTest(key=key):
                with self.assertRaises(ValueError): validate_observation(dict(self.meta, **{key:value}), self.loaded, now=self.now)

    def test_repeat_and_replay_cannot_authorize_live(self):
        with self.assertRaises(RuntimeError):
            validate_observation(self.meta, self.loaded, now=self.now, previous_frame=self.meta)
        duplicate = dict(self.meta, capture_id='new-id')
        with self.assertRaises(RuntimeError):
            validate_observation(duplicate, self.loaded, now=self.now, previous_frame=self.meta)
        replay = dict(self.meta, replay=True)
        with self.assertRaises(ValueError): validate_observation(replay, self.loaded, now=self.now)
        replay['captured_at_utc'] = '2000-01-01T00:00:00Z'
        result = validate_observation(replay, self.loaded, replay=True)
        self.assertTrue(result['replay'])
        self.assertFalse(result['execution_allowed'])

    def test_live_repeat_is_rejected_across_separate_estimate_calls(self):
        source = self.root/'live.npz'
        save_capture(source,dict(points_camera_m=np.zeros((150,3))),self.meta)
        with patch('isaac_port.fieldkit.perception.CadPoseEstimator') as cls:
            cls.return_value.models = {'pin':np.zeros((150,3))}
            cls.return_value.fit.return_value.as_dict.return_value = dict(T_cell_part=translated(.1,.2,.3))
            ledger = self.root/'ledger.json'
            first = estimate_snapshot(source,self.config_path,self.root/'first.json',parts=['pin'],ledger=ledger)
            self.assertFalse(first['execution_allowed'])
            with self.assertRaisesRegex(RuntimeError,'Repeated'):
                estimate_snapshot(source,self.config_path,self.root/'second.json',parts=['pin'],ledger=ledger)
            self.assertFalse((self.root/'second.json').exists())

    def observer_stream(self):
        outer = self
        class Stream:
            def __init__(self, config): self.index = 0
            def __enter__(self): return self
            def __exit__(self, *_): outer.stream_closed = True
            def read(self):
                self.index += 1
                return dict(points_camera_m=np.zeros((150,3))),dict(outer.meta,capture_id=f'live-{self.index}',
                             frame_number=self.index,hardware_timestamp_ms=1000+self.index)
        return Stream

    def test_persistent_observer_publishes_two_fresh_samples_and_logs(self):
        emitted = []
        def estimate(snapshot, config, output, **kwargs):
            meta = json.loads(snapshot.with_suffix('.json').read_text())
            validate_observation(meta,self.loaded)
            r = dict(state='seeded_cad_pose_proposals',capture_id=meta['capture_id'],source_metadata=meta,
                     proposals={'pin':{}},replay=False,hardware_ready=False,execution_allowed=False)
            output.write_text(json.dumps(r))
            emitted.append(r['capture_id'])
            return r
        latest, session = self.root/'latest.json', self.root/'session'
        r = run_observer(self.config_path,latest,session,parts=['pin'],frames=2,
                         stream_factory=self.observer_stream(), estimate_fn=estimate,estimator_factory=lambda:object())
        self.assertEqual(emitted,['live-1','live-2'])
        self.assertEqual(r['published'],2)
        self.assertEqual(json.loads(latest.read_text())['capture_id'],'live-2')
        self.assertEqual(len(list(session.glob('capture-*.npz'))),2)
        self.assertTrue(self.stream_closed)

    def test_persistent_observer_stale_fit_replaces_old_target_with_rejection(self):
        latest, session = self.root/'latest.json', self.root/'session'
        latest.write_text(json.dumps({'state':'seeded_cad_pose_proposals','proposals':{'pin':{}}}))
        def estimate(snapshot, config, output, **kwargs):
            raise RuntimeError('Observation stale after CAD fit; recapture')
        r = run_observer(self.config_path,latest,session,frames=1,
                         stream_factory=self.observer_stream(), estimate_fn=estimate,estimator_factory=lambda:object())
        result = json.loads(latest.read_text())
        self.assertEqual(result['state'],'observation_rejected')
        self.assertNotIn('proposals',result)
        self.assertEqual(r['rejected'],1)
        self.assertTrue(self.stream_closed)

    def test_persistent_observer_ctrl_c_clears_target_and_closes_stream(self):
        def estimate(*args,**kwargs): raise KeyboardInterrupt()
        latest = self.root/'latest.json'
        r = run_observer(self.config_path,latest,self.root/'session',frames=1,
                         stream_factory=self.observer_stream(),estimate_fn=estimate,estimator_factory=lambda:object())
        self.assertEqual(r['state'],'observer_stopped')
        self.assertNotIn('proposals',json.loads(latest.read_text()))
        self.assertTrue(self.stream_closed)

    def test_transform_direction_is_inverse_base_times_cell(self):
        p = target_proposals({'pin':dict(T_cell_part=translated(1.2,.3,.4))}, self.loaded, self.meta)['pin']
        np.testing.assert_allclose(np.array(p['T_base_part'])[:3,3], [.2,.3,.4])
        self.assertIn('T_part_tcp', p['grasp_transform_required'])
        self.assertFalse(p['execution_allowed'])

    def test_sdk_snapshot_is_compatible_with_fieldkit_loader(self):
        from isaac_port.fieldkit.perception import load_snapshot
        output = self.root/'capture.npz'
        save_capture(output, dict(points_camera_m=np.array([[.1,.2,1.]])), self.meta)
        p, labels, metadata = load_snapshot(output, self.loaded, replay=True)
        self.assertEqual(p.shape, (1,3))
        self.assertIsNone(labels)
        self.assertEqual(metadata['capture_id'], 'capture-a')

    def test_replay_end_to_end_near_seed_cad_fit_is_always_readonly(self):
        from isaac_port.fieldkit.perception import ASSETS
        with np.load(ASSETS/'part-models.npz') as models:
            rng = np.random.default_rng(9)
            pts, labels = [], []
            for label, name in enumerate(('ring1','ring2','pin'),1):
                self.c['nominal_stock_poses'][name] = translated(label*.2, 0, .5)
                source = models[name+'_points']
                subset = source[rng.choice(len(source), min(1800,len(source)), replace=False)]
                cell = subset + [label*.2,0,.5]
                cam = cell - [.2,0,1]
                pts.append(cam)
                labels.extend([label]*len(cam))
        self.write_config()
        c = load_config(self.config_path)
        # Deliberately claim live in stored metadata. CLI replay remains read-only.
        m = dict(self.meta, calibration_sha256=c['_calibration_sha256'], captured_at_utc='2000-01-01T00:00:00Z')
        source = self.root/'replay.npz'
        save_capture(source,dict(points_camera_m=np.concatenate(pts),labels=np.array(labels)),m)
        ledger = self.root/'live-ledger.json'
        r = estimate_snapshot(source,self.config_path,self.root/'poses.json',replay=True,ledger=ledger)
        self.assertTrue(r['replay'])
        self.assertFalse(r['execution_allowed'])
        self.assertFalse(ledger.exists())
        for name,p in r['proposals'].items():
            np.testing.assert_allclose(p['T_cell_part'], self.c['nominal_stock_poses'][name], atol=1e-4)
        single = estimate_snapshot(source,self.config_path,self.root/'one-part.json',replay=True,parts=['ring2'])
        self.assertEqual(set(single['proposals']), {'ring2'})


if __name__ == '__main__':
    unittest.main()
