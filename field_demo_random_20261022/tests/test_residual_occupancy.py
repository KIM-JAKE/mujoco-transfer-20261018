"""Sensor-only residual occupancy invariants; synthetic pixels, no Isaac truth."""
from datetime import datetime, timedelta, timezone
import copy
import unittest
import numpy as np
from field_demo.residual_occupancy import build_occupancy, OccupancySnapshot, UNKNOWN, FREE, OCCUPIED

NOW = datetime(2026, 10, 21, 17, 0, tzinfo=timezone.utc)
BOUNDS = [[-.04, -.04, .1], [.04, .04, .5]]


def camera(depth=None):
    return dict(depth_m=np.full((64, 64), .4) if depth is None else depth,
                K=np.array([[80., 0, 31.5], [0, 80., 31.5], [0, 0, 1.]]), T_cell_camera=np.eye(4))


def metadata(capture='image1'):
    return dict(capture_id=capture, captured_at_utc=NOW.isoformat(), calibration_sha256='a'*64,
                source_packet_sha256='b'*64, source='synthetic_camera_test', replay=True,
                scene_epoch='stationary_stock1', cell_frame='cell', rectified_registered_depth=True,
                projection_kind='rectified_pinhole', depth_uncertainty_m=.001)


def build(packet=None, meta=None, **kwargs):
    return build_occupancy(packet or camera(), meta or metadata(), BOUNDS, voxel_size_m=.02,
                           depth_margin_m=.003, now_utc=NOW, **kwargs)


def cell(snapshot, point):
    bounds = np.array(snapshot.bounds_cell_m); spacing = (bounds[1]-bounds[0])/snapshot.shape
    return tuple(np.floor((point-bounds[0])/spacing).astype(int))


class ResidualOccupancyTests(unittest.TestCase):
    def test_plane_certifies_only_observed_free_space(self):
        snapshot = build()
        self.assertEqual(snapshot.states[cell(snapshot, [0, 0, .2])], FREE)
        self.assertEqual(snapshot.states[cell(snapshot, [0, 0, .41])], OCCUPIED)
        self.assertEqual(snapshot.states[cell(snapshot, [0, 0, .47])], UNKNOWN)
        with self.assertRaises(ValueError): snapshot.states[0, 0, 0] = FREE

    def test_holes_and_narrow_foreground_never_become_free(self):
        depth = camera()['depth_m']; depth[30:35, 30:35] = np.nan
        missing = build(camera(depth)); self.assertEqual(missing.states[cell(missing, [0, 0, .2])], UNKNOWN)
        depth = camera()['depth_m']; depth[31, 31] = .23
        foreground = build(camera(depth))
        # A single foreground pixel inside the projected footprint prevents
        # free classification, even when the eight corner rays see the floor.
        self.assertEqual(foreground.states[cell(foreground, [0, 0, .3])], UNKNOWN)

    def test_rectified_nearest_ray_padding_conservatively_keeps_nearby_hits(self):
        depth = camera()['depth_m']; depth[27, 27] = .23
        direct = build(camera(depth)); meta = metadata(); meta['ray_footprint_padding_px'] = 2
        padded = build(camera(depth), meta)
        index = cell(direct, [.005, .005, .3])
        self.assertEqual(direct.states[index], FREE)
        self.assertEqual(padded.states[index], UNKNOWN)
        self.assertEqual(padded.sources[0]['ray_footprint_padding_px'], 2)
        self.assertTrue(np.all(direct.states[padded.states == FREE] == FREE))
        for invalid in (5, -1, 1.5, True):
            meta['ray_footprint_padding_px'] = invalid
            with self.assertRaises(ValueError): build(camera(depth), meta)

    def test_sdk_endpoints_use_actual_organized_rays(self):
        depth = np.full((64, 64), np.nan); depth[32, 31] = .4
        packet = camera(depth); sdk = np.full((64, 64, 3), np.nan)
        sdk[32, 31] = [.005, .0025, .4]  # authentic ray1.5pixels from nominal
        packet['points_camera_image_m'] = sdk
        meta = metadata(); meta['ray_footprint_padding_px'] = 2
        actual = build(packet, meta)
        self.assertEqual(actual.states[cell(actual, [.005, .0025, .4])], OCCUPIED)
        self.assertEqual(actual.states[cell(actual, [-.0025, .0025, .4])], UNKNOWN)
        meta['ray_footprint_padding_px'] = 0
        with self.assertRaises(ValueError): build(packet, meta)
        meta['ray_footprint_padding_px'] = 2; sdk[32, 31, 2] = .39
        with self.assertRaises(ValueError): build(packet, meta)

    def test_view_fusion_is_fresh_same_epoch_and_never_erases_hits(self):
        depth = camera()['depth_m']; depth[30:35, 30:35] = np.nan
        first = build(camera(depth)); second = build(meta=metadata('image2'), prior=first)
        self.assertEqual(second.states[cell(second, [0, 0, .2])], FREE)
        hit = camera()['depth_m']; hit[30:35, 30:35] = .23
        occupied = build(camera(hit)); fused = build(meta=metadata('image2'), prior=occupied)
        self.assertTrue(np.all(fused.states[occupied.states == OCCUPIED] == OCCUPIED))
        changed = metadata('image3'); changed['scene_epoch'] = 'stock_moved'
        with self.assertRaises(ValueError): build(meta=changed, prior=second)
        with self.assertRaises(ValueError): build(meta=metadata('image1'), prior=first)

    def test_selected_contact_does_not_clear_unknown_or_neighbours(self):
        packet = camera(); depth = packet['depth_m']; yy, xx = np.indices(depth.shape)
        surface = np.c_[(xx.ravel()-31.5)*.4/80, (yy.ravel()-31.5)*.4/80, np.full(depth.size, .4)]
        selected = dict(capture_id='image1', instance_id='image1:image:0', surface_mask=np.ones(depth.shape, bool),
                        cad_surface_points_cell_m=surface, contact_bounds_cell_m=[[-.04, -.04, .39], [.04, .04, .42]], shell_tolerance_m=.001)
        plain = build(packet); tagged = build(packet, selected_contact=selected)
        np.testing.assert_array_equal(plain.states, tagged.states)
        boxes = tagged.planner_boxes()
        self.assertTrue(any('selected_contact_instance_id' in b for b in boxes))
        self.assertTrue(all('selected_contact_instance_id' not in b for b in boxes if b['occupancy_kind'] == 'unknown'))
        # A second unselected endpoint in a shared coarse voxel vetoes the
        # entire contact exemption; no nearest-target whole-ball deletion.
        selected['surface_mask'][31, 31] = False
        mixed = build(packet, selected_contact=selected)
        point_cell = cell(mixed, [(-.5)*.4/80, (-.5)*.4/80, .4])
        self.assertEqual(np.frombuffer(mixed._selected, np.uint8).reshape(mixed.shape)[point_cell], 0)

    def test_exact_box_cover_has_no_free_voxels(self):
        snapshot = build(); boxes = snapshot.planner_boxes()
        centers = np.indices(snapshot.shape).reshape(3, -1).T
        bounds = np.array(snapshot.bounds_cell_m); spacing = (bounds[1]-bounds[0])/snapshot.shape
        centers = bounds[0]+(centers+.5)*spacing; covered = np.zeros(len(centers), int)
        for box in boxes:
            center = np.array(box['T_cell_box'])[:3, 3]; half = np.array(box['half_extents_m'])
            inside = (abs(centers-center) < half+1e-12).all(1); covered += inside
        self.assertTrue(np.all(covered[snapshot.states.ravel() == FREE] == 0))
        self.assertTrue(np.all(covered[snapshot.states.ravel() != FREE] == 1))

    def test_serialization_source_binding_age_and_projection(self):
        snapshot = build(); record = snapshot.to_record()
        restored = OccupancySnapshot.from_record(record, expected_capture_id='image1', expected_calibration_sha256='a'*64, now_utc=NOW)
        self.assertEqual(snapshot.fingerprint, restored.fingerprint)
        np.testing.assert_array_equal(snapshot.states, restored.states)
        for changed in ('capture', 'calibration', 'age', 'tamper'):
            with self.assertRaises(ValueError):
                if changed == 'capture': OccupancySnapshot.from_record(record, expected_capture_id='another', now_utc=NOW)
                elif changed == 'calibration': OccupancySnapshot.from_record(record, expected_calibration_sha256='c'*64, now_utc=NOW)
                elif changed == 'age': OccupancySnapshot.from_record(record, now_utc=NOW+timedelta(seconds=3))
                else:
                    bad = copy.deepcopy(record); bad['shape'][0] += 1; OccupancySnapshot.from_record(bad, now_utc=NOW)
        bad = metadata(); bad['rectified_registered_depth'] = False
        with self.assertRaises(ValueError): build(meta=bad)
        bad = metadata(); bad['depth_uncertainty_m'] = .004
        with self.assertRaises(ValueError): build(meta=bad)


if __name__ == '__main__': unittest.main()
