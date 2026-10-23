"""CPU-only perception contracts; test doubles are not model qualifications."""
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np

from fr3_capx.backend import CallbackBackend, RecordedBackend, ValidationRejected
from fr3_capx.perception import (
    PerceptionApi, PerceptionError, PerceptionUnavailable,
    deproject_pixel_to_camera, depth_color_to_pointcloud, depth_to_pointcloud,
    obs_get_rgb, pose_from_xyz_wxyz, pose_to_xyz_wxyz, validate_pose,
)


def fixture():
    """Synthetic optics fixture; the timestamp uses a controlled offline clock."""
    return {"rgb": np.arange(45, dtype=np.uint8).reshape(3, 5, 3),
            "depth_m": np.ones((3, 5)), "intrinsics": np.array([[2., 0, 2], [0, 2., 1], [0, 0, 1]]),
            "T_world_camera": pose_from_xyz_wxyz([.2, -.3, 1.], [0, 1, 0, 0]),
            "frame": "camera_optical", "depth_unit": "m", "rectified": True, "rgb_depth_aligned": True,
            "calibration_id": "calibration-fixture-v1", "observation_id": "image-fixture-001",
            "timestamp_s": 1000., "provenance": {"source": "recorded_rgbd", "oracle": False}}


def derived(obs, **fields):
    return {key: deepcopy(obs[key]) for key in ("calibration_id", "observation_id", "timestamp_s", "provenance")} | fields


class GeometryTests(unittest.TestCase):
    def test_pose_roundtrip_wxyz_and_transform_direction(self):
        matrix = pose_from_xyz_wxyz([.2, -.3, .4], [np.sqrt(.5), 0, 0, np.sqrt(.5)])
        xyz, wxyz = pose_to_xyz_wxyz(matrix)
        np.testing.assert_allclose(pose_from_xyz_wxyz(xyz, wxyz), matrix, atol=1e-15)
        np.testing.assert_allclose((matrix @ [1, 0, 0, 1])[:3], [.2, .7, .4], atol=1e-15)

    def test_nonrigid_reflection_bad_bottom_row_and_nonunit_quaternion_rejected(self):
        bad = []
        reflection = np.eye(4); reflection[0, 0] = -1; bad.append(reflection)
        scale = np.eye(4); scale[0, 0] = 2; bad.append(scale)
        bottom = np.eye(4); bottom[3, 0] = .01; bad.append(bottom)
        invalid = np.eye(4); invalid[0, 0] = np.nan; bad.append(invalid)
        bad.extend([np.eye(3), [0] * 16])
        for matrix in bad:
            with self.subTest(matrix=matrix), self.assertRaises(PerceptionError):
                validate_pose(matrix)
        for quat in ([2, 0, 0, 0], [0, 0, 0, 0], [0, 0, 1], [np.nan, 0, 0, 0]):
            with self.subTest(quat=quat), self.assertRaises(PerceptionError):
                pose_from_xyz_wxyz([0, 0, 0], quat)

    def test_odd_dimension_subsampling_and_color_alignment(self):
        obs = fixture()
        cloud = depth_to_pointcloud(obs["depth_m"], obs["intrinsics"], subsample_factor=2)
        self.assertEqual(cloud.shape, (6, 3))
        np.testing.assert_allclose(cloud[0], [-1, -.5, 1])
        np.testing.assert_allclose(cloud[-1], [1, .5, 1])
        points, colors = depth_color_to_pointcloud(obs["depth_m"], obs["rgb"], obs["intrinsics"], 2)
        np.testing.assert_array_equal(points, cloud)
        np.testing.assert_allclose(colors[-1], obs["rgb"][-1, -1]/255.)

    def test_invalid_depth_filtering_and_pixel_projection(self):
        obs = fixture(); depth = obs["depth_m"].copy()
        depth.ravel()[:4] = [np.nan, np.inf, 0, -1]
        points = depth_to_pointcloud(depth, obs["intrinsics"])
        self.assertEqual(len(points), 11)
        all_points = depth_to_pointcloud(depth, obs["intrinsics"], filter_invalid=False)
        self.assertEqual(len(all_points), 15)
        np.testing.assert_allclose(deproject_pixel_to_camera((2, 1), .7, obs["intrinsics"]), [0, 0, .7])
        for factor in (0, -1, 1.5, True):
            with self.subTest(factor=factor), self.assertRaises(PerceptionError):
                depth_to_pointcloud(depth, obs["intrinsics"], factor)
        with self.assertRaises(PerceptionError):
            deproject_pixel_to_camera((2, 1), np.nan, obs["intrinsics"])


class PerceptionTests(unittest.TestCase):
    def setUp(self):
        self.obs = fixture()
        self.now = 1000.
        self.responses = {
            "capture": self.obs,
            "get_object_pose": derived(self.obs, frame="world", orientation_observed=True,
                position=[.1, .2, .3], quaternion_wxyz=[1, 0, 0, 0], bbox_extent=[.01, .02, .03]),
            "sample_grasp_pose": derived(self.obs, frame="world", tool_frame="native_hand", pose=np.eye(4),
                hold_confirmed=False, proposal_only=True),
            "segment": derived(self.obs, results=[{"mask": np.ones((3, 5), dtype=np.uint8), "score": .9, "label": "pin", "box": [0, 0, 5, 3]}]),
            "detect": derived(self.obs, detections=[{"label": "pin", "box": [1, 0, 4, 3], "score": .8}]),
            "point": derived(self.obs, points={"pin": [2, 1]}),
            "plan_grasps": derived(self.obs, frame="camera_optical", tool_frame="native_hand",
                grasps=np.eye(4)[None], scores=[.8], contact_pts=[[0, 0, 1]], hold_confirmed=False),
        }
        self.responses["plan_grasps_point_clouds"] = deepcopy(self.responses["plan_grasps"])
        self.requests = []
        def callback(operation):
            def respond(payload):
                self.requests.append((operation, payload))
                return deepcopy(self.responses[operation])
            return respond
        self.backend = CallbackBackend({op: callback(op) for op in self.responses})
        self.api = PerceptionApi(self.backend, clock=lambda: self.now)

    def test_capture_through_real_callback_json_boundary(self):
        image = self.api.capture()
        self.assertEqual(image["rgb"].dtype, np.uint8)
        self.assertEqual(image["depth_m"].shape, (3, 5))
        np.testing.assert_array_equal(image["rgb"], self.obs["rgb"])
        image["rgb"][:] = 0
        image["provenance"]["oracle"] = True
        np.testing.assert_array_equal(self.api._current()["rgb"], self.obs["rgb"])
        self.assertFalse(self.api._current()["provenance"]["oracle"])

    def test_capture_rejects_wrong_units_frames_and_calibration(self):
        for changes in ({"depth_unit": "mm"}, {"frame": "world"}, {"rectified": False},
                        {"rgb_depth_aligned": False}, {"calibration_id": ""},
                        {"intrinsics": [[0, 0, 2], [0, 2, 1], [0, 0, 1]]},
                        {"T_world_camera": np.eye(3)}, {"depth_m": [[1000.] * 5] * 3},
                        {"rgb": [[[256] * 3] * 5] * 3}):
            before = deepcopy(self.responses["capture"])
            self.responses["capture"].update(changes)
            with self.subTest(changes=changes), self.assertRaises(PerceptionError):
                self.api.capture()
            self.responses["capture"] = before

    def test_stale_future_missing_timestamp_and_lineage_rejected(self):
        for changes in ({"timestamp_s": 997.9}, {"timestamp_s": 1000.2}, {"timestamp_s": None},
                        {"observation_id": ""}, {"provenance": {"source": "sim_body_pose", "oracle": False}},
                        {"provenance": {"source": "rendered_rgbd", "oracle": True}},
                        {"provenance": {"source": "rendered_rgbd", "oracle": False, "kind": "oracle"}}):
            before = deepcopy(self.responses["capture"])
            self.responses["capture"].update(changes)
            with self.subTest(changes=changes), self.assertRaises((PerceptionError, ValidationRejected)):
                self.api.capture()
            self.responses["capture"] = before

    def test_pose_return_contract_and_invalid_orientation(self):
        xyz, wxyz, extent = self.api.get_object_pose("pin")
        np.testing.assert_allclose(xyz, [.1, .2, .3])
        np.testing.assert_allclose(wxyz, [1, 0, 0, 0])
        self.assertIsNone(extent)
        np.testing.assert_allclose(self.api.get_object_pose("pin", True)[2], [.01, .02, .03])
        self.responses["get_object_pose"]["orientation_observed"] = False
        with self.assertRaises(PerceptionError):
            self.api.get_object_pose("pin")

    def test_derived_evidence_must_match_current_image_and_calibration(self):
        for field, value in (("observation_id", "old-image"), ("calibration_id", "other-calibration"), ("timestamp_s", 999.9)):
            before = self.responses["get_object_pose"][field]
            self.responses["get_object_pose"][field] = value
            with self.subTest(field=field), self.assertRaises(PerceptionError):
                self.api.get_object_pose("pin")
            self.responses["get_object_pose"][field] = before

    def test_native_hand_candidate_is_not_hold_confirmation(self):
        xyz, wxyz = self.api.sample_grasp_pose("pin")
        np.testing.assert_array_equal(xyz, [0, 0, 0])
        np.testing.assert_array_equal(wxyz, [1, 0, 0, 0])
        self.assertFalse(self.api.last_grasp_candidate["hold_confirmed"])
        self.assertTrue(self.api.last_grasp_candidate["proposal_only"])
        for changes in ({"tool_frame": "panda_tcp"}, {"frame": "camera_optical"}, {"hold_confirmed": True}):
            before = deepcopy(self.responses["sample_grasp_pose"])
            self.responses["sample_grasp_pose"].update(changes)
            with self.subTest(changes=changes), self.assertRaises(PerceptionError):
                self.api.sample_grasp_pose("pin")
            self.responses["sample_grasp_pose"] = before
        self.assertTrue(all(op in ("capture", "sample_grasp_pose") for op, _ in self.requests))

    def test_transform_uses_saved_calibration_and_rejects_unknown_frame(self):
        camera_pose = pose_from_xyz_wxyz([0, 0, .2], [1, 0, 0, 0])
        world_pose = self.api.transform_pose(camera_pose, "camera_optical", "world")
        np.testing.assert_allclose(world_pose[:3, 3], [.2, -.3, .8])
        np.testing.assert_allclose(self.api.transform_pose(world_pose, "world", "camera_optical"), camera_pose)
        with self.assertRaises(PerceptionError):
            self.api.transform_pose(world_pose, "base", "world")
        self.now += 2.1
        with self.assertRaises(PerceptionError):
            self.api.transform_pose(world_pose, "world", "camera_optical")

    def test_observation_compatibility(self):
        obs = self.api.get_observation("robot0_robotview")
        self.assertEqual(obs["robot0_robotview"]["images"]["depth"].shape, (3, 5, 1))
        np.testing.assert_array_equal(obs_get_rgb(obs)["robot0_robotview"], self.obs["rgb"])
        self.assertEqual(len(obs["robot0_robotview"]["pose"]), 7)

    def test_segmentation_compatibility_sam2_sam3_and_point_prompts(self):
        rgb = self.api.capture()["rgb"]
        self.assertTrue(self.api.init_sam3()(rgb, "pin")[0]["mask"].all())
        self.assertTrue(self.api.init_sam2()(rgb, box=[0, 0, 5, 3])[0]["mask"].all())
        rows = self.api.init_sam3_point_prompt()(rgb, (2, 1))
        self.assertEqual(rows[0]["score"], .9)
        scores, masks = self.api.init_sam2_point_prompt()(rgb, (2, 1))
        self.assertEqual(scores, [.9]); self.assertEqual(masks.shape, (1, 3, 5))
        self.assertEqual(masks.dtype, bool)
        changed = rgb.copy(); changed[0, 0, 0] += 1
        with self.assertRaises(PerceptionError):
            self.api.segment(changed, "pin")
        self.responses["segment"]["results"][0]["mask"] = np.full((3, 5), 2, dtype=int)
        with self.assertRaises(PerceptionError):
            self.api.segment(rgb, "pin")

    def test_owlvit_and_molmo_client_contracts(self):
        rgb = self.api.capture()["rgb"]
        self.assertEqual(self.api.init_owlvit()(rgb, [["pin"]])[0]["label"], "pin")
        self.assertEqual(self.api.init_owlvit(threshold=.9)(rgb, [["pin"]]), [])
        self.assertEqual(self.api.init_molmo()(rgb, ["pin"]), {"pin": (2., 1.)})
        self.responses["point"]["points"]["pin"] = [5, 1]
        with self.assertRaises(PerceptionError):
            self.api.init_molmo()(rgb, ["pin"])
        with self.assertRaises(PerceptionError):
            self.api.init_molmo(api_key="not-transmitted")
        self.responses["detect"]["detections"][0]["box"] = [-1, 0, 4, 3]
        with self.assertRaises(PerceptionError):
            self.api.init_owlvit()(rgb, [["pin"]])

    def test_contact_graspnet_client_returns_explicit_mapped_candidates(self):
        obs = self.api.capture()
        plan = self.api.init_contact_graspnet()
        args = (obs["depth_m"], obs["intrinsics"], np.ones((3, 5), dtype=int), 1)
        grasps, scores, contacts = plan(*args)
        self.assertEqual(grasps.shape, (1, 4, 4)); self.assertEqual(scores.shape, (1,))
        np.testing.assert_array_equal(contacts, [[0, 0, 1]])
        self.responses["plan_grasps"]["tool_frame"] = "network_gripper"
        with self.assertRaises(PerceptionError):
            plan(*args)

    def test_point_cloud_grasp_rejects_unrelated_geometry(self):
        obs = self.api.capture()
        points = depth_to_pointcloud(obs["depth_m"], obs["intrinsics"])
        plan = self.api.init_contact_graspnet_point_clouds()
        self.assertEqual(plan(points, points[:3])[0].shape, (1, 4, 4))
        with self.assertRaises(PerceptionError):
            plan(points + [0, 0, .01], points[:3])
        with self.assertRaises(PerceptionError):
            plan(points, points[:3] + [0, 0, .01])

    def test_model_service_unavailable_is_an_explicit_exception(self):
        api = PerceptionApi(CallbackBackend(), clock=lambda: self.now)
        with self.assertRaises(PerceptionUnavailable):
            api.capture()

    def test_functions_docs_and_no_heavy_integrations_at_import(self):
        tools = self.api.functions()
        self.assertEqual(len(tools), 18)
        self.assertTrue(all(callable(fn) for fn in tools.values()))
        self.assertIn("sample_grasp_pose", self.api.combined_doc())
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
        env.pop("PYTHONPATH", None)
        check = subprocess.run([sys.executable, "-c", "import sys; import fr3_capx.perception; assert not {'torch','capx','omni','requests','transformers'} & set(sys.modules)"],
                               env=env, capture_output=True, text=True)
        self.assertEqual(check.returncode, 0, check.stderr)

    def test_recorded_backend_supports_sensor_data_without_motion(self):
        # Convert native arrays through JSON, exactly like a remote observation.
        observation = json.loads(json.dumps(self.obs, default=lambda x: x.tolist()))
        backend = RecordedBackend({"observations": {"capture": observation}})
        api = PerceptionApi(backend, clock=lambda: self.now)
        np.testing.assert_array_equal(api.capture()["rgb"], self.obs["rgb"])
        with self.assertRaises(PerceptionUnavailable):
            api.get_object_pose("pin")


if __name__ == "__main__":
    unittest.main()
