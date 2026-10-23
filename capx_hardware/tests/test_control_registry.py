"""Contract integration tests only. Fake callbacks do not qualify robot motion."""
import copy
import time
import unittest

import numpy as np

from fr3_capx import (CallbackBackend, ControlApi, ExecutionLogger, ApiRegistry,
                     StaleProof, UnsupportedOperation, ValidationRejected,
                     build_registry, object_target_to_hand)
from fr3_capx.perception import pose_from_xyz_wxyz
from fr3_capx.backend import digest


class NativeHarness:
    """In-memory callback harness, explicitly not a physical collision checker."""
    def __init__(self):
        self.state = dict(source_id="test-source", scene_id="test-scene", config_id="test-config",
                          q={"left": [0.0]*7, "right": [0.0]*7},
                          opening_m={"left": 0.08, "right": 0.08}, held_part={"left": None, "right": None})
        self.plans, self.checked, self.executed = [], [], []
        self.reject = False
        self.plan_override = None
        self.reading = dict(timestamp_s=time.time(), confirmed=False,
                            provenance=dict(kind="rgbd", source="rendered_rgbd", oracle=False))
        self.backend = CallbackBackend(
            {"plan_motion": self.plan, "get_registered_pose": self.registered,
             "check_grasp": self.measurement,
             "check_retention": self.measurement,
             "check_completion": self.measurement,
             "get_checkpoint": self.measurement,
             "get_gripper_pose": lambda p: dict(self.measurement(p), frame="world", tool_frame="native_hand", pose=np.eye(4).tolist())},
            state_callback=lambda: copy.deepcopy(self.state), checker_callback=self.check,
            execute_callback=self.execute, source_id="test-source", scene_id="test-scene",
            config_id="test-config", joint_bounds={s: [[-3]*7, [3]*7] for s in ("left", "right")})

    def plan(self, request):
        self.plans.append(copy.deepcopy(request))
        if self.plan_override is not None:
            return copy.deepcopy(self.plan_override)
        q = copy.deepcopy(self.state["q"][request["side"]])
        target = request.get("target_q", [x + 0.01 for x in q])
        return dict(trajectory_q=[q, target], time_s=[0.0, 1.0])

    def registered(self, request):
        return dict(request, q=[0.1]*7)

    def measurement(self, request):
        return dict(request, **copy.deepcopy(self.reading))

    def check(self, request):
        self.checked.append(copy.deepcopy(request))
        return dict(approved=not self.reject, collision_free=not self.reject,
                    continuous_path_checked=True, fr3_limits_checked=True,
                    checker_id="TEST-ONLY-no-physical-check", binding=request["binding"],
                    actual_command_start_bridge_checked=True, tracking_envelope_checked=True,
                    issued_array_fr3_checked=True, tracking_contract_id=self.state.get("tracking_contract_id"),
                    target_request_checked=True,
                    target_request_digest=digest(request["motion"].get("target_request")))

    def execute(self, request):
        self.executed.append(copy.deepcopy(request))
        motion = request["motion"]
        if motion["kind"] == "joint_trajectory":
            self.state["q"][motion["side"]] = motion["trajectory_q"][-1]
        else:
            self.state["opening_m"][motion["side"]] = motion["width_m"]
        return dict(completed=True, fixture="TEST-ONLY", hold_confirmed=False)


class ControlRegistryTests(unittest.TestCase):
    def setUp(self):
        self.h = NativeHarness()
        self.log = ExecutionLogger()
        self.api = ControlApi(self.h.backend, side="right", logger=self.log)

    def test_joint_plan_is_checked_before_single_native_execution(self):
        result = self.api.goto_joint_position([0.2]*7)
        self.assertTrue(result["completed"])
        self.assertEqual(len(self.h.checked), 1)
        self.assertEqual(len(self.h.executed), 1)
        self.assertEqual(self.h.executed[0]["motion"]["side"], "right")
        self.assertEqual(self.h.state["q"]["left"], [0.0]*7)

    def test_planner_claim_cannot_override_native_collision_rejection(self):
        self.h.reject = True
        with self.assertRaises(ValidationRejected):
            self.api.goto_preinsert("fixed-pin-q")
        self.assertEqual(self.h.executed, [])

    def test_wrong_joint_endpoint_and_arm_do_not_reach_checker(self):
        for plan in [dict(trajectory_q=[[0]*7, [0.3]*7], time_s=[0, 1]),
                     dict(trajectory_q=[[0]*7, [0.2]*7], time_s=[0, 1], side="left")]:
            self.h.plan_override = plan
            with self.assertRaises(ValidationRejected):
                self.api.goto_joint_position([0.2]*7)
        self.assertEqual(self.h.checked, [])
        self.assertEqual(self.h.executed, [])

    def test_stale_planner_start_is_not_executed(self):
        self.h.plan_override = dict(trajectory_q=[[0.1]*7, [0.2]*7], time_s=[0, 1])
        with self.assertRaises(ValidationRejected):
            self.api.goto_joint_position([0.2]*7)
        self.assertFalse(self.h.executed)

    def test_cannot_move_without_connected_planner(self):
        api = ControlApi(CallbackBackend(), side="left")
        with self.assertRaises(UnsupportedOperation):
            api.goto_pose([0.3, 0.2, 1.0], [1, 0, 0, 0])

    def test_native_command_reference_preserves_original_issued_array(self):
        self.h.state["command_q"] = {"left": [0.0]*7, "right": [0.0005]*7}
        self.h.state["tracking_contract_id"] = "TEST-FIXED-CONTROLLER-ENVELOPE"
        original = [[0.0005]*7, [0.2]*7]
        self.h.plan_override = dict(trajectory_q=original, time_s=[0, 1],
            start_policy="native_command_reference", actual_start_q=[0.0]*7, command_start_q=[0.0005]*7)
        self.api.goto_joint_position([0.2]*7)
        self.assertEqual(self.h.executed[0]["motion"]["trajectory_q"], original)
        self.assertEqual(len(self.h.executed[0]["motion"]["trajectory_q"]), 2)

    def test_tool_approach_rotates_with_target_and_keeps_final_goal(self):
        root2 = np.sqrt(0.5)
        self.api.goto_pose([0.3, 0.2, 1.0], [root2, 0, root2, 0], z_approach=0.04)
        request = self.h.plans[0]
        approach, final = np.asarray(request["cartesian_waypoints_world"])
        np.testing.assert_allclose(approach[:3, 3], [0.26, 0.2, 1.0], atol=1e-10)
        np.testing.assert_allclose(final[:3, 3], [0.3, 0.2, 1.0], atol=1e-10)
        self.assertTrue(request["final_approach_linear"])
        self.assertEqual(request["tool_frame"], "native_hand")

    def test_registered_readiness_requires_explicit_name_and_no_home_export(self):
        self.api.goto_registered_pose("cell-ready-right")
        self.assertEqual(self.h.state["q"]["right"], [0.1]*7)
        self.assertNotIn("home_pose", self.api.functions())
        self.assertNotIn("goto_home_joint_position", self.api.functions())

    def test_closure_feedback_does_not_confirm_hold(self):
        result = self.api.close_gripper(width_m=0.012, force_n=20)
        self.assertFalse(result["hold_confirmed"])
        self.assertFalse(self.api.check_grasp("pin")["confirmed"])
        self.assertEqual(self.h.state["held_part"]["right"], None)

    def test_retention_rejects_stale_or_unmeasured_feedback(self):
        self.h.reading["timestamp_s"] = time.time() - 10
        with self.assertRaises(ValidationRejected):
            self.api.check_retention("pin")
        self.h.reading["timestamp_s"] = time.time()
        self.h.reading["provenance"]["oracle"] = True
        with self.assertRaises(ValidationRejected):
            self.api.breakpoint_code_block()

    def test_measured_hand_pose_uses_palm_frame(self):
        xyz, wxyz = self.api.get_gripper_pose()
        np.testing.assert_array_equal(xyz, [0, 0, 0])
        np.testing.assert_array_equal(wxyz, [1, 0, 0, 0])

    def test_checkpoint_cannot_approve_other_arm_part_or_label(self):
        for override, call in [({"side": "left"}, lambda: self.api.check_grasp("pin")),
                               ({"part_name": "ring_large"}, lambda: self.api.check_retention("pin")),
                               ({"label": "other-stage"}, lambda: self.api.breakpoint_code_block("post-lift"))]:
            self.h.reading.update(override)
            try:
                with self.assertRaises(ValidationRejected):
                    call()
            finally:
                for key in override:
                    self.h.reading.pop(key)

    def test_registry_side_prefix_and_dispatch_no_eval(self):
        registry = build_registry(self.h.backend, logger=self.log)
        registry.call("left_goto_joint_position", {"joints": [0.05]*7})
        self.assertEqual(self.h.state["q"]["left"], [0.05]*7)
        self.assertEqual(self.h.state["q"]["right"], [0.0]*7)
        self.assertEqual(len(registry.functions()), 44)
        self.assertIn("tool_frame", registry.combined_doc())
        for name in ("goto_pose", "home_pose", "__import__('os')"):
            with self.assertRaises(KeyError):
                registry.call(name, {})
        with self.assertRaises(TypeError):
            registry.call("left_goto_joint_position", {"wrong_parameter": []})

    def test_duplicate_registration_is_atomic(self):
        registry = ApiRegistry().register(self.api)
        before = registry.functions()
        with self.assertRaises(ValueError):
            registry.register(self.api)
        self.assertEqual(before, registry.functions())

    def test_rigid_grasp_target_composition(self):
        grasp = pose_from_xyz_wxyz([0.01, -0.002, 0.03], [np.sqrt(0.5), 0, 0, np.sqrt(0.5)])
        goal = pose_from_xyz_wxyz([0.2, 0.3, 1.1], [0, 1, 0, 0])
        hand = object_target_to_hand(goal, grasp)
        np.testing.assert_allclose(hand @ grasp, goal, atol=1e-12)
        with self.assertRaises(ValueError):
            object_target_to_hand(np.zeros((4, 4)), grasp)


if __name__ == "__main__":
    unittest.main()
