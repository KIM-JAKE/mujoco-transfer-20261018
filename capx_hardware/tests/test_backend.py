"""Boundary regression tests: no robot, simulator, sockets or GPU required."""
from copy import deepcopy
import unittest
from concurrent.futures import ThreadPoolExecutor

from fr3_capx.backend import CallbackBackend, RecordedBackend, StaleProof, UnsupportedOperation, ValidationRejected, digest


class BackendTests(unittest.TestCase):
    def setUp(self):
        self.time = 100.0
        self.state = {"source_id": "sha-source", "scene_id": "raised-248", "config_id": "native-v1",
                      "q": {"left": [0.] * 7, "right": [0.] * 7},
                      "opening_m": {"left": .08, "right": .02},
                      "held_part": {"left": None, "right": "pin"}, "calibration_revision": "cal-01"}
        self.executed = []
        self.checked = []
        def checker(request):
            self.checked.append(deepcopy(request))
            return {"approved": True, "collision_free": True, "continuous_path_checked": True,
                    "fr3_limits_checked": True, "checker_id": "native-full-mesh", "binding": request["binding"]}
        self.checker = checker
        self.backend = self.make_backend()
        self.motion = {"side": "right", "trajectory_q": [[0.] * 7, [.1] * 7], "time_s": [0., 2.], "speed_scale": .1}

    def make_backend(self, **kwargs):
        args = dict(state_callback=lambda: deepcopy(self.state), checker_callback=self.checker,
                    execute_callback=lambda request: self.executed.append(request) or {"executed": True},
                    source_id="sha-source", scene_id="raised-248", config_id="native-v1",
                    joint_bounds={side: [[-1.] * 7, [1.] * 7] for side in ("left", "right")},
                    clock=lambda: self.time)
        args.update(kwargs)
        return CallbackBackend(**args)

    def proof(self):
        return self.backend.call("validate_motion", deepcopy(self.motion))["proof_id"]

    def execute(self, proof_id):
        return self.backend.call("execute_motion", {"proof_id": proof_id})

    def test_approved_bound_motion_executes_once(self):
        proof_id = self.proof()
        self.assertTrue(self.execute(proof_id)["executed"])
        self.assertEqual(self.executed[0]["motion"]["trajectory_q"], self.motion["trajectory_q"])
        with self.assertRaises(StaleProof):
            self.execute(proof_id)
        self.assertEqual(len(self.executed), 1)

    def test_collision_rejected_never_executes(self):
        def blocked(request):
            result = self.checker(request)
            result["collision_free"] = False
            return result
        backend = self.make_backend(checker_callback=blocked)
        with self.assertRaises(ValidationRejected):
            backend.call("validate_motion", self.motion)
        self.assertEqual(self.executed, [])

    def test_planner_style_boolean_is_not_native_motion_qualification(self):
        backend = self.make_backend(checker_callback=lambda request: {"approved": True, "collision_free": True})
        with self.assertRaises(ValidationRejected):
            backend.call("validate_motion", self.motion)
        self.assertFalse(self.executed)

    def test_state_bindings_invalidate_proofs(self):
        for mutate in (
            lambda: self.state["q"]["left"].__setitem__(0, .01),
            lambda: self.state["opening_m"].__setitem__("right", .021),
            lambda: self.state["held_part"].__setitem__("right", None),
            lambda: self.state.__setitem__("calibration_revision", "cal-02"),
        ):
            with self.subTest(mutate=mutate):
                before = deepcopy(self.state)
                proof_id = self.proof()
                mutate()
                with self.assertRaises(StaleProof):
                    self.execute(proof_id)
                self.state = before
        self.assertFalse(self.executed)

    def test_expired_and_cross_backend_proofs_rejected(self):
        proof_id = self.proof()
        with self.assertRaises(StaleProof):
            self.make_backend().call("execute_motion", {"proof_id": proof_id})
        self.time += 5.0
        with self.assertRaises(StaleProof):
            self.execute(proof_id)
        self.assertFalse(self.executed)

    def test_source_scene_config_changes_rejected(self):
        for field in ("source_id", "scene_id", "config_id"):
            with self.subTest(field=field):
                before = deepcopy(self.state)
                proof_id = self.proof()
                self.state[field] = "changed"
                with self.assertRaises(ValidationRejected):
                    self.execute(proof_id)
                self.state = before
        self.assertFalse(self.executed)

    def test_state_changed_during_checker_does_not_issue_proof(self):
        def drifting(request):
            result = self.checker(request)
            self.state["q"]["right"][0] = .001
            return result
        backend = self.make_backend(checker_callback=drifting)
        with self.assertRaises(StaleProof):
            backend.call("validate_motion", self.motion)

    def test_checker_binding_must_match_actual_motion(self):
        def forged(request):
            result = self.checker(request)
            result["binding"]["motion_digest"] = "different"
            return result
        with self.assertRaises(ValidationRejected):
            self.make_backend(checker_callback=forged).call("validate_motion", self.motion)

    def test_targets_cannot_be_replaced_after_qualification(self):
        proof_id = self.proof()
        with self.assertRaises(ValidationRejected):
            self.backend.call("execute_motion", {"proof_id": proof_id, "trajectory_q": [[.9] * 7]})
        self.assertFalse(self.executed)

    def test_motion_shape_bounds_timing_and_speed_controls(self):
        changes = (
            {"trajectory_q": [[0.] * 6, [.1] * 6]},
            {"trajectory_q": [[0.] * 7, [2.] * 7]},
            {"trajectory_q": [[.01] * 7, [.1] * 7]},
            {"trajectory_q": [[0.] * 7, [float("nan")] * 7]},
            {"time_s": [0., 0.]}, {"time_s": [1., 2.]},
            {"time_s": [0., 61.]}, {"speed_scale": .3}, {"speed_scale": True},
        )
        for change in changes:
            with self.subTest(change=change), self.assertRaises(ValidationRejected):
                self.backend.call("validate_motion", {**self.motion, **change})
        self.assertFalse(self.checked)
        self.assertFalse(self.executed)

    def test_gripper_also_requires_checker_and_state_bound_proof(self):
        gripper = {"side": "right", "width_m": .012, "speed_m_s": .005, "force_n": 10., "speed_scale": .1}
        proof = self.backend.call("validate_gripper", gripper)
        self.backend.call("execute_gripper", {"proof_id": proof["proof_id"]})
        self.assertEqual(self.executed[-1]["motion"]["kind"], "gripper")
        with self.assertRaises(ValidationRejected):
            self.backend.call("validate_gripper", {**gripper, "force_n": 1000.})

    def test_unwired_motion_and_raw_bypasses_rejected(self):
        with self.assertRaises(UnsupportedOperation):
            CallbackBackend().call("validate_motion", self.motion)
        for operation in ("home_pose", "teleport", "set_joint_positions", "goto_pose", "open_gripper"):
            with self.subTest(operation=operation), self.assertRaises(UnsupportedOperation):
                self.backend.call(operation, {})
        with self.assertRaises(UnsupportedOperation):
            CallbackBackend({"goto_pose": lambda payload: {"executed": True}})

    def test_motion_proof_cannot_execute_as_gripper_and_is_consumed(self):
        proof_id = self.proof()
        with self.assertRaises(ValidationRejected):
            self.backend.call("execute_gripper", {"proof_id": proof_id})
        with self.assertRaises(StaleProof):
            self.execute(proof_id)
        self.assertFalse(self.executed)

    def test_camera_schema_explicitly_rejects_oracle(self):
        measured = {"observation_id": "frame-1", "provenance": {"source": "recorded_rgbd", "oracle": False}}
        backend = CallbackBackend({"capture": lambda payload: measured})
        self.assertEqual(backend.call("capture", {}), measured)
        measured["provenance"]["oracle"] = True
        with self.assertRaises(ValidationRejected):
            backend.call("capture", {})

    def test_observed_pose_is_available_but_oracle_pose_is_excluded(self):
        for value, allowed in (({"pose": [1, 2, 3], "provenance": {"kind": "rgbd", "oracle": False}}, True),
                               ({"pose": [1, 2, 3]}, False),
                               ({"ground_truth_pose": [1, 2, 3], "provenance": {"kind": "rgbd"}}, False),
                               ({"pose": [1, 2, 3], "provenance": {"kind": "rgbd", "privileged": True}}, False)):
            backend = CallbackBackend({"get_object_pose": lambda payload: value})
            if allowed:
                self.assertEqual(backend.call("get_object_pose", {"object_name": "pin"}), value)
            else:
                with self.assertRaises(ValidationRejected):
                    backend.call("get_object_pose", {"object_name": "pin"})

    def test_recorded_observations_do_not_become_live_state_or_motion(self):
        observation = {"pose": [0., 0., 1.], "provenance": {"kind": "rgbd", "oracle": False, "camera_id": "fixed-top"}}
        backend = RecordedBackend({"observations": {"get_object_pose": {"pin": observation}}, "full_task_pass": True})
        result = backend.call("get_object_pose", {"object_name": "pin"})
        self.assertFalse(result["recording"]["live"])
        for operation in ("get_state", "validate_motion", "execute_motion", "open_gripper", "get_completion"):
            with self.subTest(operation=operation), self.assertRaises(UnsupportedOperation):
                backend.call(operation, {})
        raw_pipeline = RecordedBackend({"full_task_pass": True, "sim_body_pose": [1, 2, 3]})
        with self.assertRaises(UnsupportedOperation):
            raw_pipeline.call("get_object_pose", {"object_name": "pin"})

    def test_invalid_depth_pixels_become_null_but_motion_nan_is_rejected(self):
        record = {"depth_m": [[1.0, float("nan")], [float("inf"), .7]],
                  "provenance": {"source": "rgbd", "oracle": False}}
        backend = CallbackBackend({"capture": lambda payload: record})
        self.assertEqual(backend.call("capture", {})["depth_m"], [[1.0, None], [None, .7]])
        record["timestamp_s"] = float("nan")
        with self.assertRaises(ValidationRejected):
            backend.call("capture", {})

    def test_implicit_or_malicious_provenance_is_rejected(self):
        for provenance in ({"kind": "rgbd"},
                           {"source": "rgbd", "oracle": False, "kind": "oracle"}):
            backend = CallbackBackend({"capture": lambda payload: {"provenance": provenance}})
            with self.assertRaises(ValidationRejected):
                backend.call("capture", {})

    def test_expiry_during_slow_state_read_never_executes(self):
        proof_id = self.proof()
        def slow_state():
            self.time += 6.0
            return deepcopy(self.state)
        self.backend.state_callback = slow_state
        with self.assertRaises(StaleProof):
            self.execute(proof_id)
        self.assertFalse(self.executed)

    def test_concurrent_replay_executes_only_once(self):
        proof_id = self.proof()
        def attempt():
            try:
                return self.execute(proof_id)["executed"]
            except StaleProof:
                return False
        with ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(lambda _: attempt(), range(2)))
        self.assertEqual(sum(results), 1)
        self.assertEqual(len(self.executed), 1)

    def native_command_motion(self):
        self.state["command_q"] = {"left": [0.] * 7, "right": [.0005] * 7}
        self.state["tracking_contract_id"] = "fixed-envelope-calibration-sha256"
        return {**deepcopy(self.motion), "start_policy": "native_command_reference",
                "actual_start_q": deepcopy(self.state["q"]["right"]),
                "command_start_q": deepcopy(self.state["command_q"]["right"]),
                "trajectory_q": [[.0005] * 7, [.1] * 7]}

    def native_command_backend(self, change=None):
        def checked(request):
            result = self.checker(request)
            result.update(actual_command_start_bridge_checked=True, tracking_envelope_checked=True,
                          issued_array_fr3_checked=True, tracking_contract_id=self.state.get("tracking_contract_id"))
            if change:
                change(result)
            return result
        return self.make_backend(checker_callback=checked)

    def test_native_command_start_preserves_issued_array_and_one_use(self):
        motion = self.native_command_motion()
        backend = self.native_command_backend()
        proof = backend.call("validate_motion", motion)
        self.assertEqual(proof["binding"]["tracking_contract_id"], self.state["tracking_contract_id"])
        backend.call("execute_motion", {"proof_id": proof["proof_id"]})
        issued = self.executed[0]["motion"]
        self.assertEqual(issued["trajectory_q"], motion["trajectory_q"])
        self.assertEqual(issued["time_s"], motion["time_s"])
        self.assertEqual(len(issued["trajectory_q"]), 2)
        with self.assertRaises(StaleProof):
            backend.call("execute_motion", {"proof_id": proof["proof_id"]})

    def test_native_command_start_needs_all_additional_native_checks(self):
        motion = self.native_command_motion()
        with self.assertRaises(ValidationRejected):
            self.backend.call("validate_motion", motion)
        for key in ("actual_command_start_bridge_checked", "tracking_envelope_checked", "issued_array_fr3_checked", "tracking_contract_id"):
            for remove in (True, False):
                with self.subTest(key=key, remove=remove):
                    def change(result):
                        if remove:
                            result.pop(key)
                        else:
                            result[key] = "wrong-contract" if key == "tracking_contract_id" else False
                    with self.assertRaises(ValidationRejected):
                        self.native_command_backend(change).call("validate_motion", motion)
        self.assertFalse(self.executed)

    def test_native_command_start_rejects_inconsistent_starts(self):
        motion = self.native_command_motion()
        for change in (
            {"actual_start_q": [.0005] * 7},
            {"command_start_q": [0.] * 7},
            {"command_start_q": [.001] * 7, "trajectory_q": [[.001] * 7, [.1] * 7]},
            {"actual_start_q": [0.] * 6},
        ):
            with self.subTest(change=change), self.assertRaises(ValidationRejected):
                self.native_command_backend().call("validate_motion", {**motion, **change})
        for key in ("command_q", "tracking_contract_id"):
            before = deepcopy(self.state)
            self.state.pop(key)
            with self.subTest(missing=key), self.assertRaises(ValidationRejected):
                self.native_command_backend().call("validate_motion", motion)
            self.state = before
        self.assertFalse(self.executed)

    def test_native_command_start_rejects_stale_command_or_tracking_contract(self):
        for key in ("command_q", "tracking_contract_id"):
            motion = self.native_command_motion()
            backend = self.native_command_backend()
            proof = backend.call("validate_motion", motion)
            if key == "command_q":
                self.state["command_q"]["right"][0] += .0001
            else:
                self.state["tracking_contract_id"] = "recalibrated-contract"
            with self.subTest(key=key), self.assertRaises(StaleProof):
                backend.call("execute_motion", {"proof_id": proof["proof_id"]})
        self.assertFalse(self.executed)

    def test_native_command_start_does_not_allow_agent_tracking_limits(self):
        motion = self.native_command_motion()
        for override in ({"tracking_tolerance": 1.0}, {"tracking_envelope": [1.] * 7},
                         {"metadata": {"max_tracking_error_rad": 1.}}, {"tracking_contract_id": "agent-claim"}):
            with self.subTest(override=override), self.assertRaises(ValidationRejected):
                self.native_command_backend().call("validate_motion", {**motion, **override})
        self.assertFalse(self.checked)

    def test_default_measured_policy_remains_strict_with_command_state_present(self):
        motion = self.native_command_motion()
        for key in ("start_policy", "actual_start_q", "command_start_q"):
            motion.pop(key)
        with self.assertRaises(ValidationRejected):
            self.native_command_backend().call("validate_motion", motion)
        self.assertFalse(self.executed)

    def test_target_request_needs_native_fulfillment_flag_and_matching_digest(self):
        motion = {**self.motion, "target_request": {"side": "right", "target_kind": "joint", "target_q": [.1] * 7}}
        for change in (None, {"target_request_checked": True},
                       {"target_request_checked": False, "target_request_digest": digest(motion["target_request"])},
                       {"target_request_checked": True, "target_request_digest": "planner-echo-with-wrong-request"}):
            def checker(request):
                result = self.checker(request)
                if change:
                    result.update(change)
                return result
            with self.subTest(change=change), self.assertRaises(ValidationRejected):
                self.make_backend(checker_callback=checker).call("validate_motion", motion)
        self.assertFalse(self.executed)

    def test_native_qualified_target_request_executes_exact_bound_motion_once(self):
        target = {"side": "right", "target_kind": "preinsert", "registered_pose": "pin-preinsert-v575"}
        motion = {**self.motion, "target_request": target}
        def checker(request):
            result = self.checker(request)
            result.update(target_request_checked=True, target_request_digest=digest(request["motion"]["target_request"]))
            return result
        backend = self.make_backend(checker_callback=checker)
        proof = backend.call("validate_motion", motion)
        self.assertEqual(proof["binding"]["target_request_digest"], digest(target))
        target["registered_pose"] = "different-after-validation"
        backend.call("execute_motion", {"proof_id": proof["proof_id"]})
        self.assertEqual(self.executed[0]["motion"]["target_request"]["registered_pose"], "pin-preinsert-v575")
        with self.assertRaises(StaleProof):
            backend.call("execute_motion", {"proof_id": proof["proof_id"]})
        self.assertEqual(len(self.executed), 1)

    def test_invalid_target_request_rejected_before_native_checker(self):
        for target in (None, "joint", {}, {"side": "left", "target_kind": "joint"},
                       {"side": "right", "target_kind": "teleport"}):
            with self.subTest(target=target), self.assertRaises(ValidationRejected):
                self.backend.call("validate_motion", {**self.motion, "target_request": target})
        self.assertFalse(self.checked)
        self.assertFalse(self.executed)


if __name__ == "__main__":
    unittest.main()
