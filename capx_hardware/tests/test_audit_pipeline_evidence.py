"""Structural audit fixtures; generated records never constitute physical proof."""
from datetime import datetime, timedelta, timezone
import importlib.util
import contextlib
import copy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "audit_pipeline_evidence.py"
SPEC = importlib.util.spec_from_file_location("audit_pipeline_evidence", MODULE_PATH)
audit_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit_module)
audit_pipeline = audit_module.audit_pipeline
PHASES = audit_module.PHASES


class AuditTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.native_dir = Path(self.tmp.name) / "native-run"
        self.root = self.native_dir / "capx-components" / "currentrun"
        self.pipeline_path = self.root / "pipeline-result.json"
        self.binding = {"scene_sha256": "1" * 64, "config_sha256": "2" * 64, "source_sha256": "3" * 64}
        self.clock = datetime(2026, 10, 23, tzinfo=timezone.utc)

    def stamp(self):
        self.clock += timedelta(milliseconds=1)
        return self.clock.isoformat()

    def write(self, path, data):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data))

    def read(self, path):
        return json.loads(path.read_text())

    def mutate(self, path, callback):
        value = self.read(path)
        callback(value)
        self.write(path, value)

    def fixture(self, whole=False, names=None):
        if whole:
            self.root = self.native_dir / "capx-full3" / "currentrun"
            self.pipeline_path = self.root / "pipeline-result.json"
        names = names or (["ring_small", "ring_large", "pin"] if whole else ["pin"])
        coordinator_start = self.stamp()
        parts = []
        for name in names:
            history_path = self.root / "components" / name / "execution_history_block_0.json"
            steps = []
            start = self.stamp()
            for phase in PHASES:
                observation = dict(part_name=name, phase=phase, scope="component", domain="simulation",
                                   native_measured={"source": "fresh native fixture", "value_m": 0.001},
                                   uses_oracle=False)
                for kind, status, observed in (("phase", "running", {}),
                                               ("observation", "observed", observation),
                                               ("checkpoint", "passed", observation)):
                    steps.append(dict(tool_name=phase if kind != "checkpoint" else "checkpoint",
                                      kind=kind, status=status, phase=phase, attempt=0,
                                      observation=observed, step_index=len(steps), timestamp=self.stamp()))
            history = dict(schema="fr3-capx-execution/v1", scope="component", domain="simulation",
                           binding=self.binding, required_phases=list(PHASES), recovery_used=False,
                           outcome="normal_success", start_time=start, end_time=self.stamp(), steps=steps)
            self.write(history_path, history)
            parts.append(dict(part_name=name, completed=True, recovery_used=False,
                              outcome="normal_success", completed_phases=list(PHASES),
                              history_path=str(history_path), failure=None))
        coordinator_path = self.root / "coordinator" / "execution_history_block_0.json"
        steps = [dict(tool_name="pipeline", kind="pipeline", status="running", phase="", step_index=0,
                      timestamp=coordinator_start, observation={})]
        if whole:
            steps.extend([
                dict(tool_name="fresh_final_check", kind="checkpoint", status="running", phase="final_check",
                     step_index=1, timestamp=self.stamp(), observation={}),
                dict(tool_name="checkpoint", kind="checkpoint", status="passed", phase="whole_final_check",
                     step_index=2, timestamp=self.stamp(), observation={"completed_part_count": 3})])
        history = dict(schema="fr3-capx-execution/v1", scope="full_task" if whole else "component",
                       domain="simulation", binding=self.binding, required_phases=[], recovery_used=False,
                       outcome="normal_success", start_time=coordinator_start, end_time=self.stamp(), steps=steps)
        self.write(coordinator_path, history)
        pipeline = dict(schema="fr3-capx-pipeline/v1", run_id=self.root.name, domain="simulation",
                        scope=history["scope"], binding=self.binding, expected_parts=names, parts=parts,
                        completed=True, recovery_used=False, fresh_final_check_passed=whole,
                        outcome="normal_success", history_path=str(coordinator_path),
                        artifact_path=str(self.pipeline_path), completed_part_count=len(names),
                        normal_success=True, full_task_success=whole)
        self.write(self.pipeline_path, pipeline)
        source_binding = dict(source_id="3" * 64, scene_id="1" * 64, config_id="2" * 64)
        native = dict(full_task_pass=whole, pin_only_component=not whole,
                      requires_whole_three_part_trial=not whole, hardware_tested=False,
                      pin_component_pass=not whole,
                      capx_native_bridge=dict(simulation_only=True, source_binding=source_binding,
                                              events=[dict(source_scene_config=source_binding)]))
        self.write(self.native_dir / "pipeline-results.json", native)
        return self.root / "components" / names[0] / "execution_history_block_0.json"

    def codes(self):
        return {i["code"] for i in audit_pipeline(self.pipeline_path)["issues"]}

    def authoritative_session(self, *, cancelled=False, accepted=True, finished=None):
        """Attach current-run native tokens to existing whole fixture records."""
        pipeline = self.read(self.pipeline_path)
        token = "current-native-session"
        events = []
        native_parts = {"ring_small": "ring2", "ring_large": "ring1", "pin": "pin"}
        for part in pipeline["parts"]:
            path = Path(part["history_path"])
            history = self.read(path)
            for step in history["steps"]:
                if step["kind"] != "observation":
                    continue
                event = dict(step["observation"], kind="phase", part=native_parts[part["part_name"]],
                             run_token=token, action_token=f"current-action-{len(events) + 1}",
                             generation=len(events) + 1, attempt=step["attempt"], cycle_wall_s=float(len(events)))
                events.append(event)
                step["observation"] = event
                # The matching accepted checkpoint binds exactly this event.
                for checkpoint in history["steps"]:
                    if checkpoint["kind"] == "checkpoint" and checkpoint["phase"] == step["phase"]:
                        checkpoint["observation"] = event
            self.write(path, history)
        session = dict(schema="native-phase-session/v2", run_token=token, cancelled=cancelled,
                       worker_alive_after_abort=False, terminal_failure=None, policy_error=None,
                       accepted_full3=accepted, late_coordinator_output_is_not_acceptance=True,
                       events=events)
        if finished is not None:
            session["finished"] = finished
        path = self.native_dir / "native-phase-session-final.json"
        self.write(path, session)
        return path

    def nested_pin_component(self, *, cancelled=False):
        """Real layout: token-bearing parent plus synchronous PIN wrapper log."""
        self.fixture(whole=True)
        session = self.authoritative_session(cancelled=cancelled, accepted=not cancelled)
        parent_path = self.pipeline_path
        parent = self.read(parent_path)
        pin = copy.deepcopy(next(part for part in parent["parts"] if part["part_name"] == "pin"))
        history = self.read(Path(pin["history_path"]))
        self.root = self.native_dir / "capx-components" / "pin-wrapper-currentrun"
        self.pipeline_path = self.root / "pipeline-result.json"
        pin["history_path"] = str(self.root / "components" / "pin" / "execution_history_block_0.json")
        # The inner wrapper stores the same measured values, not the parent
        # session's envelope. It does not receive independent action tokens.
        for step in history["steps"]:
            if step["kind"] in ("observation", "checkpoint"):
                step["observation"] = {key: value for key, value in step["observation"].items() if key in (
                    "part_name", "phase", "scope", "domain", "native_measured", "cycle_wall_s", "uses_oracle")}
        self.write(Path(pin["history_path"]), history)
        coordinator_path = self.root / "coordinator" / "execution_history_block_0.json"
        coordinator = dict(history, required_phases=[], steps=[dict(
            tool_name="pipeline", kind="pipeline", status="running", phase="", step_index=0,
            timestamp=history["start_time"], observation={})])
        self.write(coordinator_path, coordinator)
        component = dict(parent, run_id=self.root.name, scope="component", expected_parts=["pin"], parts=[pin],
                         history_path=str(coordinator_path), artifact_path=str(self.pipeline_path),
                         full_task_success=False, fresh_final_check_passed=False, completed_part_count=1)
        self.write(self.pipeline_path, component)
        self.mutate(self.native_dir / "pipeline-results.json", lambda n: n.update(
            pin_component_pass=False, full_task_pass=not cancelled, pin_only_component=False,
            requires_whole_three_part_trial=False))
        return parent_path, session

    def ring_component_session(self, *, accepted=True, cancelled=False):
        self.fixture(names=["ring_small"])
        phase_path = self.authoritative_session(cancelled=cancelled, accepted=False)
        self.mutate(self.native_dir / "pipeline-results.json", lambda n: n.update(
            pin_only_component=False, requires_whole_three_part_trial=False,
            component_part="ring2", pin_component_pass=False))
        phase = self.read(phase_path)
        sensor = dict(part="ring2", pose=dict(part="ring2", uses_depth_sensor=True,
                      uses_simulator_body_poses=False), captured_wall_s=10., ready_wall_s=10.1,
                      confirmed=True, observation_id="ring-final-new-observation", camera_id="fixed-camera",
                      calibration_id="calibration", raw_acquisition_sha256="a" * 64,
                      position_error_m=.0002, axis_error_rad=.005, uses_free_body_poses=False)
        final = dict(parts={"ring2": sensor}, requested_after_wall_s=9., confirmed=True,
                     scope="component", domain="simulation", uses_free_body_poses=False,
                     hands_clear=True, hand_clearance_m={"left": .2, "right": .3})
        phase["final_sensor_evidence"] = final
        self.write(phase_path, phase)
        authority = dict(phase, schema="native-ring-component-session/v1", scope="component",
                         part="ring2", hardware=False, accepted_component=accepted)
        component_path = self.native_dir / "native-ring-component-session-final.json"
        self.write(component_path, authority)
        return phase_path, component_path

    def test_ring_component_additional_final_sensor_never_promotes_to_full3(self):
        self.ring_component_session()
        result = audit_pipeline(self.pipeline_path)
        self.assertEqual(result["issues"], [])
        self.assertTrue(result["native_common_ring_component_accepted"])
        self.assertTrue(result["accepted_native_session"])
        self.assertFalse(result["native_common_full3_accepted"])
        self.assertFalse(result["physical_success_verified"])

    def test_cancelled_ring_authority_supersedes_late_generic_success(self):
        self.ring_component_session(accepted=False, cancelled=True)
        result = audit_pipeline(self.pipeline_path)
        self.assertEqual(result["issues"], [])
        self.assertFalse(result["accepted_native_session"])
        self.assertTrue(result["generic_component_claim_superseded_by_native_session"])

    def test_ring_cancelled_acceptance_and_foreign_session_are_rejected(self):
        _, authority = self.ring_component_session(accepted=True, cancelled=True)
        self.assertIn("component_session_invalid_acceptance", self.codes())
        self.mutate(authority, lambda a: a.update(run_token="another-run"))
        self.assertIn("component_session_parent_binding", self.codes())

    def test_ring_additional_final_must_be_new_and_palms_clear(self):
        phase, authority = self.ring_component_session()
        for target in (phase, authority):
            self.mutate(target, lambda a: a["final_sensor_evidence"]["parts"]["ring2"].update(
                captured_wall_s=8.))
            self.mutate(target, lambda a: a["final_sensor_evidence"]["hand_clearance_m"].update(left=.16))
        codes = self.codes()
        self.assertIn("component_session_stale_final_sensor", codes)
        self.assertIn("component_session_hand_clearance", codes)

    def test_ring_authority_cannot_claim_another_part_or_whole_task(self):
        _, authority = self.ring_component_session()
        self.mutate(authority, lambda a: a.update(part="ring1", accepted_full3=True))
        codes = self.codes()
        self.assertIn("component_session_scope_mismatch", codes)
        self.assertIn("component_session_scope_escalation", codes)

    def test_ring_authority_requires_matching_native_phase_record(self):
        phase, _ = self.ring_component_session()
        phase.unlink()
        self.assertIn("component_session_parent_binding", self.codes())

    def assert_paired_ring_event_rejected(self, mutation, code):
        phase, authority = self.ring_component_session()
        for path in (phase, authority):
            self.mutate(path, lambda a: mutation(a["events"]))
        result = audit_pipeline(self.pipeline_path)
        self.assertIn(code, {i["code"] for i in result["issues"]})
        self.assertFalse(result["native_common_ring_component_accepted"])
        self.assertFalse(result["structural_consistency_pass"])

    def test_paired_ring_authorities_cannot_share_foreign_event_token(self):
        self.assert_paired_ring_event_rejected(
            lambda events: events[0].update(run_token="foreign-run"), "native_session_event_run_mismatch")

    def test_paired_ring_authorities_cannot_share_wrong_native_part(self):
        self.assert_paired_ring_event_rejected(
            lambda events: events[0].update(part="ring1"), "native_session_event_identity")

    def test_paired_ring_authorities_cannot_share_duplicate_event(self):
        self.assert_paired_ring_event_rejected(
            lambda events: events.append(copy.deepcopy(events[-1])), "native_session_duplicate_event")

    def test_paired_ring_authorities_cannot_share_oracle_event(self):
        self.assert_paired_ring_event_rejected(
            lambda events: events[0].update(uses_oracle=True), "native_session_event_role")

    def test_paired_ring_authorities_must_match_generic_measurement(self):
        self.assert_paired_ring_event_rejected(
            lambda events: events[0]["native_measured"].update(value_m=.4),
            "native_session_generic_measurement_mismatch")

    def test_ring_parent_cannot_promote_component_to_full3(self):
        phase, _ = self.ring_component_session()
        self.mutate(phase, lambda a: a.update(accepted_full3=True))
        self.assertIn("component_session_parent_scope_escalation", self.codes())
        self.assertFalse(audit_pipeline(self.pipeline_path)["native_common_ring_component_accepted"])

    def test_paired_ring_authorities_require_boolean_worker_state(self):
        phase, authority = self.ring_component_session()
        for path in (phase, authority):
            self.mutate(path, lambda a: a.update(worker_alive_after_abort="running"))
        self.assertIn("component_session_invalid_worker_boolean", self.codes())
        self.assertFalse(audit_pipeline(self.pipeline_path)["native_common_ring_component_accepted"])

    def test_consistent_component_stays_structural_and_never_whole_or_physical(self):
        self.fixture()
        result = audit_pipeline(self.pipeline_path)
        self.assertTrue(result["structural_consistency_pass"])
        self.assertEqual(result["qualification"], "structural_consistency_only")
        self.assertFalse(result["physical_success_verified"])
        self.assertFalse(result["native_collision_proof_verified"])
        self.assertFalse(result["hardware_success_verified"])
        self.assertFalse(result["declared_full_task_success"])
        self.assertEqual(result["phases"][0]["observed_phases"], list(PHASES))
        self.assertEqual(len(result["input_sha256"]), 4)
        self.assertFalse(result["native_session_present"])
        self.assertFalse(result["accepted_native_session"])

    def test_exact_history_binding_required(self):
        history = self.fixture()
        self.mutate(history, lambda h: h["binding"].update(source_sha256="9" * 64))
        self.assertIn("binding_mismatch", self.codes())

    def test_phase_missing_or_out_of_order_detected(self):
        history = self.fixture()
        self.mutate(history, lambda h: h["steps"][0].update(phase="carry"))
        self.assertIn("phase_order_mismatch", self.codes())
        self.assertIn("phase_evidence_mismatch", self.codes())

    def test_generated_pass_boolean_without_observation_rejected(self):
        history = self.fixture()
        self.mutate(history, lambda h: h["steps"][1].update(kind="generated_boolean", observation={"passed": True}))
        self.assertIn("missing_observed_evidence", self.codes())
        self.assertIn("checkpoint_evidence_mismatch", self.codes())

    def test_oracle_true_or_missing_and_empty_native_measurement_rejected(self):
        for value in (True, None):
            history = self.fixture()
            self.mutate(history, lambda h: h["steps"][1]["observation"].update(uses_oracle=value))
            self.assertIn("oracle_or_missing_declaration", self.codes())
        self.mutate(history, lambda h: h["steps"][1]["observation"].update(native_measured={}))
        self.assertIn("missing_native_measured", self.codes())

    def test_native_oracle_conflict_and_hidden_recovery_detected(self):
        history = self.fixture()
        def corrupt(h):
            h["steps"][1]["observation"]["native_measured"].update(uses_simulator_body_poses=True, corrected=True)
        self.mutate(history, corrupt)
        self.assertIn("native_oracle_conflict", self.codes())
        self.assertIn("recovery_claim_mismatch", self.codes())

    def test_cross_run_history_claim_rejected_even_local_copy_exists(self):
        self.fixture()
        self.mutate(self.pipeline_path, lambda p: p["parts"][0].update(history_path="/other/run/components/pin/execution_history_block_0.json"))
        self.assertIn("cross_run_path", self.codes())

    def test_native_binding_and_component_scope_escalation_rejected(self):
        self.fixture()
        self.mutate(self.native_dir / "pipeline-results.json", lambda n: n["capx_native_bridge"]["source_binding"].update(scene_id="9" * 64))
        self.assertIn("native_binding_mismatch", self.codes())
        self.mutate(self.pipeline_path, lambda p: p.update(full_task_success=True, fresh_final_check_passed=True))
        self.assertIn("component_promoted_to_whole", self.codes())
        self.assertIn("native_scope_escalation", self.codes())

    def test_consistent_whole_is_only_structural_not_physical(self):
        self.fixture(whole=True)
        result = audit_pipeline(self.pipeline_path)
        self.assertTrue(result["structural_consistency_pass"], result["issues"])
        self.assertTrue(result["declared_full_task_success"])
        self.assertFalse(result["physical_success_verified"])

    def test_whole_needs_explicit_current_fresh_record(self):
        self.fixture(whole=True)
        coordinator = self.root / "coordinator" / "execution_history_block_0.json"
        self.mutate(coordinator, lambda h: h.update(steps=h["steps"][:1]))
        self.assertIn("missing_explicit_fresh_final_check", self.codes())

    def test_whole_check_before_last_component_is_stale(self):
        self.fixture(whole=True)
        coordinator = self.root / "coordinator" / "execution_history_block_0.json"
        self.mutate(coordinator, lambda h: h["steps"][1].update(timestamp=h["start_time"]))
        self.assertIn("stale_final_check", self.codes())

    def test_whole_duplicate_component_and_count_mismatch(self):
        self.fixture(whole=True)
        self.mutate(self.pipeline_path, lambda p: (p["parts"].append(p["parts"][0]), p.update(completed_part_count=99)))
        self.assertIn("duplicate_or_invalid_parts", self.codes())
        self.assertIn("completed_count_mismatch", self.codes())
        self.assertIn("whole_part_set_mismatch", self.codes())

    def test_missing_native_report_or_native_hardware_promotion_rejected(self):
        self.fixture()
        (self.native_dir / "pipeline-results.json").unlink()
        self.assertIn("missing_native_source", self.codes())
        self.fixture()
        self.mutate(self.pipeline_path, lambda p: p.update(domain="hardware"))
        self.assertIn("native_domain_mismatch", self.codes())
        self.assertIn("native_hardware_claim_missing", self.codes())

    def test_audit_is_read_only_for_all_inputs(self):
        self.fixture()
        before = {p: p.read_bytes() for p in self.native_dir.rglob("*.json")}
        audit_pipeline(self.pipeline_path)
        after = {p: p.read_bytes() for p in self.native_dir.rglob("*.json")}
        self.assertEqual(before, after)

    def test_native_component_completion_boolean_must_match(self):
        self.fixture()
        self.mutate(self.native_dir / "pipeline-results.json", lambda n: n.update(pin_component_pass=False))
        self.assertIn("native_component_claim_missing", self.codes())

    def test_advancing_after_rejected_checkpoint_is_detected(self):
        history = self.fixture()
        self.mutate(history, lambda h: h["steps"][2].update(status="failed"))
        self.assertIn("advance_after_failed_phase", self.codes())

    def test_truthy_generated_success_is_not_explicit_boolean(self):
        self.fixture()
        self.mutate(self.pipeline_path, lambda p: p.update(completed=1))
        self.assertIn("invalid_boolean_claim", self.codes())

    def test_current_authoritative_session_matches_generic_whole_events(self):
        self.fixture(whole=True)
        session = self.authoritative_session()
        result = audit_pipeline(self.pipeline_path)
        self.assertTrue(result["structural_consistency_pass"], result["issues"])
        self.assertTrue(result["native_session_present"])
        self.assertTrue(result["accepted_native_session"])
        self.assertTrue(result["native_common_full3_accepted"])
        self.assertFalse(result["physical_success_verified"])
        self.assertIn(str(session), result["input_sha256"])

    def test_cancelled_session_overrides_late_generic_whole_success(self):
        self.fixture(whole=True)
        self.authoritative_session(cancelled=True, accepted=False)
        self.mutate(self.native_dir / "pipeline-results.json", lambda n: n.update(full_task_pass=False))
        result = audit_pipeline(self.pipeline_path)
        self.assertTrue(result["structural_consistency_pass"], result["issues"])
        self.assertTrue(result["declared_full_task_success"])
        self.assertTrue(result["generic_claim_superseded_by_native_session"])
        self.assertEqual(result["native_session_status"], "cancelled")
        self.assertFalse(result["accepted_native_session"])
        self.assertFalse(result["physical_success_verified"])

    def test_unfinished_or_unaccepted_session_never_accepts_generic_success(self):
        for finished in (None, False):
            with self.subTest(finished=finished):
                self.fixture(whole=True)
                self.authoritative_session(accepted=False, finished=finished)
                result = audit_pipeline(self.pipeline_path)
                self.assertTrue(result["structural_consistency_pass"], result["issues"])
                self.assertFalse(result["accepted_native_session"])
                self.assertEqual(result["native_session_status"], "incomplete")

    def test_invalid_acceptance_cannot_overrule_cancellation_or_live_worker(self):
        for updates in (dict(cancelled=True), dict(finished=False), dict(worker_alive_after_abort=True),
                        dict(terminal_failure={"code": "native_collision"})):
            with self.subTest(updates=updates):
                self.fixture(whole=True)
                session = self.authoritative_session()
                self.mutate(session, lambda s: s.update(**updates))
                result = audit_pipeline(self.pipeline_path)
                self.assertIn("native_session_invalid_acceptance", {i["code"] for i in result["issues"]})
                self.assertFalse(result["accepted_native_session"])

    def test_foreign_run_repeated_token_and_duplicate_phase_events_rejected(self):
        for field, value, code in (("run_token", "old-run", "native_session_event_run_mismatch"),
                                  ("action_token", "current-action-1", "native_session_duplicate_action"),
                                  ("generation", 1, "native_session_generation_order"),
                                  ("phase", "perception", "native_session_duplicate_event")):
            with self.subTest(field=field):
                self.fixture(whole=True)
                session = self.authoritative_session()
                self.mutate(session, lambda s: s["events"][1].update(**{field: value}))
                self.assertIn(code, self.codes())

    def test_native_measured_payload_must_equal_generic_observation(self):
        self.fixture(whole=True)
        session = self.authoritative_session()
        self.mutate(session, lambda s: s["events"][0]["native_measured"].update(value_m=0.2))
        self.assertIn("native_session_generic_measurement_mismatch", self.codes())

    def test_session_part_role_and_measured_identity_are_checked(self):
        for update, code in (({"scope": "full_task"}, "native_session_event_role"),
                             ({"uses_oracle": True}, "native_session_event_role"),
                             ({"part_name": "pin"}, "native_session_event_identity"),
                             ({"native_measured": {"sensor": {"part": "pin"}}}, "native_session_measured_part_mismatch")):
            with self.subTest(update=update):
                self.fixture(whole=True)
                session = self.authoritative_session()
                self.mutate(session, lambda s: s["events"][0].update(**update))
                self.assertIn(code, self.codes())

    def test_legacy_whole_claim_without_session_is_not_common_native_qualified(self):
        self.fixture(whole=True)
        result = audit_pipeline(self.pipeline_path)
        self.assertTrue(result["structural_consistency_pass"])
        self.assertTrue(result["declared_full_task_success"])
        self.assertFalse(result["native_common_full3_accepted"])
        self.assertFalse(result["accepted_native_session"])

    def test_failed_session_can_consistently_record_an_unconsumed_tail_event(self):
        self.fixture(whole=True)
        session = self.authoritative_session(cancelled=True, accepted=False)
        pipeline = self.read(self.pipeline_path)
        # Native emitted perception, but cancellation prevented the worker from
        # consuming it. The failed generic component has no observed success.
        part = pipeline["parts"][0]
        history_path = Path(part["history_path"])
        history = self.read(history_path)
        history.update(steps=history["steps"][:1], outcome="failed")
        self.write(history_path, history)
        part.update(completed=False, outcome="failed", completed_phases=[])
        pipeline.update(parts=[part], completed=False, outcome="failed", normal_success=False,
                        full_task_success=False, fresh_final_check_passed=False, completed_part_count=0)
        self.write(self.pipeline_path, pipeline)
        coordinator = self.read(Path(pipeline["history_path"]))
        coordinator.update(outcome="failed", steps=coordinator["steps"][:1])
        self.write(Path(pipeline["history_path"]), coordinator)
        self.mutate(session, lambda s: s.update(events=s["events"][:1]))
        self.mutate(self.native_dir / "pipeline-results.json", lambda n: n.update(full_task_pass=False))
        result = audit_pipeline(self.pipeline_path)
        self.assertTrue(result["structural_consistency_pass"], result["issues"])
        self.assertEqual(result["unmatched_native_event_count"], 1)
        self.assertFalse(result["accepted_native_session"])

    def test_cli_discovers_only_bounded_current_run_component_and_full3_paths(self):
        declared = [self.native_dir / "capx-full3" / "pipeline-result.json",
                    self.native_dir / "capx-full3" / "current-whole" / "pipeline-result.json",
                    self.native_dir / "capx-components" / "current-component" / "pipeline-result.json"]
        for path in declared:
            self.write(path, {})
        self.write(self.native_dir / "archive" / "other-run" / "pipeline-result.json", {})
        output = self.directory_output = Path(self.tmp.name) / "audit-output.json"
        called = []
        def fake(path, **kwargs):
            called.append(path)
            return dict(structural_consistency_pass=True, issue_count=0)
        with patch.object(audit_module, "audit_pipeline", side_effect=fake), contextlib.redirect_stdout(io.StringIO()):
            status = audit_module.main([str(self.native_dir), "--output", str(output)])
        self.assertEqual(status, 0)
        self.assertEqual(set(called), set(declared))
        self.assertEqual(self.read(output)["audited_runs"], 3)

    def test_nested_pin_wrapper_matches_current_parent_without_whole_promotion(self):
        parent, _ = self.nested_pin_component()
        result = audit_pipeline(self.pipeline_path)
        self.assertTrue(result["structural_consistency_pass"], result["issues"])
        self.assertTrue(result["nested_component_bound_to_parent"])
        self.assertEqual(result["parent_common_pipeline_path"], str(parent))
        self.assertEqual(result["native_session_comparison_role"], "nested_component")
        self.assertEqual(result["native_session_status"], "parent_accepted")
        self.assertFalse(result["accepted_native_session"])
        self.assertFalse(result["native_common_full3_accepted"])
        self.assertFalse(result["declared_full_task_success"])
        self.assertFalse(result["physical_success_verified"])

    def test_nested_component_can_preserve_local_success_after_parent_cancel(self):
        _, _ = self.nested_pin_component(cancelled=True)
        result = audit_pipeline(self.pipeline_path)
        self.assertTrue(result["structural_consistency_pass"], result["issues"])
        self.assertTrue(result["declared_completed"])
        self.assertTrue(result["nested_component_bound_to_parent"])
        self.assertEqual(result["native_session_status"], "cancelled")
        self.assertFalse(result["accepted_native_session"])

    def test_parent_success_flag_without_current_matching_part_is_insufficient(self):
        for corruption in ("binding", "part", "measurement", "token"):
            with self.subTest(corruption=corruption):
                parent, _ = self.nested_pin_component()
                parent_data = self.read(parent)
                pin_history = Path(next(p for p in parent_data["parts"] if p["part_name"] == "pin")["history_path"])
                if corruption == "binding":
                    self.mutate(parent, lambda p: p["binding"].update(source_sha256="9" * 64))
                elif corruption == "part":
                    self.mutate(parent, lambda p: p.update(parts=p["parts"][:2]))
                elif corruption == "measurement":
                    self.mutate(pin_history, lambda h: h["steps"][1]["observation"]["native_measured"].update(value_m=999.))
                else:
                    self.mutate(pin_history, lambda h: h["steps"][1]["observation"].update(run_token="old-session"))
                result = audit_pipeline(self.pipeline_path)
                codes = {issue["code"] for issue in result["issues"]}
                self.assertIn("nested_component_parent_unbound", codes)
                self.assertIn("native_scope_mismatch", codes)
                self.assertFalse(result["nested_component_bound_to_parent"])
                self.assertFalse(result["accepted_native_session"])

    def test_pin_only_native_source_cannot_gain_parent_whole_exemption(self):
        self.nested_pin_component()
        self.mutate(self.native_dir / "pipeline-results.json", lambda n: n.update(
            pin_only_component=True, requires_whole_three_part_trial=True))
        result = audit_pipeline(self.pipeline_path)
        self.assertFalse(result["nested_component_bound_to_parent"])
        self.assertFalse(result["accepted_native_session"])
        self.assertIn("native_scope_escalation", {issue["code"] for issue in result["issues"]})

    def test_component_namespace_cannot_accept_a_full_task_session_claim(self):
        self.fixture(whole=True)
        self.authoritative_session()
        # Retain the internally complete fixture, but claim it from the wrong
        # pipeline namespace. Native whole acceptance is reserved for capx-full3.
        old_root = self.root
        new_root = self.native_dir / "capx-components" / "currentrun"
        new_root.parent.mkdir(parents=True, exist_ok=True)
        old_root.rename(new_root)
        for path in new_root.rglob("*.json"):
            path.write_text(path.read_text().replace(str(old_root), str(new_root)))
        self.root, self.pipeline_path = new_root, new_root / "pipeline-result.json"
        result = audit_pipeline(self.pipeline_path)
        self.assertIn("native_session_pipeline_role_mismatch", {issue["code"] for issue in result["issues"]})
        self.assertFalse(result["accepted_native_session"])

    def test_nested_failure_before_first_pin_observation_remains_failed_evidence(self):
        parent_path, session = self.nested_pin_component(cancelled=True)
        parent = self.read(parent_path)
        parent_pin = next(part for part in parent["parts"] if part["part_name"] == "pin")
        inner = self.read(self.pipeline_path)
        for pipeline, pin in ((parent, parent_pin), (inner, inner["parts"][0])):
            path = Path(pin["history_path"])
            self.mutate(path, lambda h: h.update(steps=h["steps"][:1], outcome="failed"))
            pin.update(completed=False, completed_phases=[], outcome="failed")
            pipeline.update(completed=False, outcome="failed", normal_success=False,
                            full_task_success=False, fresh_final_check_passed=False,
                            completed_part_count=sum(part["completed"] for part in pipeline["parts"]))
            coordinator_path = Path(pipeline["history_path"])
            self.mutate(coordinator_path, lambda h: h.update(outcome="failed", steps=h["steps"][:1]))
        self.write(parent_path, parent)
        self.write(self.pipeline_path, inner)
        self.mutate(session, lambda s: s.update(events=[e for e in s["events"] if e["part"] != "pin"]))
        result = audit_pipeline(self.pipeline_path)
        self.assertTrue(result["structural_consistency_pass"], result["issues"])
        self.assertTrue(result["nested_component_bound_to_parent"])
        self.assertFalse(result["declared_completed"])
        self.assertFalse(result["accepted_native_session"])


if __name__ == "__main__":
    unittest.main()
