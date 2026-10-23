"""Three-part accounting tests with fake callbacks only; no device services."""
from dataclasses import FrozenInstanceError
import json
from pathlib import Path
import tempfile
import unittest

from fr3_capx.execution import ExecutionHistory, Failure, FULL_TASK_PHASES, PhaseFailure, PhaseResult
from fr3_capx.pipeline import PartTask, PipelineCoordinator, REQUIRED_PART_NAMES
from fr3_capx.skills import EvidenceBinding


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.directory = Path(self.tmp.name)
        self.binding = EvidenceBinding("1" * 64, "2" * 64, "3" * 64)
        self.coordinator = PipelineCoordinator(binding=self.binding, domain="simulation",
                                               output_dir=self.directory)
        self.calls = []

    def task(self, name, *, fail_phase=None, fail_once=False, recovery=False):
        attempts = {}
        def operation(phase):
            def perform():
                self.calls.append((name, phase))
                attempts[phase] = attempts.get(phase, 0) + 1
                if phase == fail_phase and (not fail_once or attempts[phase] == 1):
                    raise PhaseFailure(Failure("native_reject", "Native fixture rejected path",
                                               retryable=recovery))
                return PhaseResult({"part": name, "phase": phase}, (f"/audit/{name}-{phase}.png",))
            return perform
        options = {}
        if recovery:
            options = dict(recoveries={fail_phase: lambda failure, attempt: self.calls.append(
                (name, "recovery", failure.code, attempt))}, retry_budget=1, max_retries=1)
        return PartTask(name, {phase: operation(phase) for phase in FULL_TASK_PHASES},
                        {phase: lambda _: True for phase in FULL_TASK_PHASES}, **options)

    def test_component_success_remains_component_and_hash_bound(self):
        result = self.coordinator.run_component(self.task("pin"))
        self.assertTrue(result.completed)
        self.assertTrue(result.normal_success)
        self.assertFalse(result.full_task_success)
        self.assertEqual(result.scope, "component")
        self.assertEqual(result.completed_part_count, 1)
        self.assertFalse(result.fresh_final_check_passed)
        artifact = json.loads(Path(result.artifact_path).read_text())
        self.assertEqual(artifact["binding"], dict(scene_sha256="1" * 64, config_sha256="2" * 64,
                                                   source_sha256="3" * 64))
        self.assertFalse(artifact["full_task_success"])
        self.assertFalse(artifact["native_path_qualification"])
        self.assertFalse(artifact["hardware_qualification"])
        for path in (result.history_path, result.parts[0].history_path):
            history = ExecutionHistory.from_file(path)
            self.assertEqual(history.scope, "component")
            self.assertEqual(history.binding, artifact["binding"])
        component = ExecutionHistory.from_file(result.parts[0].history_path)
        self.assertEqual(component.required_phases, list(FULL_TASK_PHASES))
        self.assertEqual(result.parts[0].completed_phases, FULL_TASK_PHASES)

    def test_whole_success_requires_all_current_parts_and_fresh_check_in_caller_order(self):
        order = ("pin", "ring_large", "ring_small")
        def final_check():
            self.calls.append(("whole", "fresh_final_check"))
            return True
        result = self.coordinator.run_three_parts([self.task(name) for name in order],
                                                  fresh_final_check=final_check)
        self.assertTrue(result.full_task_success)
        self.assertTrue(result.normal_success)
        self.assertTrue(result.fresh_final_check_passed)
        self.assertEqual(result.expected_parts, order)
        self.assertEqual(result.completed_part_count, 3)
        self.assertEqual([part.part_name for part in result.parts], list(order))
        self.assertEqual(self.calls[-1], ("whole", "fresh_final_check"))
        self.assertEqual(self.calls[:-1], [(name, phase) for name in order for phase in FULL_TASK_PHASES])
        whole_history = ExecutionHistory.from_file(result.history_path)
        self.assertEqual(whole_history.scope, "full_task")
        for part in result.parts:
            self.assertEqual(ExecutionHistory.from_file(part.history_path).scope, "component")
        accounting = whole_history.steps[-1].observation
        self.assertEqual(accounting["completed_part_count"], 3)
        self.assertEqual(accounting["completed_parts"], list(order))

    def test_missing_duplicate_or_extra_part_rejected_before_execution(self):
        for order in (("pin",), ("pin", "pin", "ring_large"),
                      ("pin", "ring_small", "ring_large", "pin")):
            with self.subTest(order=order), self.assertRaises(ValueError):
                self.coordinator.run_three_parts([self.task(name) for name in order],
                                                 fresh_final_check=lambda: True)
        self.assertEqual(self.calls, [])
        self.assertEqual(list(self.directory.iterdir()), [])
        with self.assertRaises(ValueError):
            self.task("unregistered_part")
        self.assertEqual(REQUIRED_PART_NAMES, frozenset({"ring_small", "ring_large", "pin"}))

    def test_final_callback_required_before_motion(self):
        tasks = [self.task(name) for name in REQUIRED_PART_NAMES]
        with self.assertRaises(TypeError):
            self.coordinator.run_three_parts(tasks, fresh_final_check=None)
        self.assertEqual(self.calls, [])

    def test_component_failure_stops_later_parts_and_whole_final_check(self):
        tasks = [self.task("ring_small"), self.task("ring_large", fail_phase="carry"), self.task("pin")]
        result = self.coordinator.run_three_parts(tasks,
                                                  fresh_final_check=lambda: self.calls.append("must_not_check"))
        self.assertFalse(result.completed)
        self.assertFalse(result.full_task_success)
        self.assertEqual(result.outcome, "failed")
        self.assertEqual(result.completed_part_count, 1)
        self.assertEqual(len(result.parts), 2)
        self.assertEqual(result.failure.code, "native_reject")
        self.assertNotIn("must_not_check", self.calls)
        self.assertFalse(any(call[0] == "pin" for call in self.calls))
        self.assertFalse(any(call[:2] == ("ring_large", "preinsert") for call in self.calls))
        artifact = json.loads(Path(result.artifact_path).read_text())
        self.assertEqual(artifact["completed_part_count"], 1)
        self.assertEqual(artifact["failure"]["code"], "native_reject")

    def test_false_or_truthy_final_check_cannot_report_whole_success(self):
        for value in (False, None, 1, "yes", {"passed": True}):
            with self.subTest(value=value):
                result = self.coordinator.run_three_parts(
                    [self.task(name) for name in REQUIRED_PART_NAMES], fresh_final_check=lambda: value)
                self.assertEqual(result.completed_part_count, 3)
                self.assertFalse(result.completed)
                self.assertFalse(result.full_task_success)
                self.assertFalse(result.normal_success)
                self.assertEqual(result.failure.code, "fresh_final_check_failed")

    def test_final_check_exception_structured_and_component_count_preserved(self):
        def broken():
            raise RuntimeError("No fresh RGB-D completion frame")
        result = self.coordinator.run_three_parts([self.task(name) for name in REQUIRED_PART_NAMES],
                                                  fresh_final_check=broken)
        self.assertFalse(result.completed)
        self.assertEqual(result.completed_part_count, 3)
        self.assertEqual(result.failure.code, "fresh_final_check_exception")
        self.assertEqual(result.failure.details["exception_type"], "RuntimeError")

    def test_recovered_component_and_whole_do_not_become_normal(self):
        tasks = [self.task("pin", fail_phase="grasp", fail_once=True, recovery=True),
                 self.task("ring_large"), self.task("ring_small")]
        result = self.coordinator.run_three_parts(tasks, fresh_final_check=lambda: True)
        self.assertTrue(result.completed)
        self.assertTrue(result.full_task_success)
        self.assertTrue(result.recovery_used)
        self.assertFalse(result.normal_success)
        self.assertEqual(result.outcome, "recovered_success")
        self.assertFalse(result.parts[0].normal_success)
        self.assertEqual(result.parts[0].outcome, "recovered_success")
        self.assertTrue(all(part.normal_success for part in result.parts[1:]))
        history = ExecutionHistory.from_file(result.history_path)
        self.assertTrue(history.recovery_used)
        self.assertEqual(history.outcome, "recovered_success")

    def test_recovery_budget_exhaustion_stops_following_components(self):
        result = self.coordinator.run_three_parts(
            [self.task("pin", fail_phase="carry", recovery=True), self.task("ring_large"),
             self.task("ring_small")], fresh_final_check=lambda: True)
        self.assertFalse(result.completed)
        self.assertTrue(result.recovery_used)
        self.assertEqual(result.completed_part_count, 0)
        self.assertEqual(len(result.parts), 1)
        self.assertEqual(self.calls.count(("pin", "carry")), 2)
        self.assertFalse(any(call[0] != "pin" for call in self.calls))

    def test_operations_and_checkpoint_maps_immutable_and_exact(self):
        operations = {phase: lambda: PhaseResult() for phase in FULL_TASK_PHASES}
        checkpoints = {phase: lambda _: True for phase in FULL_TASK_PHASES}
        task = PartTask("pin", operations, checkpoints)
        operations.clear()
        checkpoints.clear()
        self.assertEqual(set(task.operations), set(FULL_TASK_PHASES))
        with self.assertRaises(TypeError):
            task.operations["carry"] = lambda: PhaseResult()
        with self.assertRaises(FrozenInstanceError):
            task.part_name = "ring_small"
        for missing_map in ("operations", "checkpoints"):
            maps = dict(operations=dict(task.operations), checkpoints=dict(task.checkpoints))
            maps[missing_map].pop("perception")
            with self.assertRaises(ValueError):
                PartTask("pin", **maps)

    def test_recovery_requires_both_explicit_budgets_and_callbacks(self):
        source = self.task("pin")
        for options in (dict(retry_budget=1), dict(max_retries=1),
                        dict(retry_budget=1, max_retries=1),
                        dict(recoveries={"carry": lambda *_: None}),
                        dict(retry_budget=True, max_retries=1),
                        dict(retry_budget=-1, max_retries=1),
                        dict(retry_budget=1, max_retries=1, recoveries={"other": lambda *_: None})):
            with self.subTest(options=options), self.assertRaises(ValueError):
                PartTask("pin", source.operations, source.checkpoints, **options)

    def test_old_component_artifacts_cannot_replace_current_whole_callbacks(self):
        previous = self.coordinator.run_component(self.task("pin"))
        self.calls.clear()
        with self.assertRaises(ValueError):
            self.coordinator.run_three_parts([previous, self.task("ring_small"), self.task("ring_large")],
                                             fresh_final_check=lambda: True)
        self.assertEqual(self.calls, [])

    def test_domains_explicit_and_run_artifacts_do_not_overwrite(self):
        for domain in (None, "full_task", "sim", "unknown"):
            with self.subTest(domain=domain), self.assertRaises(ValueError):
                PipelineCoordinator(binding=self.binding, domain=domain, output_dir=self.directory)
        with self.assertRaises(TypeError):
            PipelineCoordinator(binding={}, domain="simulation", output_dir=self.directory)
        hardware = PipelineCoordinator(binding=self.binding, domain="hardware", output_dir=self.directory)
        first = hardware.run_component(self.task("pin"))
        second = hardware.run_component(self.task("pin"))
        self.assertNotEqual(first.run_id, second.run_id)
        self.assertNotEqual(first.artifact_path, second.artifact_path)
        self.assertEqual(first.domain, "hardware")
        self.assertEqual(ExecutionHistory.from_file(first.history_path).domain, "hardware")
        self.assertFalse(json.loads(Path(first.artifact_path).read_text())["hardware_qualification"])


if __name__ == "__main__":
    unittest.main()
