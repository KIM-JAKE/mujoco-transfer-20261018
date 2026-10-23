"""Proof promotion and bounded feedback, independent of GPU/robot services."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from fr3_capx.execution import (
    BoundedExecutor, ExecutionHistory, ExecutionLogger, FULL_TASK_PHASES, Failure, PhaseFailure,
    PhaseResult, TerminalStop,
)
from fr3_capx.skills import (
    EvidenceBinding, EvidenceRejected, REQUIRED_PHASES, SkillLibrary,
    ValidationEvidence, extract_docstring, extract_functions, sha256_text,
)


CODE = '''def fixed_insert(api):
    """Insert only along the independently qualified axis."""
    api.move_joint_preinsert()

    api.insert_fixed_axis()
'''


class SkillsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / "skills.json"
        self.binding = EvidenceBinding("1" * 64, "2" * 64, "3" * 64)
        self.proof = Path(self.tmp.name) / "native-proof.json"
        self.proof.write_text('{"fixture": "externally verified test proof"}')
        # Real integrations must check full artifact semantics here. This fake
        # verifier demonstrates that promotion checks artifact identity again
        # at every reuse, rather than trusting a persisted boolean.
        def verify(e):
            return (Path(e.proof_path).is_file() and
                    hashlib.sha256(Path(e.proof_path).read_bytes()).hexdigest() == e.proof_sha256)
        self.verify = verify
        self.lib = SkillLibrary(self.path, verify_evidence=verify)
        self.lib.extract_from_code(CODE, "raised_component")
        self.evidence = ValidationEvidence(
            evidence_id="fixture-not-real-validation", binding=self.binding,
            function_sha256=self.lib.skills["fixed_insert"].code_sha256,
            proof_path=str(self.proof), proof_sha256=hashlib.sha256(self.proof.read_bytes()).hexdigest(),
            completed=True, fixed_insert=True, collision_method="native_full_path",
            phases=tuple(sorted(REQUIRED_PHASES)),
        )

    def test_original_extraction_docstring_and_full_body(self):
        functions = extract_functions(CODE)
        self.assertEqual([f["name"] for f in functions], ["fixed_insert"])
        self.assertIn("api.insert_fixed_axis()", functions[0]["code"])
        self.assertEqual(functions[0]["docstring"], "Insert only along the independently qualified axis.")
        self.assertEqual(extract_docstring(CODE), functions[0]["docstring"])
        partial = 'def demo(x):\n    """Documented."""\n    return x\n\nif ('
        self.assertEqual(extract_functions(partial)[0]["name"], "demo")

    def test_frequency_never_promotes(self):
        for _ in range(10):
            self.lib.extract_from_code(CODE, "old575_visual_align")
        self.assertEqual(self.lib.get_promoted_skills(binding=self.binding), {})
        self.assertFalse(self.lib.skills["fixed_insert"].promoted)

    def test_untrusted_or_incomplete_claims_rejected(self):
        lib = SkillLibrary(Path(self.tmp.name) / "no-verifier.json")
        lib.extract_from_code(CODE)
        with self.assertRaises(EvidenceRejected):
            lib.promote("fixed_insert", evidence=self.evidence, binding=self.binding)
        for changes in (
            {"scope": "component"}, {"scope": "hardware"}, {"mode": "recovery"},
            {"completed": False}, {"recovery_used": True}, {"fixed_insert": False},
            {"collision_method": "curobo_proposal"}, {"phases": ("carry", "fixed_insert")},
            {"function_sha256": "0" * 64}, {"domain": "hardware"},
        ):
            with self.subTest(changes=changes), self.assertRaises(EvidenceRejected):
                self.lib.promote("fixed_insert", evidence=replace(self.evidence, **changes),
                                 binding=self.binding)

    def test_binding_hash_changes_reject_reuse_and_hardware(self):
        self.lib.promote("fixed_insert", evidence=self.evidence, binding=self.binding)
        self.assertIn("fixed_insert", self.lib.get_promoted_skills(binding=self.binding))
        for field in ("scene_sha256", "config_sha256", "source_sha256"):
            changed = replace(self.binding, **{field: "9" * 64})
            self.assertEqual(self.lib.get_promoted_skills(binding=changed), {})
        self.assertEqual(self.lib.get_promoted_skills(binding=self.binding, domain="hardware"), {})
        self.proof.write_text("changed proof")
        self.assertEqual(self.lib.get_promoted_skills(binding=self.binding), {})

    def test_library_roundtrip_and_code_update_revokes(self):
        self.lib.promote("fixed_insert", evidence=self.evidence, binding=self.binding)
        self.lib.save()
        restored = SkillLibrary(self.path, verify_evidence=self.verify)
        self.assertIn("fixed_insert", restored.get_promoted_skills(binding=self.binding))
        restored.extract_from_code(CODE.replace("api.insert_fixed_axis()", "api.other_axis()"))
        self.assertFalse(restored.skills["fixed_insert"].promoted)
        self.assertEqual(restored.skills["fixed_insert"].evidence, [])
        restored.save()
        corrupted = json.loads(self.path.read_text())
        corrupted["skills"]["fixed_insert"]["code"] += "\n# unverified change"
        self.path.write_text(json.dumps(corrupted))
        with self.assertRaises(ValueError):
            SkillLibrary(self.path, verify_evidence=self.verify)

    def test_no_namespace_execution(self):
        namespace = {}
        with self.assertRaises(RuntimeError):
            self.lib.inject_into_namespace(namespace)
        self.assertEqual(namespace, {})
        self.assertIn("extract_from_code", self.lib.functions())
        with self.assertRaises(ValueError):
            EvidenceBinding("unhashed", "2" * 64, "3" * 64)


class ExecutionTests(unittest.TestCase):
    def test_declared_full_task_phase_order_and_result(self):
        history = ExecutionHistory(domain="simulation", scope="full_task")
        runner = BoundedExecutor(ExecutionLogger(history), required_phases=FULL_TASK_PHASES)
        observed = []
        for phase in FULL_TASK_PHASES:
            def operation(phase=phase):
                observed.append(phase)
                return PhaseResult({"verified_phase": phase})
            runner.run_phase(phase, operation, checkpoint=lambda _: True)
        result = runner.finish()
        self.assertTrue(result.normal_success)
        self.assertEqual(result.required_phases, FULL_TASK_PHASES)
        self.assertEqual(result.completed_phases, FULL_TASK_PHASES)
        self.assertEqual(history.required_phases, list(FULL_TASK_PHASES))
        self.assertEqual(observed, list(FULL_TASK_PHASES))
        self.assertEqual(set(FULL_TASK_PHASES), REQUIRED_PHASES)

    def test_declared_phase_order_duplicate_and_unknown_block_before_motion(self):
        for first, invalid in ((None, "carry"), ("perception", "perception"),
                               ("perception", "unknown"), ("perception", "fixed_insert")):
            with self.subTest(first=first, invalid=invalid):
                runner = BoundedExecutor(required_phases=FULL_TASK_PHASES)
                if first:
                    runner.run_phase(first, lambda: PhaseResult(), checkpoint=lambda _: True)
                calls = []
                with self.assertRaises(TerminalStop):
                    runner.run_phase(invalid, lambda: calls.append("must_not_move"),
                                     checkpoint=lambda _: True)
                self.assertEqual(calls, [])
                self.assertEqual(runner.logger.history.outcome, "failed")
                self.assertEqual(runner.logger.history.steps[-1].failure.code, "phase_order_invalid")

    def test_declared_missing_phase_blocks_finish(self):
        runner = BoundedExecutor(required_phases=FULL_TASK_PHASES)
        runner.run_phase("perception", lambda: PhaseResult(), checkpoint=lambda _: True)
        with self.assertRaises(TerminalStop):
            runner.finish()
        self.assertEqual(runner.logger.history.outcome, "failed")
        self.assertEqual(runner.logger.history.steps[-1].failure.code, "incomplete_phase_plan")
        with self.assertRaises(TerminalStop):
            runner.run_phase("grasp", lambda: PhaseResult(), checkpoint=lambda _: True)

    def test_declared_plan_recovery_retries_same_phase_without_advancing(self):
        runner = BoundedExecutor(required_phases=("grasp", "carry"), retry_budget=1)
        calls = []
        def operation():
            calls.append("grasp")
            return PhaseResult({"retained": len(calls) > 1})
        runner.run_phase("grasp", operation, checkpoint=lambda r: r.observation["retained"],
                         recovery=lambda *_: None, max_retries=1)
        self.assertEqual(runner.completed_phases, ["grasp"])
        runner.run_phase("carry", lambda: PhaseResult(), checkpoint=lambda _: True)
        result = runner.finish()
        self.assertFalse(result.normal_success)
        self.assertEqual(result.completed_phases, ("grasp", "carry"))

    def test_declared_plan_validated_and_history_binding_retained(self):
        for phases in ([], ["grasp", "grasp"], [""], ["grasp", 1], "grasp"):
            with self.subTest(phases=phases), self.assertRaises(ValueError):
                BoundedExecutor(required_phases=phases)
        history = ExecutionHistory(required_phases=["grasp", "carry"])
        runner = BoundedExecutor(ExecutionLogger(history))
        self.assertEqual(runner.required_phases, ("grasp", "carry"))
        with self.assertRaises(ValueError):
            BoundedExecutor(ExecutionLogger(history), required_phases=["perception", "grasp"])

    def test_generic_phase_repetition_remains_supported(self):
        runner = BoundedExecutor()
        for _ in range(2):
            runner.run_phase("sample", lambda: PhaseResult(), checkpoint=lambda _: True)
        result = runner.finish()
        self.assertTrue(result.normal_success)
        self.assertEqual(result.required_phases, ())
        self.assertEqual(result.completed_phases, ("sample", "sample"))

    def test_checkpoint_log_roundtrip_and_normal_completion(self):
        history = ExecutionHistory(scope="full_task", binding={"source_sha256": "3" * 64})
        logger = ExecutionLogger(history)
        runner = BoundedExecutor(logger)
        runner.run_phase("carry", lambda: PhaseResult({"slip_mm": 0.0}, ("/audit/carry.png",)),
                         checkpoint=lambda result: result.observation["slip_mm"] < 0.1)
        logger.log_step_update("Carry checkpoint retained", images=["/audit/after.png"])
        outcome = runner.finish()
        self.assertTrue(outcome.normal_success)
        self.assertEqual(outcome.scope, "full_task")
        self.assertEqual(outcome.domain, "simulation")
        with tempfile.TemporaryDirectory() as directory:
            path = history.save_to_directory(directory)
            restored = ExecutionHistory.from_file(path)
            self.assertEqual(history.to_dict(), restored.to_dict())
        self.assertEqual([step.step_index for step in history.steps], list(range(len(history.steps))))
        self.assertTrue(all(step.timestamp.endswith("+00:00") for step in history.steps))
        self.assertEqual(sorted(step.elapsed_s for step in history.steps),
                         [step.elapsed_s for step in history.steps])
        self.assertTrue(any(s.kind == "checkpoint" for s in history.steps))
        self.assertNotIn("image_refs", history.get_steps_for_vlm()[1])
        self.assertEqual(history.get_steps_for_vlm(True)[1]["image_refs"], ["/audit/carry.png"])
        with self.assertRaises(TerminalStop):
            runner.run_phase("late_motion", lambda: PhaseResult(), checkpoint=lambda _: True)

    def test_recovery_success_never_normal(self):
        runner = BoundedExecutor(retry_budget=1)
        count = {"calls": 0, "recoveries": 0}
        def operation():
            count["calls"] += 1
            return PhaseResult({"retained": count["calls"] > 1})
        def recovery(failure, attempt):
            self.assertEqual(failure.code, "checkpoint_failed")
            self.assertEqual(attempt, 1)
            count["recoveries"] += 1
        runner.run_phase("grasp", operation, checkpoint=lambda r: r.observation["retained"],
                         recovery=recovery, max_retries=5)
        outcome = runner.finish()
        self.assertTrue(outcome.completed)
        self.assertFalse(outcome.normal_success)
        self.assertEqual(outcome.outcome, "recovered_success")
        self.assertTrue(runner.logger.history.recovery_used)
        self.assertEqual(count, {"calls": 2, "recoveries": 1})

    def test_retry_budget_and_terminal_stop(self):
        runner = BoundedExecutor(retry_budget=2)
        calls = []
        def operation():
            calls.append("operation")
            raise PhaseFailure(Failure("slip", "Pin slipped", retryable=True, details={"mm": 1.2}))
        with self.assertRaises(TerminalStop):
            runner.run_phase("carry", operation, checkpoint=lambda _: True,
                             recovery=lambda *_: calls.append("recovery"), max_retries=99)
        self.assertEqual(calls, ["operation", "recovery", "operation", "recovery", "operation"])
        self.assertEqual(runner.retries_used, 2)
        self.assertEqual(runner.logger.history.outcome, "failed")
        with self.assertRaises(TerminalStop):
            runner.run_phase("insert", lambda: calls.append("unsafe"), checkpoint=lambda _: True)
        self.assertNotIn("unsafe", calls)
        with self.assertRaises(TerminalStop):
            runner.finish()

    def test_nonretryable_and_recovery_exception_stop(self):
        for recovery_exception in (False, True):
            with self.subTest(recovery_exception=recovery_exception):
                runner = BoundedExecutor(retry_budget=4)
                calls = []
                def operation():
                    raise PhaseFailure(Failure("collision", "Native guard rejected",
                                               retryable=recovery_exception))
                def recovery(*_):
                    calls.append("recovery")
                    raise RuntimeError("Backend unavailable")
                with self.assertRaises(TerminalStop):
                    runner.run_phase("insert", operation, checkpoint=lambda _: True,
                                     recovery=recovery, max_retries=4)
                self.assertEqual(len(calls), int(recovery_exception))
                self.assertEqual(runner.logger.history.outcome, "failed")

    def test_explicit_checkpoint_true_and_no_empty_success(self):
        runner = BoundedExecutor()
        with self.assertRaises(TerminalStop):
            runner.run_phase("guard", lambda: PhaseResult(), checkpoint=lambda _: "yes")
        self.assertEqual(runner.logger.history.outcome, "failed")
        with self.assertRaises(TerminalStop):
            BoundedExecutor().finish()

    def test_exception_is_structured_and_histories_are_isolated(self):
        runner = BoundedExecutor()
        def broken():
            raise RuntimeError("Controller feedback unavailable")
        with self.assertRaises(TerminalStop):
            runner.run_phase("carry", broken, checkpoint=lambda _: True)
        failure = runner.logger.history.steps[-1].failure
        self.assertEqual(failure.code, "operation_exception")
        self.assertEqual(failure.details["exception_type"], "RuntimeError")
        other = ExecutionLogger()
        self.assertEqual(other.history.steps, [])
        self.assertIn("log_step", other.functions())


if __name__ == "__main__":
    unittest.main()
