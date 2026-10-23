"""Evidence accounting tests, with no simulation or device execution."""
from dataclasses import FrozenInstanceError, replace
import json
import unittest

from fr3_capx.benchmark import FULL_TASK_PARTS, TrialRecord, aggregate_trials


class BenchmarkTests(unittest.TestCase):
    def record(self, trial_id="one", **changes):
        values = dict(trial_id=trial_id, condition="nominal", source_hash="a" * 64,
                      config_hash="b" * 64, scene_hash="c" * 64, domain="simulation", scope="full_task",
                      completed=True, recovery_used=False, completed_parts=FULL_TASK_PARTS,
                      cycle_wall_s=40., commanded_duration_s=12., simulation_time_s=15., warmth="warm")
        values.update(changes)
        return TrialRecord(**values)

    def test_all_failed_and_incomplete_trials_stay_in_denominator(self):
        records = [self.record("normal", cycle_wall_s=29.),
                   self.record("recovered", recovery_used=True, cycle_wall_s=50.),
                   self.record("blocked", completed=False, completed_parts=("ring_small",), failure="collision", cycle_wall_s=3.),
                   self.record("incomplete", completed=False, completed_parts=(), cycle_wall_s=None)]
        group = aggregate_trials(records)["groups"][0]
        self.assertEqual(group["counts"], dict(attempted=4, normal_success=1, recovered_success=1,
                                             failed_or_incomplete=2, incomplete_without_failure_detail=1))
        self.assertEqual(group["normal_success_fraction"], .25)
        self.assertEqual(group["recovered_success_fraction"], .25)
        self.assertEqual(group["failure_reasons"], {"collision": 1, "incomplete_without_failure_detail": 1})
        stats = group["timing_statistics"]["successful_full_three_part_cycle_wall_s"]
        self.assertEqual(stats, dict(measured_count=2, fastest=29., median=39.5, max=50.))

    def test_component_is_separate_and_never_counts_as_full_three_parts(self):
        component = self.record("component", scope="component", component_name="pin_carry", completed_parts=(), cycle_wall_s=.5)
        report = aggregate_trials([self.record(), component])
        self.assertEqual(report["cohort_count"], 2)
        group = next(g for g in report["groups"] if g["cohort"]["scope"] == "component")
        self.assertFalse(group["whole_three_part_evidence"])
        self.assertIsNone(group["timing_statistics"])
        self.assertIsNone(group["stretch_target"]["normal_successes_at_or_below_target"])

    def test_source_config_scene_domain_condition_poses_and_warmth_split(self):
        base = self.record()
        differences = [dict(source_hash="d" * 64), dict(config_hash="d" * 64), dict(scene_hash="d" * 64),
                       dict(domain="hardware", simulation_time_s=None), dict(condition="PLUS"),
                       dict(offsets_mm=(10., 0., 0.)), dict(angles_rad=(0., 0., .01)), dict(warmth="cold")]
        records = [base] + [replace(base, trial_id=str(index), **change) for index, change in enumerate(differences)]
        report = aggregate_trials(records)
        self.assertEqual(report["cohort_count"], len(records))
        self.assertIsNone(report["pooled_success_fraction"])

    def test_overlapping_counters_have_unknown_unattributed_time(self):
        row = self.record(cycle_wall_s=40., simulation_time_s=100., timing_stage_s={"physics": 30., "callback": 25.})
        timing = aggregate_trials([row])["groups"][0]["trial_timings"][0]
        self.assertEqual(timing["simulation_time_s"], 100.)
        self.assertEqual(timing["timing_stage_s"], {"callback": 25., "physics": 30.})
        self.assertIsNone(timing["exclusive_attributed_wall_s"])
        self.assertIsNone(timing["unattributed_wall_s"])
        self.assertEqual(timing["unattributed_status"], "unknown")

    def test_exclusive_wall_counters_can_report_remainder_without_sim_time(self):
        row = self.record(cycle_wall_s=40., timing_stage_s={"planning": 8., "actuation": 22.}, timing_exclusive=True)
        timing = aggregate_trials([row])["groups"][0]["trial_timings"][0]
        self.assertEqual(timing["exclusive_attributed_wall_s"], 30.)
        self.assertEqual(timing["unattributed_wall_s"], 10.)
        self.assertEqual(timing["unattributed_status"], "measured_remainder")
        with self.assertRaises(ValueError):
            self.record(cycle_wall_s=2., timing_stage_s={"a": 3.}, timing_exclusive=True)

    def test_exclusive_counters_without_measured_total_still_unknown(self):
        row = self.record(cycle_wall_s=None, timing_stage_s={"a": 3.}, timing_exclusive=True)
        timing = aggregate_trials([row])["groups"][0]["trial_timings"][0]
        self.assertIsNone(timing["unattributed_wall_s"])

    def test_missing_success_times_are_not_zero(self):
        group = aggregate_trials([self.record(cycle_wall_s=None)])["groups"][0]
        self.assertEqual(group["counts"]["normal_success"], 1)
        self.assertEqual(group["timing_statistics"]["successful_full_three_part_cycle_wall_s"],
                         dict(measured_count=0, fastest=None, median=None, max=None))

    def test_full_task_requires_explicit_all_parts_and_no_terminal_failure(self):
        for change in (dict(completed_parts=()), dict(completed_parts=("pin",)),
                       dict(completed_parts=(*FULL_TASK_PARTS, "pin")), dict(failure="collision"),
                       dict(component_name="pin")):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.record(**change)
        failed_after_parts = self.record(completed=False, failure="withdrawal_collision")
        group = aggregate_trials([failed_after_parts])["groups"][0]
        self.assertEqual(group["counts"]["normal_success"], 0)
        self.assertEqual(group["counts"]["failed_or_incomplete"], 1)

    def test_record_deeply_freezes_supplied_lists_and_mappings(self):
        parts, offsets, stages = list(FULL_TASK_PARTS), [0., 0., 0.], {"planning": 3.}
        row = self.record(completed_parts=parts, offsets_mm=offsets, timing_stage_s=stages)
        parts.clear(); offsets[0] = 99.; stages["planning"] = 999.
        self.assertEqual(row.completed_parts, FULL_TASK_PARTS)
        self.assertEqual(row.offsets_mm, (0., 0., 0.))
        self.assertEqual(row.timing_stage_s, (("planning", 3.),))
        with self.assertRaises(FrozenInstanceError):
            row.completed = False

    def test_strict_structured_roundtrip_does_not_parse_raw_pipeline_reports(self):
        row = self.record()
        restored = TrialRecord.from_dict(json.loads(json.dumps(row.to_dict())))
        self.assertEqual(restored, row)
        for raw in ({"full_task_pass": True, "wall_seconds": 1.}, {**row.to_dict(), "unknown": 1}):
            with self.assertRaises(ValueError):
                TrialRecord.from_dict(raw)
        with self.assertRaises(ValueError):
            aggregate_trials([row.to_dict()])

    def test_invalid_numbers_scope_and_metadata_fail_explicitly(self):
        for change in (dict(cycle_wall_s=float("nan")), dict(cycle_wall_s=True), dict(angles_rad=(0., float("inf"), 0.)),
                       dict(offsets_mm=(0., 0.)), dict(completed=1), dict(source_hash="unsealed"),
                       dict(scope="component"), dict(domain="hardware"), dict(timing_stage_s=[("a", 1.), ("a", 2.)])):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.record(**change)

    def test_duplicate_ids_rejected_and_empty_report_has_no_invented_rate(self):
        with self.assertRaises(ValueError):
            aggregate_trials([self.record(), self.record()])
        report = aggregate_trials([])
        self.assertEqual(report["attempted_total"], 0)
        self.assertEqual(report["groups"], [])
        self.assertIsNone(report["pooled_success_fraction"])

    def test_target_is_optional_and_uses_normal_full_task_only(self):
        rows = [self.record("normal", cycle_wall_s=29.), self.record("recovered", recovery_used=True, cycle_wall_s=15.)]
        stretch = aggregate_trials(rows)["groups"][0]["stretch_target"]
        self.assertEqual(stretch["normal_successes_at_or_below_target"], 1)
        self.assertIn("not a guarantee", stretch["label"])
        self.assertIsNone(aggregate_trials(rows, stretch_target_s=None)["groups"][0]["stretch_target"])

    def test_same_condition_label_with_different_ring_variations_is_separate(self):
        small = ("ring_small", (10., -10., 0.), (.04, 0., 0.))
        large = ("ring_large", (-10., 10., 0.), (0., -.04, 0.))
        changed_large = ("ring_large", (-9., 10., 0.), (0., -.04, 0.))
        one = self.record("plus-one", condition="PLUS", part_variations=(small, large))
        two = self.record("plus-two", condition="PLUS", part_variations=(small, changed_large))
        report = aggregate_trials([one, two])
        self.assertEqual(report["cohort_count"], 2)
        self.assertTrue(all(group["cohort"]["condition"] == "PLUS" for group in report["groups"]))
        self.assertNotEqual(report["groups"][0]["cohort"]["part_variations"], report["groups"][1]["cohort"]["part_variations"])

    def test_per_part_variations_are_deeply_frozen_canonical_and_roundtrip(self):
        rows = [["ring_small", [10., -10., 0.], [.04, 0., 0.]],
                ["ring_large", [-10., 10., 0.], [0., -.04, 0.]]]
        record = self.record(part_variations=rows)
        opposite_order = self.record("reordered", part_variations=list(reversed(rows)))
        self.assertEqual(record.part_variations, opposite_order.part_variations)
        self.assertEqual(aggregate_trials([record, opposite_order])["cohort_count"], 1)
        self.assertEqual(TrialRecord.from_dict(json.loads(json.dumps(record.to_dict()))), record)
        rows[0][1][0] = 99.
        rows.clear()
        self.assertEqual(dict((name, offsets) for name, offsets, _ in record.part_variations)["ring_small"], (10., -10., 0.))

    def test_invalid_part_variations_reject_and_labels_never_infer_offsets(self):
        for variations in (
            {"ring_small": [0., 0., 0.]},
            [("small_ring", [0., 0., 0.], [0., 0., 0.])],
            [("pin", [0., 0.], [0., 0., 0.])],
            [("pin", [0., 0., 0.], [0., float("nan"), 0.])],
            [("pin", [0., 0., 0.], [0., 0., 0.])] * 2,
        ):
            with self.subTest(variations=variations), self.assertRaises(ValueError):
                self.record(part_variations=variations)
        self.assertEqual(self.record(condition="PLUS").part_variations, ())


if __name__ == "__main__":
    unittest.main()
