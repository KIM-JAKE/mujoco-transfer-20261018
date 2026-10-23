"""Structured, immutable benchmark records; no run-report inference or execution.

Condition, source, configuration, scene, physical domain, scope and warm/cold
status form distinct cohorts. Components never become three-part task evidence.
Times are descriptive measurements, not hardware/runtime predictions.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, fields
import json
import math
import re
from statistics import median
from typing import Any, Iterable, Mapping


FULL_TASK_PARTS = ("ring_small", "ring_large", "pin")
SMALL_SAMPLE_LIMITATION = (
    "Observed frequencies describe these recorded trials only. Small cohorts do not "
    "establish reliability; independence, unseen conditions and hardware transfer are not inferred."
)


def _text(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")
    return value


def _number(value: Any, name: str, *, optional: bool = False) -> float | None:
    if value is None and optional:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
        raise ValueError(f"{name} must be a finite nonnegative number")
    return float(value)


def _triple(value: Any, name: str) -> tuple[float, float, float]:
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        raise ValueError(f"{name} must explicitly contain x/y/z values")
    if any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in value):
        raise ValueError(f"{name} must contain finite numbers")
    return tuple(float(v) for v in value)


@dataclass(frozen=True)
class TrialRecord:
    """One attempted trial with explicit evidence scope and timing semantics.

    Hashes are lowercase SHA-256 hex digests of stable source/config/scene content,
    not run labels, timestamps or output folder names. A completed full task must explicitly
    list all FULL_TASK_PARTS. A failed/incomplete trial still belongs in the input;
    use completed=False even if all parts were inserted but the required withdrawal
    or final confirmation failed. Missing time measurements remain None.

    timing_stage_s is frozen into sorted (name, seconds) pairs. Mark timing_exclusive
    only when the producer established that all listed counters are disjoint wall
    intervals inside cycle_wall_s. Otherwise their sum and unattributed time are
    unknown: simulation time and overlapping detailed counters are never added.

    part_variations lists independent (part_name, offsets_mm_xyz, angles_rad_xyz)
    triples. Parts are canonicalized by name without rounding or inferring a
    perturbation from the condition label. Missing entries mean unspecified,
    not an automatically inferred zero offset.
    """
    trial_id: str
    condition: str
    source_hash: str
    config_hash: str
    scene_hash: str
    domain: str
    scope: str
    completed: bool
    recovery_used: bool
    completed_parts: tuple[str, ...] = ()
    offsets_mm: tuple[float, float, float] = (0., 0., 0.)
    angles_rad: tuple[float, float, float] = (0., 0., 0.)
    cycle_wall_s: float | None = None
    commanded_duration_s: float | None = None
    simulation_time_s: float | None = None
    timing_stage_s: tuple[tuple[str, float], ...] = ()
    timing_exclusive: bool = False
    warmth: str = "unknown"
    failure: str | None = None
    component_name: str | None = None
    part_variations: tuple[tuple[str, tuple[float, float, float], tuple[float, float, float]], ...] = ()

    def __post_init__(self):
        for key in ("trial_id", "condition"):
            _text(getattr(self, key), key)
        for key in ("source_hash", "config_hash", "scene_hash"):
            value = getattr(self, key)
            if not isinstance(value, str) or re.fullmatch(r"[a-f0-9]{64}", value) is None:
                raise ValueError(f"{key} must be a lowercase SHA-256 hex digest")
        if self.domain not in ("simulation", "hardware"):
            raise ValueError("domain must explicitly be simulation or hardware")
        if self.scope not in ("full_task", "component"):
            raise ValueError("scope must explicitly be full_task or component")
        if self.warmth not in ("warm", "cold", "unknown"):
            raise ValueError("warmth must explicitly be warm, cold or unknown")
        if any(type(getattr(self, key)) is not bool for key in ("completed", "recovery_used", "timing_exclusive")):
            raise ValueError("Completion, recovery and timing-exclusivity flags must be booleans")
        if not isinstance(self.completed_parts, (list, tuple)):
            raise ValueError("completed_parts must be an explicit list/tuple")
        parts = tuple(_text(part, "completed_parts entry") for part in self.completed_parts)
        if len(set(parts)) != len(parts) or not set(parts).issubset(FULL_TASK_PARTS):
            raise ValueError("completed_parts must be unique known task parts")
        object.__setattr__(self, "completed_parts", parts)
        if self.scope == "full_task":
            if self.component_name is not None:
                raise ValueError("A full-task record cannot name a component scope")
            if self.completed and set(parts) != set(FULL_TASK_PARTS):
                raise ValueError("Full-task completion requires explicit completion of all three parts")
        else:
            _text(self.component_name, "component_name")
        if self.failure is not None:
            _text(self.failure, "failure")
            if self.completed:
                raise ValueError("A completed trial cannot also have a terminal failure")
        object.__setattr__(self, "offsets_mm", _triple(self.offsets_mm, "offsets_mm"))
        object.__setattr__(self, "angles_rad", _triple(self.angles_rad, "angles_rad"))
        if not isinstance(self.part_variations, (list, tuple)):
            raise ValueError("part_variations must be explicit part/offset/angle triples")
        variations = []
        for row in self.part_variations:
            if not isinstance(row, (list, tuple)) or len(row) != 3:
                raise ValueError("Each part variation must contain part_name, offsets_mm_xyz and angles_rad_xyz")
            name, offsets, angles = row
            if not isinstance(name, str) or name not in FULL_TASK_PARTS:
                raise ValueError("Part variations must use ring_small, ring_large or pin")
            variations.append((name, _triple(offsets, f"{name} offsets_mm"), _triple(angles, f"{name} angles_rad")))
        if len({name for name, _, _ in variations}) != len(variations):
            raise ValueError("Duplicate part variation names")
        object.__setattr__(self, "part_variations", tuple(sorted(variations)))
        for key in ("cycle_wall_s", "commanded_duration_s", "simulation_time_s"):
            object.__setattr__(self, key, _number(getattr(self, key), key, optional=True))
        if self.domain == "hardware" and self.simulation_time_s is not None:
            raise ValueError("A hardware trial cannot claim elapsed simulation time")
        counters = self.timing_stage_s
        if isinstance(counters, Mapping):
            counters = tuple(counters.items())
        if not isinstance(counters, (list, tuple)):
            raise ValueError("timing_stage_s must be explicit name/seconds pairs or a mapping")
        checked = []
        for row in counters:
            if not isinstance(row, (list, tuple)) or len(row) != 2:
                raise ValueError("timing stage must have name and seconds")
            checked.append((_text(row[0], "timing stage name"), _number(row[1], "timing stage duration")))
        if len({name for name, _ in checked}) != len(checked):
            raise ValueError("Duplicate timing stage names")
        object.__setattr__(self, "timing_stage_s", tuple(sorted(checked)))
        if self.timing_exclusive and self.cycle_wall_s is not None and sum(value for _, value in checked) > self.cycle_wall_s + 1e-9:
            raise ValueError("Declared exclusive wall counters exceed measured total wall time")

    @classmethod
    def from_dict(cls, record: dict) -> "TrialRecord":
        """Accept this structured schema only; never infer fields from simulator logs."""
        if not isinstance(record, dict):
            raise ValueError("Trial input must be a structured dictionary")
        unknown = set(record) - {field.name for field in fields(cls)}
        if unknown:
            raise ValueError(f"Unknown structured trial fields: {sorted(unknown)}")
        try:
            return cls(**record)
        except TypeError as exc:
            raise ValueError(f"Missing or invalid structured trial field: {exc}") from exc

    def to_dict(self) -> dict:
        result = {field.name: getattr(self, field.name) for field in fields(self)}
        result["completed_parts"] = list(self.completed_parts)
        result["offsets_mm"], result["angles_rad"] = list(self.offsets_mm), list(self.angles_rad)
        result["timing_stage_s"] = dict(self.timing_stage_s)
        result["part_variations"] = [[name, list(offsets), list(angles)] for name, offsets, angles in self.part_variations]
        return result


def _distribution(values: Iterable[float | None]) -> dict:
    measured = [value for value in values if value is not None]
    return {"measured_count": len(measured), "fastest": min(measured) if measured else None,
            "median": median(measured) if measured else None, "max": max(measured) if measured else None}


def _timing(record: TrialRecord) -> dict:
    known_exclusive = record.timing_exclusive and record.cycle_wall_s is not None
    exclusive_sum = sum(value for _, value in record.timing_stage_s) if known_exclusive else None
    return {"trial_id": record.trial_id, "cycle_wall_s": record.cycle_wall_s,
            "commanded_duration_s": record.commanded_duration_s,
            "simulation_time_s": record.simulation_time_s,
            "timing_stage_s": dict(record.timing_stage_s),
            "counter_coverage": "exclusive_wall_intervals" if record.timing_exclusive else "overlap_or_coverage_unknown",
            "exclusive_attributed_wall_s": exclusive_sum,
            "unattributed_wall_s": max(0., record.cycle_wall_s - exclusive_sum) if known_exclusive else None,
            "unattributed_status": "measured_remainder" if known_exclusive else "unknown"}


def aggregate_trials(records: Iterable[TrialRecord], *, stretch_target_s: float | None = 30.) -> dict:
    """Compare exact cohorts; every failure and incomplete attempt stays in denominator.

    Fastest/median/max summarize successful full-three-part tasks only. Component
    durations remain raw trial measurements. The optional target is a stretch label
    and a descriptive count of normal successes, never a readiness assertion.
    """
    target = _number(stretch_target_s, "stretch_target_s", optional=True)
    rows = list(records)
    if any(not isinstance(row, TrialRecord) for row in rows):
        raise ValueError("aggregate_trials accepts TrialRecord objects only; use from_dict explicitly")
    if len({row.trial_id for row in rows}) != len(rows):
        raise ValueError("Duplicate trial IDs would double count evidence")
    cohorts: dict[str, tuple[dict, list[TrialRecord]]] = {}
    for row in rows:
        key = {name: getattr(row, name) for name in (
            "condition", "offsets_mm", "angles_rad", "source_hash", "config_hash", "scene_hash",
            "domain", "scope", "component_name", "warmth", "part_variations")}
        serialized = json.dumps(key, sort_keys=True)
        cohorts.setdefault(serialized, (key, []))[1].append(row)
    groups = []
    for serialized in sorted(cohorts):
        key, trials = cohorts[serialized]
        normal = [row for row in trials if row.completed and not row.recovery_used]
        recovered = [row for row in trials if row.completed and row.recovery_used]
        failed = [row for row in trials if not row.completed]
        successful = normal + recovered
        full = key["scope"] == "full_task"
        denominator = len(trials)
        counts = {"attempted": denominator, "normal_success": len(normal), "recovered_success": len(recovered),
                  "failed_or_incomplete": len(failed), "incomplete_without_failure_detail": sum(row.failure is None for row in failed)}
        stats = None
        if full:
            stats = {"successful_full_three_part_cycle_wall_s": _distribution(row.cycle_wall_s for row in successful),
                     "normal_success_cycle_wall_s": _distribution(row.cycle_wall_s for row in normal),
                     "recovered_success_cycle_wall_s": _distribution(row.cycle_wall_s for row in recovered),
                     "successful_full_three_part_commanded_duration_s": _distribution(row.commanded_duration_s for row in successful)}
        stretch = None if target is None else {
            "label": "stretch target; not a guarantee", "target_s": target,
            "scope": "normal full-three-part wall-cycle success only",
            "normal_successes_at_or_below_target": sum(row.cycle_wall_s is not None and row.cycle_wall_s <= target for row in normal) if full else None}
        groups.append({"cohort": key, "counts": counts,
                       "normal_success_fraction": len(normal) / denominator,
                       "recovered_success_fraction": len(recovered) / denominator,
                       "whole_three_part_evidence": full, "timing_statistics": stats,
                       "failure_reasons": dict(sorted(Counter(row.failure or "incomplete_without_failure_detail" for row in failed).items())),
                       "stretch_target": stretch, "trial_ids": [row.trial_id for row in trials],
                       "trial_timings": [_timing(row) for row in trials],
                       "statistical_limitations": SMALL_SAMPLE_LIMITATION})
    return {"schema": "fr3-capx-benchmark/v1", "attempted_total": len(rows), "cohort_count": len(groups),
            "groups": groups, "pooled_success_fraction": None,
            "pooling_note": "Different code/configuration/scene, conditions, domains, scopes and warm/cold states are not pooled.",
            "timing_note": "Wall time, commanded duration and elapsed simulation time are separate. Overlapping counters are not added or normalized to 100%.",
            "statistical_limitations": SMALL_SAMPLE_LIMITATION}
