#!/usr/bin/env python3
"""Read-only structural audit of native CaP-X pipeline evidence, stdlib only.

This checks record consistency, never physical success. It does not execute
callbacks, load the sealed adapter, mutate source artifacts or print protocols.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any

PHASES = ("perception", "grasp", "carry", "preinsert", "fixed_insert", "release", "withdraw", "final_check")
PARTS = frozenset({"ring_small", "ring_large", "pin"})
BINDING_KEYS = ("scene_sha256", "config_sha256", "source_sha256")


def audit_pipeline(path: str | Path, *, native_path: str | Path | None = None,
                   workspace_root: str | Path | None = None) -> dict[str, Any]:
    """Audit actual local records; all success fields remain declared claims.

    Paths recorded inside /workspace/project are mapped to workspace_root.
    Local component records are always read from this pipeline's own directory,
    never from a path supplied by another run or a protocol payload.
    """
    path = Path(path).resolve()
    root = path.parent
    workspace = Path(workspace_root).resolve() if workspace_root else Path(__file__).resolve().parents[2]
    issues: list[dict[str, str]] = []
    files: dict[str, str] = {}

    def issue(code: str, location: str, message: str) -> None:
        issues.append(dict(code=code, location=location, message=message))

    def read(target: Path) -> dict[str, Any]:
        try:
            raw = target.read_bytes()
            files[str(target)] = hashlib.sha256(raw).hexdigest()
            value = json.loads(raw)
            if not isinstance(value, dict):
                raise ValueError("JSON object required")
            return value
        except (OSError, ValueError) as exc:
            issue("unreadable_artifact", str(target), type(exc).__name__)
            return {}

    def record_path(recorded: Any, expected: Path, label: str) -> None:
        if not isinstance(recorded, str):
            issue("missing_recorded_path", label, "Path is missing")
            return
        candidate = Path(recorded)
        if candidate.is_relative_to("/workspace/project"):
            candidate = workspace / candidate.relative_to("/workspace/project")
        if not candidate.is_absolute():
            candidate = root / candidate
        if candidate.resolve() != expected.resolve():
            issue("cross_run_path", label, "Recorded path does not identify the current-run artifact")

    def timestamp(value: Any, label: str) -> datetime | None:
        try:
            parsed = datetime.fromisoformat(value)
            if parsed.tzinfo is None:
                raise ValueError("timezone missing")
            return parsed
        except (TypeError, ValueError):
            issue("invalid_timestamp", label, "Timezone-aware ISO timestamp required")
            return None

    pipeline = read(path)
    binding = pipeline.get("binding")
    if (not isinstance(binding, dict) or set(binding) != set(BINDING_KEYS) or
            any(not isinstance(binding.get(k), str) or not re.fullmatch(r"[a-fA-F0-9]{64}", binding[k])
                for k in BINDING_KEYS)):
        issue("invalid_binding", "pipeline.binding", "Exact scene/config/source SHA256 binding required")
    if pipeline.get("schema") != "fr3-capx-pipeline/v1":
        issue("schema_mismatch", "pipeline.schema", "Unsupported pipeline schema")
    if pipeline.get("run_id") != root.name:
        issue("run_id_mismatch", "pipeline.run_id", "run_id must identify its current directory")
    scope, domain = pipeline.get("scope"), pipeline.get("domain")
    if scope not in ("component", "full_task") or domain not in ("simulation", "hardware"):
        issue("invalid_scope_domain", "pipeline", "Explicit component/full_task and simulation/hardware required")
    record_path(pipeline.get("artifact_path"), path, "pipeline.artifact_path")
    parts = pipeline.get("parts", [])
    if not isinstance(parts, list) or any(not isinstance(p, dict) for p in parts):
        issue("invalid_parts", "pipeline.parts", "Part records must be a list of objects")
        parts = []
    names = [p.get("part_name") for p in parts]
    expected = pipeline.get("expected_parts", [])
    if (not isinstance(expected, list) or any(name not in PARTS for name in expected) or
            len(set(expected)) != len(expected)):
        issue("invalid_expected_parts", "pipeline.expected_parts", "Expected parts must be unique registered names")
    if any(name not in PARTS for name in names) or len(set(names)) != len(names):
        issue("duplicate_or_invalid_parts", "pipeline.parts", "Each attempted registered part may occur only once")
    if names != expected[:len(names)]:
        issue("part_order_mismatch", "pipeline.parts", "Attempted parts must be the declared execution prefix")
    if scope == "component" and (len(expected) != 1 or len(parts) != 1):
        issue("component_scope_mismatch", "pipeline", "Component scope requires exactly one part")
    if scope == "full_task" and (len(expected) != 3 or frozenset(expected) != PARTS):
        issue("whole_part_set_mismatch", "pipeline", "Whole task requires both rings and pin exactly once")
    completed = pipeline.get("completed") is True
    for field in ("completed", "recovery_used", "fresh_final_check_passed", "normal_success", "full_task_success"):
        if type(pipeline.get(field)) is not bool:
            issue("invalid_boolean_claim", "pipeline." + field, "An explicit boolean claim is required")
    declared_recovery = pipeline.get("recovery_used") is True
    derived_recovery = False
    component_ends: list[datetime] = []
    phase_summary = []
    generic_phase_observations: list[dict[str, Any]] = []

    def check_history(history: dict[str, Any], label: str, history_scope: str) -> list[dict[str, Any]]:
        if history.get("schema") != "fr3-capx-execution/v1":
            issue("schema_mismatch", label, "Unsupported execution schema")
        if history.get("binding") != binding:
            issue("binding_mismatch", label, "History scene/config/source binding differs")
        if history.get("scope") != history_scope or history.get("domain") != domain:
            issue("history_scope_domain_mismatch", label, "History scope/domain differs from its role")
        steps = history.get("steps", [])
        if not isinstance(steps, list) or any(not isinstance(s, dict) for s in steps):
            issue("invalid_steps", label, "Steps must be objects")
            return []
        previous = None
        for index, step in enumerate(steps):
            if step.get("step_index") != index:
                issue("step_index_mismatch", label, "Step indexes must be sequential")
            stamp = timestamp(step.get("timestamp"), label)
            if stamp is not None and previous is not None and stamp < previous:
                issue("timestamp_order", label, "Step timestamps moved backwards")
            previous = stamp if stamp is not None else previous
        start = timestamp(history.get("start_time"), label + ".start_time")
        end = timestamp(history.get("end_time"), label + ".end_time")
        if start is not None and end is not None and end < start:
            issue("timestamp_order", label, "History ended before it started")
        return steps

    for part_index, part in enumerate(parts):
        name = part.get("part_name")
        if name not in PARTS:
            continue  # Never use an untrusted label as a filesystem path.
        label = "components." + name
        history_path = root / "components" / name / "execution_history_block_0.json"
        record_path(part.get("history_path"), history_path, label + ".history_path")
        history = read(history_path)
        steps = check_history(history, label, "component")
        if history.get("required_phases") != list(PHASES):
            issue("declared_phase_mismatch", label, "All eight declared phases must match exact order")
        end = timestamp(history.get("end_time"), label + ".end_time")
        if end is not None:
            component_ends.append(end)
        starts = [i for i, s in enumerate(steps) if s.get("kind") == "phase" and s.get("status") == "running"]
        observed_phases, passed_phases = [], []
        previous_phase_index, previous_attempt = -1, -1
        previous_segment_passed, previous_segment_recovery = False, False
        recovery_seen = any(s.get("kind") == "recovery" for s in steps)
        for offset, start in enumerate(starts):
            first = steps[start]
            phase, attempt = first.get("phase"), first.get("attempt")
            segment = steps[start:starts[offset + 1] if offset + 1 < len(starts) else len(steps)]
            if phase not in PHASES or type(attempt) is not int or attempt < 0:
                issue("invalid_phase_attempt", label, "Known phase and integer attempt required")
                continue
            phase_index = PHASES.index(phase)
            if phase_index == previous_phase_index:
                if attempt != previous_attempt + 1 or not previous_segment_recovery or previous_segment_passed:
                    issue("invalid_retry_order", label, "A repeated phase needs bounded recovery and next attempt")
            elif phase_index != previous_phase_index + 1 or attempt != 0:
                issue("phase_order_mismatch", label, "Phase was omitted or executed out of order")
            elif previous_phase_index >= 0 and not previous_segment_passed:
                issue("advance_after_failed_phase", label, "A later phase began before the preceding phase passed")
            previous_phase_index, previous_attempt = phase_index, attempt
            observations = [s for s in segment if s.get("kind") == "observation" and s.get("status") == "observed"]
            checkpoints = [s for s in segment if s.get("kind") == "checkpoint" and s.get("status") == "passed"]
            if len(observations) > 1 or len(checkpoints) > 1:
                issue("duplicate_phase_evidence", label + "." + phase, "One observation and accepted checkpoint per attempt required")
            if not observations and (checkpoints or part.get("completed") is True):
                issue("missing_observed_evidence", label + "." + phase, "Phase lacks observed native evidence")
            for step in observations + checkpoints:
                observation = step.get("observation", {})
                if (step.get("phase") != phase or step.get("attempt") != attempt or
                        not isinstance(observation, dict)):
                    issue("phase_evidence_mismatch", label + "." + phase, "Observation belongs to another phase/attempt")
                    continue
                identities = dict(part_name=name, phase=phase, scope="component", domain=domain)
                if any(observation.get(k) != value for k, value in identities.items()):
                    issue("observation_identity_mismatch", label + "." + phase, "Native observation role differs")
                if observation.get("uses_oracle") is not False:
                    issue("oracle_or_missing_declaration", label + "." + phase, "Explicit uses_oracle=False required")
                native = observation.get("native_measured")
                if not isinstance(native, dict) or not native:
                    issue("missing_native_measured", label + "." + phase, "Nonempty measured evidence required; generated bool is insufficient")
                elif any(native.get(k) is True for k in ("uses_free_body_poses", "uses_simulator_body_poses")):
                    issue("native_oracle_conflict", label + "." + phase, "Native evidence contradicts oracle-free declaration")
                if isinstance(native, dict) and (native.get("corrected") is True or
                        native.get("normal_fixed_execution") is False or
                        (isinstance(native.get("alignment_index"), int) and native["alignment_index"] > 0)):
                    recovery_seen = True
            if observations:
                observed_phases.append(phase)
                generic_phase_observations.extend(dict(s["observation"], phase=s.get("phase"), attempt=s.get("attempt")) for s in observations
                                                  if isinstance(s.get("observation"), dict))
            if checkpoints:
                passed_phases.append(phase)
                if not observations or checkpoints[0].get("observation") != observations[0].get("observation"):
                    issue("checkpoint_evidence_mismatch", label + "." + phase, "Passed checkpoint does not bind the observed evidence")
            previous_segment_passed = bool(checkpoints)
            previous_segment_recovery = any(s.get("kind") == "recovery" for s in segment)
        if part.get("completed_phases") != passed_phases:
            issue("completed_phase_claim_mismatch", label, "Declared completion differs from accepted checkpoints")
        if part.get("completed") is True and passed_phases != list(PHASES):
            issue("incomplete_component_success", label, "Component success requires eight ordered accepted observations")
        if part.get("completed") is not True and part_index + 1 < len(parts):
            issue("execution_after_failure", label, "Another part executed after component failure")
        if (part.get("recovery_used") is True) != recovery_seen or (history.get("recovery_used") is True) != recovery_seen:
            issue("recovery_claim_mismatch", label, "Recovery event/correction and reported mode differ")
        derived_recovery |= recovery_seen
        wanted = "failed" if part.get("completed") is not True else "recovered_success" if recovery_seen else "normal_success"
        if part.get("outcome") != wanted or history.get("outcome") != wanted:
            issue("component_outcome_mismatch", label, "Normal/recovered/failed outcome contradicts execution")
        phase_summary.append(dict(part=name, observed_phases=observed_phases, passed_phases=passed_phases,
                                  declared_completed=part.get("completed") is True, recovery_observed=recovery_seen))

    count = sum(part.get("completed") is True for part in parts)
    if pipeline.get("completed_part_count") != count:
        issue("completed_count_mismatch", "pipeline", "Completion count differs from part records")
    if declared_recovery != derived_recovery:
        issue("recovery_claim_mismatch", "pipeline", "Aggregate recovery mode differs from component evidence")
    wanted_outcome = "failed" if not completed else "recovered_success" if derived_recovery else "normal_success"
    if pipeline.get("outcome") != wanted_outcome or pipeline.get("normal_success") is not (completed and not derived_recovery):
        issue("pipeline_outcome_mismatch", "pipeline", "Aggregate normal/recovered outcome is inconsistent")
    if completed and (names != expected or count != len(expected)):
        issue("incomplete_pipeline_success", "pipeline", "Pipeline success omits expected completed parts")
    coordinator_path = root / "coordinator" / "execution_history_block_0.json"
    record_path(pipeline.get("history_path"), coordinator_path, "pipeline.history_path")
    coordinator = read(coordinator_path)
    coordinator_steps = check_history(coordinator, "coordinator", scope)
    if coordinator.get("outcome") != pipeline.get("outcome") or coordinator.get("recovery_used") != pipeline.get("recovery_used"):
        issue("coordinator_outcome_mismatch", "coordinator", "Coordinator outcome differs from pipeline")
    requests = [s for s in coordinator_steps if s.get("tool_name") == "fresh_final_check" and s.get("status") == "running"]
    final_passes = [s for s in coordinator_steps if s.get("phase") == "whole_final_check" and s.get("kind") == "checkpoint" and s.get("status") == "passed"]
    whole_claim = pipeline.get("full_task_success") is True
    fresh_claim = pipeline.get("fresh_final_check_passed") is True
    if scope == "component" and (whole_claim or fresh_claim or requests or final_passes):
        issue("component_promoted_to_whole", "pipeline", "Component records cannot establish whole-three-part completion")
    if scope == "full_task" and completed:
        if not whole_claim or not fresh_claim or len(requests) != 1 or len(final_passes) != 1:
            issue("missing_explicit_fresh_final_check", "coordinator", "Whole success needs one current-run fresh request and accepted checkpoint")
        if len(parts) != 3 or count != 3 or frozenset(names) != PARTS:
            issue("whole_part_set_mismatch", "pipeline", "Whole success lacks three current-run completions")
        if requests:
            request_time = timestamp(requests[0].get("timestamp"), "coordinator.fresh_request")
            if request_time and component_ends and request_time < max(component_ends):
                issue("stale_final_check", "coordinator", "Whole check began before all current components ended")
        if final_passes and final_passes[0].get("observation", {}).get("completed_part_count") != 3:
            issue("fresh_final_count_mismatch", "coordinator", "Fresh checkpoint does not cover three components")
    if whole_claim and (scope != "full_task" or not completed or not fresh_claim):
        issue("invalid_whole_success_claim", "pipeline", "Whole claim contradicts declared scope/completion")
    # Source/native record scope is independent from the generated coordinator.
    if native_path is None:
        native_path = next((ancestor / "pipeline-results.json" for ancestor in path.parents
                            if (ancestor / "pipeline-results.json").is_file()), None)
    native = read(Path(native_path).resolve()) if native_path is not None else {}
    if not native:
        issue("missing_native_source", "native", "Original native pipeline report required for scope comparison")
    component_session_path = (Path(native_path).resolve().parent / "native-ring-component-session-final.json"
                              if native_path else None)
    component_session_present = bool(component_session_path and component_session_path.exists())
    component_session = read(component_session_path) if component_session_present else {}
    native_component = (native.get("pin_only_component") is True or
                        native.get("requires_whole_three_part_trial") is True or component_session_present)
    # Optional authority is only loaded from this exact native report directory.
    # Its cancellation latch supersedes any late generic coordinator success.
    session_path = Path(native_path).resolve().parent / "native-phase-session-final.json" if native_path else None
    session_present = bool(session_path is not None and session_path.exists())
    session = read(session_path) if session_present else {}
    session_claim = session.get("accepted_full3") if session_present else None
    session_cancelled = session.get("cancelled") is True
    session_unfinished = session.get("finished") is False or session.get("unfinished") is True
    session_candidate_accepted = bool(session_present and session_claim is True and
                                      not session_cancelled and not session_unfinished and scope == "full_task")
    session_status = "not_present_legacy"
    unmatched_native_events = 0
    nested_component = False
    parent_common_path = None
    parent_part_observations: list[dict[str, Any]] = []
    if session_present and scope == "component" and not native_component and len(expected) == 1:
        # The PIN wrapper legitimately runs its own component coordinator while
        # the outer common full3 coordinator is paused at these same hooks.
        # Bind that role to an actual sibling current-run parent, never infer it
        # from a parent's success boolean or merely the presence of a session.
        native_directory = Path(native_path).resolve().parent
        candidates = sorted(set(native_directory.glob("capx-full3/*/pipeline-result.json")) |
                            ({native_directory / "capx-full3" / "pipeline-result.json"}
                             if (native_directory / "capx-full3" / "pipeline-result.json").is_file() else set()))
        matches = []
        for candidate in candidates:
            parent = read(candidate)
            parent_expected, parent_parts = parent.get("expected_parts"), parent.get("parts")
            if (parent.get("schema") != "fr3-capx-pipeline/v1" or parent.get("scope") != "full_task" or
                    parent.get("binding") != binding or parent.get("domain") != domain or
                    not isinstance(parent_expected, list) or len(parent_expected) != 3 or
                    any(not isinstance(name, str) for name in parent_expected) or frozenset(parent_expected) != PARTS or
                    not isinstance(parent_parts, list)):
                continue
            selected = [part for part in parent_parts if isinstance(part, dict) and part.get("part_name") == expected[0]]
            if len(selected) != 1 or (completed and selected[0].get("completed") is not True):
                continue
            history_path = candidate.parent / "components" / expected[0] / "execution_history_block_0.json"
            record_path(selected[0].get("history_path"), history_path, "nested_parent.part_history")
            history = read(history_path)
            if (history.get("binding") != binding or history.get("scope") != "component" or
                    history.get("domain") != domain or history.get("required_phases") != list(PHASES)):
                continue
            observations = [s.get("observation") for s in history.get("steps", []) if isinstance(s, dict) and
                            s.get("kind") == "observation" and s.get("status") == "observed" and
                            isinstance(s.get("observation"), dict)]
            indexed = {(obs.get("phase"), obs.get("attempt")): obs for obs in observations}
            if ((not observations and (completed or generic_phase_observations)) or
                    len(indexed) != len(observations) or any(obs.get("run_token") != session.get("run_token") or
                    obs.get("part_name") != expected[0] or obs.get("scope") != "component" or
                    obs.get("domain") != domain or obs.get("uses_oracle") is not False for obs in observations)):
                continue
            if any((obs.get("phase"), obs.get("attempt")) not in indexed or
                   indexed[(obs.get("phase"), obs.get("attempt"))].get("native_measured") != obs.get("native_measured")
                   for obs in generic_phase_observations):
                continue
            matches.append((candidate, observations))
        if len(matches) == 1:
            nested_component = True
            parent_common_path, parent_part_observations = matches[0]
        else:
            issue("nested_component_parent_unbound", "native_session", "Nested component requires one same-run parent with matching binding, part and measured phase evidence")
    if session_present:
        session_status = "cancelled" if session_cancelled else "incomplete" if not session_candidate_accepted else "accepted_claim"
        if nested_component and session_claim is True and not session_cancelled and not session_unfinished:
            session_status = "parent_accepted"
        if session_path.resolve().parent != Path(native_path).resolve().parent:
            issue("native_session_cross_run_path", "native_session", "Session authority must be inside this exact native run")
        if session.get("schema") not in ("native-phase-session/v2", "native-phase-session/v3"):
            issue("native_session_schema_mismatch", "native_session", "Unsupported authoritative session schema")
        for key in ("accepted_full3", "cancelled"):
            if type(session.get(key)) is not bool:
                issue("native_session_invalid_boolean", "native_session." + key, "Explicit boolean required")
        run_token = session.get("run_token")
        if not isinstance(run_token, str) or not run_token:
            issue("native_session_missing_run_token", "native_session", "Current native run token required")
        if session_claim is True and (session_cancelled or session_unfinished or
                session.get("worker_alive_after_abort") is True or session.get("terminal_failure") is not None or
                session.get("policy_error") is not None):
            issue("native_session_invalid_acceptance", "native_session", "Cancelled, unfinished or failed session cannot accept full3")
            session_candidate_accepted = False
        if session_candidate_accepted and (scope != "full_task" or not completed or not whole_claim or not fresh_claim):
            issue("native_session_generic_scope_mismatch", "native_session", "Native full3 acceptance requires matching current whole-task generic evidence")
        if session_candidate_accepted and root.name != "capx-full3" and root.parent.name != "capx-full3":
            issue("native_session_pipeline_role_mismatch", "native_session", "Whole native acceptance belongs only to a full_task capx-full3 artifact")
        events = session.get("events", [])
        if not isinstance(events, list) or any(not isinstance(event, dict) for event in events):
            issue("native_session_invalid_events", "native_session.events", "Session events must be a list of objects")
            events = []
        if scope == "component" and not component_session_present:
            events = [event for event in events if event.get("part_name") == expected[0]] if nested_component else []
        compared_observations = (parent_part_observations if nested_component else
                                 generic_phase_observations if scope == "full_task" or component_session_present else [])
        native_part_names = {"ring1": "ring_large", "ring2": "ring_small", "pin": "pin"}
        seen_actions, seen_keys = set(), set()
        event_by_action = {}
        previous_generation, previous_order, previous_attempt = 0, None, -1
        for event in events:
            native_part = event.get("part")
            part_name = native_part_names.get(native_part) if isinstance(native_part, str) else None
            phase, attempt, action, generation = (event.get(k) for k in ("phase", "attempt", "action_token", "generation"))
            if (event.get("kind") != "phase" or part_name is None or event.get("part_name") != part_name or
                    phase not in PHASES or type(attempt) is not int or attempt < 0):
                issue("native_session_event_identity", "native_session.events", "Native part/phase/attempt identity is invalid")
                continue
            if event.get("scope") != "component" or event.get("domain") != domain or event.get("uses_oracle") is not False:
                issue("native_session_event_role", "native_session.events", "Session phase events need component scope, matching domain and uses_oracle=False")
            if event.get("run_token") != run_token:
                issue("native_session_event_run_mismatch", "native_session.events", "Event belongs to another native session")
            if not isinstance(action, str) or not action or action in seen_actions:
                issue("native_session_duplicate_action", "native_session.events", "Every event must consume one unique nonempty action token")
            else:
                seen_actions.add(action)
                event_by_action[action] = event
            if type(generation) is not int or generation <= previous_generation:
                issue("native_session_generation_order", "native_session.events", "Event generations must increase strictly")
            else:
                previous_generation = generation
            event_key = (part_name, phase, attempt)
            if event_key in seen_keys:
                issue("native_session_duplicate_event", "native_session.events", "Part/phase/attempt event repeated")
            seen_keys.add(event_key)
            if part_name not in expected:
                issue("native_session_unexpected_part", "native_session.events", "Event is outside this pipeline's declared parts")
            else:
                order = (expected.index(part_name), PHASES.index(phase))
                if previous_order is None:
                    order_ok = order == (0, 0) and attempt == 0
                elif order == previous_order:
                    order_ok = attempt == previous_attempt + 1
                else:
                    next_order = (previous_order[0], previous_order[1] + 1) if previous_order[1] < len(PHASES) - 1 else (previous_order[0] + 1, 0)
                    order_ok = order == next_order and attempt == 0
                if not order_ok:
                    issue("native_session_phase_order", "native_session.events", "Native events must be the current-run declared phase prefix with explicit retries")
                previous_order, previous_attempt = order, attempt
            measured = event.get("native_measured")
            if not isinstance(measured, dict) or not measured:
                issue("native_session_missing_measurement", "native_session.events", "Actual nonempty measured evidence must accompany each event")
            elif any(isinstance(packet, dict) and "part" in packet and
                     packet["part"] not in (event.get("part"), part_name)
                     for packet in (measured, measured.get("sensor"))):
                issue("native_session_measured_part_mismatch", "native_session.events", "Measured part identity differs from the current native event")
            provenance = event.get("provenance")
            if isinstance(provenance, dict) and provenance.get("oracle") is not False:
                issue("native_session_provenance_conflict", "native_session.events", "Measured provenance contradicts oracle-free event")
        matched_actions = set()
        for observation in compared_observations:
            action = observation.get("action_token")
            event = event_by_action.get(action) if isinstance(action, str) else None
            if event is None:
                issue("native_session_generic_event_missing", "native_session", "Generic observation has no matching current native event")
                continue
            matched_actions.add(action)
            fields = ("run_token", "generation", "action_token", "part", "part_name", "phase", "attempt",
                      "scope", "domain", "uses_oracle", "native_measured")
            if any(observation.get(key) != event.get(key) for key in fields):
                issue("native_session_generic_measurement_mismatch", "native_session", "Generic observation differs from its native event identity or actual measurement")
        unmatched_prefix_started = False
        for event in events:
            matched = isinstance(event.get("action_token"), str) and event["action_token"] in matched_actions
            if not matched:
                unmatched_native_events += 1
                unmatched_prefix_started = True
            elif unmatched_prefix_started:
                issue("native_session_missing_middle_event", "native_session", "An unconsumed native event occurs before a later consumed event")
        if session_candidate_accepted and unmatched_native_events:
            issue("native_session_unconsumed_acceptance", "native_session", "Accepted native session has events missing from generic observations")
        if session_candidate_accepted:
            completed_native_phases = {(e.get("part_name"), e.get("phase")) for e in events}
            required_native_phases = {(name, phase) for name in PARTS for phase in PHASES}
            if completed_native_phases != required_native_phases:
                issue("native_session_incomplete_full3", "native_session", "Accepted session requires all eight phases of all three current-run parts")
    component_session_accepted = False
    component_session_status = "not_present_legacy"
    if component_session_present:
        cs = component_session
        native_names = {"ring_small": "ring2", "ring_large": "ring1"}
        expected_native = native_names.get(expected[0]) if len(expected) == 1 else None
        if (scope != "component" or domain != "simulation" or not expected_native or cs.get("scope") != "component" or
                cs.get("part") != expected_native or cs.get("schema") != "native-ring-component-session/v1"):
            issue("component_session_scope_mismatch", "component_session", "Single-ring authority must name this component only")
        if component_session_path.resolve().parent != Path(native_path).resolve().parent:
            issue("component_session_cross_run_path", "component_session", "Component authority must be in this exact native run")
        for key in ("accepted_component", "accepted_full3", "cancelled", "hardware"):
            if type(cs.get(key)) is not bool:
                issue("component_session_invalid_boolean", "component_session." + key, "Explicit boolean required")
        if cs.get("accepted_full3") is not False or cs.get("hardware") is not False:
            issue("component_session_scope_escalation", "component_session", "Simulation component authority cannot accept full3 or hardware")
        if session_present and session.get("accepted_full3") is not False:
            issue("component_session_parent_scope_escalation", "component_session", "Single-ring native phase authority must not accept full3")
        for label, authority in (("component_session", cs), ("native_session", session)):
            if type(authority.get("worker_alive_after_abort")) is not bool:
                issue("component_session_invalid_worker_boolean", label + ".worker_alive_after_abort", "Explicit worker state boolean required")
        if (not session_present or cs.get("run_token") != session.get("run_token") or
                cs.get("events") != session.get("events") or
                cs.get("final_sensor_evidence") != session.get("final_sensor_evidence") or
                any(cs.get(key) != session.get(key) for key in (
                    "cancelled", "worker_alive_after_abort", "terminal_failure", "policy_error"))):
            issue("component_session_parent_binding", "component_session", "Component authority must match the same native phase session and sensor data")
        claimed = cs.get("accepted_component") is True
        blocked = (cs.get("cancelled") is True or session_cancelled or session_unfinished or
                   cs.get("finished") is False or cs.get("unfinished") is True or
                   cs.get("worker_alive_after_abort") is True or cs.get("terminal_failure") is not None or
                   cs.get("policy_error") is not None)
        if claimed and blocked:
            issue("component_session_invalid_acceptance", "component_session", "Cancelled, unfinished, live-worker or failed component cannot accept completion")
        component_session_accepted = bool(claimed and not blocked and scope == "component" and expected_native)
        component_session_status = "cancelled" if cs.get("cancelled") is True else "accepted_claim" if component_session_accepted else "incomplete"
        if component_session_accepted:
            if not completed or whole_claim or fresh_claim or native.get("full_task_pass") is True:
                issue("component_session_generic_scope_mismatch", "component_session", "Native component acceptance requires a completed component without whole-task promotion")
            events = cs.get("events", [])
            events = events if isinstance(events, list) else []
            covered = {(e.get("part_name"), e.get("phase")) for e in events if isinstance(e, dict)}
            if covered != {(expected[0], phase) for phase in PHASES} or unmatched_native_events:
                issue("component_session_incomplete_phases", "component_session", "Accepted component requires its eight actual consumed phases")
            e = cs.get("final_sensor_evidence")
            e = e if isinstance(e, dict) else {}
            values = e.get("parts")
            values = values if isinstance(values, dict) else {}
            observation = values.get(expected_native)
            observation = observation if isinstance(observation, dict) else {}
            def finite_scalar(value):
                return type(value) in (int, float) and math.isfinite(value)
            times = [event.get("cycle_wall_s") for event in events if isinstance(event, dict)]
            captured, ready, requested = (observation.get("captured_wall_s"), observation.get("ready_wall_s"),
                                          e.get("requested_after_wall_s"))
            if (not times or not all(finite_scalar(t) for t in times) or
                    not all(finite_scalar(t) for t in (captured, ready, requested)) or
                    not (captured > requested > max(times) and ready >= captured and ready - captured <= 3.)):
                issue("component_session_stale_final_sensor", "component_session.final_sensor", "Final acquisition must follow the last phase and its new request")
            pose = observation.get("pose")
            pose = pose if isinstance(pose, dict) else {}
            if (set(values) != {expected_native} or e.get("confirmed") is not True or
                    e.get("scope") != "component" or e.get("domain") != domain or
                    e.get("uses_free_body_poses") is not False or observation.get("confirmed") is not True or
                    observation.get("part") != expected_native or observation.get("uses_free_body_poses") is not False or
                    pose.get("part") != expected_native or pose.get("uses_depth_sensor") is not True or
                    pose.get("uses_simulator_body_poses") is not False):
                issue("component_session_final_sensor_scope", "component_session.final_sensor", "Fresh direct sensor must cover this one component without an oracle")
            for key in ("observation_id", "camera_id", "calibration_id", "raw_acquisition_sha256"):
                if not isinstance(observation.get(key), str) or not observation[key]:
                    issue("component_session_final_sensor_binding", "component_session.final_sensor." + key, "Fresh acquisition identity is required")
            phase_sample_ids = set()
            for event in events:
                measured = event.get("native_measured", {}) if isinstance(event, dict) else {}
                measured = measured if isinstance(measured, dict) else {}
                sensor = measured.get("sensor", measured)
                if isinstance(sensor, dict) and isinstance(sensor.get("observation_id"), str):
                    phase_sample_ids.add(sensor["observation_id"])
            if isinstance(observation.get("observation_id"), str) and observation["observation_id"] in phase_sample_ids:
                issue("component_session_reused_final_observation", "component_session.final_sensor", "Additional final sensor cannot reuse a phase observation")
            if (not finite_scalar(observation.get("position_error_m")) or
                    not finite_scalar(observation.get("axis_error_rad")) or
                    not 0 <= observation["position_error_m"] < .0015 or
                    not 0 <= observation["axis_error_rad"] < .05):
                issue("component_session_final_sensor_limits", "component_session.final_sensor", "Confirmed ring observation contradicts original final limits")
            distances = e.get("hand_clearance_m")
            distances = distances if isinstance(distances, dict) else {}
            if (e.get("hands_clear") is not True or set(distances) != {"left", "right"} or
                    not all(finite_scalar(d) and d > .16 for d in distances.values())):
                issue("component_session_hand_clearance", "component_session.final_sensor", "Both measured palms must be more than 0.16m clear")
            session_status = "component_accepted_claim"
    bridge = native.get("capx_native_bridge", {})
    source_binding = bridge.get("source_binding", {}) if isinstance(bridge, dict) else {}
    translated = {"scene_sha256": source_binding.get("scene_id"),
                  "config_sha256": source_binding.get("config_id"),
                  "source_sha256": source_binding.get("source_id")}
    if translated != binding:
        issue("native_binding_mismatch", "native.source_binding", "Native and coordinator scene/config/source identities differ")
    if isinstance(bridge, dict):
        for event in bridge.get("events", []):
            if isinstance(event, dict) and event.get("source_scene_config") != source_binding:
                issue("native_event_binding_mismatch", "native.events", "Native execution event binding differs")
        if domain == "hardware" and bridge.get("simulation_only") is True:
            issue("native_domain_mismatch", "native", "Simulation-only native run cannot become hardware evidence")
    if native_component and (scope != "component" or whole_claim or native.get("full_task_pass") is True):
        issue("native_scope_escalation", "native", "Original component-only native run was promoted to whole task")
    if scope == "component" and native.get("full_task_pass") is True and not nested_component:
        issue("native_scope_mismatch", "native", "Native whole flag contradicts component evidence")
    native_component_name = native.get("component_part")
    if isinstance(native_component_name, str):
        native_component_name = {"ring1": "ring_large", "ring2": "ring_small"}.get(
            native_component_name, native_component_name)
    if scope == "component" and native_component and native_component_name not in (None, *expected):
        issue("native_component_identity_mismatch", "native", "Original native part identity differs from component record")
    if (scope == "component" and expected == ["pin"] and completed and
            native.get("pin_component_pass") is not True and not nested_component):
        issue("native_component_claim_missing", "native", "Pin completion lacks matching original native component report")
    if (scope == "full_task" and completed and native.get("full_task_pass") is not True and
            not (session_present and not session_candidate_accepted)):
        issue("native_whole_claim_missing", "native", "Whole callback claim lacks matching original native report")
    if domain == "hardware" and native.get("hardware_tested") is not True:
        issue("native_hardware_claim_missing", "native", "Hardware-labelled callbacks lack matching native hardware report")
    return dict(schema="fr3-capx-structural-evidence-audit/v1", pipeline_path=str(path),
                structural_consistency_pass=not issues, issue_count=len(issues), issues=issues,
                binding=binding, declared_scope=scope, declared_domain=domain,
                declared_completed=completed, declared_full_task_success=whole_claim,
                declared_completed_part_count=pipeline.get("completed_part_count"), phases=phase_summary,
                native_declared_component_only=native_component,
                native_declared_full_task_pass=native.get("full_task_pass") is True,
                native_session_present=session_present,
                native_session_path=str(session_path) if session_present else None,
                native_session_status=session_status,
                native_session_acceptance_claim=session_claim,
                native_session_comparison_role="nested_component" if nested_component else "whole_task" if scope == "full_task" else "component_only",
                nested_component_bound_to_parent=nested_component,
                parent_common_pipeline_path=str(parent_common_path) if parent_common_path is not None else None,
                accepted_native_session=bool((session_candidate_accepted or component_session_accepted) and not issues),
                native_common_full3_accepted=bool(session_candidate_accepted and not issues),
                native_ring_component_session_present=component_session_present,
                native_ring_component_session_status=component_session_status,
                native_ring_component_acceptance_claim=component_session.get("accepted_component") if component_session_present else None,
                native_ring_component_session_path=str(component_session_path) if component_session_present else None,
                native_common_ring_component_accepted=bool(component_session_accepted and not issues),
                generic_component_claim_superseded_by_native_session=bool(component_session_present and completed and not (component_session_accepted and not issues)),
                generic_claim_superseded_by_native_session=bool(session_present and not session_candidate_accepted and whole_claim),
                unmatched_native_event_count=unmatched_native_events,
                native_session_evidence_limits=[
                    "Acceptance means the current-run authoritative claim is structurally consistent, not physical verification.",
                    "Runtime clock origin, source immutability and raw capture authenticity are not independently re-proved by archived status records.",
                    "Session status does not archive independent final all-three raw sensor data; generic fresh-check records and the authoritative claim are compared only.",
                    "Original native collision/measurement validation remains required; this standalone audit never authorizes execution."
                ],
                physical_success_verified=False, native_collision_proof_verified=False,
                hardware_success_verified=False, qualification="structural_consistency_only",
                limitation="Record consistency does not verify physical success; original native observations and full path validation remain required.",
                input_sha256=files)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", help="Pipeline JSON or native run directory")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--workspace-root", type=Path)
    args = parser.parse_args(argv)
    paths = []
    for value in args.inputs:
        candidate = Path(value)
        if candidate.is_dir():
            paths.extend(sorted(set(candidate.glob("capx-components/*/pipeline-result.json")) |
                                set(candidate.glob("capx-full3/*/pipeline-result.json")) |
                                ({candidate / "capx-full3" / "pipeline-result.json"}
                                 if (candidate / "capx-full3" / "pipeline-result.json").is_file() else set())))
        else:
            paths.append(candidate)
    if not paths:
        parser.error("No current pipeline artifacts found")
    audits = [audit_pipeline(path, workspace_root=args.workspace_root) for path in paths]
    report = dict(schema="fr3-capx-structural-evidence-batch/v1", audited_runs=len(audits),
                  structurally_consistent_runs=sum(a["structural_consistency_pass"] for a in audits),
                  physical_success_verified=False, full_task_physical_success_verified=False, audits=audits)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"audited_runs": len(audits), "structurally_consistent_runs": report["structurally_consistent_runs"],
                      "issues": sum(a["issue_count"] for a in audits), "physical_success_verified": False,
                      "output": str(args.output.resolve())}))
    return 0 if all(a["structural_consistency_pass"] for a in audits) else 1


if __name__ == "__main__":
    raise SystemExit(main())
