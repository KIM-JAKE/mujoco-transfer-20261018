#!/usr/bin/env python3
"""Read-only native-run artifact accounting, never a robot/geometry qualification.

Only known literal report fields are interpreted. Missing legacy evidence remains
unknown. NPZ headers are inspected without loading array data, simulator body
poses, pickles or ground-truth traces. No control code, services or GPU are used.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import zipfile


class ScopeContradiction(ValueError):
    """Requested or claimed evidence scope contradicts an explicit component run."""


def _hash(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def _json(path: Path):
    def invalid(value):
        raise ValueError(f"Nonfinite JSON literal {value} in {path}")
    return json.loads(path.read_text(), parse_constant=invalid)


def _canonical_hash(value) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _bool(value) -> bool | None:
    return value if type(value) is bool else None


def _positive_int(value) -> bool:
    return type(value) is int and value > 0


def _npy_header(stream) -> dict:
    if stream.read(6) != b"\x93NUMPY":
        raise ValueError("Invalid NPY magic")
    version = tuple(stream.read(2))
    length_size = 2 if version == (1, 0) else 4 if version in ((2, 0), (3, 0)) else 0
    if not length_size:
        raise ValueError("Unknown NPY version")
    raw_length = stream.read(length_size)
    if len(raw_length) != length_size:
        raise ValueError("Truncated NPY header length")
    length = int.from_bytes(raw_length, "little")
    if not 0 < length <= 65536:
        raise ValueError("Unbounded or empty NPY header")
    header = ast.literal_eval(stream.read(length).decode("utf-8" if version == (3, 0) else "latin1").strip())
    shape, dtype = header.get("shape"), header.get("descr")
    if not isinstance(shape, tuple) or any(type(d) is not int or d < 0 for d in shape) or not isinstance(dtype, str):
        raise ValueError("Unsupported NPY shape/dtype")
    if re.fullmatch(r"[<>=|][fiu][1248]", dtype) is None:
        raise ValueError("Only numeric command/timing arrays are inspected")
    return {"shape": list(shape), "dtype": dtype, "fortran_order": header.get("fortran_order")}


def _issued(path: Path) -> dict:
    if not path.is_file():
        return {"status": "unknown", "reason": "issued-command artifact absent"}
    result = {"sha256": _hash(path), "numeric_values_checked": False,
              "ground_truth_or_trace_arrays_read": False, "status": "unknown"}
    try:
        with zipfile.ZipFile(path) as archive:
            headers = {}
            for name in ("command.npy", "time_s.npy", "actual_dof_position.npy", "actual_dof_velocity.npy"):
                if name in archive.namelist():
                    with archive.open(name) as stream:
                        headers[name] = _npy_header(stream)
                        expected_bytes = math.prod(headers[name]["shape"]) * int(headers[name]["dtype"][-1])
                        if archive.getinfo(name).file_size - stream.tell() != expected_bytes:
                            raise ValueError(f"Missing/truncated numeric payload for {name}")
            result["headers"] = headers
            command, times = headers.get("command.npy", {}), headers.get("time_s.npy", {})
            shape = command.get("shape", [])
            if len(shape) == 2 and shape[0] > 0 and shape[1] == 18 and times.get("shape") == [shape[0]]:
                result.update(status="present_header_only", rows=shape[0], dofs=18)
            else:
                result["reason"] = "Missing, empty or inconsistent command/time array headers"
    except (ValueError, OSError, KeyError, SyntaxError, zipfile.BadZipFile, struct.error) as exc:
        result["reason"] = str(exc)
    return result


def _source(directory: Path) -> dict:
    path = directory / "code-manifest.json"
    if not path.is_file():
        return {"status": "unknown", "source_hash": None, "reason": "code-manifest absent"}
    data = _json(path)
    if not isinstance(data, dict) or not data:
        return {"status": "unknown", "source_hash": None, "reason": "source manifest is not a nonempty file/hash map"}
    actual, missing, mismatched = {}, [], []
    base = (directory / "code_snapshot").resolve()
    for name, expected in data.items():
        if not isinstance(name, str) or not isinstance(expected, str) or re.fullmatch(r"[0-9a-f]{64}", expected) is None:
            return {"status": "unknown", "source_hash": None, "reason": "invalid source manifest entry"}
        source = (base / name).resolve()
        if not source.is_relative_to(base):
            raise ValueError("Source manifest path escapes its snapshot")
        if not source.is_file():
            missing.append(name)
        else:
            actual[name] = _hash(source)
            if actual[name] != expected:
                mismatched.append(name)
    verified = not missing and not mismatched
    return {"status": "hashes_match" if verified else "incomplete_or_changed", "source_hash": _canonical_hash(actual) if verified else None,
            "declared_source_hash": _canonical_hash(data), "manifest_sha256": _hash(path),
            "checked_files": len(actual), "missing": missing, "mismatched": mismatched}


def _rgbd(path: Path) -> dict:
    if not path.is_file():
        return {"status": "unknown", "measured_record_count": None, "raw_rgb_depth_frames_checked": False}
    rows = _json(path)
    if not isinstance(rows, list):
        return {"status": "unknown", "reason": "RGB-D record file is not a list", "sha256": _hash(path)}
    measured = 0
    for row in rows:
        if not isinstance(row, dict) or not isinstance(row.get("result"), dict):
            continue
        result = row["result"]
        poses = result.get("poses", [result.get("pose")])
        if not isinstance(poses, list):
            continue
        if any(isinstance(pose, dict) and pose.get("uses_depth_sensor") is True and pose.get("uses_simulator_body_poses") is False
               and _positive_int(pose.get("depth_points")) for pose in poses):
            measured += 1
    return {"status": "measurement_records_present" if measured else "unknown", "sha256": _hash(path),
            "records": len(rows), "measured_record_count": measured, "raw_rgb_depth_frames_checked": False,
            "note": "Depth-derived pose records only; camera calibration, raw frames and hardware accuracy are not qualified."}


def _bridge(data, report_path: Path | None = None) -> dict:
    if not isinstance(data, dict):
        return {"status": "unknown_or_not_applicable_legacy", "actual_physics_executions": None}
    count, events = data.get("actual_physics_executions"), data.get("events")
    execution_events = [event for event in events if isinstance(event, dict)
                        and event.get("side") in ("left", "right") and _positive_int(event.get("rows"))
                        and isinstance(event.get("field_id"), str) and event["field_id"].strip()
                        and isinstance(event.get("native_evidence"), dict) and bool(event["native_evidence"])] if isinstance(events, list) else []
    consistent = _positive_int(count) and len(execution_events) == count and data.get("simulation_only") is True
    result = {"status": "reported_execution_counter_and_event_records" if consistent else "unknown_or_inconsistent",
              "actual_physics_executions": count if type(count) is int and count >= 0 else None,
              "matching_execution_event_count": len(execution_events), "source_binding": data.get("source_binding"),
              "tracking_contract_id": data.get("tracking_contract_id"),
              "whole_three_part_completion_inferred": False,
              "native_event_claims_independently_verified": False,
              "note": "Counts are arm-field executions, not completed parts. Native event claims are recorded, not re-qualified by this audit."}
    if report_path:
        result["artifact_sha256"] = _hash(report_path)
    return result


def audit_native_artifacts(report_path: str | Path, *, expected_scope: str, bridge_report: str | Path | None = None) -> dict:
    """Account for literal native evidence without promoting flags into qualification."""
    if expected_scope not in ("component", "full_task"):
        raise ValueError("expected_scope must explicitly be component or full_task")
    report_path = Path(report_path).resolve()
    report = _json(report_path)
    if not isinstance(report, dict):
        raise ValueError("Pipeline report must be a JSON object")
    directory = report_path.parent
    scattered = report.get("scattered_pick", {})
    scattered = scattered if isinstance(scattered, dict) else {}
    component = report.get("pin_only_component") is True or scattered.get("component_only") is True
    full_claim = report.get("full_task_pass") is True
    if component and (expected_scope == "full_task" or full_claim or scattered.get("full_task_pass") is True):
        raise ScopeContradiction("Explicit PIN-only/component trial cannot supply whole-three-part evidence")
    if full_claim and (report.get("requires_whole_three_part_trial") is True or report.get("ring_tasks_executed") == 0):
        raise ScopeContradiction("Whole-task claim contradicts an explicit zero-ring-task/requires-whole-trial declaration")
    if expected_scope == "component" and full_claim and not component:
        raise ScopeContradiction("A full-task claim cannot be silently reclassified as a component trial")
    parts = scattered.get("parts", [])
    ring_flow = {}
    for part in parts if isinstance(parts, list) else []:
        if isinstance(part, dict) and part.get("part") in ("ring1", "ring2"):
            pickup, insertion = part.get("pickup"), part.get("insertion_fresh_RGBD")
            ring_flow[part["part"]] = (isinstance(pickup, dict) and pickup.get("state") == "RGBD_grasp_and_lift_confirmed"
                and isinstance(insertion, dict) and insertion.get("uses_depth_sensor") is True
                and insertion.get("uses_simulator_body_poses") is False and _positive_int(insertion.get("depth_points")))
    pin = scattered.get("pin", {})
    retention = pin.get("pin_postrelease_RGBD_retention") if isinstance(pin, dict) else None
    pin_flow = (isinstance(pin, dict) and pin.get("pin_single_stroke_pass") is True and isinstance(retention, dict)
                and retention.get("confirmed") is True and retention.get("uses_free_body_poses") is False
                and retention.get("after_release_withdraw_and_original_retention_wait") is True)
    same_run_three = not component and all(ring_flow.get(name) is True for name in ("ring1", "ring2")) and pin_flow
    issued = _issued(directory / "executed-actuator-commands.npz")
    fr3 = report.get("FR3_command_limit_audit")
    fr3_status = "unknown"
    if isinstance(fr3, dict):
        rows = issued.get("rows")
        if fr3.get("command_limit_checks") is True and fr3.get("complete_issued_field_checked_at_end") is True and _positive_int(fr3.get("samples")) and fr3.get("validated_sample_count") == fr3["samples"] and rows == fr3["samples"]:
            fr3_status = "reported_full_issued_field_audit_with_matching_row_count"
    bridge_path = Path(bridge_report).resolve() if bridge_report else directory / "native-bridge-evidence.json"
    bridge_data = _json(bridge_path) if bridge_path.is_file() else report.get("capx_native_bridge")
    claims = {key: _bool(report.get(key)) for key in ("full_task_pass", "pin_component_pass", "vision_end_to_end_pass", "hardware_tested", "part_attachments")}
    teleport = report.get("workpiece_teleports_after_initialization")
    known_teleport = teleport if type(teleport) is int and teleport >= 0 else None
    return {"schema": "native-artifact-audit/v1", "audit_only": True, "expected_scope": expected_scope,
            "declared_scope": "component" if component else "full_task" if full_claim else "unknown",
            "report_path": str(report_path), "report_sha256": _hash(report_path), "auditor_sha256": _hash(Path(__file__)),
            "raw_claims": claims, "workpiece_teleports_after_initialization": known_teleport,
            "same_report_three_part_flow_records": "present" if same_run_three else "unverified",
            "part_flow_records": {"ring1": ring_flow.get("ring1"), "ring2": ring_flow.get("ring2"), "pin": pin_flow or None},
            "ring_tasks_executed": report.get("ring_tasks_executed") if type(report.get("ring_tasks_executed")) is int else None,
            "source": _source(directory), "issued_commands": issued,
            "fr3_command_audit": {"status": fr3_status, "reported": fr3 if isinstance(fr3, dict) else None,
                                  "limits_independently_recomputed": False},
            "rgbd": _rgbd(directory / "rgb-observations.json"), "bridge": _bridge(bridge_data, bridge_path if bridge_path.is_file() else None),
            "qualification": {"motion_pass_inferred": False, "geometry_or_no_slip_qualified": False,
                              "hardware_qualified": False, "ground_truth_used_for_control": False,
                              "trace_ground_truth_arrays_read": False},
            "limitations": ["Raw pass/attachment/teleport flags are producer claims, not independently re-proved.",
                            "A PIN component and an arm execution count do not establish whole-three-part completion.",
                            "Absent or unfamiliar evidence remains unknown; no mock callback outcome is treated as physical motion."]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("--expected-scope", required=True, choices=("component", "full_task"))
    parser.add_argument("--bridge-report", type=Path)
    args = parser.parse_args()
    try:
        result = audit_native_artifacts(args.report, expected_scope=args.expected_scope, bridge_report=args.bridge_report)
    except (ValueError, OSError) as exc:
        print(json.dumps({"schema": "native-artifact-audit/v1", "status": "rejected", "error": str(exc)}))
        return 2
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
