"""Explicit callback boundary for CaP-X APIs; importing this never connects a robot.

Native motion integrations must provide state, continuous collision/FR3 limit checks,
and execution callbacks. A planner result alone is never an execution permission.
All callbacks receive and return dictionaries. No sockets or services are started.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import secrets
import threading
import time
from typing import Any, Callable, Mapping


class BackendError(RuntimeError):
    """Base adapter boundary error."""


class UnsupportedOperation(BackendError):
    """A native implementation has not been explicitly connected."""


class ValidationRejected(BackendError):
    """A request or trusted checker result does not satisfy the contract."""


class StaleProof(ValidationRejected):
    """A permission expired, was consumed, or no longer describes current state."""


class Backend(ABC):
    @abstractmethod
    def call(self, operation: str, payload: dict) -> dict:
        """Call an explicitly supported operation, or raise UnsupportedOperation."""


READ_OPERATIONS = frozenset({
    "capture", "detect", "point", "plan_grasps", "plan_grasps_point_clouds",
    "get_observation", "get_object_pose", "sample_grasp_pose", "get_gripper_pose",
    "get_arm0_gripper_pose", "get_arm1_gripper_pose", "get_held_part_pose",
    "detect_objects", "segment", "estimate_pose", "plan_grasp", "plan_motion",
    "plan_to_preinsert", "get_completion", "check_grasp", "check_retention",
    "check_completion", "get_fixed_path", "get_registered_pose", "get_checkpoint",
})
PERCEPTION_OPERATIONS = READ_OPERATIONS - {
    "plan_motion", "plan_to_preinsert", "get_fixed_path", "get_registered_pose",
    "get_checkpoint",
}
PROVENANCE_KINDS = frozenset({"rgbd", "camera", "encoder", "fused", "measured", "recorded_observation"})
PROVENANCE_SOURCES = frozenset({"real_rgbd", "rendered_rgbd", "recorded_rgbd", "rgbd", "rgbd_estimate", "vision", "segmentation_model", "grasp_model"})
FORBIDDEN_KEYS = frozenset({"ground_truth", "ground_truth_pose", "privileged_pose", "sim_body_pose", "oracle_pose"})


def _json_copy(value: Any) -> Any:
    """Canonical JSON data only, including ndarray/list conversion when available."""
    def convert(obj):
        if hasattr(obj, "tolist"):
            return obj.tolist()
        raise TypeError(f"Not JSON data: {type(obj).__name__}")
    try:
        return json.loads(json.dumps(value, default=convert, allow_nan=False))
    except (TypeError, ValueError) as exc:
        raise ValidationRejected("Payload/state must contain finite JSON data") from exc


def _sensor_copy(value: Any) -> Any:
    """JSON depth nulls preserve invalid sensor pixels; all other data stay finite."""
    def visit(obj, depth=False):
        if hasattr(obj, "tolist"):
            obj = obj.tolist()
        if isinstance(obj, dict):
            return {key: visit(child, depth or key in ("depth", "depth_m")) for key, child in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [visit(child, depth) for child in obj]
        if depth and isinstance(obj, float) and not math.isfinite(obj):
            return None
        return obj
    return _json_copy(visit(value))


def digest(value: Any) -> str:
    return hashlib.sha256(json.dumps(_json_copy(value), sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def _assert_observed(value: dict) -> None:
    def visit(obj):
        if isinstance(obj, dict):
            if FORBIDDEN_KEYS.intersection(obj) or obj.get("privileged") is True or obj.get("oracle") is True:
                raise ValidationRejected("Privileged simulator/oracle data is excluded from observations")
            for child in obj.values():
                visit(child)
        elif isinstance(obj, list):
            for child in obj:
                visit(child)
    visit(value)
    provenance = value.get("provenance", {})
    if not isinstance(provenance, dict) or provenance.get("oracle") is not False or provenance.get("kind") in {"oracle", "ground_truth", "simulator_body_pose", "sim_body_pose", "privileged"} or not (
        provenance.get("kind") in PROVENANCE_KINDS or
        (provenance.get("source") in PROVENANCE_SOURCES and provenance.get("oracle") is False)
    ):
        raise ValidationRejected("Observation requires explicit camera/encoder/measurement provenance")


def _vector(values: Any, size: int, name: str) -> list[float]:
    if not isinstance(values, list) or len(values) != size or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValidationRejected(f"{name} must have {size} finite numeric values")
    return [float(v) for v in values]


class CallbackBackend(Backend):
    """Bind audited native callbacks. The default instance supports no motion.

    State must include source_id/scene_id/config_id and q, opening_m, held_part,
    each keyed by left/right. Include calibration/environment revisions in scene_id
    and state_id in state when available. Source/config IDs must be content hashes
    or revisions managed by the native integration, not LLM-supplied identifiers.

    checker_callback({motion,state,binding}) must return approved, collision_free,
    continuous_path_checked, fr3_limits_checked all True, checker_id and the exact
    binding. FR3 checks must include interpolation, position-dependent velocity,
    acceleration and jerk for the actual native controller (see fieldkit servo_core).
    execute_callback receives {motion,proof}; it must atomically recheck this state
    binding under the native controller lock before sending actuator commands.

    Optional start_policy='native_command_reference' preserves an existing native
    issued trajectory exactly: actual_start_q binds measured state.q[side], and
    command_start_q binds both trajectory_q[0] and trusted state.command_q[side].
    No measured waypoint is inserted. A nonempty trusted tracking_contract_id and
    checker approvals for the actual-to-command bridge, fixed tracking envelope,
    and issued-array FR3 limits are mandatory. The contract ID must identify the
    native tracking calibration; include its content/revision and fixed envelope
    in state/config identity. No tracking tolerance can be supplied by an agent.

    If target_request is present, it binds the original joint/pose/preinsert/fixed
    path goal. The trusted checker must additionally return target_request_checked
    True and target_request_digest == digest(motion['target_request']). This means
    the actually issued path satisfies endpoint FK/registered joints or path and
    approach constraints under fixed native tolerances. Echoing a planner's goal
    is not qualification. The digest is also included in the execution proof.
    """
    def __init__(self, callbacks: Mapping[str, Callable[[dict], dict]] | None = None, *,
                 state_callback: Callable[[], dict] | None = None,
                 checker_callback: Callable[[dict], dict] | None = None,
                 execute_callback: Callable[[dict], dict] | None = None,
                 source_id: str = "unbound", scene_id: str = "unbound", config_id: str = "unbound",
                 joint_bounds: Mapping[str, list] | None = None,
                 max_speed_scale: float = 0.2, proof_ttl_s: float = 5.0,
                 clock: Callable[[], float] = time.monotonic):
        self.callbacks = dict(callbacks or {})
        unknown = set(self.callbacks) - READ_OPERATIONS
        if unknown:
            raise UnsupportedOperation(f"Callbacks cannot bypass validation: {sorted(unknown)}")
        self.state_callback, self.checker_callback, self.execute_callback = state_callback, checker_callback, execute_callback
        self.identity = dict(source_id=source_id, scene_id=scene_id, config_id=config_id)
        self.joint_bounds = _json_copy(dict(joint_bounds or {}))
        if not 0 < max_speed_scale <= 0.95 or not 0 < proof_ttl_s <= 60:
            raise ValueError("Invalid FR3 speed cap or proof TTL")
        self.max_speed_scale, self.proof_ttl_s, self.clock = max_speed_scale, proof_ttl_s, clock
        self._proofs: dict[str, dict] = {}
        self._lock = threading.RLock()

    def _state(self) -> dict:
        if self.state_callback is None:
            raise UnsupportedOperation("No native state callback connected")
        state = _json_copy(self.state_callback())
        if not isinstance(state, dict) or any(state.get(k) != v or v == "unbound" for k, v in self.identity.items()):
            raise ValidationRejected("Source/scene/config identity is unbound or changed")
        for side in ("left", "right"):
            _vector(state.get("q", {}).get(side), 7, f"{side} q")
            opening = state.get("opening_m", {}).get(side)
            if isinstance(opening, bool) or not isinstance(opening, (float, int)) or not 0 <= opening <= 0.08:
                raise ValidationRejected("Both Native Hand openings must be measured in meters")
            if side not in state.get("held_part", {}):
                raise ValidationRejected("Both held-part identities must be present (None means empty)")
        return state

    def _motion(self, payload: dict, state: dict, gripper: bool = False) -> dict:
        motion = _json_copy(payload)
        def reject_tracking_overrides(value):
            if isinstance(value, dict):
                if any("tracking" in str(key).lower() for key in value):
                    raise ValidationRejected("Tracking limits/contracts are native configuration; agent overrides are forbidden")
                for child in value.values():
                    reject_tracking_overrides(child)
            elif isinstance(value, list):
                for child in value:
                    reject_tracking_overrides(child)
        reject_tracking_overrides(motion)
        side = motion.get("side")
        if side not in ("left", "right"):
            raise ValidationRejected("Explicit left/right side required")
        if "target_request" in motion:
            target_request = motion["target_request"]
            if not isinstance(target_request, dict) or target_request.get("side") != side or target_request.get("target_kind") not in ("joint", "pose", "preinsert", "fixed_path"):
                raise ValidationRejected("target_request must bind the same side and a joint/pose/preinsert/fixed_path goal")
        speed = motion.get("speed_scale")
        if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not 0 < speed <= self.max_speed_scale:
            raise ValidationRejected("Explicit speed_scale exceeds configured FR3 cap")
        if gripper:
            if any(key in motion for key in ("start_policy", "actual_start_q", "command_start_q")):
                raise ValidationRejected("Joint command-reference start policy does not apply to gripper commands")
            width = motion.get("width_m")
            if isinstance(width, bool) or not isinstance(width, (int, float)) or not 0 <= width <= 0.08:
                raise ValidationRejected("Native Hand width_m must lie in [0, 0.08]")
            for key, maximum in (("speed_m_s", 0.1), ("force_n", 70.0)):
                value = motion.get(key)
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 < value <= maximum:
                    raise ValidationRejected(f"Explicit {key} and native Hand limit required")
            motion["kind"] = "gripper"
            return motion
        path = motion.get("trajectory_q")
        if not isinstance(path, list) or not 2 <= len(path) <= 100000:
            raise ValidationRejected("trajectory_q must contain 2..100000 seven-joint waypoints")
        path = [_vector(q, 7, "trajectory waypoint") for q in path]
        bounds = self.joint_bounds.get(side)
        if not isinstance(bounds, list) or len(bounds) != 2:
            raise ValidationRejected("Native joint bounds not connected")
        lower, upper = (_vector(x, 7, "joint bounds") for x in bounds)
        if any(lo >= hi for lo, hi in zip(lower, upper)) or any(not lo <= q[j] <= hi for q in path for j, (lo, hi) in enumerate(zip(lower, upper))):
            raise ValidationRejected("Trajectory exceeds native joint bounds")
        policy = motion.get("start_policy", "measured")
        if policy == "native_command_reference":
            actual_start = _vector(motion.get("actual_start_q"), 7, "actual_start_q")
            command_start = _vector(motion.get("command_start_q"), 7, "command_start_q")
            commands = state.get("command_q")
            if not isinstance(commands, dict):
                raise ValidationRejected("Native command-reference start requires trusted state.command_q")
            trusted_command = _vector(commands.get(side), 7, f"trusted {side} command_q")
            tracking_id = state.get("tracking_contract_id")
            if not isinstance(tracking_id, str) or not tracking_id.strip():
                raise ValidationRejected("Native command-reference start requires trusted tracking_contract_id")
            for expected, current, description in (
                (actual_start, state["q"][side], "actual_start_q differs from measured current state"),
                (path[0], command_start, "Trajectory does not start at command_start_q"),
                (command_start, trusted_command, "command_start_q differs from trusted current command"),
            ):
                if any(abs(a-b) > 1e-6 for a, b in zip(expected, current)):
                    raise ValidationRejected(description)
            motion["actual_start_q"], motion["command_start_q"] = actual_start, command_start
        elif policy == "measured":
            if "actual_start_q" in motion or "command_start_q" in motion:
                raise ValidationRejected("Actual/command start fields require explicit native_command_reference policy")
            if any(abs(a-b) > 1e-6 for a, b in zip(path[0], state["q"][side])):
                raise ValidationRejected("Trajectory must start at the measured current joint state")
        else:
            raise ValidationRejected("Unknown trajectory start policy")
        times = _vector(motion.get("time_s"), len(path), "waypoint time_s")
        if times[0] != 0 or times[-1] > 60 or any(b <= a for a, b in zip(times, times[1:])):
            raise ValidationRejected("Strictly increasing waypoint times must start at zero and end within 60 s")
        motion["trajectory_q"], motion["time_s"], motion["kind"] = path, times, "joint_trajectory"
        return motion

    def _validate(self, payload: dict, gripper: bool = False) -> dict:
        if self.checker_callback is None or self.execute_callback is None:
            raise UnsupportedOperation("Native collision/FR3 checker and executor are not connected")
        state = self._state()
        motion = self._motion(payload, state, gripper)
        binding = {**self.identity, "state_digest": digest(state), "motion_digest": digest(motion)}
        if "target_request" in motion:
            binding["target_request_digest"] = digest(motion["target_request"])
        native_command_start = motion.get("start_policy") == "native_command_reference"
        if native_command_start:
            binding["tracking_contract_id"] = state["tracking_contract_id"]
        checked = _json_copy(self.checker_callback({"motion": deepcopy(motion), "state": deepcopy(state), "binding": deepcopy(binding)}))
        required = ("approved", "collision_free", "continuous_path_checked", "fr3_limits_checked")
        if not isinstance(checked, dict) or any(checked.get(k) is not True for k in required) or checked.get("binding") != binding or not checked.get("checker_id"):
            raise ValidationRejected("Trusted checker did not approve the complete bound native motion")
        if native_command_start and (
            any(checked.get(key) is not True for key in (
                "actual_command_start_bridge_checked", "tracking_envelope_checked", "issued_array_fr3_checked"))
            or checked.get("tracking_contract_id") != state["tracking_contract_id"]
        ):
            raise ValidationRejected("Native checker did not qualify the command bridge, fixed tracking envelope and original issued-array FR3 limits")
        if "target_request" in motion and (
            checked.get("target_request_checked") is not True
            or checked.get("target_request_digest") != binding["target_request_digest"]
        ):
            raise ValidationRejected("Native checker did not qualify issued-path target/approach fulfillment against the bound original request")
        if digest(self._state()) != binding["state_digest"]:
            raise StaleProof("State changed during native validation")
        proof_id = secrets.token_urlsafe(24)
        expires = self.clock() + self.proof_ttl_s
        proof = {"proof_id": proof_id, "expires_at_monotonic_s": expires, "binding": binding, "checker_id": checked["checker_id"]}
        self._proofs[proof_id] = {"motion": motion, "proof": proof}
        return deepcopy({"approved": True, **proof, "hardware_connected": False, "note": "Callback qualification does not establish hardware commissioning"})

    def _execute(self, payload: dict, gripper: bool = False) -> dict:
        if set(payload) != {"proof_id"}:
            raise ValidationRejected("Execution accepts only an issued proof_id; targets cannot be replaced")
        proof_id = payload.get("proof_id")
        if not isinstance(proof_id, str):
            raise StaleProof("Missing proof")
        record = self._proofs.pop(proof_id, None)
        if record is None:
            raise StaleProof("Proof is unknown, already consumed, or belongs to another backend")
        proof = record["proof"]
        expected_kind = "gripper" if gripper else "joint_trajectory"
        if record["motion"].get("kind") != expected_kind:
            raise ValidationRejected("Proof operation kind does not match requested execution operation")
        if self.clock() >= proof["expires_at_monotonic_s"]:
            raise StaleProof("Native motion proof expired")
        if digest(self._state()) != proof["binding"]["state_digest"] or self.identity != {k: proof["binding"][k] for k in self.identity}:
            raise StaleProof("Current joints/opening/held part/source/scene/config differ from proof")
        if self.clock() >= proof["expires_at_monotonic_s"]:
            raise StaleProof("Native motion proof expired while reading current state")
        if self.execute_callback is None:
            raise UnsupportedOperation("No native executor connected")
        result = _json_copy(self.execute_callback(deepcopy(record)))
        if not isinstance(result, dict):
            raise ValidationRejected("Native executor must return a dictionary")
        return result

    def call(self, operation: str, payload: dict) -> dict:
        if not isinstance(payload, dict):
            raise ValidationRejected("Operation payload must be a dictionary")
        with self._lock:
            if operation == "get_state":
                return deepcopy(self._state())
            if operation in ("validate_motion", "validate_gripper"):
                return self._validate(payload, operation == "validate_gripper")
            if operation in ("execute_motion", "execute_gripper"):
                return self._execute(payload, operation == "execute_gripper")
            if operation not in READ_OPERATIONS or operation not in self.callbacks:
                raise UnsupportedOperation(f"No explicit native callback for {operation}")
            result = _sensor_copy(self.callbacks[operation](_sensor_copy(payload)))
            if not isinstance(result, dict):
                raise ValidationRejected("Native callback must return a dictionary")
            if operation in PERCEPTION_OPERATIONS:
                _assert_observed(result)
            return result


class RecordedBackend(Backend):
    """Read only explicit recorded observations, never simulator body-pose or control.

    Record layout: {observations: {operation: dict | {query_key: dict}}, ...}.
    query_key is payload.object_name, payload.part_name, or '*' for an unkeyed
    operation. A raw pipeline JSON may be supplied, but only an explicit
    `observations` map can be exposed. full_task_pass is not an observation.
    """
    def __init__(self, record: dict | str | Path):
        self.path = str(Path(record).resolve()) if isinstance(record, (str, Path)) else None
        self.record = _json_copy(json.loads(Path(record).read_text()) if self.path else record)
        if not isinstance(self.record, dict):
            raise ValidationRejected("Recorded backend requires a JSON object")
        self.record_digest = digest(self.record)

    def call(self, operation: str, payload: dict) -> dict:
        if not isinstance(payload, dict) or operation not in PERCEPTION_OPERATIONS:
            raise UnsupportedOperation("RecordedBackend only supports explicit recorded measurements; control is unavailable")
        result = deepcopy(self.record.get("observations", {}).get(operation))
        key = payload.get("object_name", payload.get("part_name", "*"))
        if isinstance(result, dict) and "provenance" not in result:
            result = result.get(key)
        if not isinstance(result, dict):
            raise UnsupportedOperation(f"No explicit recorded observation for {operation}/{key}")
        _assert_observed(result)
        result["recording"] = {"path": self.path, "sha256": self.record_digest, "live": False}
        return result
