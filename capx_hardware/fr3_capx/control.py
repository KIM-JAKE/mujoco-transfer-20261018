"""CaP-X-style dual FR3 control surface over trusted native callbacks.

No device connections or motion at import. Planner outputs are proposals only;
each execution goes through the bound native checker and one-use permission.
The Native Hand palm is the Cartesian tool frame. Native callbacks own its
explicit registered transform to the real controller TCP.
"""
from __future__ import annotations

import inspect
import math
import time
from typing import Any, Callable

import numpy as np

from .backend import Backend, ValidationRejected
from .execution import ExecutionLogger
from .perception import pose_from_xyz_wxyz, pose_to_xyz_wxyz, validate_pose


START_FIELDS = ("start_policy", "actual_start_q", "command_start_q")


def _name(value: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValidationRejected("A nonempty registered identifier is required")
    return value


def _vector(value: Any, length: int, label: str) -> list[float]:
    if np.asarray(value).dtype.kind == "b":
        raise ValidationRejected(f"{label} cannot contain booleans")
    arr = np.asarray(value, dtype=float)
    if arr.shape != (length,) or not np.isfinite(arr).all():
        raise ValidationRejected(f"{label} requires {length} finite values")
    return arr.tolist()


def object_target_to_hand(T_world_object_goal: Any, T_hand_object: Any) -> np.ndarray:
    """Rigid-body placement: T_world_hand = goal @ inv(measured T_hand_object).

    Both matrices must describe the same physical object origin. This algebra
    neither estimates an occluded axis nor compensates an unknown slipping grasp.
    Use a measured post-grasp relation and verify retention before applying it.
    """
    goal, grasp = validate_pose(T_world_object_goal), validate_pose(T_hand_object)
    return validate_pose(goal @ np.linalg.inv(grasp))


class ControlApi:
    """An explicit left or right API; never selects an arm implicitly.

    Native planners implement plan_motion. They must return the exact issued
    trajectory_q and time_s, already retimed for the requested speed scale.
    No controller home configuration or unverified direct edge is synthesized.
    """

    def __init__(self, backend: Backend, *, side: str,
                 logger: ExecutionLogger | None = None, speed_scale: float = 0.2,
                 max_observation_age_s: float = 2.0,
                 clock: Callable[[], float] = time.time):
        if side not in ("left", "right"):
            raise ValueError("ControlApi requires side='left' or 'right'")
        if isinstance(speed_scale, bool) or not 0 < speed_scale <= 0.95:
            raise ValueError("Invalid speed scale")
        if not math.isfinite(max_observation_age_s) or max_observation_age_s <= 0:
            raise ValueError("Invalid observation age")
        self.backend, self.side, self.logger = backend, side, logger
        self.speed_scale, self.clock = float(speed_scale), clock
        self.max_observation_age_s = max_observation_age_s

    def _log(self, tool: str, text: str, **metadata: Any) -> None:
        if self.logger is not None:
            self.logger.log_step(f"{self.side}_{tool}", text, **metadata)

    def get_state(self) -> dict:
        """Read both measured arms, openings, held-part and scene identities."""
        return self.backend.call("get_state", {})

    def _plan(self, target_kind: str, **target: Any) -> dict:
        request = dict(side=self.side, target_kind=target_kind,
                       speed_scale=self.speed_scale, **target)
        self._log("plan_motion", f"Planning {target_kind}", kind="planning", status="running")
        plan = self.backend.call("plan_motion", request)
        if not isinstance(plan, dict) or plan.get("supported") is False:
            raise ValidationRejected("Native planner returned no trajectory proposal")
        if "trajectory_q" not in plan or "time_s" not in plan:
            raise ValidationRejected("Native planner must return issued trajectory_q and time_s")
        if "side" in plan and plan["side"] != self.side:
            raise ValidationRejected("Native planner returned another arm's trajectory")
        # Keep the requested target with the issued field for the native checker.
        # Backend rejects stale start, out-of-bounds waypoints and invalid timing.
        motion = dict(side=self.side, trajectory_q=plan["trajectory_q"],
                      time_s=plan["time_s"], speed_scale=self.speed_scale,
                      target_request=request)
        for key in START_FIELDS:
            if key in plan:
                motion[key] = plan[key]
        if target_kind == "joint":
            path = np.asarray(plan["trajectory_q"], dtype=float)
            if path.ndim != 2 or path.shape[1] != 7 or len(path) < 2 or not np.allclose(
                    path[-1], target["target_q"], atol=1e-6, rtol=0):
                raise ValidationRejected("Planned joint endpoint differs from requested target")
        return motion

    def _execute(self, motion: dict) -> dict:
        proof = self.backend.call("validate_motion", motion)
        if proof.get("approved") is not True or not proof.get("proof_id"):
            raise ValidationRejected("No native motion permission")
        self._log("validate_motion", "Native issued motion approved", kind="validation",
                  status="passed", observation={"proof_id": proof["proof_id"],
                                                "checker_id": proof.get("checker_id")})
        result = self.backend.call("execute_motion", {"proof_id": proof["proof_id"]})
        self._log("execute_motion", "Native executor returned feedback", kind="motion",
                  status="observed", observation=result)
        return result

    def goto_joint_position(self, joints: Any) -> dict:
        """Plan, fully validate and execute a seven-joint target in radians."""
        return self._execute(self._plan("joint", target_q=_vector(joints, 7, "joints")))

    def goto_pose(self, position: Any, quaternion_wxyz: Any, z_approach: float = 0.0) -> dict:
        """Move palm to a world XYZ/WXYZ pose via native collision-aware planning.

        CaP-X-compatible z_approach is metres along NEGATIVE TARGET TOOL Z,
        not world up. With nonzero approach, request a fully checked path via
        that approach pose then a linear approach to the final pose. A native
        planner unable to honor this constraint must reject it, not omit it.
        No Panda TCP offset or controller home position is applied.
        """
        if isinstance(z_approach, bool) or not math.isfinite(z_approach) or z_approach < 0:
            raise ValidationRejected("z_approach must be finite nonnegative metres")
        target = pose_from_xyz_wxyz(position, quaternion_wxyz)
        xyz, quat = pose_to_xyz_wxyz(target)
        payload = dict(position=xyz.tolist(), quaternion_wxyz=quat.tolist(),
                       frame="world", tool_frame="native_hand", z_approach=float(z_approach))
        if z_approach:
            approach = target.copy()
            approach[:3, 3] -= target[:3, :3] @ np.array([0.0, 0.0, z_approach])
            payload.update(cartesian_waypoints_world=[approach.tolist(), target.tolist()],
                           final_approach_linear=True, approach_frame="target_tool")
        return self._execute(self._plan("pose", **payload))

    def goto_preinsert(self, endpoint_id: str) -> dict:
        """Move to a registered insertion-preceding JOINT pose, checking the live scene."""
        return self._execute(self._plan("preinsert", endpoint_id=_name(endpoint_id)))

    def execute_fixed_path(self, path_id: str) -> dict:
        """Execute a registered fixed stroke/withdrawal with live native admission."""
        return self._execute(self._plan("fixed_path", path_id=_name(path_id)))

    def goto_registered_pose(self, pose_id: str) -> dict:
        """Move to an explicitly registered custom readiness pose; never generic home."""
        pose = self.backend.call("get_registered_pose", {"side": self.side, "pose_id": _name(pose_id)})
        if pose.get("side") != self.side or pose.get("pose_id") != pose_id:
            raise ValidationRejected("Registered posture identity mismatch")
        return self.goto_joint_position(pose.get("q"))

    def _gripper(self, width_m: float, speed_m_s: float, force_n: float) -> dict:
        motion = dict(side=self.side, width_m=width_m, speed_m_s=speed_m_s,
                      force_n=force_n, speed_scale=self.speed_scale)
        proof = self.backend.call("validate_gripper", motion)
        if proof.get("approved") is not True or not proof.get("proof_id"):
            raise ValidationRejected("No native gripper permission")
        result = self.backend.call("execute_gripper", {"proof_id": proof["proof_id"]})
        self._log("gripper", "Gripper feedback; hold is not yet certified", kind="gripper",
                  status="observed", observation=result)
        return result

    def open_gripper(self, width_m: float = 0.08, *, speed_m_s: float = 0.02,
                     force_n: float = 20.0) -> dict:
        """Open Native Hand through native aperture-sweep admission; width in metres."""
        return self._gripper(width_m, speed_m_s, force_n)

    def close_gripper(self, width_m: float = 0.0, *, speed_m_s: float = 0.02,
                      force_n: float = 20.0) -> dict:
        """Close within configured native limits; then independently check the grasp."""
        return self._gripper(width_m, speed_m_s, force_n)

    def _measured(self, operation: str, **payload: Any) -> dict:
        result = self.backend.call(operation, dict(side=self.side, **payload))
        if result.get("side") != self.side:
            raise ValidationRejected("Native checkpoint response belongs to another arm")
        for key in ("part_name", "label"):
            if key in payload and result.get(key) != payload[key]:
                raise ValidationRejected(f"Native checkpoint {key} differs from requested identity")
        provenance = result.get("provenance", {})
        if (not isinstance(provenance, dict) or provenance.get("oracle") is not False or
                provenance.get("kind") not in {"camera", "rgbd", "encoder", "fused", "measured", "recorded_observation"}):
            raise ValidationRejected("Native checkpoint requires explicit sensor provenance")
        timestamp = result.get("timestamp_s")
        if isinstance(timestamp, bool) or not isinstance(timestamp, (float, int)) or not math.isfinite(timestamp):
            raise ValidationRejected("Native checkpoint requires a measured timestamp_s")
        age = self.clock() - timestamp
        if age < -0.1 or age > self.max_observation_age_s:
            raise ValidationRejected("Native checkpoint observation is stale")
        self._log(operation, "Fresh measured checkpoint feedback", kind="checkpoint",
                  status="observed", observation=result)
        return result

    def get_gripper_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """Read current encoder-derived world pose of the physical Native Hand palm."""
        result = self._measured("get_gripper_pose", frame="world", tool_frame="native_hand")
        if result.get("frame") != "world" or result.get("tool_frame") != "native_hand":
            raise ValidationRejected("Measured Hand pose frame mismatch")
        pose = result.get("pose")
        if pose is None:
            pose = pose_from_xyz_wxyz(result.get("position"), result.get("quaternion_wxyz"))
        return pose_to_xyz_wxyz(pose)

    def check_grasp(self, part_name: str) -> dict:
        """Read measured grasp/controlled-lift evidence; a model proposal is insufficient."""
        return self._measured("check_grasp", part_name=_name(part_name))

    def check_retention(self, part_name: str) -> dict:
        """Read fresh measured held-object relation/drift evidence after transport."""
        return self._measured("check_retention", part_name=_name(part_name))

    def check_completion(self, part_name: str) -> dict:
        """Read fresh post-release completion evidence, separate from commanded depth."""
        return self._measured("check_completion", part_name=_name(part_name))

    def breakpoint_code_block(self, label: str = "checkpoint") -> dict:
        """Return explicit native checkpoint feedback for higher-level reconsideration.

        Does not call a VLM, regenerate code, retry motion or move a camera.
        """
        return self._measured("get_checkpoint", label=_name(label))

    def functions(self) -> dict[str, Callable]:
        """CaP-X API mapping, scoped to this explicitly selected arm."""
        names = ("get_state", "goto_joint_position", "goto_pose", "goto_preinsert",
                 "execute_fixed_path", "goto_registered_pose", "open_gripper", "close_gripper",
                 "get_gripper_pose", "check_grasp", "check_retention", "check_completion",
                 "breakpoint_code_block")
        return {name: getattr(self, name) for name in names}

    def combined_doc(self) -> str:
        return "\n\n".join(f"{name}{inspect.signature(fn)}\n{inspect.getdoc(fn) or ''}"
                            for name, fn in self.functions().items())
