"""Structured execution feedback and bounded callback recovery.

Adapted from CaP-X ``capx/utils/execution_logger.py`` at commit
53e9966d7a8e2fa7494676772bccc35280f5c0ed, Copyright (c) 2026 Max Fu, MIT.
See THIRD_PARTY_LICENSES/CaP-X-MIT.txt. This module has no robot driver, code
``exec``, model client, websocket, global history or automatic VLM invocation.
Image references replace embedded image data to preserve auditable artifacts.
"""
from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence


FULL_TASK_PHASES = (
    "perception", "grasp", "carry", "preinsert", "fixed_insert", "release", "withdraw", "final_check"
)


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass(frozen=True)
class Failure:
    code: str
    message: str
    retryable: bool = False
    details: Mapping[str, Any] = field(default_factory=dict)


class PhaseFailure(RuntimeError):
    def __init__(self, failure: Failure):
        super().__init__(failure.message)
        self.failure = failure


class TerminalStop(RuntimeError):
    """Run must remain stopped; constructing a new run is an explicit action."""


@dataclass
class ExecutionStep:
    """CaP-X feedback record extended with phase and structured checkpoints."""
    tool_name: str
    text: str
    images: list[str] = field(default_factory=list)
    timestamp: str = field(default_factory=_utc_now_iso)
    step_index: int = 0
    highlight: bool = False
    phase: str = ""
    kind: str = "observation"
    status: str = "observed"
    elapsed_s: float = 0.0
    attempt: int = 0
    observation: dict[str, Any] = field(default_factory=dict)
    failure: Failure | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def to_vlm_format(self, include_images: bool = False) -> dict[str, Any]:
        """Format only; this function does not invoke a model."""
        result = dict(tool=self.tool_name, description=self.text,
                      timestamp=self.timestamp, phase=self.phase, status=self.status)
        if self.failure:
            result["failure"] = asdict(self.failure)
        if include_images and self.images:
            result["image_refs"] = list(self.images)
            result["num_images"] = len(self.images)
        return result


@dataclass
class ExecutionHistory:
    steps: list[ExecutionStep] = field(default_factory=list)
    code_block_index: int = 0
    start_time: str = field(default_factory=_utc_now_iso)
    end_time: str | None = None
    domain: str = "simulation"
    scope: str = "component"
    binding: dict[str, str] = field(default_factory=dict)
    recovery_used: bool = False
    outcome: str = "running"
    required_phases: list[str] = field(default_factory=list)

    def add_step(self, step: ExecutionStep) -> None:
        step.step_index = len(self.steps)
        self.steps.append(step)

    def to_dict(self) -> dict[str, Any]:
        return {"schema": "fr3-capx-execution/v1", **asdict(self)}

    def get_steps_for_vlm(self, include_images: bool = False) -> list[dict[str, Any]]:
        return [step.to_vlm_format(include_images) for step in self.steps]

    def to_vlm_summary(self, include_image_counts: bool = True) -> str:
        lines = [f"## Execution History (Code Block {self.code_block_index})",
                 f"Started: {self.start_time}", f"Domain: {self.domain}; scope: {self.scope}",
                 f"Outcome: {self.outcome}; recovery_used: {self.recovery_used}"]
        if self.end_time:
            lines.append(f"Ended: {self.end_time}")
        for i, step in enumerate(self.steps):
            lines.extend([f"### Step {i + 1}: {step.tool_name}", step.text])
            if include_image_counts and step.images:
                lines.append(f"*({len(step.images)} referenced image(s))*")
        return "\n".join(lines)

    def save_to_directory(self, output_dir: Path | str) -> Path:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        target = output_dir / f"execution_history_block_{self.code_block_index}.json"
        target.write_text(json.dumps(self.to_dict(), indent=2, allow_nan=False) + "\n")
        return target

    @classmethod
    def from_file(cls, path: Path | str) -> "ExecutionHistory":
        data = json.loads(Path(path).read_text())
        if data.pop("schema", None) != "fr3-capx-execution/v1":
            raise ValueError("Unsupported execution history schema")
        steps = []
        for record in data.pop("steps", []):
            if record.get("failure") is not None:
                record["failure"] = Failure(**record["failure"])
            steps.append(ExecutionStep(**record))
        return cls(steps=steps, **data)


class ExecutionLogger:
    """Instance-local CaP-X-style step logging; no implicit model feedback."""
    def __init__(self, history: ExecutionHistory | None = None):
        self.history = history if history is not None else ExecutionHistory()
        self._started = time.monotonic()

    def log_step(self, tool_name: str, text: str, images: list[str] | None = None,
                 highlight: bool = False, **metadata: Any) -> ExecutionStep:
        step = ExecutionStep(tool_name=tool_name, text=text, images=list(images or []),
                             highlight=highlight, elapsed_s=time.monotonic() - self._started,
                             **metadata)
        # Fail before recording observations that cannot round-trip to JSON.
        json.dumps(step.to_dict(), allow_nan=False)
        self.history.add_step(step)
        return step

    def log_step_update(self, text: str | None = None,
                        images: list[str] | None = None) -> None:
        if not self.history.steps:
            return
        last_step = self.history.steps[-1]
        if text:
            last_step.text = f"{last_step.text}\n\n{text}"
        if images:
            last_step.images.extend(images)

    def checkpoint(self, phase: str, passed: bool, *, observation: dict[str, Any] | None = None,
                   images: list[str] | None = None, attempt: int = 0) -> ExecutionStep:
        return self.log_step("checkpoint", f"{phase}: {'passed' if passed else 'failed'}",
                             phase=phase, kind="checkpoint", status="passed" if passed else "failed",
                             observation=observation or {}, images=images, attempt=attempt)

    def finalize(self, outcome: str, *, recovery_used: bool = False) -> ExecutionHistory:
        if self.history.end_time is not None:
            raise TerminalStop("Execution history is already finalized")
        self.history.outcome = outcome
        self.history.recovery_used = recovery_used
        self.history.end_time = _utc_now_iso()
        return self.history

    def functions(self) -> dict[str, Callable[..., Any]]:
        return {name: getattr(self, name) for name in (
            "log_step", "log_step_update", "checkpoint"
        )}


@dataclass(frozen=True)
class PhaseResult:
    observation: dict[str, Any] = field(default_factory=dict)
    image_refs: tuple[str, ...] = ()


@dataclass(frozen=True)
class RunResult:
    completed: bool
    recovery_used: bool
    domain: str
    scope: str
    outcome: str
    required_phases: tuple[str, ...] = ()
    completed_phases: tuple[str, ...] = ()

    @property
    def normal_success(self) -> bool:
        return self.completed and not self.recovery_used and self.outcome == "normal_success"


class BoundedExecutor:
    """Invoke reviewed operations with explicit checkpoints and bounded recovery.

    The caller owns operations and native safety checks. A recovery callback is
    only selected after a structured retryable failure and is followed by a
    fresh operation and checkpoint. A recovered run is never a normal success.
    Exceptions, exhausted budgets and non-retryable failures stop all later
    phases. An optional required_phases declaration blocks missing, duplicate
    or reordered phases before an operation runs, and blocks early finish.
    Without a declaration this remains a generic callback executor; its result
    does not certify full-task coverage. There is no automatic VLM call or
    unsafe action generation.
    """
    def __init__(self, logger: ExecutionLogger | None = None, *, retry_budget: int = 0,
                 required_phases: Sequence[str] | None = None):
        if type(retry_budget) is not int or retry_budget < 0:
            raise ValueError("retry_budget must be a non-negative integer")
        self.logger = logger if logger is not None else ExecutionLogger()
        if required_phases is None and self.logger.history.required_phases:
            required_phases = self.logger.history.required_phases
        if required_phases is not None:
            if isinstance(required_phases, (str, bytes)):
                raise ValueError("required_phases must be a sequence of unique phase names")
            required_phases = tuple(required_phases)
            if (not required_phases or
                    any(not isinstance(phase, str) or not phase.strip() for phase in required_phases) or
                    len(required_phases) != len(set(required_phases))):
                raise ValueError("required_phases must be nonempty and contain unique phase names")
            if (self.logger.history.required_phases and
                    tuple(self.logger.history.required_phases) != required_phases):
                raise ValueError("required_phases conflicts with execution history")
        self.required_phases = required_phases
        self.logger.history.required_phases = list(required_phases or ())
        self.retry_budget = retry_budget
        self.retries_used = 0
        self.recovery_used = False
        self.stopped = False
        self.finished = False
        self.completed_phases: list[str] = []

    def _stop(self, failure: Failure, phase: str, attempt: int) -> None:
        self.stopped = True
        self.logger.log_step("terminal_stop", failure.message, phase=phase, kind="failure",
                             status="stopped", attempt=attempt, failure=failure)
        self.logger.finalize("failed", recovery_used=self.recovery_used)
        raise TerminalStop(f"{phase}: {failure.code}: {failure.message}")

    def run_phase(self, phase: str, operation: Callable[[], PhaseResult], *,
                  checkpoint: Callable[[PhaseResult], bool],
                  recovery: Callable[[Failure, int], None] | None = None,
                  max_retries: int = 0) -> PhaseResult:
        if self.stopped or self.finished:
            raise TerminalStop("Execution has stopped or completed")
        if type(max_retries) is not int or max_retries < 0:
            raise ValueError("max_retries must be a non-negative integer")
        if self.required_phases is not None:
            index = len(self.completed_phases)
            expected = self.required_phases[index] if index < len(self.required_phases) else None
            if phase != expected:
                self._stop(Failure("phase_order_invalid", "Phase is missing, duplicate or out of order",
                                   details={"requested": phase, "expected": expected,
                                            "completed": list(self.completed_phases),
                                            "required": list(self.required_phases)}), phase, 0)
        attempt = 0
        while True:
            self.logger.log_step(phase, "Operation started", phase=phase, kind="phase",
                                 status="running", attempt=attempt)
            try:
                result = operation()
                if not isinstance(result, PhaseResult):
                    raise TypeError("operation must return PhaseResult")
                self.logger.log_step(phase, "Operation feedback", phase=phase, kind="observation",
                                     observation=result.observation, images=list(result.image_refs),
                                     attempt=attempt)
                # An explicit positive bool is necessary; truthy strings/objects
                # must not accidentally approve a checkpoint.
                passed = checkpoint(result) is True
                self.logger.checkpoint(phase, passed, observation=result.observation,
                                       images=list(result.image_refs), attempt=attempt)
                if not passed:
                    raise PhaseFailure(Failure("checkpoint_failed", f"{phase} checkpoint rejected",
                                               retryable=True, details=result.observation))
                self.completed_phases.append(phase)
                return result
            except PhaseFailure as exc:
                failure = exc.failure
            except Exception as exc:
                failure = Failure("operation_exception", str(exc), retryable=False,
                                  details={"exception_type": type(exc).__name__})
            self.logger.log_step(phase, failure.message, phase=phase, kind="failure",
                                 status="failed", attempt=attempt, failure=failure)
            if (not failure.retryable or recovery is None or attempt >= max_retries or
                    self.retries_used >= self.retry_budget):
                self._stop(failure, phase, attempt)
            attempt += 1
            self.retries_used += 1
            self.recovery_used = True
            self.logger.log_step("recovery", "Bounded recovery selected after failure",
                                 phase=phase, kind="recovery", status="running", attempt=attempt,
                                 failure=failure)
            try:
                recovery(failure, attempt)
            except Exception as exc:
                self._stop(Failure("recovery_exception", str(exc), details={
                    "exception_type": type(exc).__name__}), phase, attempt)

    def finish(self) -> RunResult:
        if self.stopped or self.finished:
            raise TerminalStop("Execution has stopped or completed")
        if not self.completed_phases:
            self._stop(Failure("empty_run", "No phase completed"), "finalize", 0)
        if (self.required_phases is not None and
                tuple(self.completed_phases) != self.required_phases):
            self._stop(Failure("incomplete_phase_plan", "Declared phase plan is incomplete",
                               details={"completed": list(self.completed_phases),
                                        "required": list(self.required_phases)}), "finalize", 0)
        self.finished = True
        outcome = "recovered_success" if self.recovery_used else "normal_success"
        self.logger.finalize(outcome, recovery_used=self.recovery_used)
        return RunResult(True, self.recovery_used, self.logger.history.domain,
                         self.logger.history.scope, outcome, tuple(self.required_phases or ()),
                         tuple(self.completed_phases))

    def functions(self) -> dict[str, Callable[..., Any]]:
        return {"run_phase": self.run_phase, "finish": self.finish}
