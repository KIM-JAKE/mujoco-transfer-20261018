"""Evidence-bound component and three-part orchestration over reviewed callbacks.

This coordinator never selects a robot backend, invokes a VLM, or promotes a
component result into whole-task proof. Each component must complete the same
eight explicit phases. Whole completion additionally needs a fresh trusted
final observation made after all three components in the current run.
"""
from __future__ import annotations

import json
import uuid
from dataclasses import asdict, dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Any, Callable, Mapping, Sequence

from .execution import (
    BoundedExecutor, ExecutionHistory, ExecutionLogger, Failure, FULL_TASK_PHASES,
    PhaseResult, RunResult, TerminalStop,
)
from .skills import EvidenceBinding


REQUIRED_PART_NAMES = frozenset({"ring_small", "ring_large", "pin"})
Operation = Callable[[], PhaseResult]
Checkpoint = Callable[[PhaseResult], bool]
Recovery = Callable[[Failure, int], None]


@dataclass(frozen=True)
class PartTask:
    """A declared part workflow with explicit operations and checkpoints.

    Callback maps are copied and made immutable on construction. Recovery is
    off by default. To enable it, supply both positive retry_budget (per part)
    and max_retries (per phase), and callbacks for the recoverable phases.
    A callback may still refuse recovery by raising a nonretryable failure.
    """
    part_name: str
    operations: Mapping[str, Operation]
    checkpoints: Mapping[str, Checkpoint]
    recoveries: Mapping[str, Recovery] = field(default_factory=dict)
    retry_budget: int = 0
    max_retries: int = 0

    def __post_init__(self) -> None:
        if self.part_name not in REQUIRED_PART_NAMES:
            raise ValueError("Part name must be ring_small, ring_large or pin")
        required = set(FULL_TASK_PHASES)
        for name in ("operations", "checkpoints"):
            mapping = dict(getattr(self, name))
            if set(mapping) != required or any(not callable(callback) for callback in mapping.values()):
                raise ValueError(f"{name} must provide one callable for each declared phase")
            object.__setattr__(self, name, MappingProxyType(mapping))
        recoveries = dict(self.recoveries)
        if not set(recoveries).issubset(required) or any(not callable(cb) for cb in recoveries.values()):
            raise ValueError("Recovery callbacks must name declared phases")
        for name in ("retry_budget", "max_retries"):
            value = getattr(self, name)
            if type(value) is not int or value < 0:
                raise ValueError(f"{name} must be a nonnegative integer")
        enabled = self.retry_budget > 0 and self.max_retries > 0
        if bool(self.retry_budget) != bool(self.max_retries):
            raise ValueError("Recovery requires both retry_budget and max_retries")
        if bool(recoveries) != enabled:
            raise ValueError("Recovery callbacks and both positive budgets must be declared together")
        object.__setattr__(self, "recoveries", MappingProxyType(recoveries))


@dataclass(frozen=True)
class PartResult:
    part_name: str
    completed: bool
    recovery_used: bool
    outcome: str
    completed_phases: tuple[str, ...]
    history_path: str
    failure: Failure | None = None

    @property
    def normal_success(self) -> bool:
        return self.completed and not self.recovery_used and self.outcome == "normal_success"


@dataclass(frozen=True)
class PipelineResult:
    """Current-run report, not a native collision or hardware qualification."""
    run_id: str
    domain: str
    scope: str
    binding: EvidenceBinding
    expected_parts: tuple[str, ...]
    parts: tuple[PartResult, ...]
    completed: bool
    recovery_used: bool
    fresh_final_check_passed: bool
    outcome: str
    history_path: str
    artifact_path: str
    failure: Failure | None = None

    @property
    def completed_part_count(self) -> int:
        return sum(part.completed for part in self.parts)

    @property
    def normal_success(self) -> bool:
        return self.completed and not self.recovery_used and self.outcome == "normal_success"

    @property
    def full_task_success(self) -> bool:
        names = [part.part_name for part in self.parts if part.completed]
        return (self.scope == "full_task" and self.completed and self.fresh_final_check_passed and
                len(names) == 3 and frozenset(names) == REQUIRED_PART_NAMES)

    def to_dict(self) -> dict[str, Any]:
        return {"schema": "fr3-capx-pipeline/v1", **asdict(self),
                "completed_part_count": self.completed_part_count,
                "normal_success": self.normal_success,
                "full_task_success": self.full_task_success,
                "native_path_qualification": False,
                "hardware_qualification": False}


class PipelineCoordinator:
    """Coordinate explicit callbacks, preserving component/full-task boundaries.

    Source/config/scene hashes bind every history and aggregate artifact.
    The caller is responsible for native guards inside operations and truthful
    checkpoint/fresh-final observations. This class records callback outcomes;
    it cannot manufacture native collision or physical qualification evidence.
    """
    def __init__(self, *, binding: EvidenceBinding, domain: str, output_dir: Path | str):
        if not isinstance(binding, EvidenceBinding):
            raise TypeError("An explicit EvidenceBinding is required")
        if domain not in ("simulation", "hardware"):
            raise ValueError("domain must explicitly be simulation or hardware")
        self.binding = binding
        self.domain = domain
        self.output_dir = Path(output_dir).resolve()

    def _begin(self, scope: str) -> tuple[str, Path, ExecutionLogger]:
        run_id = uuid.uuid4().hex
        directory = self.output_dir / run_id
        directory.mkdir(parents=True, exist_ok=False)
        history = ExecutionHistory(domain=self.domain, scope=scope, binding=asdict(self.binding))
        logger = ExecutionLogger(history)
        logger.log_step("pipeline", "Explicit callback pipeline started", kind="pipeline",
                        status="running", observation={"run_id": run_id, "scope": scope,
                                                       "domain": self.domain})
        return run_id, directory, logger

    def _component(self, task: PartTask, directory: Path) -> PartResult:
        history = ExecutionHistory(domain=self.domain, scope="component", binding=asdict(self.binding))
        logger = ExecutionLogger(history)
        executor = BoundedExecutor(logger, retry_budget=task.retry_budget,
                                   required_phases=FULL_TASK_PHASES)
        run_result: RunResult | None = None
        failure: Failure | None = None
        try:
            for phase in FULL_TASK_PHASES:
                executor.run_phase(phase, task.operations[phase], checkpoint=task.checkpoints[phase],
                                   recovery=task.recoveries.get(phase), max_retries=task.max_retries)
            run_result = executor.finish()
        except TerminalStop as exc:
            failure = next((step.failure for step in reversed(history.steps) if step.failure),
                           Failure("terminal_stop", str(exc)))
        history_path = history.save_to_directory(directory / "components" / task.part_name)
        return PartResult(
            part_name=task.part_name, completed=run_result is not None and run_result.completed,
            recovery_used=executor.recovery_used,
            outcome=run_result.outcome if run_result is not None else "failed",
            completed_phases=tuple(executor.completed_phases), history_path=str(history_path),
            failure=failure,
        )

    def _complete(self, *, run_id: str, directory: Path, logger: ExecutionLogger,
                  expected: tuple[str, ...], parts: list[PartResult], completed: bool,
                  fresh_final_check_passed: bool = False, failure: Failure | None = None) -> PipelineResult:
        recovered = any(part.recovery_used for part in parts)
        outcome = "failed" if not completed else "recovered_success" if recovered else "normal_success"
        logger.log_step("pipeline", "Pipeline completion accounting", kind="completion",
                        status=outcome, observation={
                            "expected_parts": list(expected), "attempted_parts": [p.part_name for p in parts],
                            "completed_parts": [p.part_name for p in parts if p.completed],
                            "completed_part_count": sum(p.completed for p in parts),
                            "fresh_final_check_passed": fresh_final_check_passed,
                            "recovery_used": recovered}, failure=failure)
        logger.finalize(outcome, recovery_used=recovered)
        history_path = logger.history.save_to_directory(directory / "coordinator")
        artifact_path = directory / "pipeline-result.json"
        result = PipelineResult(
            run_id=run_id, domain=self.domain, scope=logger.history.scope, binding=self.binding,
            expected_parts=expected, parts=tuple(parts), completed=completed, recovery_used=recovered,
            fresh_final_check_passed=fresh_final_check_passed, outcome=outcome,
            history_path=str(history_path), artifact_path=str(artifact_path), failure=failure,
        )
        artifact_path.write_text(json.dumps(result.to_dict(), indent=2, allow_nan=False) + "\n")
        return result

    def run_component(self, task: PartTask) -> PipelineResult:
        """Run a component only; even its normal success is never whole success."""
        if not isinstance(task, PartTask):
            raise TypeError("run_component requires a PartTask")
        run_id, directory, logger = self._begin("component")
        result = self._component(task, directory)
        return self._complete(run_id=run_id, directory=directory, logger=logger,
                              expected=(task.part_name,), parts=[result], completed=result.completed,
                              failure=result.failure)

    def run_three_parts(self, tasks: Sequence[PartTask], *,
                        fresh_final_check: Callable[[], bool]) -> PipelineResult:
        """Run each required part once, in caller order, then a fresh final check.

        The trusted check is called only after all eight phases of all three
        current-run parts succeed. Returning a truthy object instead of the
        explicit boolean True rejects completion. Prior component artifacts
        cannot be supplied in place of current-run callbacks.
        """
        tasks = tuple(tasks)
        if (len(tasks) != 3 or any(not isinstance(task, PartTask) for task in tasks) or
                frozenset(task.part_name for task in tasks) != REQUIRED_PART_NAMES):
            raise ValueError("Whole task requires ring_small, ring_large and pin exactly once")
        if not callable(fresh_final_check):
            raise TypeError("A trusted fresh final-check callback is required")
        expected = tuple(task.part_name for task in tasks)
        run_id, directory, logger = self._begin("full_task")
        parts: list[PartResult] = []
        for task in tasks:
            part = self._component(task, directory)
            parts.append(part)
            logger.log_step("component", f"{task.part_name}: {part.outcome}", phase=task.part_name,
                            kind="component", status=part.outcome, failure=part.failure,
                            observation={"completed": part.completed, "recovery_used": part.recovery_used,
                                         "completed_phases": list(part.completed_phases),
                                         "component_history": part.history_path})
            if not part.completed:
                return self._complete(run_id=run_id, directory=directory, logger=logger,
                                      expected=expected, parts=parts, completed=False, failure=part.failure)
        logger.log_step("fresh_final_check", "Fresh whole-task observation requested after all components",
                        phase="final_check", kind="checkpoint", status="running")
        failure = None
        try:
            passed = fresh_final_check() is True
            if not passed:
                failure = Failure("fresh_final_check_failed", "Fresh whole-task check did not return True")
        except Exception as exc:
            passed = False
            failure = Failure("fresh_final_check_exception", str(exc),
                              details={"exception_type": type(exc).__name__})
        logger.checkpoint("whole_final_check", passed,
                          observation={"completed_part_count": len(parts),
                                       "parts": [part.part_name for part in parts]})
        return self._complete(run_id=run_id, directory=directory, logger=logger,
                              expected=expected, parts=parts, completed=passed,
                              fresh_final_check_passed=passed, failure=failure)

    def functions(self) -> dict[str, Callable[..., Any]]:
        return {"run_component": self.run_component, "run_three_parts": self.run_three_parts}
