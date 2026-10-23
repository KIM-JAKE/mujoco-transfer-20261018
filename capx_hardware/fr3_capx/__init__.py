"""CaP-X-inspired dual FR3 API package. Importing performs no external actions."""
from .backend import (Backend, BackendError, CallbackBackend, RecordedBackend,
                      StaleProof, UnsupportedOperation, ValidationRejected)
from .benchmark import TrialRecord, aggregate_trials
from .control import ControlApi, object_target_to_hand
from .execution import (BoundedExecutor, ExecutionHistory, ExecutionLogger,
                        Failure, FULL_TASK_PHASES, PhaseFailure, PhaseResult,
                        RunResult, TerminalStop)
from .perception import PerceptionApi
from .pipeline import PartTask, PipelineCoordinator
from .registry import ApiRegistry, build_registry
from .skills import EvidenceBinding, EvidenceRejected, SkillLibrary, ValidationEvidence
from .timing import WallProfiler

__version__ = "0.1.0"

__all__ = ["Backend", "BackendError", "CallbackBackend", "RecordedBackend", "StaleProof",
           "UnsupportedOperation", "ValidationRejected", "ControlApi", "object_target_to_hand",
           "BoundedExecutor", "ExecutionHistory", "ExecutionLogger", "Failure", "FULL_TASK_PHASES",
           "PhaseFailure", "PhaseResult", "RunResult", "TerminalStop", "PerceptionApi",
           "ApiRegistry", "build_registry", "EvidenceBinding", "EvidenceRejected", "SkillLibrary",
           "ValidationEvidence", "PartTask", "PipelineCoordinator", "TrialRecord",
           "aggregate_trials", "WallProfiler"]
