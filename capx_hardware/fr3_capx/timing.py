"""Nested wall-time profiling for actual callbacks, separate from simulated time.

Measurements in one thread have exclusive nested accounting. Concurrent threads
are kept separate: their CPU/service time cannot be added to derive wall time.
No inference about physical controller speed is made from simulation wall time.
"""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import asdict, dataclass
from functools import wraps
import math
import threading
import time
from typing import Callable


@dataclass(frozen=True)
class TimingSpan:
    category: str
    label: str
    thread_id: int
    depth: int
    start_s: float
    end_s: float
    wall_s: float
    exclusive_s: float
    outcome: str


class WallProfiler:
    """Profile reviewed callbacks. Construction records a monotonic origin only."""
    def __init__(self, *, clock: Callable[[], float] = time.perf_counter):
        self.clock = clock
        self.origin = clock()
        self._local = threading.local()
        self._lock = threading.Lock()
        self.spans: list[TimingSpan] = []
        self.metrics: dict[str, dict] = {}

    @contextmanager
    def measure(self, category: str, label: str = ""):
        if not isinstance(category, str) or not category.strip():
            raise ValueError("Explicit timing category required")
        stack = getattr(self._local, "stack", None)
        if stack is None:
            stack = self._local.stack = []
        entry = {"start": self.clock(), "child_s": 0.0}
        depth = len(stack)
        stack.append(entry)
        outcome = "returned"
        try:
            yield
        except BaseException:
            outcome = "raised"
            raise
        finally:
            end = self.clock()
            duration = end - entry["start"]
            stack.pop()
            if duration < 0 or entry["child_s"] > duration + 1e-9:
                raise RuntimeError("Nonmonotonic profiler clock")
            if stack:
                stack[-1]["child_s"] += duration
            span = TimingSpan(category, label, threading.get_ident(), depth,
                              entry["start"] - self.origin, end - self.origin,
                              duration, max(0.0, duration-entry["child_s"]), outcome)
            with self._lock:
                self.spans.append(span)

    def wrap(self, category: str, callback: Callable, *, label: str = "") -> Callable:
        """Wrap a native callback with timing; preserve its return and exceptions."""
        @wraps(callback)
        def measured(*args, **kwargs):
            with self.measure(category, label):
                return callback(*args, **kwargs)
        return measured

    def record_metric(self, name: str, value: float, *, unit: str, basis: str) -> None:
        """Record commanded/simulated duration explicitly; never merge it into wall time."""
        if (not name or not unit or not basis or isinstance(value, bool) or
                not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0):
            raise ValueError("A nonnegative finite metric with explicit unit/basis is required")
        with self._lock:
            if name in self.metrics:
                raise ValueError("Metric already recorded; use a separate name for another basis")
            self.metrics[name] = dict(value=float(value), unit=unit, basis=basis)

    def report(self) -> dict:
        now = self.clock()
        with self._lock:
            spans, metrics = list(self.spans), {k: dict(v) for k, v in self.metrics.items()}
        by_thread: dict[str, dict] = {}
        for span in spans:
            entry = by_thread.setdefault(str(span.thread_id), {
                "root_covered_wall_s": 0.0, "exclusive_category_s": {}, "span_count": 0})
            entry["span_count"] += 1
            category = entry["exclusive_category_s"]
            category[span.category] = category.get(span.category, 0.0) + span.exclusive_s
            if span.depth == 0:
                entry["root_covered_wall_s"] += span.wall_s
        return dict(schema="fr3-capx-wall-profile/v1", total_wall_s=now-self.origin,
                    thread_accounting=by_thread, metrics=metrics,
                    concurrent_thread_durations_are_additive_wall_time=False,
                    includes_in_progress_spans=False,
                    spans=[asdict(span) for span in spans],
                    note="Exclusive categories sum only within each thread's measured roots. "
                         "Uninstrumented time and concurrent overlap are not inferred as waiting. "
                         "Simulation wall time is not a hardware runtime estimate.")
