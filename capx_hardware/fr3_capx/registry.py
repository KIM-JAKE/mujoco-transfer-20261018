"""Explicit CaP-X function registry and tool documentation, without code exec."""
from __future__ import annotations

import inspect
from typing import Any, Callable


class ApiRegistry:
    """Combine API instances; prefix side-specific controls to avoid arm ambiguity."""

    def __init__(self):
        self._functions: dict[str, Callable] = {}

    def register(self, api: Any, *, prefix: str = "") -> "ApiRegistry":
        functions = api.functions()
        if not isinstance(functions, dict):
            raise TypeError("API functions() must return a dictionary")
        staged = {}
        for name, function in functions.items():
            full_name = prefix + name
            if not full_name.isidentifier() or not callable(function):
                raise ValueError("Tool names must be identifiers and functions callable")
            if full_name in self._functions or full_name in staged:
                raise ValueError(f"Duplicate tool: {full_name}")
            staged[full_name] = function
        self._functions.update(staged)
        return self

    def functions(self) -> dict[str, Callable]:
        return dict(self._functions)

    def combined_doc(self) -> str:
        return "\n\n".join(f"{name}{inspect.signature(fn)}\n{inspect.getdoc(fn) or ''}"
                            for name, fn in sorted(self._functions.items()))

    def describe(self) -> list[dict]:
        return [dict(name=name, signature=str(inspect.signature(fn)),
                     documentation=inspect.getdoc(fn) or "")
                for name, fn in sorted(self._functions.items())]

    def call(self, name: str, arguments: dict) -> Any:
        """Dispatch an already registered callable; never eval/exec an expression."""
        if not isinstance(arguments, dict):
            raise TypeError("Tool arguments must be a dictionary")
        if name not in self._functions:
            raise KeyError(f"Unknown tool: {name}")
        function = self._functions[name]
        bound = inspect.signature(function).bind(**arguments)
        return function(*bound.args, **bound.kwargs)


def build_registry(backend: Any, *, logger=None, speed_scale: float = 0.2,
                   perception_clock=None) -> ApiRegistry:
    """Create observation tools and explicit left_/right_ motion tools; no device I/O."""
    from .control import ControlApi
    from .perception import PerceptionApi
    kwargs = {} if perception_clock is None else {"clock": perception_clock}
    registry = ApiRegistry().register(PerceptionApi(backend, **kwargs))
    for side in ("left", "right"):
        registry.register(ControlApi(backend, side=side, logger=logger, speed_scale=speed_scale),
                          prefix=side + "_")
    return registry
