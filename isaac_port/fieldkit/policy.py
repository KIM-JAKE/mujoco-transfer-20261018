"""Code-as-Policies style task contract, independent of a 1 kHz servo backend.

The supplied adapter is a deterministic dry run. A site-specific adapter must
implement actual robot motion, Robotiq feedback, PLC acknowledgements and
calibrated part observations; successful API return alone is not an observation.
"""
from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class TaskState:
    grasped: frozenset[str] = frozenset()
    inserted: frozenset[str] = frozenset()
    clamped: frozenset[str] = frozenset()
    released: frozenset[str] = frozenset()
    fault: str | None = None


class Cell(Protocol):
    def execute_parallel(self, actions: list[tuple[str, str, str]]) -> None: ...
    def observe(self) -> TaskState: ...
    def press_button(self, *, minimum_travel_m: float, dwell_s: float) -> None: ...
    def stop(self) -> None: ...


def require(cell: Cell, attribute: str, parts: set[str]) -> None:
    state = cell.observe()
    if state.fault or not parts <= getattr(state, attribute):
        raise RuntimeError(f'{attribute} not confirmed for {sorted(parts)}: {state}')


def assemble(cell: Cell) -> None:
    """Only progress after measured grasp, seat, clamp and release evidence.

    Each adapter action must be bounded in time and return only after completing
    or reporting a fault. A fault stops the task and is never retried blindly.
    """
    rings = {'ring1', 'ring2'}
    try:
        cell.execute_parallel([('right', 'grasp', 'ring1'), ('left', 'grasp', 'ring2')])
        require(cell, 'grasped', rings)
        cell.execute_parallel([('right', 'insert', 'ring1'), ('left', 'insert', 'ring2')])
        require(cell, 'inserted', rings)
        cell.execute_parallel([('fixture', 'clamp', name) for name in sorted(rings)])
        require(cell, 'clamped', rings)
        cell.execute_parallel([('right', 'release', 'ring1'), ('left', 'release', 'ring2')])
        require(cell, 'released', rings)
        cell.execute_parallel([('left', 'grasp', 'pin'), ('right', 'approach', 'button')])
        require(cell, 'grasped', {'pin'})
        cell.execute_parallel([('left', 'insert', 'pin')])
        require(cell, 'inserted', {'pin'})
        cell.execute_parallel([('fixture', 'clamp', 'pin')])
        require(cell, 'clamped', {'pin'})
        cell.execute_parallel([('left', 'release', 'pin')])
        require(cell, 'released', {'ring1', 'ring2', 'pin'})
        cell.press_button(minimum_travel_m=.0018, dwell_s=.1)
    except BaseException:
        cell.stop()
        raise


class DryRunCell:
    """Contract exercise only: no physics, timing prediction or robot socket."""
    def __init__(self):
        self.state = TaskState()
        self.events = []
        self.stopped = False

    def execute_parallel(self, actions):
        fields = {key: set(getattr(self.state, key)) for key in
                  ['grasped', 'inserted', 'clamped', 'released']}
        for side, action, part in actions:
            if action == 'insert' and part not in fields['grasped']:
                raise RuntimeError('Insertion without grasp')
            if action == 'clamp' and part not in fields['inserted']:
                raise RuntimeError('Clamp without seat')
            if action == 'release' and part not in fields['clamped']:
                raise RuntimeError('Release without clamp')
            key = dict(grasp='grasped', insert='inserted', clamp='clamped', release='released').get(action)
            if key:
                fields[key].add(part)
            self.events.append(dict(side=side, action=action, part=part))
        self.state = TaskState(**{k: frozenset(v) for k, v in fields.items()})

    def observe(self):
        return self.state

    def press_button(self, **parameters):
        require(self, 'released', {'ring1', 'ring2', 'pin'})
        self.events.append(dict(action='press_button', **parameters))

    def stop(self):
        self.stopped = True
        self.events.append(dict(action='stop'))
