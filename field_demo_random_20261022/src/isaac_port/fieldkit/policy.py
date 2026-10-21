"""Code-as-Policies style task contract, independent of a 1 kHz servo backend.

The supplied adapter is a deterministic dry run. A site-specific adapter must
implement actual robot motion, Robotiq feedback, PLC acknowledgements and
calibrated part observations; successful API return alone is not an observation.
"""
from dataclasses import dataclass, replace
from concurrent.futures import ThreadPoolExecutor, wait, FIRST_EXCEPTION
from threading import RLock
from typing import Protocol


@dataclass(frozen=True)
class TaskState:
    grasped: frozenset[str] = frozenset()
    inserted: frozenset[str] = frozenset()
    clamped: frozenset[str] = frozenset()
    released: frozenset[str] = frozenset()
    fault: str | None = None
    button_verified: bool = False


class Cell(Protocol):
    """Thread-safe field adapter with bounded actions and an idempotent stop.

    Observations belong to the current cycle. button_verified means measured
    travel and continuous dwell passed, rather than a command acknowledgement.
    """
    def execute_parallel(self, actions: list[tuple[str, str, str]]) -> None: ...
    def observe(self) -> TaskState: ...
    def press_button(self, *, minimum_travel_m: float, dwell_s: float) -> None: ...
    def stop(self) -> None: ...


def require(cell: Cell, attribute: str, parts: set[str]) -> None:
    state = cell.observe()
    if state.fault or not parts <= getattr(state, attribute):
        raise RuntimeError(f'{attribute} not confirmed for {sorted(parts)}: {state}')


def finish_button(cell: Cell) -> None:
    require(cell, 'released', {'ring1', 'ring2', 'pin'})
    cell.press_button(minimum_travel_m=.0018, dwell_s=.1)
    state=cell.observe()
    if state.fault or not state.button_verified:
        raise RuntimeError('Measured button travel and dwell were not confirmed')


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
        finish_button(cell)
    except BaseException:
        cell.stop()
        raise


PART_SIDES = {'ring1': 'right', 'ring2': 'left', 'pin': 'left'}


def assemble_parts(cell: Cell, parts=('ring1', 'ring2', 'pin'), *, button=False) -> None:
    """Complete each selected part from measured taught starts; no retries.

    This field profile has no handover and no automatic contact search. The
    optional button requires all three parts in the same run.
    """
    parts = tuple(parts)
    if not parts or len(set(parts)) != len(parts) or any(p not in PART_SIDES for p in parts):
        raise ValueError('Select unique known parts')
    if button and set(parts) != set(PART_SIDES):
        raise ValueError('Button profile requires all three parts')
    try:
        for part in parts:
            for side, action, evidence in [(PART_SIDES[part], 'grasp', 'grasped'),
                                          (PART_SIDES[part], 'insert', 'inserted'),
                                          ('fixture', 'clamp', 'clamped'),
                                          (PART_SIDES[part], 'release', 'released')]:
                cell.execute_parallel([(side, action, part)])
                require(cell, evidence, {part})
        if button:
            cell.execute_parallel([('right', 'approach', 'button')])
            finish_button(cell)
    except BaseException:
        cell.stop()
        raise


def assemble_concurrent(cell: Cell) -> None:
    """Independent left ring2→pin and right ring1→button approach lanes.

    The field Cell must bind its calibrated plan, enforce servo crossing gates,
    support concurrent calls and stop all in-flight actions on stop(). Actions
    must be bounded and observations must belong to the current cycle. Python
    threads express dependencies; they are not synchronized 1kHz control loops.
    """
    def part(side,name):
        for actor,action,evidence in [(side,'grasp','grasped'),(side,'insert','inserted'),
                                      ('fixture','clamp','clamped'),(side,'release','released')]:
            cell.execute_parallel([(actor,action,name)])
            require(cell,evidence,{name})
    def left():
        part('left','ring2')
        part('left','pin')
    def right():
        part('right','ring1')
        cell.execute_parallel([('right','approach','button')])
    pool=ThreadPoolExecutor(max_workers=2);futures=[]
    try:
        futures=[pool.submit(left),pool.submit(right)]
        done,_=wait(futures,return_when=FIRST_EXCEPTION)
        for task in done:task.result()
        for task in futures:task.result()
        finish_button(cell)
    except BaseException:
        # Stop before joining a lane that may be waiting on hardware feedback.
        cell.stop()
        for task in futures:task.cancel()
        raise
    finally:
        pool.shutdown(wait=True,cancel_futures=True)


class DryRunCell:
    """Contract exercise only: no physics, timing prediction or robot socket."""
    def __init__(self):
        self.state = TaskState()
        self.events = []
        self.stopped = False
        self.lock = RLock()

    def execute_parallel(self, actions):
        with self.lock:
            if self.stopped:raise RuntimeError('Cell is stopped')
            self._apply_actions(actions)

    def _apply_actions(self, actions):
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
        with self.lock:return self.state

    def press_button(self, **parameters):
        with self.lock:
            if self.stopped:raise RuntimeError('Cell is stopped')
            require(self, 'released', {'ring1', 'ring2', 'pin'})
            self.events.append(dict(action='press_button', **parameters))
            self.state=replace(self.state,button_verified=True)

    def stop(self):
        with self.lock:
            self.stopped = True
            self.events.append(dict(action='stop'))
