"""Independent arm clocks with per-arm contact interlocks.

This changes the simultaneous path. Collision and PhysX acceptance must be
rerun; an offline timing result alone is never an assembly success.
"""
import json
import numpy as np
from scipy.interpolate import CubicSpline, PPoly
from fr3_motion_control import PathMotionLimiter, position_velocity_bounds, LOWER, UPPER
from fr3_reference_profile import reference_fingerprint

SIDES = ('left', 'right')


class ArmClock:
    def __init__(self, archive, prefix):
        self.bounds = archive[prefix + 'bounds'].copy()
        self.durations = archive[prefix + 'durations'].copy()
        self.ends = np.cumsum(self.durations)
        self.clocks = [PPoly(archive[f'{prefix}clock_{i}_c'], archive[f'{prefix}clock_{i}_x'])
                       for i in range(len(self.durations))]
        self.scales = [float(archive[f'{prefix}clock_{i}_scale']) for i in range(len(self.durations))]
        self.elapsed = 0.
        self.held = False
        self.started = False

    def at(self, time):
        if time >= self.ends[-1]:
            return float(self.bounds[-1]), 0., 0.
        i = int(np.searchsorted(self.ends, time, side='right'))
        start = 0. if i == 0 else self.ends[i - 1]
        clock, scale = self.clocks[i], self.scales[i]
        tick = float(np.clip((time - start) / scale, clock.x[0], clock.x[-1]))
        s = float(np.clip(clock(tick), self.bounds[i], self.bounds[i + 1]))
        if s <= self.bounds[i] + 1e-13 or s >= self.bounds[i + 1] - 1e-13:
            return s, 0., 0.
        return s, float(clock(tick, 1) / scale), float(clock(tick, 2) / scale**2)

    def peek(self, dt):
        return self.at(self.elapsed + dt if self.started and not self.held else self.elapsed)[0]

    def gate(self, reference, held, started):
        # Every possible interlock must be a zero-speed boundary of this arm.
        if held:
            nearest = int(np.argmin(abs(self.bounds - reference)))
            if abs(self.bounds[nearest] - reference) > 2e-7:
                raise RuntimeError('Unexpected moving-path stop requested')
            self.elapsed = 0. if nearest == 0 else float(self.ends[nearest - 1])
        self.held, self.started = bool(held), bool(started)

    def advance(self, dt):
        if self.started and not self.held:
            self.elapsed = min(self.elapsed + dt, float(self.ends[-1]))
        s, v, a = self.at(self.elapsed)
        return (s, 0., 0.) if self.held or not self.started else (s, v, a)


class ConcurrentProfile:
    def __init__(self, times, q, names, schedule, archive):
        self.indices = np.array([names.index(f'{side}_arm_fr3v2_joint{j}')
                                 for side in SIDES for j in range(1, 8)])
        self.driver_indices = [names.index(side + '_grip_left_driver_joint') for side in SIDES]
        self.side_indices = {side: np.array([i for i, n in enumerate(names) if n.startswith(side + '_')])
                             for side in SIDES}
        self.arm_splines = {side: CubicSpline(times, q[:, self.indices[k*7:(k+1)*7]], axis=0)
                            for k, side in enumerate(SIDES)}
        self.spline = CubicSpline(times, q, axis=0)
        with np.load(archive) as data:
            if str(data['fingerprint']) != reference_fingerprint(times, q, names, schedule):
                raise RuntimeError('Concurrent clocks do not match frozen inputs')
            self.arms = {side: ArmClock(data, side + '_') for side in SIDES}
        self.metadata = json.loads(archive.with_suffix('.json').read_text())
        self.last_reference = {side: float(arm.bounds[0]) for side, arm in self.arms.items()}
        self.last_rate = {side: 0. for side in SIDES}

    def path(self, references, derivative=0):
        return np.concatenate([self.arm_splines[side](references[side], derivative) for side in SIDES])

    def full_path(self, references, derivative=0):
        q = np.array(self.spline(references['left'], derivative), copy=True)
        for side in SIDES:
            ii = self.side_indices[side]
            q[ii] = self.spline(references[side], derivative)[ii]
        return q

    def full_velocity(self):
        v = self.full_path(self.last_reference, 1)
        for side in SIDES:
            v[self.side_indices[side]] *= self.last_rate[side]
        return v

    def peek(self, dt):
        return {side: arm.peek(dt) for side, arm in self.arms.items()}

    def gate(self, references, holds, started):
        for side in SIDES:
            self.arms[side].gate(references[side], holds.get(side), started)

    def advance(self, dt):
        p, v, a = [], [], []
        for side in SIDES:
            s, sd, sdd = self.arms[side].advance(dt)
            self.last_reference[side], self.last_rate[side] = s, sd
            sp = self.arm_splines[side]
            p.append(sp(s)); v.append(sp(s, 1) * sd)
            a.append(sp(s, 2) * sd**2 + sp(s, 1) * sdd)
        return np.concatenate(p), np.concatenate(v), np.concatenate(a)

    def report(self):
        return self.metadata | dict(state='executing_concurrent_clocks')


class ConcurrentMotionLimiter(PathMotionLimiter):
    def __init__(self, names, initial, worker_path, profile, dt=1/480):
        super().__init__(names, initial, worker_path, profile, dt)
        self.reference = dict(profile.last_reference)

    def set_reference(self, reference, hold, started):
        self.reference = dict(reference)
        self.profile.gate(reference, hold, started)
