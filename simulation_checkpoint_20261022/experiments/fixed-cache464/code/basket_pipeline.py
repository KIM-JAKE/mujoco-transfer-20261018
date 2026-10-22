"""Overlap proven command paths. Physical execution is in run_basket_pipeline.

Recorded commands are reusable motion primitives, NOT replayed object states.
Only the initial state is set; contacts determine all subsequent part poses.
"""
import numpy as np


def hermite(tape, t):
    times = tape['time_s']; values = tape['command']
    i = int(np.clip(np.searchsorted(times, t)-1, 0, len(times)-2))
    a, b = times[i:i+2]; h = b-a; u = float(np.clip((t-a)/h, 0, 1))
    # Uniform cubic B-spline: C2, convex weights, no PBV-band overshoot.
    # This smooths sampled commands, without moving any simulated objects.
    controls=[max(0,i-1),i,i+1,min(len(times)-1,i+2)]
    weights=[(1-u)**3,(3*u**3-6*u*u+4),(-3*u**3+3*u*u+3*u+1),u**3]
    result=values[i].astype(float).copy()
    for index,weight in zip(controls,weights):
        result+=(weight/6)*(values[index]-values[i])
    return result



class Schedule:
    def __init__(self, rings, pin, overlap=True,ring2_carry_s=8.1854166667):
        self.rings = {k:rings[k] for k in ['time_s','command']}
        self.pin = {k:pin[k] for k in ['time_s','command']}
        self.overlap = overlap
        self.first_release = 32.06875
        self.first_done = 43.1041666667
        self.next_handoff = 56.4041666667
        self.second_release = 68.3958333333
        self.second_done = 84.2854166667
        self.left_park = 89.38125
        self.pin_start = 1.8020833333
        self.pin_free = 11.875
        self.pin_end = 38.1
        self.saved1 = self.first_done-self.first_release if overlap else 0.
        self.second_start = self.second_release-self.saved1
        self.ring2_carry_s=float(ring2_carry_s)
        if not 1.5<=self.ring2_carry_s<=8.1854166668:raise ValueError('Invalid ring2 carry duration')
        self.carry_saved=8.1854166667-self.ring2_carry_s if overlap else 0.
        self.park_done = self.left_park-self.saved1-self.carry_saved
        self.pin_begin = self.second_start if overlap else self.park_done
        self.pin_resume = max(self.pin_begin+self.pin_free-self.pin_start,
                              self.park_done)
        self.duration = self.pin_resume+self.pin_end-self.pin_free

    def clocks(self, t):
        if not self.overlap:
            return min(t, self.left_park), min(t, self.second_done), (
                self.pin_start+max(0., t-self.pin_begin))
        if t <= self.first_release:
            l, r = t, t
        elif t < self.next_handoff-self.saved1:
            l = min(t, self.first_done)
            r = self.first_done+t-self.first_release
        else:
            l = t+self.saved1
            r = t+self.saved1
        if t>=self.second_start:
            elapsed=t-self.second_start
            l=self.second_release+elapsed*8.1854166667/self.ring2_carry_s if elapsed<self.ring2_carry_s else t+self.saved1+self.carry_saved
        p = self.pin_start+max(0., t-self.pin_begin)
        if t >= self.pin_begin+self.pin_free-self.pin_start:
            p = self.pin_free+max(0., t-self.pin_resume)
        return min(l, self.left_park), min(r, self.second_release), p

    def command(self, t, left_indices, right_indices):
        l, r, p = self.clocks(t)
        values = hermite(self.rings, min(t, self.left_park))
        values[left_indices] = hermite(self.rings, l)[left_indices]
        if t >= self.pin_begin:
            values[right_indices] = hermite(self.pin, p)[right_indices]
        else:
            values[right_indices] = hermite(self.rings, r)[right_indices]
        return values
