"""Screen measured joint positions against FR3 nominal motion limits.

Secant velocities and divided differences are computed without smoothing.
Exceedances reject feasibility; absence of exceedances at 30 Hz cannot certify
the 1 kHz FCI command stream, position-dependent limits, jerk or torque rate.
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys

import numpy as np

SOURCE = 'https://frankarobotics.github.io/docs/robot_specifications.html'
VELOCITY = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
ACCELERATION = np.full(7, 10.0)
LOWER = np.array([-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, .4398, -3.0508])
UPPER = np.array([2.9007, 1.8361, 2.9007, -.1169, 2.8763, 4.6216, 3.0508])


def audit(run):
    run = Path(run)
    tape_path = run / 'physics-tape.npz'
    tape = np.load(tape_path)
    names = json.loads((run / 'dof-names.json').read_text())
    t = tape['time_s'].astype(float)
    assert len(t) >= 3 and np.all(np.diff(t) > 0)
    arms = {}
    for side in ['left', 'right']:
        indices = [names.index(f'{side}_arm_fr3v2_joint{i}') for i in range(1, 8)]
        q = tape['joint_positions'][:, indices].astype(float)
        assert np.isfinite(q).all()
        v = np.diff(q, axis=0) / np.diff(t)[:, None]
        tv = (t[1:] + t[:-1]) / 2
        acc = np.diff(v, axis=0) / np.diff(tv)[:, None]
        rows = []
        for j in range(7):
            iv, ia = np.argmax(abs(v[:, j])), np.argmax(abs(acc[:, j]))
            peak_v, peak_a = float(abs(v[iv, j])), float(abs(acc[ia, j]))
            position_pass = bool(q[:, j].min() >= LOWER[j] and q[:, j].max() <= UPPER[j])
            rows.append(dict(joint=j + 1, peak_secant_velocity_rad_s=peak_v,
                             nominal_velocity_limit_rad_s=float(VELOCITY[j]),
                             velocity_ratio=float(peak_v / VELOCITY[j]),
                             peak_velocity_interval_s=[float(t[iv]), float(t[iv + 1])],
                             peak_divided_difference_acceleration_rad_s2=peak_a,
                             acceleration_limit_rad_s2=float(ACCELERATION[j]),
                             acceleration_ratio=float(peak_a / ACCELERATION[j]),
                             peak_acceleration_interval_s=[float(t[ia]), float(t[ia + 2])],
                             position_within_nominal_range=position_pass,
                             velocity_pass=bool(peak_v <= VELOCITY[j]),
                             acceleration_pass=bool(peak_a <= ACCELERATION[j])))
        arms[side] = rows
    exceeds = any(not all(row[key] for key in ['position_within_nominal_range',
                  'velocity_pass', 'acceleration_pass']) for rows in arms.values() for row in rows)
    result = dict(state='failed' if exceeds else 'inconclusive',
                  fr3_motion_limit_violation_detected=exceeds,
                  fr3_hardware_feasibility_certified=False,
                  official_source=SOURCE, source_checked_date='2026-10-18',
                  physics_tape_sha256=hashlib.sha256(tape_path.read_bytes()).hexdigest(),
                  sample_count=len(t), median_sample_interval_s=float(np.median(np.diff(t))),
                  method='Unsmoothed secant velocity and adjacent-secant divided-difference acceleration from measured joint positions',
                  limitations=['30 Hz samples do not resolve all 480 Hz simulation peaks or 1 kHz FCI commands',
                               'Position-dependent velocity limits, command continuity, jerk, torque and torque-rate compliance are not certified',
                               'Nominal-limit exceedance is sufficient to reject this trajectory; no exceedance would not certify hardware feasibility'],
                  arms=arms)
    (run / 'fr3-motion-audit.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    report = audit(parser.parse_args().run)
    print(json.dumps(report, indent=2))
    sys.exit(1 if report['fr3_motion_limit_violation_detected'] else 0)
