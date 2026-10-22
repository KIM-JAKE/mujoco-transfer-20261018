"""Compare early hand rejection with full PhysX queries at recorded robot poses.

Diagnostic only: reads robot commands, never free-workpiece transforms, and
does not move any actor or report a grasp/task success.
"""
from pathlib import Path
import numpy as np


def execute(c):
    here = Path(__file__).resolve().parent
    archive = np.load(here.parents[1] / 'runs/native-roas-table-scatter-full-v25-304/executed-actuator-commands.npz')
    labels = archive['pin_phase']
    rows = []
    checker, arm, ids = c['checker'], c['arm'], c['ids']
    for label in ['loose_ring_pregrasp', 'loose_ring_descend', 'loose_ring_close', 'loose_ring_lift']:
        selected = np.flatnonzero(labels == label)
        if not len(selected):
            continue
        q = archive['command'][selected[-1], ids]
        tcp = arm.fk(q)
        bodies = arm.bodies(q, .08)
        subset = {body: pose for body, pose in bodies.items() if body.startswith(arm.hand_path)}
        early = checker.hand_clear(tcp, .08)
        same = checker._clear_transforms(subset, None)
        full = checker.clear(q, .08)
        row = dict(phase=label, tcp_m=tcp[:3, 3].tolist(), early_hand_query=early,
                   hand_subset_of_full_query=same, full_robot_query=full,
                   early_and_full_hand_agree=bool(early[0] == same[0]))
        rows.append(row)
        print('EARLY_HAND_CLEARANCE_DIAGNOSTIC', row, flush=True)
    return dict(state='clearance_query_equivalence_diagnostic', records=rows,
                all_hand_queries_agree=all(row['early_and_full_hand_agree'] for row in rows),
                full_task_pass=False, physical_task_run=False)
