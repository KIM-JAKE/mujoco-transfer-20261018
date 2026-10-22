"""At a stopped insertion gate, real seating error must remain observable."""
from pathlib import Path
import json
import unittest
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from pose_feedback import PoseFeedback


class TerminalFeedbackTests(unittest.TestCase):
    def test_motor_lag_is_compensated_in_motion_but_corrected_at_seat(self):
        root = Path(__file__).resolve().parent
        base = root / 'runs/contact-under15-07'
        # The bundled seed is also sufficient in a fresh clone.
        recording = base / 'recording'
        if not recording.exists():
            recording = root / 'generated/contact_task/fast'
        names = json.loads((root/'assets/reference-dof-names.json').read_text())
        feedback = PoseFeedback(root, recording, names, 1.)
        feedback.motion_limited = True
        feedback.fast_cycle = True
        feedback.planned_q = feedback.source['qpos'][:, feedback.qadr].copy()
        plan_path = base / 'arm-posture-fr3.npz'
        if not plan_path.exists():
            plan_path = root/'generated/contact_task/arm-posture-fr3.npz'
        with np.load(plan_path) as plan:
            for i, name in enumerate(names):
                if '_arm_' in name:
                    feedback.planned_q[:, i] = plan[name.split('_')[0]+'_qpos'][:, int(name[-1])-1]
        feedback.windows = {'ring2': feedback.windows['ring2']}
        end = feedback.windows['ring2'][2]
        hand = feedback.model.body('left_grip_base').id

        def observation(time):
            nominal = np.array([np.interp(time, feedback.ts, feedback.planned_q[:, j]) for j in range(len(names))])
            actual = nominal.copy()
            actual[names.index('left_arm_fr3v2_joint2')] += .003
            planned = mujoco.MjData(feedback.model)
            measured = mujoco.MjData(feedback.model)
            planned.qpos[feedback.qadr] = nominal
            measured.qpos[feedback.qadr] = actual
            mujoco.mj_kinematics(feedback.model, planned)
            mujoco.mj_kinematics(feedback.model, measured)
            delta = Rotation.from_matrix(measured.xmat[hand].reshape(3,3)) * Rotation.from_matrix(planned.xmat[hand].reshape(3,3)).inv()
            positions, quaternions = [], []
            for part in feedback.parts:
                p, r = feedback.target(part, time)
                positions.append(measured.xpos[hand] + delta.apply(p-planned.xpos[hand]))
                quaternions.append((delta*r).as_quat()[[3,0,1,2]])
            return actual, np.array(positions), np.array(quaternions)

        actual, positions, quaternions = observation(end-.05)
        feedback.update(end-.05, actual, positions, quaternions, {})
        self.assertLess(np.linalg.norm(feedback.offset), 1e-10)
        actual, positions, quaternions = observation(end-1e-8)
        error, _ = feedback.errors('ring2', end, positions, quaternions)
        self.assertGreater(np.linalg.norm(error), .00045)
        feedback.update(end-1e-8, actual, positions, quaternions, {})
        self.assertGreater(np.linalg.norm(feedback.offset), 1e-6)


if __name__ == '__main__':
    unittest.main()
