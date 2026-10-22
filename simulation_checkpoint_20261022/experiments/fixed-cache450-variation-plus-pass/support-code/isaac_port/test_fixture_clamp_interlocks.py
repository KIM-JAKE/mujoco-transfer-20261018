"""Do not close adaptive fixture jaws onto a moving or misaligned workpiece."""
from types import SimpleNamespace
import unittest
import numpy as np
from fixture_clamps import Controller


class ClampInterlockTests(unittest.TestCase):
    def test_adaptive_clamp_waits_for_stopped_arm_and_tight_alignment(self):
        feedback=SimpleNamespace(
            windows={'ring2':('left',0.,1.,1.1)}, insertion={'ring2':.8},
            fast_cycle=True,strict_clamp_alignment=True,
            names=[f'left_arm_fr3v2_joint{j}'for j in range(1,8)],
            measured_qv=np.zeros(7))
        error=np.array([.0001,0,0]);feedback.errors=lambda *args:(error,np.zeros(3))
        controller=Controller([])
        controller.update(0.,.99,feedback,None,None,{})
        self.assertFalse(controller.requested['ring2'])
        feedback.measured_qv[0]=.04
        controller.update(.01,1.,feedback,None,None,{})
        self.assertFalse(controller.requested['ring2'])
        feedback.measured_qv[:]=0.;error[0]=.0002
        controller.update(.02,1.,feedback,None,None,{})
        self.assertFalse(controller.requested['ring2'])
        error[0]=.0001
        controller.update(.03,1.,feedback,None,None,{})
        self.assertTrue(controller.requested['ring2'])
        self.assertFalse(controller.verified['ring2'])


if __name__=='__main__':unittest.main()
