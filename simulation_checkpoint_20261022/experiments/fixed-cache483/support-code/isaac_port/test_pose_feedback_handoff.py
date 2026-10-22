"""A released hand may settle; a still-gripping hand must retain correction."""
import unittest
import numpy as np
from pose_feedback import PoseFeedback


class ReleaseHandoffTests(unittest.TestCase):
    def setUp(self):
        self.f=PoseFeedback.__new__(PoseFeedback)
        self.f.windows={'ring1':('right',0.,1.,1.1)}
        self.f.names=[f'right_arm_fr3v2_joint{j}'for j in range(1,8)]+['right_grip_left_driver_joint']
        self.f.ts=np.array([0.,2.]);self.f.planned_q=np.zeros((2,8))
        self.f.offset=np.full(8,.01);self.f.measured_q=np.zeros(8)
        self.f.clamped={'ring1':True};self.f.clear_counts={'ring1':3}
        self.f.settled_handoff_checkpoints={'right':[1.1]}
        self.indices=np.arange(7)

    def relax(self):
        return self.f.relax_released_handoff('ring1',1.1-1e-8,self.indices,1/120)

    def test_confirmed_open_hand_settles_without_altering_gripper(self):
        self.assertTrue(self.relax())
        np.testing.assert_allclose(self.f.offset[:7],.01*np.exp(-(1/120)/.35))
        self.assertEqual(self.f.offset[7],.01)

    def test_no_checkpoint_or_unconfirmed_release_keeps_seating_offset(self):
        for field,value in [('settled_handoff_checkpoints',{}),('clamped',{'ring1':False}),
                            ('clear_counts',{'ring1':2})]:
            self.setUp();setattr(self.f,field,value)
            self.assertFalse(self.relax());np.testing.assert_array_equal(self.f.offset,.01)
        for driver_position in [.02,float('nan')]:
            self.setUp();self.f.measured_q[7]=driver_position
            self.assertFalse(self.relax());np.testing.assert_array_equal(self.f.offset,.01)

    def test_correction_cannot_decay_before_release_endpoint(self):
        self.assertFalse(self.f.relax_released_handoff('ring1',1.,self.indices,1/120))
        np.testing.assert_array_equal(self.f.offset,.01)


if __name__=='__main__':unittest.main()
