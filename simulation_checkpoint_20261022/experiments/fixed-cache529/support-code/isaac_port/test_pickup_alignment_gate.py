"""Perturbed pickups align above the fixture at a planned zero-speed gate."""
import unittest
import numpy as np
from pose_feedback import PoseFeedback


class PickupAlignmentTests(unittest.TestCase):
    def feedback(self):
        f=PoseFeedback.__new__(PoseFeedback)
        f.windows={'pin':('left',3.17,6.369,6.448)};f.arrivals={};f.motion_limited=True
        f.grip_counts={'pin':0};f.clear_counts={'pin':0}
        f.insertion={'pin':5.257916666666667};f.pin_prealignment_reference=5.107916666666667
        f._gate_side='left'
        return f

    def test_transverse_or_angular_error_holds_above_fixture(self):
        f=self.feedback()
        for pe,re in [(np.array([.0002,0,0]),np.zeros(3)),
                      (np.zeros(3),np.array([0,0,.004]))]:
            f.errors=lambda *args:(pe,re)
            reference,reason=f.gate(5.10,5.12,None,None,{'pin':0.})
            self.assertAlmostEqual(reference,f.pin_prealignment_reference-1e-8)
            self.assertEqual(reason,'pin_clearance_alignment')

    def test_aligned_part_can_advance_without_weakening_insertion_gate(self):
        f=self.feedback();f.errors=lambda *args:(np.zeros(3),np.zeros(3))
        reference,reason=f.gate(5.10,5.12,None,None,{'pin':0.})
        self.assertEqual(reference,5.12);self.assertIsNone(reason)
        f.errors=lambda *args:(np.array([.00003,0,0]),np.zeros(3))
        reference,reason=f.gate(5.25,5.27,None,None,{'pin':0.})
        self.assertEqual(reason,'pin_precision_alignment')
        self.assertLess(reference,f.insertion['pin'])


if __name__=='__main__':unittest.main()
