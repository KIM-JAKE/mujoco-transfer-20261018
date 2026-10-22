"""Instantaneous pose agreement cannot open a moving-pin alignment gate."""
import unittest
import numpy as np
from pose_feedback import PoseFeedback


class PinSettlingTests(unittest.TestCase):
    def setUp(self):
        self.f=PoseFeedback.__new__(PoseFeedback)
        self.f.parts=['ring1','ring2','pin'];self.f.pin_entry_settled=True;self.f.pin_entry_stable_count=0
        self.f.part_linear_velocity=np.zeros((3,3));self.f.part_angular_velocity=np.zeros((3,3))

    def test_moving_pin_resets_count_and_three_stationary_reads_are_required(self):
        self.f.part_linear_velocity[2,0]=.02
        self.assertFalse(self.f.pin_entry_is_settled())
        self.f.part_linear_velocity[:]=0
        self.assertFalse(self.f.pin_entry_is_settled());self.assertFalse(self.f.pin_entry_is_settled())
        self.assertTrue(self.f.pin_entry_is_settled())
        self.f.part_angular_velocity[2,0]=.04
        self.assertFalse(self.f.pin_entry_is_settled());self.assertEqual(self.f.pin_entry_stable_count,0)

    def test_missing_or_nonfinite_velocity_cannot_confirm_stationarity(self):
        self.f.part_linear_velocity=None
        with self.assertRaisesRegex(RuntimeError,'measured part velocities'):self.f.pin_entry_is_settled()
        self.f.part_linear_velocity=np.full((3,3),np.nan)
        with self.assertRaisesRegex(RuntimeError,'Invalid measured'):self.f.pin_entry_is_settled()

    def test_legacy_mode_does_not_change_its_original_gate(self):
        self.f.pin_entry_settled=False;self.f.part_linear_velocity=None
        self.assertTrue(self.f.pin_entry_is_settled())


if __name__=='__main__':unittest.main()
