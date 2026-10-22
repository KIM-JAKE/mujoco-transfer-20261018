"""An actuator that has not applied a correction must not accumulate a new target."""
import unittest
import numpy as np
from .pose_feedback import PoseFeedback
class EntryAntiWindupTests(unittest.TestCase):
    def feedback(self):
        f=PoseFeedback.__new__(PoseFeedback)
        f.offset=np.full(30,.02)
        f.applied_arm_offset=np.full(30,.001)
        return f
    def test_stalled_applied_offset_cannot_accumulate_target_windup(self):
        f=self.feedback();indices=np.arange(7);increment=np.full(7,.0001)
        for _ in range(100):f.accumulate_offset(indices,increment,track_applied=True)
        np.testing.assert_allclose(f.offset[indices],.0011,rtol=0,atol=1e-15)
        np.testing.assert_array_equal(f.offset[7:],np.full(23,.02))
        f.applied_arm_offset[indices]=f.offset[indices]
        f.accumulate_offset(indices,increment,track_applied=True)
        np.testing.assert_allclose(f.offset[indices],.0012,rtol=0,atol=1e-15)
    def test_missing_nonfinite_or_wrong_shape_applied_state_rejects_without_mutation(self):
        for applied in [None,np.zeros(14),np.full(30,np.nan),np.full(30,np.inf)]:
            with self.subTest(applied=applied):
                f=self.feedback();f.applied_arm_offset=applied;before=f.offset.copy()
                with self.assertRaisesRegex(RuntimeError,'finite applied'):
                    f.accumulate_offset(np.arange(7),np.full(7,.0001),track_applied=True)
                np.testing.assert_array_equal(f.offset,before)
    def test_legacy_accumulation_and_global_correction_bound_are_preserved(self):
        f=self.feedback();indices=np.arange(7)
        f.accumulate_offset(indices,np.full(7,.0001))
        np.testing.assert_allclose(f.offset[indices],.0201,rtol=0,atol=1e-15)
        f.applied_arm_offset[indices]=.24995
        f.accumulate_offset(indices,np.full(7,.0001),track_applied=True)
        np.testing.assert_array_equal(f.offset[indices],np.full(7,.25))
    def test_nonfinite_increment_never_reaches_motor_target(self):
        f=self.feedback();before=f.offset.copy()
        with self.assertRaisesRegex(RuntimeError,'correction increment'):
            f.accumulate_offset(np.arange(7),np.full(7,np.nan),track_applied=True)
        np.testing.assert_array_equal(f.offset,before)
if __name__=='__main__':unittest.main()
