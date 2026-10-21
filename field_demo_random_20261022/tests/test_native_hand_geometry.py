import unittest,numpy as np
from field_demo.native_hand_geometry import actual_hand_points,surfaces
class HandGeometryTests(unittest.TestCase):
 def test_actual_desk_tcp_offset_cancels_in_measured_flange_mount(self):
  F=np.eye(4);F[2,3]=.12;M=np.eye(4);M[0,3]=.001
  P=actual_hand_points(F,F,M,.08);raw=surfaces()['palm_points'];np.testing.assert_allclose(P[:len(raw)],raw+[.001,0,0])
 def test_native_fingers_follow_actual_open_width_not_taught_width(self):
  A=actual_hand_points(np.eye(4),np.eye(4),np.eye(4),.02);B=actual_hand_points(np.eye(4),np.eye(4),np.eye(4),.08);n=len(surfaces()['palm_points']);m=len(surfaces()['finger_points'])
  np.testing.assert_allclose(B[n:n+m]-A[n:n+m],np.tile([0,.03,0],(m,1)),atol=1e-12)
  np.testing.assert_allclose(B[n+m:]-A[n+m:],np.tile([0,-.03,0],(m,1)),atol=1e-12)
