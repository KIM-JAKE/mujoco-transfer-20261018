import threading,time,unittest
from field_demo.hand_monitor import HandMonitor
class HandMonitorTests(unittest.TestCase):
 def test_stalled_or_failed_actual_read_cannot_keep_old_grasp_alive(self):
  block=threading.Event();clock=[0.]
  m=HandMonitor(lambda:block.wait(1),clock=lambda:clock[0]);self.addCleanup(block.set);self.addCleanup(m.close)
  value=dict(completed=True,is_grasped=True,width_m=.05,robot_time_s=10.)
  m.publish(value);self.assertTrue(m.status()['is_grasped']);clock[0]=.201
  with self.assertRaisesRegex(RuntimeError,'stale'):m.status()
 def test_older_sdk_packet_cannot_overwrite_completed_opening(self):
  block=threading.Event();m=HandMonitor(lambda:block.wait(1));self.addCleanup(block.set);self.addCleanup(m.close)
  m.publish(dict(completed=True,is_grasped=False,width_m=.08,robot_time_s=11.))
  m.publish(dict(completed=True,is_grasped=True,width_m=.05,robot_time_s=10.))
  self.assertFalse(m.status()['is_grasped'])

 def test_repeated_same_sdk_timestamp_does_not_refresh_stale_hold(self):
  block=threading.Event();clock=[0.];m=HandMonitor(lambda:block.wait(1),clock=lambda:clock[0]);self.addCleanup(block.set);self.addCleanup(m.close)
  sample=dict(completed=True,is_grasped=True,width_m=.05,robot_time_s=10.)
  m.publish(sample);clock[0]=.15;m.publish(sample);clock[0]=.201
  with self.assertRaisesRegex(RuntimeError,'stale'):m.status()
