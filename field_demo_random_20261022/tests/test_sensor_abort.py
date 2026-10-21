"""No robot calls: arm heartbeat stop and completed native grasp command policy."""
import io,threading,unittest
from types import SimpleNamespace
from unittest.mock import Mock
from isaac_port.fieldkit.site import Site

class SensorAbortTests(unittest.TestCase):
 def site(self,policy='preserve_completed_grasp'):
  s=Site.__new__(Site);s.config=dict(cell_io_mode='SDK_stop',part_evidence_source='RGBD_SDK',abort_hand_policy=policy);s.cancelled=threading.Event();s.expected_held={'right':'ring1'};s.plc=None;s.events=[];s.event=lambda name,**kw:s.events.append((name,kw));s.grippers={k:SimpleNamespace(stop=Mock(),status=Mock(return_value=dict(is_grasped=k=='right',width_m=.062 if k=='right'else .08)))for k in ['left','right']};s.process=SimpleNamespace(poll=lambda:None,stdin=io.StringIO());return s
 def test_normal_abort_stops_active_arm_without_cancelling_completed_grasp(self):
  s=self.site();s.stop();self.assertEqual(s.process.stdin.getvalue(),'STOP\n');self.assertTrue(s.cancelled.is_set())
  for g in s.grippers.values():g.stop.assert_not_called()
  self.assertTrue(any(name=='abort_hand_command_preserved'and value['retention_guaranteed']is False and value['resume_allowed']is False for name,value in s.events))
 def test_preserved_command_is_not_automatic_resume_authorization(self):
  s=self.site();s.stop()
  with self.assertRaisesRegex(RuntimeError,'stopped'):s.require_ready()
 def test_SDK_stop_mode_requires_real_current_grasp_without_partPLC(self):
  s=self.site();s.require_ready();s.grippers['right'].status.return_value=dict(is_grasped=False,width_m=.08)
  with self.assertRaises(RuntimeError):s.require_ready()
 def test_legacy_explicit_cancel_policy_unchanged(self):
  s=self.site('cancel_grasp');s.stop()
  for g in s.grippers.values():g.stop.assert_called_once()
if __name__=='__main__':unittest.main()
