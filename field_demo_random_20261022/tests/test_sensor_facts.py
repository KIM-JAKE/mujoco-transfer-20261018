"""Measured-image + SDK part facts; no privileged PLC part sensors."""
from datetime import datetime,timezone,timedelta
import copy
from types import SimpleNamespace
import unittest
import numpy as np
from field_demo.sensor_facts import SensorFacts
from field_demo import random_cycle
from test_random_cycle import pose
import test_payload_cycle_integration as payload_tests


def image(part='ring1',stamp=None,capture='camera-1',position=None):
 stamp=stamp or datetime.now(timezone.utc).isoformat();v=dict(part=part,instance_id=capture+':pixel',T_cell_part=position or pose(.65,0,.1))
 return dict(capture_id=capture,captured_at_utc=stamp,calibration_sha256='measured-camera',instances=[v],selected_instance=v)

class SensorFactsTests(unittest.TestCase):
 def setUp(self):
  self.now=datetime.now(timezone.utc);self.recipe=dict(max_observation_age_s=2.,insert_pose_confirmation_tolerance_m=.001,target_angle_tolerance_rad=.01,retention_observation_dwell_s=.3,insertion_targets={'ring1':pose(.65,0,.1)})
  self.f=SensorFacts(self.recipe,clock=lambda:self.now);self.held=dict(is_grasped=True,width_m=.062);self.open=dict(is_grasped=False,width_m=.08)
 def test_grasp_requires_both_actual_image_and_SDK(self):
  with self.assertRaisesRegex(RuntimeError,'SDK'):self.f.confirm_held('ring1','right',image(),pose(),self.open)
  with self.assertRaisesRegex(RuntimeError,'Fresh'):self.f.confirm_held('ring1','right',image(stamp=(self.now-timedelta(seconds=10)).isoformat()),pose(),self.held)
  self.f.confirm_held('ring1','right',image(stamp=self.now.isoformat()),pose(),self.held);self.assertTrue(self.f.grasped('ring1','right',self.held));self.assertFalse(self.f.grasped('ring1','right',self.open))
 def test_seating_checks_axis_and_center_not_only_RGB_class(self):
  p=pose(.65,0,.1);p[0][3]+=.01
  with self.assertRaisesRegex(RuntimeError,'center/axis'):self.f.confirm_seated('ring1',image(position=p,stamp=self.now.isoformat()))
  p=pose(.65,0,.1);p[:]=np.array(p)@np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1]])
  with self.assertRaisesRegex(RuntimeError,'center/axis'):self.f.confirm_seated('ring1',image(position=p,stamp=self.now.isoformat()))
 def test_retention_needs_clamp_and_distinct_postrelease_persistence(self):
  self.f.confirm_seated('ring1',image(stamp=self.now.isoformat()));self.f.confirm_clamp('ring1',source='operator_manual_clamp',evidence={'operator':'SYNTHETIC'})
  self.f.confirm_release('ring1','left',self.open,.079);self.now+=timedelta(seconds=.5)
  a=image(capture='a',stamp=(self.now-timedelta(seconds=.4)).isoformat());b=image(capture='b',stamp=self.now.isoformat())
  with self.assertRaisesRegex(RuntimeError,'persistence'):self.f.confirm_retained('ring1','left',[b,b],self.open)
  self.f.confirm_retained('ring1','left',[a,b],self.open);self.assertTrue(self.f.fixture_fact('ring1','retained'))
 def test_unknown_or_unobserved_clamp_not_invented(self):
  with self.assertRaisesRegex(RuntimeError,'Actual clamp'):self.f.confirm_clamp('ring1',source='constant_true',evidence={'flag':True})
 def test_transfer_receiver_photo_must_follow_measured_donor_release(self):
  self.f.confirm_held('ring1','right',image(stamp=self.now.isoformat()),pose(),self.held);old=image(stamp=(self.now-timedelta(seconds=.1)).isoformat());self.f.confirm_release('ring1','right',self.open,.079)
  with self.assertRaisesRegex(RuntimeError,'predates'):self.f.confirm_transfer('ring1','right','left',old,pose(),self.open,self.held,.079)
  self.now+=timedelta(seconds=.1);self.f.confirm_transfer('ring1','right','left',image(stamp=self.now.isoformat()),pose(),self.open,self.held,.079);self.assertTrue(self.f.transfer_confirmed('ring1','right','left'))

class NoPartPLCCycle(payload_tests.IntegratedPayloadCycle):
 # Reuse integration setup but exercise the new evidence route end to end.
 def setUp(self):
  super().setUp();r=self.r;r.facts=SensorFacts(self.recipe);r.payload.facts=r.facts
  r.site.plc=SimpleNamespace(read=lambda name:(_ for _ in ()).throw(AssertionError('No privileged PLC part sensors')))
  r.read=lambda name:random_cycle.LiveRuntime.read(r,name)
  r.wait=lambda name,timeout,dwell:random_cycle.LiveRuntime.wait(r,name,timeout,dwell)
  def clamp(part):
   r.signals[part+'_clamped']=True  # Fake fixture physically retains after explicit fake operator action.
   return r.facts.confirm_clamp(part,source='operator_manual_clamp',evidence={'operator':'SYNTHETIC TEST ONLY'})
  r.clamp=clamp
  r.wait_signal=lambda *a:(_ for _ in ()).throw(AssertionError('No privileged PLC donor signal'))
  original=r.observe
  def observe(*args,**kwargs):
   value=original(*args,**kwargs);stamp=datetime.now(timezone.utc).isoformat();value['captured_at_utc']=stamp;value['source_metadata']['captured_at_utc']=stamp;return value
  r.observe=observe
  r.observe_fixture=lambda part:r.observe('fixture',part,side=next(e['insert_side']for e in r.recipe['parts']if e['part']==part))
 # Legacy PLC-failure tests describe the old adapter and intentionally do not
 # apply to this class; inherited happy path plus payload failure tests do.
 def test_stale_or_missing_independent_donor_release_signal_blocks_transfer(self):
  r=self.r;original=r.hand_status
  def missing(side):
   status=original(side)
   if r.facts.releases.get('right',{}).get('part')=='ring1'and side=='right':return dict(status,is_grasped=True)
   return status
  r.hand_status=missing
  with self.assertRaisesRegex(RuntimeError,'Donor has not actually released'):random_cycle.run(r,self.recipe)
  self.assertFalse(any(e[:2]==('move','donor_retreat')for e in r.events))
if __name__=='__main__':unittest.main()
