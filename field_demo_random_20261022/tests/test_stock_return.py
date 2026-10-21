"""Actual return orchestrator with explicit fake SDK/camera/planner boundaries."""
import tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from field_demo.random_cycle import LiveRuntime
class ReturnTests(unittest.TestCase):
 def fixture(self,root):
  I=np.eye(4).tolist();m=dict(verified=True,by='offline_synthetic',at_utc='2026-10-21T00:00:00+00:00',evidence_file='measure.txt');(root/'measure.txt').write_text('SYNTHETIC ONLY')
  spec=dict(T_cell_tcp=I,measurement=m,release_region=dict(T_cell_box=I,half_extents_m=[.3,.3,.3],measurement=m),held_extent_radius_m=.08,release_action=dict(width_m=.08,speed_m_s=.005),empty_hand_views=[dict(T_tcp_inspection=I,measurement=m)])
  events=[];rt=SimpleNamespace(root=root,output=root,recipe=dict(max_observation_age_s=2,inventory_inspection=dict(return_over_bin=dict(right=spec))),site=SimpleNamespace(expected_held=dict(right='ring1')))
  def plan(phase,*a,**kw):
   if phase=='empty_hand_inspection':self.assertTrue(a[-1], 'Possible snagged inventory must remain enclosed until empty proof')
   events.append(('plan',phase));return dict(phase=phase)
  rt.plan=plan;rt.execute=lambda p,o:events.append(('execute',p['phase']));rt.tcp=lambda s:I
  rt.hand_move=lambda *a:(events.append(('open',))or dict(completed=True,is_grasped=False,width_m=.08))
  rt.facts=SimpleNamespace(confirm_release=lambda *a:events.append(('SDK_release',)))
  rt.payload=SimpleNamespace(after_bin_return_release=lambda *a:events.append(('empty_load',)))
  rt.observe=lambda *a:dict(instances=[dict(part='ring1',instance_id='observed',T_cell_part=I)])
  rt._camera=lambda *a:(SimpleNamespace(read=lambda:({},dict(capture_id='actual_fake_exposure'))),{},root/'camera.json')
  rt._inventory_frame=lambda *a:{};rt.hand_status=lambda s:dict(completed=True,is_grasped=False,width_m=.08);rt.event=lambda *a,**kw:None;rt.flange_frame=lambda s:I
  return rt,events,dict(part='ring1'),dict(T_tcp_part=I,payload_dimensions=dict(radius_m=.06,length_m=.02))
 def test_blocked_return_never_opens_hand(self):
  with tempfile.TemporaryDirectory()as d:
   r,e,entry,carry=self.fixture(Path(d));r.plan=lambda *a,**kw:(_ for _ in ()).throw(RuntimeError('actual geometry blocked'))
   with self.assertRaisesRegex(RuntimeError,'geometry blocked'):LiveRuntime.return_stock_to_bin(r,entry,'right',{},carry,np.eye(4),'epoch')
   self.assertFalse(e)
 def test_no_release_outside_measured_enclosing_region(self):
  with tempfile.TemporaryDirectory()as d:
   r,e,entry,carry=self.fixture(Path(d));T=np.eye(4);T[0,3]=.5;r.tcp=lambda s:T
   with self.assertRaisesRegex(RuntimeError,'release region'):LiveRuntime.return_stock_to_bin(r,entry,'right',{},carry,np.eye(4),'epoch')
   self.assertFalse(any(row[0]=='open'for row in e))
 def test_empty_load_only_after_real_release_and_two_view_empty_proof(self):
  for verified in [False,True]:
   with self.subTest(verified=verified),tempfile.TemporaryDirectory()as d:
    r,e,entry,carry=self.fixture(Path(d))
    with patch('field_demo.camera.save_capture',side_effect=lambda p,*a:p.write_bytes(b'actual-fake-packet')),patch('field_demo.selected_inventory_guard.verify_empty_hand',return_value=dict(verified=verified,reason='synthetic',source_capture_ids=['a','b'])):
     if verified:LiveRuntime.return_stock_to_bin(r,entry,'right',{},carry,np.eye(4),'epoch')
     else:
      with self.assertRaisesRegex(RuntimeError,'not optically confirmed'):LiveRuntime.return_stock_to_bin(r,entry,'right',{},carry,np.eye(4),'epoch')
    self.assertEqual(any(row[0]=='empty_load'for row in e),verified)
    self.assertLess(e.index(('execute','inventory_return')),e.index(('open',)))
