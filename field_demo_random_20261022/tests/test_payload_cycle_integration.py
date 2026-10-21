"""Actual cycle hooks + real mass transforms; fake devices/driver only."""
from contextlib import contextmanager
import copy,json,tempfile,threading
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from field_demo import random_cycle
from field_demo.payload_boundary import PayloadBoundaries
from test_random_cycle import FakeRuntime,recipe,pose
from test_payload_boundary import row

class IntegratedPayloadCycle(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name);(self.root/'measured.txt').write_text('SYNTHETIC OFFLINE ONLY')
  self.recipe=recipe();self.r=FakeRuntime(self.root,self.recipe);s=self.r.site;s.process=None;s.cancelled=threading.Event();s.depth=0;s.log_dir=self.root
  @contextmanager
  def lock():
   s.depth+=1
   try:yield
   finally:s.depth-=1
  s.control_lock=lock;s.require_ready=self.r.ready;s.plc=SimpleNamespace(read=self.r.read);s.event=lambda *a,**kw:None
  s.grippers={side:SimpleNamespace(status=lambda side=side:self.r.hand_status(side))for side in ['left','right']}
  self.profile=dict(schema_version=1,driver_binary='FAKE_OFFLINE',robots={side:dict(tool=row(.73),empty_payload=row(0.,diagonal=(0,0,0)))for side in ['left','right']},parts={p:row(.1,frame='part',center=(.001,0,0),diagonal=(1e-5,2e-5,2e-5))for p in ['ring1','ring2','pin']})
  self.writes=[];self.fail_write=None
  def apply(site,binary,side,tool,before,after,frame):
   self.assertGreater(site.depth,0);self.assertIsNone(site.process)
   self.writes.append((side,copy.deepcopy(after)))
   self.r.events.append(('load',side,after['mass_kg']))
   if self.fail_write is not None and len(self.writes)==self.fail_write:raise RuntimeError('SYNTHETIC unknown write/readback')
  self.r.payload=PayloadBoundaries(s,self.profile,self.root,apply=apply);self.r.flange_frame=lambda side:np.eye(4)
  self.r.wait_signal=lambda name,expected:self.assertEqual(self.r.read(name),expected)
 def test_all_three_real_cycle_boundaries_and_no_hand_doublecount(self):
  result=random_cycle.run(self.r,self.recipe);self.assertTrue(result['automatic_payload_boundaries_exercised']);self.assertEqual(result['payload_write_scope'],'fake_offline');self.assertFalse(result['motion_executed'])
  self.assertTrue(all(v[1]['mass_kg'] in [0.,.1]for v in self.writes));self.assertEqual(self.r.payload.owner,dict(left=None,right=None))
  for p in ['ring1','ring2','pin']:
   load=next(i for i,e in enumerate(self.r.events)if e[0]=='payload_grasp_confirmed_before_lift'and e[1]['part']==p)
   lift=next(i for i,e in enumerate(self.r.events)if e[:3]==('move','lift',p));self.assertLess(load,lift)
   release=next(i for i,e in enumerate(self.r.events)if e[0]=='payload_empty_confirmed_before_withdraw'and e[1]['part']==p)
   withdraw=next(i for i,e in enumerate(self.r.events)if e[:3]==('move','withdraw',p));self.assertLess(release,withdraw)
  for p in ['ring1','ring2']:
   load=next(i for i,e in enumerate(self.r.events)if e[0]=='payload_transfer_confirmed_before_retreat'and e[1]['part']==p)
   retreat=next(i for i,e in enumerate(self.r.events)if e[:3]==('move','donor_retreat',p));self.assertLess(load,retreat)
 def test_load_write_failure_stops_before_first_lift(self):
  self.fail_write=1
  with self.assertRaisesRegex(RuntimeError,'unknown write'):random_cycle.run(self.r,self.recipe)
  self.assertFalse(self.r.payload.valid);self.assertTrue(self.r.stopped);self.assertFalse(any(e[:2]==('move','lift')for e in self.r.events))
 def test_second_robot_transfer_write_failure_no_retreat_or_insert(self):
  # Identify exact transfer write index using one complete fake run.
  apply=self.r.payload.apply;counter=0
  def fail_donor_empty(*args):
   if self.r.payload.owner['right']=='ring1'and self.r.payload.owner['left']=='ring1'and args[2]=='right':raise RuntimeError('transfer donor reset unknown')
   return apply(*args)
  self.r.payload.apply=fail_donor_empty
  with self.assertRaisesRegex(RuntimeError,'donor reset unknown'):random_cycle.run(self.r,self.recipe)
  self.assertFalse(self.r.payload.valid);self.assertFalse(any(e[:2]==('move','donor_retreat')for e in self.r.events));self.assertTrue(self.r.stopped)
 def test_stale_or_missing_independent_donor_release_signal_blocks_transfer(self):
  self.r.wait_signal=lambda name,expected:(_ for _ in ()).throw(RuntimeError('donor PLC has not released'))
  with self.assertRaisesRegex(RuntimeError,'PLC has not released'):random_cycle.run(self.r,self.recipe)
  self.assertFalse(any(e[:2]==('move','donor_retreat')for e in self.r.events))
 def test_invalid_payload_rejects_before_any_motion(self):
  self.r.payload.valid=False
  with self.assertRaisesRegex(RuntimeError,'Payload state unknown'):random_cycle.run(self.r,self.recipe)
  self.assertFalse(any(e[0]in ['move','hand']for e in self.r.events))
 def test_actual_live_run_without_manager_cannot_move(self):
  self.r.payload=None;self.r.replay=False
  with self.assertRaisesRegex(RuntimeError,'requires measured automatic payload'):random_cycle.run(self.r,self.recipe)
  self.assertFalse(any(e[0]in ['move','hand']for e in self.r.events))
 def test_missing_null_profile_rejected_without_device_calls(self):
  with self.assertRaisesRegex(ValueError,'payload_profile required'):random_cycle.load_payload_profile({},self.root)
  profile=copy.deepcopy(self.profile);profile['parts']['pin']['center_m']=None;p=self.root/'payload.json';p.write_text(json.dumps(profile))
  with self.assertRaises(ValueError):random_cycle.load_payload_profile(dict(payload_profile='payload.json',payload_profile_sha256=random_cycle.sha(p)),self.root)
 def test_execute_missing_actual_inputs_rejected_before_any_device_constructor(self):
  p=self.root/'recipe.json';p.write_text(json.dumps(dict(site_config='site.json')));(self.root/'site.json').write_text('{}')
  with patch.object(random_cycle,'validate_recipe',return_value=dict(_root=str(self.root))),patch.object(random_cycle,'Site',side_effect=AssertionError('Device must not open')):
   with self.assertRaisesRegex(ValueError,'payload_profile required'):random_cycle.main(['--recipe',str(p),'--execute'])
 def test_startup_sdk_tool_load_mismatch_does_not_overwrite(self):
  s=dict(robot_mode_name='idle',dq=[0.]*7,dq_d=[0.]*7,m_ee_kg=.73,F_x_Cee_m=[0.]*3,I_ee_column_major_kg_m2=np.eye(3).ravel().tolist(),m_load_kg=0.,F_x_Cload_m=[0.]*3,I_load_column_major_kg_m2=[0.]*9)
  with self.assertRaisesRegex(RuntimeError,'differs'):random_cycle.validate_payload_sdk_state(self.r.payload,'left',s)
  self.assertFalse(self.writes)
if __name__=='__main__':unittest.main()
