import contextlib,io,json,subprocess,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from capx_autoring.offline import IntegrationRuntime,integration_check,fixture_recipe
from capx_autoring.runner import RingSession,run_runtime,PHASES,ORDER,phase_evidence,strict_value
from capx_autoring.config import initialize,engine_identity,preflight
from capx_autoring.__main__ import main
from capx_autoring.field_demo.random_cycle import screen_pickup,check_observation
from capx_autoring.admission import AdmittedHand,AdmittedPLC,CurrentAdmission

class RunnerTests(unittest.TestCase):
 def test_complete_original_policy_two_rings_and_fresh_both_no_hardware_claim(self):
  with tempfile.TemporaryDirectory()as d:
   r=integration_check(Path(d)/'run');events=r['events']
   self.assertEqual([(e['part'],e['phase'])for e in events],[(p,f)for p in ORDER for f in PHASES])
   self.assertEqual(len({e['action_token']for e in events}),16)
   self.assertTrue(r['completed_offline_contract']);self.assertFalse(r['accepted_ring_pair']);self.assertFalse(r['accepted_full3'])
   self.assertEqual(r['components']['completed_parts'],list(ORDER))
   self.assertTrue(set(x['capture_id']for x in r['fresh_both_final']['rings']).isdisjoint(e['evidence']['capture_id']for e in events))
   trace=json.loads((Path(d)/'run/offline-command-trace.json').read_text());self.assertTrue(any(x.get('side')=='left'and x.get('kind')=='arm'for x in trace));self.assertTrue(any(x.get('side')=='right'and x.get('kind')=='arm'for x in trace))
 def test_native_pickup_rejection_bounded_no_grasp_or_motion(self):
  with tempfile.TemporaryDirectory()as d:
   out=Path(d)/'fail'
   with self.assertRaises(RuntimeError):integration_check(out,fail_phase='pickup')
   record=json.loads((out/'native-ring-pair-session-final.json').read_text());trace=json.loads((out/'offline-command-trace.json').read_text())
   self.assertTrue(record['cancelled']);self.assertFalse(record['accepted_ring_pair']);self.assertEqual(len(record['events']),1);self.assertFalse(any(x.get('kind')=='arm'or x.get('mode')=='grasp'for x in trace))
 def test_insert_refusal_no_clamp_release_or_withdraw(self):
  with tempfile.TemporaryDirectory()as d:
   out=Path(d)/'fail'
   with self.assertRaises(RuntimeError):integration_check(out,fail_phase='insert')
   trace=json.loads((out/'offline-command-trace.json').read_text());self.assertFalse(any(x['kind']=='clamp'or x.get('phase')=='withdraw'for x in trace))
   record=json.loads((out/'native-ring-pair-session-final.json').read_text());self.assertEqual(record['events'][-1]['phase'],'preinsert')
 def test_large_failure_preserves_small_completion_but_not_pair_success(self):
  with tempfile.TemporaryDirectory()as d:
   output=Path(d)/'failed-large'
   with self.assertRaises(RuntimeError):integration_check(output,fail_phase=('ring1','insert'))
   r=json.loads((output/'native-ring-pair-session-final.json').read_text());self.assertEqual(r['component_progress']['ring2'],list(PHASES));self.assertEqual(r['component_progress']['ring1'],list(PHASES[:4]));self.assertFalse(r['accepted_ring_pair']);self.assertFalse(r['accepted_full3'])
 def test_failed_receiver_grasp_does_not_open_donor(self):
  with tempfile.TemporaryDirectory()as d:
   session=RingSession(domain='offline_contract',source_sha256='0'*64,config_sha256='0'*64);runtime=IntegrationRuntime(Path(d)/'captures',session);original=runtime.hand_move
   def hand(side,part,mode,parameters):
    if side=='left'and mode=='grasp':return runtime.hand_status(side)
    return original(side,part,mode,parameters)
   with patch.object(runtime,'hand_move',side_effect=hand),self.assertRaises(RuntimeError):run_runtime(runtime,runtime.recipe,session,Path(d)/'failed')
   self.assertFalse(any(c.get('side')=='right'and c.get('mode')=='release'for c in runtime.commands));self.assertTrue(runtime.hand_status('right')['is_grasped'])
 def test_phase_replay_wrongorder_cancel_refused(self):
  for part,phase in [('ring1','perception'),('ring2','grasp')]:
   s=RingSession(domain='offline_contract',source_sha256='0'*64,config_sha256='0'*64)
   with self.assertRaises(RuntimeError):s.advance(part,phase,{})
  s.advance('ring2','perception',{})
  with self.assertRaises(RuntimeError):s.advance('ring2','perception',{})
  s.cancelled=True
  with self.assertRaises(RuntimeError):s.advance('ring2','grasp',{})
 def test_fake_devices_cannot_be_relabeled_as_hardware(self):
  with tempfile.TemporaryDirectory()as d:
   s=RingSession(domain='hardware',source_sha256='0'*64,config_sha256='0'*64);rt=IntegrationRuntime(d,s)
   with self.assertRaises(ValueError):run_runtime(rt,rt.recipe,s,Path(d)/'run')
 def test_final_failure_and_keyboardinterrupt_preserve_partial_attempt(self):
  for error in [RuntimeError('final image missing'),KeyboardInterrupt()]:
   with self.subTest(type=type(error).__name__),tempfile.TemporaryDirectory()as d:
    s=RingSession(domain='offline_contract',source_sha256='0'*64,config_sha256='0'*64);rt=IntegrationRuntime(Path(d)/'captures',s)
    with patch.object(rt,'fresh_both',side_effect=error):
     with self.assertRaises(type(error)):run_runtime(rt,rt.recipe,s,Path(d)/'run')
    r=json.loads((Path(d)/'run/native-ring-pair-session-final.json').read_text());self.assertTrue(r['cancelled']);self.assertFalse(r['accepted_ring_pair']);self.assertEqual(len(r['events']),16);self.assertTrue(rt.stopped)
 def test_default_preflight_and_execute_missing_site_do_not_connect(self):
  with tempfile.TemporaryDirectory()as d:
   cfg=initialize(d)
   with patch('socket.socket',side_effect=AssertionError('No network')),patch('subprocess.run',side_effect=AssertionError('No subprocess')),patch('subprocess.Popen',side_effect=AssertionError('No process')):
    self.assertFalse(preflight(cfg)[0]['local_config_complete'])
    with contextlib.redirect_stdout(io.StringIO()):
     self.assertEqual(main(['run','--config',str(cfg)]),2)
     self.assertEqual(main(['run','--config',str(cfg),'--execute','--output',str(Path(d)/'blocked')]),2)
   self.assertTrue((Path(d)/'blocked/blocked-launch.json').exists())
 def test_init_never_overwrites_existing_measurements(self):
  with tempfile.TemporaryDirectory()as d:
   camera=Path(d)/'camera.json';camera.write_text('ORIGINAL MEASUREMENTS');initialize(d)
   before={p.name:p.read_bytes()for p in Path(d).iterdir()if p.is_file()}
   with self.assertRaises(FileExistsError):initialize(d)
   self.assertEqual(before,{p.name:p.read_bytes()for p in Path(d).iterdir()if p.is_file()})
 def test_gate_refusals_prevent_real_hand_and_clamp_writes(self):
  from types import SimpleNamespace
  calls=[];gate=SimpleNamespace(admit=lambda *a:(_ for _ in ()).throw(RuntimeError('denied')));site=SimpleNamespace(control_lock=contextlib.nullcontext)
  hand=AdmittedHand(SimpleNamespace(move=lambda **a:calls.append('hand')),site,'right',gate)
  plc=AdmittedPLC(SimpleNamespace(write=lambda *a:calls.append(a)),site,gate)
  with self.assertRaises(RuntimeError):hand.move(width_m=.01)
  with self.assertRaises(RuntimeError):plc.write('ring1_clamp_command',True)
  self.assertEqual(calls,[]);plc.write('cell_stop',True);self.assertEqual(calls,[('cell_stop',True)])
 def test_measured_camera_wrong_K_size_and_nonfinite_refused(self):
  from capx_autoring.camera import MeasuredCamera
  original='capx_autoring.field_demo.camera.RealSenseCapture.read'
  expected={'width':32,'height':32,'fps':15,'max_age_s':.2,'K':np.eye(3).tolist(),'intrinsic_tolerance_px':1e-6};cfg={'camera_serial':'OFFLINE'}
  packet={'K':np.eye(3),'depth_m':np.ones((32,32)),'rgb':np.zeros((32,32,3),np.uint8)}
  camera=MeasuredCamera(cfg,expected)
  with patch(original,return_value=(packet,{})):self.assertIs(camera.read()[0],packet)
  for key,value in [('K',np.diag([2.,1.,1.])),('K',np.full((3,3),np.nan)),('depth_m',np.ones((31,32)))]:
   bad={**packet,key:value}
   with patch(original,return_value=(bad,{})),self.assertRaises(RuntimeError):camera.read()
 def test_nonfinite_diagnostics_preserved_in_strict_json(self):
  value=strict_value({'failure':{'details':[float('nan'),float('inf')]}});json.dumps(value,allow_nan=False);self.assertEqual(value['failure']['details'][0],{'nonfinite':'nan'})
 def test_engine_and_build_dependency_closure(self):
  self.assertEqual(len(engine_identity()),64)
  from capx_autoring.config import ROOT
  import re
  cmake=(ROOT/'isaac_port/fieldkit/CMakeLists.txt').read_text()
  for name in re.findall(r'\b([A-Za-z0-9_]+\.cpp)\b',cmake):self.assertTrue((ROOT/'isaac_port/fieldkit'/name).is_file(),name)

if __name__=='__main__':unittest.main()
