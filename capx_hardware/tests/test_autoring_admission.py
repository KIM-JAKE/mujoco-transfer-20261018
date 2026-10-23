import contextlib,hashlib,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from capx_autoring.admission import CurrentAdmission,COVERAGE
from hardware_v1.config import canonical
from hardware_v1.backend import MotionDisabled

class AdmissionTests(unittest.TestCase):
 def fixture(self,d):
  g=object.__new__(CurrentAdmission);g.identity='1'*64;g.used=set();g.recipe={'insertion_targets':{'ring1':[],'ring2':[]}};g.runtime=SimpleNamespace(_latest_plan_context=None,installed_parts=[],carried_state=lambda:[]);g.unchanged=lambda:None
  f=Path(d)/'checker';f.write_text('OFFLINE INERT FIXTURE');sha=hashlib.sha256(f.read_bytes()).hexdigest()
  g.backend=SimpleNamespace(base=Path(d),config_digest='2'*64,config={'site_config_sha256':'3'*64,'geometry_record':{},'tools':{},'admission':{'executable':{'path':str(f),'sha256':sha}}})
  states={s:{'state':{'q':[0.]*7,'q_d':[0.]*7}}for s in ['left','right']};hands={s:{'width_m':.01}for s in states};g._states=lambda site:(states,hands)
  site=SimpleNamespace(require_ready=lambda:None,autoring_io_context=None,event=lambda *a,**kw:None)
  return g,site,states,hands
 def responder(self,change=None):
  def respond(args,**kw):
   request=json.loads(kw['input']);proof={'schema':'capx-hardware-autoring-approval/v1','domain':'hardware','request_sha256':hashlib.sha256(kw['input'].encode()).hexdigest(),'approved':True,**{k:True for k in COVERAGE}}
   if change:change(proof,request)
   return SimpleNamespace(stdout=json.dumps(proof))
  return respond
 def test_complete_request_is_bound_to_actual_states_current_goal_context(self):
  with tempfile.TemporaryDirectory()as d:
   g,s,_,_=self.fixture(d)
   with patch('capx_autoring.admission.subprocess.run',side_effect=self.responder())as call:g.admit(s,{'kind':'arm'})
   r=json.loads(call.call_args.kwargs['input']);self.assertEqual(r['domain'],'hardware');self.assertEqual(set(r['current_states']),{'left','right'});self.assertIn('fixed_world_part_goals',r['task_context']);self.assertEqual(len(g.used),1)
 def test_every_coverage_field_requires_exact_true(self):
  for key in COVERAGE:
   for bad in [False,1,'true',None]:
    with self.subTest(key=key,bad=bad),tempfile.TemporaryDirectory()as d:
     g,s,_,_=self.fixture(d)
     with patch('capx_autoring.admission.subprocess.run',side_effect=self.responder(lambda p,r:p.update({key:bad}))),self.assertRaises(MotionDisabled):g.admit(s,{'kind':'hand'})
 def test_foreign_proof_and_old_domain_rejected(self):
  for change in [{'request_sha256':'0'*64},{'domain':'simulation'},{'approved':1},{'schema':'old'}]:
   with self.subTest(change=change),tempfile.TemporaryDirectory()as d:
    g,s,_,_=self.fixture(d)
    with patch('capx_autoring.admission.subprocess.run',side_effect=self.responder(lambda p,r:p.update(change))),self.assertRaises(MotionDisabled):g.admit(s,{'kind':'arm'})
 def test_stale_proof_refused(self):
  with tempfile.TemporaryDirectory()as d:
   g,s,_,_=self.fixture(d)
   with patch('capx_autoring.admission.subprocess.run',side_effect=self.responder()),patch('capx_autoring.admission.time.monotonic',side_effect=[0.,3.]),self.assertRaises(MotionDisabled):g.admit(s,{'kind':'arm'})
 def test_actual_command_or_width_change_refused(self):
  import copy
  for key in ['q','q_d','width']:
   with self.subTest(key=key),tempfile.TemporaryDirectory()as d:
    g,s,states,hands=self.fixture(d);ns=copy.deepcopy(states);nh=copy.deepcopy(hands)
    if key=='width':nh['left']['width_m']+=.001
    else:ns['left']['state'][key][0]+=.001
    responses=iter([(states,hands),(ns,nh)]);g._states=lambda site:next(responses)
    with patch('capx_autoring.admission.subprocess.run',side_effect=self.responder()),self.assertRaises(MotionDisabled):g.admit(s,{'kind':'arm'})
 def test_changed_checker_fails_before_spawn(self):
  with tempfile.TemporaryDirectory()as d:
   g,s,_,_=self.fixture(d);(Path(d)/'checker').write_text('CHANGED')
   with patch('capx_autoring.admission.subprocess.run')as call,self.assertRaises(MotionDisabled):g.admit(s,{'kind':'arm'})
   call.assert_not_called()

if __name__=='__main__':unittest.main()
