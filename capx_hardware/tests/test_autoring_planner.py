import hashlib,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from capx_autoring.planner import CurrentPlanner
from capx_autoring.field_demo.random_planner import DynamicPlanner

class PlannerTests(unittest.TestCase):
 def build(self,d):
  p=Path(d);exe=p/'gpu';exe.write_text('INERT');model=p/'model.json';model.write_text('{}')
  spec={'executable':{'path':'gpu','sha256':hashlib.sha256(exe.read_bytes()).hexdigest()},'model_record':{'path':'model.json','sha256':hashlib.sha256(model.read_bytes()).hexdigest()},'timeout_s':1.}
  setting=p/'planner.json';setting.write_text(json.dumps({'planner_binary':'native','cuRobo_proposal':spec}))
  site=SimpleNamespace(config={'robots':{s:{'ip':s,'T_cell_base':np.eye(4).tolist()}for s in ['left','right']}},grippers={s:SimpleNamespace(status=lambda:{'width_m':.01})for s in ['left','right']},event=lambda *a,**kw:None)
  planner=CurrentPlanner(site,setting,p/'out')
  state={'read_only':True,'motion_executed':False,'server_protocol_version':10,'state':{'q':[0.]*7,'q_d':[0.]*7,'dq':[0.]*7,'dq_d':[0.]*7,'F_T_EE_column_major':np.eye(4).ravel(order='F').tolist(),'O_T_EE_column_major':np.eye(4).ravel(order='F').tolist()}}
  return planner,state
 def reply(self,change=None):
  def call(args,**kw):
   r=json.loads(kw['input']);start=r['T_cell_tcp_start'];segments=[]
   for goal in r['mandatory_T_cell_tcp_targets']:segments.append([start,goal]);start=goal
   a={'schema':'capx-current-curobo-proposal-reply/v1','request_sha256':hashlib.sha256(kw['input'].encode()).hexdigest(),'model_record_sha256':r['model_record_sha256'],'segments':segments}
   if change:change(a)
   return SimpleNamespace(stdout=json.dumps(a))
  return call
 def test_native_success_uses_no_gpu_and_gpu_candidate_is_rechecked_natively(self):
  with tempfile.TemporaryDirectory()as d:
   p,s=self.build(d);target=np.eye(4).tolist()
   with patch.object(DynamicPlanner,'plan',return_value={'native':True}),patch('capx_autoring.planner.subprocess.run')as call:self.assertTrue(p.plan('carry','ring1','right',[target],{})['native']);call.assert_not_called()
   with patch.object(DynamicPlanner,'plan',side_effect=[RuntimeError('original native refused'),{}])as native,patch('capx_autoring.planner.run_native',return_value=s),patch('capx_autoring.planner.subprocess.run',side_effect=self.reply()):
    result=p.plan('carry','ring1','right',[target],{});self.assertEqual(native.call_count,2);self.assertEqual(result['proposal_source'],'optional_site_cuRobo_then_original_native')
 def test_wrong_id_model_goal_nan_and_mandatory_target_omission_refused(self):
  changes=[lambda a:a.update(request_sha256='0'*64),lambda a:a.update(model_record_sha256='0'*64),lambda a:a['segments'][0][-1][0].__setitem__(3,.001),lambda a:a['segments'][0][-1][0].__setitem__(0,float('nan')),lambda a:a.update(segments=[]),lambda a:a.update(ik_seed=[float('nan')]*7)]
  for change in changes:
   with self.subTest(change=change),tempfile.TemporaryDirectory()as d:
    p,s=self.build(d)
    with patch('capx_autoring.planner.run_native',return_value=s),patch('capx_autoring.planner.subprocess.run',side_effect=self.reply(change)),self.assertRaises((ValueError,KeyError)):p._propose('carry','ring1','right',[np.eye(4).tolist()],{}, {})
 def test_gpu_cannot_override_original_native_rejection(self):
  with tempfile.TemporaryDirectory()as d:
   p,s=self.build(d)
   with patch.object(DynamicPlanner,'plan',side_effect=RuntimeError('native denied')),patch('capx_autoring.planner.run_native',return_value=s),patch('capx_autoring.planner.subprocess.run',side_effect=self.reply()),self.assertRaises(RuntimeError):p.plan('carry','ring1','right',[np.eye(4).tolist()],{})

if __name__=='__main__':unittest.main()
