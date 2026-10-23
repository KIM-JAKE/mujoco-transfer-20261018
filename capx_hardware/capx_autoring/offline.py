"""Explicit synthetic integration devices for the REAL production ring policy.

No simulation/SDK/socket/process imports. These poses are test inputs, not site
settings or sensor evidence. The resulting session can never accept hardware.
"""
from datetime import datetime,timezone,timedelta
from pathlib import Path
from types import SimpleNamespace
import copy,hashlib,json,uuid
import numpy as np
from .runner import RingSession,run_runtime,phase_evidence,ORDER
from .field_demo.sensor_facts import SensorFacts
from .field_demo import random_cycle as cycle

def pose(x=0,y=0,z=0):
 T=np.eye(4);T[:3,3]=[x,y,z];return T.tolist()

def fixture_recipe():
 action={'open':{'width_m':.078,'speed_m_s':.01},'grasp':{'width_m':.02,'speed_m_s':.01,'force_n':10.,'epsilon_inner_m':.001,'epsilon_outer_m':.001},'release':{'width_m':.078,'speed_m_s':.01}}
 entries=[]
 for part,side in [('ring2','left'),('ring1','right')]:
  entries.append(dict(part=part,pickup_side='right',insert_side=side,grasp_families=[{'name':'OFFLINE_TEST_ONLY','T_part_tcp':pose()}],payload_dimensions={'radius_m':.03,'length_m':.02},pregrasp_offset_part_m=[0,0,.05],lift_offsets_cell_m=[[0,0,.05]],insert_entry_offset_part_m=[0,0,-.045],withdraw_offset_cell_m=[0,0,-.08],min_tcp_clearance_m=.16,handover={'donor_grasped_signal':'donor_ring2_grasped','receiver_grasped_signal':'receiver_ring2_grasped','receiver_open':action['open'],'receiver_grasp':action['grasp'],'receiver_release':action['release'],'donor_release':action['release'],'donor_T_cell_tcp':pose(0,0,1.),'donor_waypoints_cell_tcp':[],'receiver_T_part_tcp':pose(),'receiver_approach_offset_part_m':[0,0,-.05],'donor_retreat_offset_cell_m':[.2,0,0]}))
 return dict(parts=entries,insertion_targets={'ring2':pose(-.2,.2,1.),'ring1':pose(.2,.2,1.)},feedback_timeout_s=1.,feedback_dwell_s=.1,max_task_s=30.,max_observation_age_s=2.,target_position_tolerance_m=.001,target_angle_tolerance_rad=.01,release_width_tolerance_m=.001,insert_pose_confirmation_tolerance_m=.001,retention_observation_dwell_s=.3,_actions=action)

class IntegrationRuntime:
 replay=True
 def __init__(self,output,session,*,fail_phase=None):
  self.recipe=fixture_recipe();self.output=Path(output);self.output.mkdir(parents=True,exist_ok=True);self.session=session;self.fail_phase=fail_phase;self.commands=[];self.events=[];self.stopped=False;self.monitor=lambda:None;self.counter=0;self.now=datetime(2026,1,1,tzinfo=timezone.utc)
  self.tcps={'left':pose(-.4,0,.8),'right':pose(.4,0,.8)};self.hands={s:dict(is_grasped=False,width_m=.078)for s in self.tcps};self.parts={p:pose((-.1 if p=='ring2'else.1),-.3,.8)for p in ORDER};self.held={};self.installed_parts=[]
  self.site=SimpleNamespace(expected_held={},config={'gripper_actions':{p:copy.deepcopy(self.recipe['_actions'])for p in ORDER}},process=None)
  self.facts=SensorFacts(self.recipe,clock=lambda:self.now)
  runtime=self
  class Payload:
   valid=True
   def __init__(self):self.owner={s:None for s in ['left','right']}
   def after_grasp(self,s,p,*args):self.owner[s]=p
   def refresh_held(self,*args):pass
   def after_transfer_release(self,d,r,p,*args):self.owner[d]=None;self.owner[r]=p
   def after_retained_release(self,s,*args):self.owner[s]=None
  self.payload=Payload()
 def ready(self):
  if self.stopped:raise RuntimeError('Offline devices stopped')
  self.monitor()
 read=cycle.LiveRuntime.read
 def wait(self,name,timeout,dwell):
  part=next(e['part']for e in self.recipe['parts']if name.startswith(e['part']))
  if name.endswith('_retained'):
   side=next(e['insert_side']for e in self.recipe['parts']if e['part']==part)
   images=[self.observe('fixture',part)];self.now+=timedelta(seconds=.31);images.append(self.observe('fixture',part));self.facts.confirm_retained(part,side,images,self.hand_status(side))
  if not self.read(name):raise RuntimeError('Offline actual policy fact absent '+name)
 def hand_status(self,side):return copy.deepcopy(self.hands[side])
 def hand_move(self,side,part,mode,parameters):
  self.ready();self.commands.append({'kind':'hand','side':side,'part':part,'mode':mode})
  if mode=='grasp':
   self.hands[side]={'is_grasped':True,'width_m':.03};self.held[side]=(part,np.linalg.inv(np.asarray(self.tcps[side]))@np.asarray(self.parts[part]))
  else:self.hands[side]={'is_grasped':False,'width_m':parameters['width_m']};self.held.pop(side,None)
  return self.hand_status(side)
 def observe(self,scope,part,*,side=None,previous=None):
  self.ready();self.now+=timedelta(milliseconds=1);self.counter+=1;capture='offline-'+str(self.counter)
  T=self.parts[part];instance={'part':part,'instance_id':capture+':0','T_cell_part':copy.deepcopy(T),'geometrically_accessible':True}
  report={'state':'random_ring_observation','replay':True,'capture_id':capture,'captured_at_utc':self.now.isoformat(),'calibration_sha256':'0'*64,'source_metadata':{'capture_id':capture,'captured_at_utc':self.now.isoformat(),'calibration_sha256':'0'*64,'source':'explicit_offline_fixture','replay':True},'instances':[instance],'selected_instance':instance,'T_cell_part':copy.deepcopy(T),'_scope':scope,'_part':part,'_side':side,'ring_pose_seed_used':False}
  p=self.output/(capture+'.json');p.write_text(json.dumps(report));report['_path']=str(p);report['_sha256']=hashlib.sha256(p.read_bytes()).hexdigest();return report
 def tcp(self,side):return copy.deepcopy(self.tcps[side])
 def flange_frame(self,side):return np.eye(4)
 def plan(self,phase,part,side,targets,observation,held_parts,**kwargs):
  if phase==self.fail_phase or (part,phase)==self.fail_phase:raise RuntimeError('Explicit offline preflight refusal '+phase)
  return {'phase':phase,'part':part,'side':side,'targets':copy.deepcopy(targets),'held_parts':copy.deepcopy(held_parts),'scope':'fake_planner_contract_only'}
 def execute(self,plan,observation):
  self.ready();side=plan['side'];self.tcps[side]=copy.deepcopy(plan['targets'][-1]);self.commands.append({'kind':'arm','phase':plan['phase'],'part':plan['part'],'side':side})
  self.site.last_arm_receipt={'part':plan['part'],'phase':plan['phase'],'side':side,'result':{'stage_completed':True,'scope':'OFFLINE_FAKE_DEVICE'}}
  for s,(part,G)in self.held.items():self.parts[part]=(np.asarray(self.tcps[s])@G).tolist()
 def clamp(self,part):self.facts.confirm_clamp(part,source='actual_PLC',evidence={'scope':'FAKE_OFFLINE_PLC_NOT_HARDWARE'});self.commands.append({'kind':'clamp','part':part})
 def mark_installed(self,part,pose,dimensions,observation,**kwargs):self.installed_parts.append({'part':part,'T_cell_part':copy.deepcopy(pose)})
 def verify_clearance(self,recipe):
  for row in self.installed_parts:self.wait(row['part']+'_retained',1.,.1)
 def phase(self,part,phase,observation):self.session.advance(part,phase,phase_evidence(self,part,phase,observation))
 def fresh_both(self):
  images=[self.observe('fixture',p)for p in ORDER]
  for part,image in zip(ORDER,images):self.facts.target(part,image)
  if len({i['capture_id']for i in images})!=2:raise RuntimeError('Offline final IDs duplicate')
  return {'rings':[{'part':p,'capture_id':i['capture_id']}for p,i in zip(ORDER,images)],'scope':'explicit_offline_fixture'}
 def event(self,name,**values):self.events.append({'name':name,**values})
 def stop(self):self.stopped=True
 def close(self):pass

def integration_check(output,*,fail_phase=None):
 output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
 session=RingSession(domain='offline_contract',source_sha256='0'*64,config_sha256='0'*64)
 runtime=IntegrationRuntime(output/'captures',session,fail_phase=fail_phase)
 try:return run_runtime(runtime,runtime.recipe,session,output)
 finally:(output/'offline-command-trace.json').write_text(json.dumps(runtime.commands,indent=2)+'\n')
