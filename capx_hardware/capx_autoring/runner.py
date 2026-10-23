"""Concrete SMALL->LARGE seed-free policy, bounded search and fresh BOTH final.

The policy is the existing field runtime, not a list of arbitrary callbacks.
Offline replay and synthetic integration fixtures can exercise it but can never
produce a hardware acceptance. Device construction is after complete preflight.
"""
from datetime import datetime,timezone
from pathlib import Path
import copy,hashlib,json,math,time,uuid
import numpy as np
from .field_demo import random_cycle as cycle
from .field_demo.calibration import rigid,utc_stamp
from hardware_v1.config import canonical,sha

PHASES=('perception','grasp','carry','preinsert','fixed_insert','release','withdraw','final_check')
ORDER=('ring2','ring1')

def strict_value(value):
 if isinstance(value,float)and not math.isfinite(value):return {'nonfinite':repr(value)}
 if isinstance(value,np.ndarray):return strict_value(value.tolist())
 if isinstance(value,dict):return {str(k):strict_value(v)for k,v in value.items()}
 if isinstance(value,(tuple,list)):return [strict_value(v)for v in value]
 if isinstance(value,np.generic):return strict_value(value.item())
 return value

class RingSession:
 def __init__(self,*,domain,source_sha256,config_sha256):
  if domain not in ('hardware','offline_contract','recorded_pixel_replay'):raise ValueError('Explicit supported run domain required')
  self.run_id=uuid.uuid4().hex;self.domain=domain;self.source=source_sha256;self.config=config_sha256;self.events=[];self.expected=[(p,f)for p in ORDER for f in PHASES];self.cancelled=False
 def advance(self,part,phase,evidence):
  if self.cancelled or len(self.events)>=16 or (part,phase)!=self.expected[len(self.events)]:raise RuntimeError('Ring phase order or cancelled run')
  row={'run_token':self.run_id,'part':part,'phase':phase,'action_token':uuid.uuid4().hex,'permission':'next','evidence':copy.deepcopy(evidence)}
  self.events.append(row);return 'next'

def phase_evidence(runtime,part,phase,observation):
 entry=next(e for e in runtime.recipe['parts']if e['part']==part)
 facts=getattr(runtime,'facts',None)
 if facts is not None and phase in ('release','final_check'):
  retained=facts.retained.get(part,{})
  rows=retained.get('observations',[])
  if not rows:raise RuntimeError('Postrelease persistence evidence missing')
  latest=rows[-1];source=Path(latest['image_record']);observation=json.loads(source.read_text())
  if observation.get('capture_id')!=latest['capture_id']:raise RuntimeError('Retention image identity changed')
  observation['_path']=str(source);observation['_sha256']=sha(source)
 if phase!='withdraw':cycle.check_observation(runtime,observation)
 evidence={'capture_id':observation['capture_id'],'captured_at_utc':observation['captured_at_utc'],'calibration_sha256':observation['calibration_sha256'],'raw_path':observation.get('source_packet'),'raw_sha256':observation.get('source_packet_sha256')}
 facts=getattr(runtime,'facts',None)
 if facts is not None:
  if phase in ('grasp','carry','preinsert'):
   side=entry['pickup_side']if phase=='grasp'else entry['insert_side'];held=facts.held.get(side)
   if not held or held['part']!=part or not facts.grasped(part,side,runtime.hand_status(side)):raise RuntimeError('Current measured Hand/image hold missing')
   evidence['held']=copy.deepcopy(held)
  if phase=='preinsert':
   actual,_=facts.observation(observation,part);target=cycle.offset(runtime.recipe['insertion_targets'][part],entry['insert_entry_offset_part_m'],local=True)
   pos=float(np.linalg.norm(actual[:3,3]-target[:3,3]));axis=float(np.arccos(np.clip(actual[:3,2]@target[:3,2],-1,1)))
   if pos>runtime.recipe['target_position_tolerance_m']or axis>runtime.recipe['target_angle_tolerance_rad']:raise RuntimeError('Fresh measured preinsert center/axis gate failed')
   evidence.update(position_error_m=pos,axis_error_rad=axis)
  if phase=='fixed_insert':
   if part not in facts.seats:raise RuntimeError('Fresh actual seating evidence missing')
   evidence['seat']=copy.deepcopy(facts.seats[part])
  if phase=='withdraw':
   receipt=getattr(runtime.site,'last_arm_receipt',None)
   if not receipt or receipt.get('part')!=part or receipt.get('phase')!='withdraw'or receipt.get('result',{}).get('stage_completed')is not True:raise RuntimeError('Completed current withdraw field acknowledgement missing')
   evidence['native_withdraw']=copy.deepcopy(receipt)
  if phase in ('release','withdraw','final_check'):
   if part not in facts.retained or runtime.hand_status(entry['insert_side'])['is_grasped']is not False:raise RuntimeError('Actual released/retained evidence missing')
   evidence['retention']=copy.deepcopy(facts.retained[part])
 return evidence

class HardwareRingRuntime(cycle.LiveRuntime):
 def __init__(self,site,recipe,output,gate,session):
  self.session=session;self.gate=gate
  from .planner import CurrentPlanner
  planner=CurrentPlanner(site,Path(recipe['_root'])/recipe['planner_config'],Path(output)/'plans')
  super().__init__(site,recipe,output,planner=planner)
  gate.runtime=self
  self.payload.apply=self._payload_apply
 def _camera(self,scope,side):
  from .field_demo.random_perception import load_random_config
  from .camera import MeasuredCamera
  role='held_'+side if scope=='held'else scope
  path=(self.root/(self.recipe.get('camera_configs',{}).get(role)or self.recipe['camera_config'])).resolve()
  key=str(path)
  if key not in self.camera_configs:self.camera_configs[key]=load_random_config(path)
  cfg=self.camera_configs[key];hardware=self.gate.backend.config['camera'];serial=cfg['camera_serial']
  if serial not in self.streams:self.streams[serial]=MeasuredCamera(cfg,hardware).start()
  stream=self.streams[serial]
  if stream.config['camera_frame']!=cfg['camera_frame']or stream.config['T_cell_camera']!=cfg['T_cell_camera']:raise RuntimeError('One physical camera cannot change extrinsics between roles')
  stream.config=cfg
  return stream,cfg,path
 def _payload_apply(self,site,binary,side,tool,before,after,F_T_EE):
  from .field_demo.payload_boundary import execute_boundary
  self.gate.admit(site,{'kind':'stopped_payload','side':side,'driver_path':str(binary),'driver_sha256':sha(binary),'tool':tool,'before':before,'after':after,'F_T_EE':np.asarray(F_T_EE).tolist()})
  return execute_boundary(site,binary,side,tool,before,after,F_T_EE)
 def phase(self,part,phase,observation):
  self.ready();self.gate.unchanged();evidence=phase_evidence(self,part,phase,observation)
  if self.session.advance(part,phase,evidence)!='next':raise RuntimeError('Exact NEXT permission required')
 def fresh_both(self):
  requested=datetime.now(timezone.utc);results=[]
  for part in ORDER:
   image=cycle.check_observation(self,self.observe_fixture(part));values=cycle.instances(image,part)
   if len(values)!=1:raise RuntimeError('Final BOTH ring not uniquely visible')
   image=cycle.select_observation(image,values[0]);record=self.facts.target(part,image)
   if utc_stamp(record['captured_at_utc'])<=requested:raise RuntimeError('Final capture predates new BOTH request')
   raw=Path(image['source_packet'])
   if sha(raw)!=image['source_packet_sha256']:raise RuntimeError('Final raw changed')
   record.update(raw_path=str(raw),raw_sha256=sha(raw));results.append(record)
  if len({x['capture_id']for x in results})!=2:raise RuntimeError('Duplicate final captures')
  if any(self.hand_status(s)['is_grasped']is not False for s in ['left','right']):raise RuntimeError('Actual Hand still grasped at final')
  self.verify_clearance(self.recipe)
  return {'request_utc':requested.isoformat(),'rings':results,'original_clearance_and_completion_gates_passed':True}

class PacketReplayRuntime(cycle.ReplayRuntime):
 """Original actual-pixel detector + fake actuators; never hardware permission."""
 def __init__(self,*args,session,**kwargs):self.session=session;super().__init__(*args,**kwargs)
 def phase(self,part,phase,observation):
  if self.session.advance(part,phase,phase_evidence(self,part,phase,observation))!='next':raise RuntimeError('NEXT required')
 def fresh_both(self):
  # Replay contract only; absence of trailing actual packets is an honest failure.
  images=[]
  for part in ORDER:
   image=cycle.check_observation(self,self.observe('fixture',part));images.append({'part':part,'capture_id':image['capture_id'],'scope':'replay_only'})
  return {'rings':images,'hardware_qualified':False}

def run_runtime(runtime,recipe,session,output):
 """Shared production policy exercised by concrete live and explicit offline devices."""
 if session.domain=='hardware'and (not isinstance(runtime,HardwareRingRuntime)or runtime.replay is not False):raise ValueError('Offline/replay devices cannot produce hardware acceptance')
 if session.domain=='hardware'and (runtime.session is not session or session.source!=runtime.gate.identity or session.config!=sha(runtime.gate.path)):raise ValueError('Actual current run source/config identity mismatch')
 output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
 path=output/'native-ring-pair-session-final.json'
 if path.exists():raise FileExistsError(path)
 result=None;final=None;failure=None;completed=False
 try:
  result=cycle.run(runtime,recipe)
  if result.get('completed_parts')!=list(ORDER)or len(session.events)!=16:raise RuntimeError('Both actual eight-phase components required')
  final=runtime.fresh_both();completed=True
 except BaseException as e:
  session.cancelled=True;failure={'type':type(e).__name__,'message':str(e)}
  # Cover a new final-observation failure too; retain grips, never auto-open.
  try:
   cancelled=getattr(runtime.site,'cancelled',None)
   if not getattr(runtime,'stopped',False)and not(cancelled is not None and cancelled.is_set()):runtime.stop()
  except BaseException as stop_error:failure['stop_error']={'type':type(stop_error).__name__,'message':str(stop_error)}
  raise
 finally:
  record={'schema':'capx-native-ring-pair-session/v1','run_token':session.run_id,'domain':session.domain,'source_sha256':session.source,'config_sha256':session.config,'accepted_ring_pair':completed and session.domain=='hardware','accepted_full3':False,'completed_offline_contract':completed and session.domain!='hardware','cancelled':session.cancelled,'worker_alive_after_abort':bool(getattr(runtime.site,'process',None)is not None and runtime.site.process.poll()is None),'events':session.events,'components':result,'component_progress':{part:[e['phase']for e in session.events if e['part']==part]for part in ORDER},'fresh_both_final':final,'failure':failure,'hardware_execution_qualified':False}
  path.write_text(json.dumps(strict_value(record),indent=2,allow_nan=False)+'\n')
 return record

def execute(config_path,output,*,execute=False,replay_manifest=None):
 from .config import preflight
 report,c,p,files,recipe=preflight(config_path)
 if not execute:return report
 # Must fail before device construction if anything is missing/unqualified.
 from .admission import CurrentAdmission,make_site
 gate=CurrentAdmission(config_path)
 output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
 session=RingSession(domain='hardware',source_sha256=gate.identity,config_sha256=sha(p))
 site=make_site(gate,output/'devices');runtime=None
 try:
  runtime=HardwareRingRuntime(site,gate.recipe,output,gate,session)
  from .field_demo.execution import StopListener
  with site.control_lock(),StopListener(site):return run_runtime(runtime,gate.recipe,session,output)
 finally:
  if runtime is not None:runtime.close()
