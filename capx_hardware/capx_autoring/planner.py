"""Original measured-SDK native planning with optional cuRobo proposal fallback.

A site-installed, hash-pinned cuRobo client executable may propose Cartesian
waypoints. Its answer never supplies native approval, CSV timing, or a new goal.
Every segment is re-solved/retimed by the original measured-SDK planner and then
passes the original whole-path checks plus the execution-time hardware gate.
"""
from pathlib import Path
import hashlib,json,subprocess,uuid
import numpy as np
from hardware_v1.config import canonical,resolve,sha
from .field_demo.calibration import rigid
from .field_demo.random_planner import DynamicPlanner,run_native,native_output

class CurrentPlanner(DynamicPlanner):
 def __init__(self,*args,**kwargs):
  super().__init__(*args,**kwargs);self.settings=json.loads(self.config.read_text());self.proposal=self.settings.get('cuRobo_proposal');self.diagnostics=[]
 def plan(self,phase,part,side,T_cell_tcp_targets,observation,**kwargs):
  try:return super().plan(phase,part,side,T_cell_tcp_targets,observation,**kwargs)
  except (RuntimeError,ValueError)as original:
   if self.proposal is None:raise
   diagnostic={'phase':phase,'part':part,'native_refusal':str(original),'gpu_proposal_qualified':False};self.diagnostics.append(diagnostic)
   try:targets,seed=self._propose(phase,part,side,T_cell_tcp_targets,observation,kwargs)
   except Exception as e:
    diagnostic['proposal_refusal']=str(e);raise original from e
   options=dict(kwargs)
   if seed is not None:options['ik_seed']=seed
   try:
    result=super().plan(phase,part,side,targets,observation,**options);diagnostic['new_native_path_approved']=True
    result['proposal_source']='optional_site_cuRobo_then_original_native';return result
   except Exception as e:diagnostic['new_native_refusal']=str(e);raise
   finally:self.site.event('capx_current_proposal_diagnostic',record=diagnostic)
 def _propose(self,phase,part,side,targets,observation,context):
  spec=self.proposal;exe=resolve(self.config.parent,spec['executable']['path']);model=resolve(self.config.parent,spec['model_record']['path'])
  if sha(exe)!=spec['executable']['sha256']or sha(model)!=spec['model_record']['sha256']:raise ValueError('cuRobo client/model identity changed')
  timeout=spec['timeout_s']
  if type(timeout)not in (int,float)or not np.isfinite(timeout)or not 0<timeout<=2:raise ValueError('Bounded cuRobo proposal timeout <=2s required')
  binary=resolve(self.config.parent,self.settings['planner_binary']);states={}
  for arm in ['left','right']:
   value=run_native(binary,'--read-state',self.site.config['robots'][arm]['ip']);native_output(value);states[arm]=value
  body=states[side]['state'];start=rigid(self.site.config['robots'][side]['T_cell_base'])@rigid(np.asarray(body['O_T_EE_column_major']).reshape(4,4,order='F'))
  request={'schema':'capx-current-curobo-proposal/v1','domain':'hardware','request_id':uuid.uuid4().hex,'side':side,'part':part,'phase':phase,'model_record_sha256':sha(model),'states':states,'hands':{s:self.site.grippers[s].status()for s in states},'T_cell_tcp_start':start.tolist(),'mandatory_T_cell_tcp_targets':targets,'observation':observation,'context':context}
  digest=hashlib.sha256(canonical(request).encode()).hexdigest()
  answer=json.loads(subprocess.run([str(exe)],input=canonical(request),text=True,capture_output=True,check=True,timeout=timeout).stdout,parse_constant=lambda v:(_ for _ in ()).throw(ValueError('Nonfinite proposal')))
  if answer.get('schema')!='capx-current-curobo-proposal-reply/v1'or answer.get('request_sha256')!=digest or answer.get('model_record_sha256')!=sha(model):raise ValueError('Proposal identity mismatch')
  segments=answer['segments']
  if not isinstance(segments,list)or len(segments)!=len(targets):raise ValueError('Every original mandatory target must remain')
  current=start;result=[]
  for segment,goal in zip(segments,targets):
   if not isinstance(segment,list)or not 2<=len(segment)<=33:raise ValueError('Bounded Cartesian segment required')
   poses=[rigid(t)for t in segment]
   if np.max(abs(poses[0]-current))>1e-8 or np.max(abs(poses[-1]-rigid(goal)))>1e-8:raise ValueError('Current start/original goal changed')
   result.extend(p.tolist()for p in poses[1:]);current=rigid(goal)
  if not 1<=len(result)<=32:raise ValueError('Original native 32-target bound')
  seed=answer.get('ik_seed')
  if seed is not None:
   a=np.asarray(seed,float)
   if a.shape!=(7,)or not np.isfinite(a).all():raise ValueError('Finite native seven-joint IK seed required')
   seed=a.tolist()
  if sha(exe)!=spec['executable']['sha256']or sha(model)!=spec['model_record']['sha256']:raise ValueError('Proposal dependency changed')
  return result,seed
