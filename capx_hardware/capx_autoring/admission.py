"""Current full-world executable gate around ORIGINAL concrete field drivers.

This client is not a collision checker. Missing/unqualified checker fails before
constructing live devices. Every later arm/Hand/clamp/payload action is checked
again; an old planner certificate or registration is never an execution permit.
"""
from pathlib import Path
import copy,hashlib,json,subprocess,time,uuid
import numpy as np
from hardware_v1.config import canonical,resolve,sha
from hardware_v1.backend import HardwareBackend,MotionDisabled
from .config import preflight,engine_identity

COVERAGE=('full_world','both_arms','hands','payload','installed_and_free_parts','measured_frames','actual_command_bridge','original_FR3_limits','continuous_interpolant','continuous_aperture','supported_release')

class CurrentAdmission:
 def __init__(self,config_path):
  report,self.config,self.path,self.files,self.recipe=preflight(config_path)
  if not report['local_config_complete']:raise MotionDisabled('Automatic ring execution blocked: '+canonical(report['blockers']))
  self.backend=HardwareBackend(resolve(self.path.parent,self.config['hardware']['path']))
  self.backend._motion_gate() # no connection; must be explicitly enabled and complete
  self.identity=engine_identity();self.used=set();self.runtime=None
 def unchanged(self):
  if engine_identity()!=self.identity or any(sha(p)!=h for p,h in self.files.items()):raise MotionDisabled('Source/site/recipe/calibration dependency changed')
  self.backend._motion_gate()
 def _states(self,site):
  from .field_demo.random_planner import run_native,native_output
  native=resolve(Path(self.recipe['_root']),json.loads(resolve(Path(self.recipe['_root']),self.recipe['planner_config']).read_text())['planner_binary'])
  states={}
  for side in ['left','right']:
   raw=run_native(native,'--read-state',site.config['robots'][side]['ip']);native_output(raw);states[side]=raw
  hands={s:site.grippers[s].status()for s in states}
  return states,hands
 def admit(self,site,action):
  self.unchanged();site.require_ready();began=time.monotonic();states,hands=self._states(site)
  rt=self.runtime
  context={'runtime_plan':copy.deepcopy(getattr(rt,'_latest_plan_context',None)),'carried_parts':getattr(rt,'carried_state',lambda:[])(),'installed_parts':copy.deepcopy(getattr(rt,'installed_parts',[])),'io':copy.deepcopy(getattr(site,'autoring_io_context',None)),'fixed_world_part_goals':self.recipe['insertion_targets']}
  # Plans hold JSON-only original proof, fresh raw/calibration identity and current G.
  request={'schema':'capx-hardware-autoring-admission/v1','domain':'hardware','request_id':uuid.uuid4().hex,'engine_source_sha256':self.identity,'hardware_config_sha256':self.backend.config_digest,'site_config_sha256':self.backend.config['site_config_sha256'],'action':action,'current_states':states,'current_hands':hands,'task_context':context,'geometry':self.backend.config['geometry_record'],'tools':self.backend.config['tools']}
  digest=hashlib.sha256(canonical(request).encode()).hexdigest();self.used.add(request['request_id'])
  checker=self.backend.config['admission']['executable'];executable=resolve(self.backend.base,checker['path'])
  if sha(executable)!=checker['sha256']:raise MotionDisabled('Hardware checker changed')
  proof=json.loads(subprocess.run([str(executable)],input=canonical(request),capture_output=True,text=True,check=True,timeout=30).stdout,parse_constant=lambda v:(_ for _ in ()).throw(ValueError('Nonfinite proof')))
  if proof.get('schema')!='capx-hardware-autoring-approval/v1'or proof.get('domain')!='hardware'or proof.get('request_sha256')!=digest or proof.get('approved')is not True or any(proof.get(k)is not True for k in COVERAGE):raise MotionDisabled('Missing exact full current hardware-world proof')
  self.unchanged()
  if time.monotonic()-began>2:raise MotionDisabled('Current hardware approval expired')
  fresh,fh=self._states(site)
  for side in states:
   for key in ['q','q_d']:
    if np.max(np.abs(np.asarray(states[side]['state'][key])-fresh[side]['state'][key]))>1e-6:raise MotionDisabled('Actual/command state changed during approval')
   if abs(hands[side]['width_m']-fh[side]['width_m'])>1e-6:raise MotionDisabled('Hand opening changed during approval')
  if time.monotonic()-began>2:raise MotionDisabled('Approval expired during final state check')
  site.event('capx_current_hardware_admission',request=request,request_sha256=digest,proof=proof)
  return proof

class AdmittedHand:
 def __init__(self,hand,site,side,gate):self._hand,self._site,self._side,self._gate=hand,site,side,gate
 def __getattr__(self,k):return getattr(self._hand,k)
 def move(self,**action):
  # Original Hand validates bounds/motor profile; gate covers whole swept aperture.
  with self._site.control_lock():
   self._gate.admit(self._site,{'kind':'hand','side':self._side,'action':{k:v for k,v in action.items()if k!='cancelled'}})
   return self._hand.move(**action)

class AdmittedPLC:
 def __init__(self,plc,site,gate):self._plc,self._site,self._gate=plc,site,gate
 def __getattr__(self,k):return getattr(self._plc,k)
 def write(self,name,value):
  if name=='cell_stop':return self._plc.write(name,value) # independent STOP bypasses held motion lock
  with self._site.control_lock():
   self._gate.admit(self._site,{'kind':'clamp','signal':name,'value':value})
   return self._plc.write(name,value)

def make_site(gate,output):
 from .isaac_port.fieldkit.site import Site
 class AdmittedSite(Site):
  def motion(self,side,mode,target):
   if mode!='csv'or len(target)!=1:raise MotionDisabled('Automatic policy emits only original native CSV')
   csv=Path(target[0]).resolve();plan=self.autoring_gate.runtime._latest_plan_context
   if Path(plan['csv']).resolve()!=csv or plan['csv_sha256']!=sha(csv)or plan['robot']!=side or plan['part']not in ('ring2','ring1')or plan['stage_row'].get('planning_source')!='dynamic_field_envelope'or plan['stage_row'].get('field_path_reviewed')is not False:raise MotionDisabled('Exact generated current-context native field required')
   with self.control_lock():
    self.autoring_gate.admit(self,{'kind':'arm','side':side,'csv_path':str(csv),'csv_sha256':sha(csv),'interpolation':'original_fieldkit_quintic_1kHz'})
    # All ORIGINAL dynamic revalidation/native guards still run under this lock.
    result=super().motion(side,mode,target)
    self.last_arm_receipt={'part':plan['part'],'phase':plan['phase'],'side':side,'csv_sha256':sha(csv),'result':result}
    return result
 site=AdmittedSite(resolve(Path(gate.recipe['_root']),gate.recipe['site_config']),output)
 site.autoring_gate=gate;site.autoring_io_context=None
 site.grippers={s:AdmittedHand(h,site,s,gate)for s,h in site.grippers.items()};site.plc=AdmittedPLC(site.plc,site,gate)
 return site
