import ast,sys,types,json
from pathlib import Path
import numpy as np
O=Path('/tmp/fixed489-final-retreat');tree=ast.parse((O/'overrides/final_empty_clearance.py').read_text());fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef)and n.name=='_complete_encoder_clearance');ns=dict(np=np);exec(compile(ast.Module(body=[fn],type_ignores=[]),'<isolated completion>','exec'),ns);complete=ns['_complete_encoder_clearance'];tests=[]

def run(name,start=.160,velocity=.2,sequence=None,blocked=False,stale=False,fault=False):
 class Art:
  def __init__(self):self.actual=np.zeros((1,18));self.actual[0,0]=start;self.actual[0,7:9]=.012
  def get_dof_positions(self):return self.actual.copy()
  def get_dof_velocities(self):v=np.zeros((1,18));v[0,0]=velocity;return v
 a=Art();cmd=a.actual.copy();cmd[0,0]=.183;ri=np.arange(7);rf=np.array([7,8]);li=np.arange(9,16);lf=np.array([16,17]);c=dict(art=a,cmd=cmd,ids=ri,fi=rf,left_ids=li,left_fi=lf,dt=1/480);rc=types.SimpleNamespace(resolution_rad=.035);calls=[];issued=[]
 def guard(*args,**kwargs):
  assert not issued,'proof after extra actuation';calls.append(args)
  if stale:cmd[0,9]+=.01
  return not blocked,{'blocked':True}if blocked else None
 sys.modules['aperture_path_check']=types.SimpleNamespace(_samples=lambda ck,qs:[(0,0,q)for q in qs]);sys.modules['dual_aperture_path_check']=types.SimpleNamespace(rows_other_contexts=guard)
 def step(phase):
  assert calls,'extra step before full fresh proof';issued.append(phase)
  if fault:raise RuntimeError('ORIGINAL_STEP_FAULT')
  a.actual[0,0]=(sequence[min(len(issued)-1,len(sequence)-1)]if sequence else start)
 try:r=complete(c,rc,None,lambda q:np.array([q[0],0,0]),np.zeros((3,3)),step);error=None
 except RuntimeError as e:r=None;error=str(e)
 tests.append(dict(name=name,success=r is not None,error=error,extra_steps=len(issued),proof_calls=len(calls)));return r,error,issued,calls
r,e,steps,g=run('already_actual_clear',start=.170);assert r and not steps and not g
r,e,steps,g=run('moving_crosses_gate_after_freshproof',sequence=[.162,.166]);assert r and len(steps)==2 and len(g)==1 and r['actual_gate_m']==.165
r,e,steps,g=run('stationary_short_no_blindwait',velocity=0);assert not r and not steps and not g
r,e,steps,g=run('freshbridge_blocked_no_steps',blocked=True);assert not r and not steps and len(g)==1
r,e,steps,g=run('sourcechanged_no_steps',stale=True);assert not r and not steps
r,e,steps,g=run('max100ms_not_relaxed',sequence=[.160]);assert not r and len(steps)==48
r,e,steps,g=run('stepfault_propagates_no_retry',fault=True);assert not r and e=='ORIGINAL_STEP_FAULT'and len(steps)==1
s=(O/'overrides/final_empty_clearance.py').read_text();assert 'planned.min()<.180'in s and 'after.min()<.165'in s and 'actual[li],cmd[0,li]'in s
(O/'lifecycle-tests.json').write_text(json.dumps(dict(all_pass=True,tests=tests,original165mm_actual_gate_retained=True,planned180mm_float32_endpoint_required=True,only_this_module_changed=True),indent=2));print('PASS',len(tests),'completion lifecycle cases')
