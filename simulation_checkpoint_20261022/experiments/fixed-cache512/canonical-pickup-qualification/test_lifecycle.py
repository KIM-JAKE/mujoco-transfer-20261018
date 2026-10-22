import ast,json,types,sys,textwrap
from pathlib import Path
import numpy as np
O=Path(__file__).parent;src=(O/'overrides/direct_pin_skill.py').read_text();base=Path('/tmp/fixed506-rebranch-pin-component-full/overrides/direct_pin_skill.py').read_text()
a=src.index('    robot_hint_data=');b=src.index('    H=arm.fk(art.get_dof_positions()[0,ids]);line(H,pickupH',a);block=textwrap.dedent(src[a:b]);tests=[]
# Existing catalogue/taught seeds are literal public data; mock only numerical
# solve and original guards to assert scheduling and failure boundaries.
H=Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/native_hand_layout/roas-table');data=json.loads((H/'bin-picking/direct-pin-robot-seeds.json').read_text());teach=np.array(data['seeds']['pin_tail_approach']);cat=np.arange(7.)+.1;pre=np.arange(7.)+.2;GPU=np.arange(7.)+20
class Arm:
 bounds=np.tile([-3,5],(7,1))
 def solve(self,P,R,seed):
  seed=np.asarray(seed)
  if np.array_equal(seed,teach):return np.arange(7.)+1
  if np.array_equal(seed,cat):return np.arange(7.)+2
  if np.array_equal(seed,pre):return np.arange(7.)+3
  return None
class Checker:
 def __init__(self,blocked=(),noroute=(),fault=None):self.blocked=blocked;self.noroute=noroute;self.fault=fault;self.calls=[]
 def clear(self,q,*args):self.calls.append(('clear',float(q[0])));return q[0]not in self.blocked,{'test':float(q[0])}
 def route(self,start,q,*args,**kw):
  self.calls.append(('route',float(q[0])));assert kw=={'seconds':.75}
  if self.fault:raise RuntimeError(self.fault)
  return None if q[0]in self.noroute else [start.copy(),q.copy()]
def run(enabled=True,blocked=(),noroute=(),fault=None,motionfault=False):
 ck=Checker(blocked,noroute,fault);cmd=np.zeros((1,9),dtype=np.float32);cmd[0,7:]=.012;motions=[];mod=types.ModuleType('fr3_gpu_ik');mod.propose_ik=lambda *args:[GPU.copy()];sys.modules['fr3_gpu_ik']=mod
 def motion(*args):
  motions.append(args)
  if motionfault:raise RuntimeError('partialissuedfault')
 env=dict(json=json,np=np,HERE=H,robot_seeds=data['seeds'],cmd=cmd,ids=np.arange(7),fi=[7,8],preq=pre,cand={'pickup_q':cat},arm=Arm(),newpre=np.eye(4),ctx={},pin_config={'pin_prefer_canonical_pickup_branch':enabled},art=types.SimpleNamespace(get_dof_positions=lambda:cmd.copy()),pickup_opening_m=.024,checker=ck,paths={'workpiece_pin':'/pin'},report={},motion=motion,print=lambda *a,**k:None)
 error=None
 try:exec(block,env)
 except RuntimeError as e:error=str(e)
 return env,ck,motions,error
for label,kwargs,selected in [('disabled',{'enabled':False},20),('canonicalfirst',{},1),('firstblocked',{'blocked':(1,)},2),('firstnoroute',{'noroute':(1,)},2),('allcanonicalblocked',{'blocked':(1,2,3)},20)]:
 e,c,m,err=run(**kwargs);assert err is None and len(m)==1 and m[0][0][-1][0]==selected,(label,err,c.calls)
 if kwargs.get('enabled',True):assert e['report']['pin_canonical_pickup_branch']['fallback_to_original_groups']==(selected==20)
 tests.append(label)
for label,kwargs in [('routefault',{'fault':'originalguardfault'}),('partialmotion',{'motionfault':True})]:
 e,c,m,err=run(**kwargs);assert err and (len(m)==(1 if label=='partialmotion'else 0));assert sum(k=='route'for k,v in c.calls)==1;tests.append(label)
e,c,m,err=run(blocked=(1,2,3,20));assert err and not m;tests.append('allblocked_noactuation')
# Guard executes before any context robot access.
first=ast.parse(src).body;execute=next(x for x in first if isinstance(x,ast.FunctionDef)and x.name=='execute');guard=compile(ast.Module(body=[execute.body[0]],type_ignores=[]),'<guard>','exec')
try:exec(guard,{'ctx':{'config':{'pin_prefer_canonical_pickup_branch':True}}});raise AssertionError('nonfixed accepted')
except ValueError:tests.append('nonfixed_reject_before_robotaccess')
# No implementation changes to original control, proof, pixel and gain functions.
oldtree=ast.parse(base);newtree=ast.parse(src)
def funcs(t):return {x.name:ast.dump(x,include_attributes=False) for x in ast.walk(t)if isinstance(x,ast.FunctionDef)and x.name!='execute'}
assert funcs(oldtree)==funcs(newtree);tests.append('all_original_nested_and_top_level_functions_AST_identical')
(O/'lifecycle-tests.json').write_text(json.dumps({'passed':True,'tests':tests,'cases':len(tests),'scope':'Scheduling/fallback/fault lifecycle with mocks; public numericIK checked separately, no fullscene physics proof.'},indent=2)+'\n');print(tests)
