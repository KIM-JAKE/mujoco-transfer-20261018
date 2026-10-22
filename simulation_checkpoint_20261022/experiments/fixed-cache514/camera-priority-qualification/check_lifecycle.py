"""Controlled lifecycle test using the literal changed request AST nodes only."""
from pathlib import Path
import ast,json,hashlib,types
import numpy as np
BASE=Path('/tmp/fixed510-canonical-pin-full/overrides/loose_bin_task.py')
NEW=Path(__file__).parent/'overrides/loose_bin_task.py'
original=ast.parse(BASE.read_text());changed=ast.parse(NEW.read_text())
def requests(tree):
 return [n for n in ast.walk(tree)if isinstance(n,ast.If)and 900<n.lineno<1080 and len(n.body)==1 and isinstance(n.body[0],ast.Assign)and any(isinstance(t,ast.Name)and t.id=='next_initial'for t in n.body[0].targets)and isinstance(n.body[0].value,ast.Call)and isinstance(n.body[0].value.func,ast.Attribute)and n.body[0].value.func.attr=='request'and ast.unparse(n.test).startswith('part == ring_order[0]')]
old=requests(original);new=sorted(requests(changed),key=lambda n:n.lineno)
assert len(old)==1 and len(new)==2
assert all(ast.dump(n.body[0],include_attributes=False)==ast.dump(old[0].body[0],include_attributes=False)for n in new)
# No new observe/wait/native aperture call has been introduced or changed.
for target in ['wait','observe','LoadedContactAperture']:
 def calls(tree):return [ast.dump(n,include_attributes=False)for n in ast.walk(tree)if isinstance(n,ast.Call)and isinstance(n.func,ast.Name)and n.func.id==target]
 assert calls(original)==calls(changed),target
before,after=[compile(ast.fix_missing_locations(ast.Module(body=[n],type_ignores=[])),str(NEW),'exec')for n in new]
legacy=compile(ast.fix_missing_locations(ast.Module(body=[old[0]],type_ignores=[])),str(BASE),'exec')
def exercise(flag,first=True,one=False,fail=False,legacy_mode=False):
 log=[];sent=[]
 class Vision:
  def request(self,event,H):
   log.append('next_initial_requested');sent.append((event,H.copy()));return ('same-future',len(sent))
 art=types.SimpleNamespace(get_dof_positions=lambda:np.arange(18,dtype=float)[None,:])
 env={'part':'ring2'if first else'ring1','ring_order':['ring2','ring1'],'c':{'config':dict(defer_next_initial_until_receiver_bound=flag,one_ring_only=one)},'vision':Vision(),'initial_ring_event':lambda:dict(mode='initial',part='ring1'),'arms':{'right':types.SimpleNamespace(fk=lambda q:q.copy())},'art':art,'indices':{'right':list(range(7))},'next_initial':None}
 exec(legacy if legacy_mode else before,env)
 log.append('required_receiver_stable_hold_RGBD')
 if fail:
  log.append('original_binding_rejected');return log,sent,env['next_initial']
 log.append('original_LoadedContactAperture_completed')
 if not legacy_mode:exec(after,env)
 return log,sent,env['next_initial']
rows=[]
for first in [False,True]:
 for one in [False,True]:
  for flag in [False,True]:
   log,sent,future=exercise(flag,first,one)
   eligible=first and not one
   assert len(sent)==int(eligible)
   if eligible:
    assert sent[0][0]==dict(mode='initial',part='ring1') and np.array_equal(sent[0][1],np.arange(7,dtype=float))
    assert log.index('next_initial_requested')>(log.index('original_LoadedContactAperture_completed'))if flag else log.index('next_initial_requested')<log.index('required_receiver_stable_hold_RGBD')
   if not flag:
    oldlog,oldsent,oldfuture=exercise(False,first,one,legacy_mode=True)
    assert log==oldlog and future==oldfuture
   rows.append(dict(flag=flag,first_part=first,one_ring_only=one,requests=len(sent),lifecycle=log))
log,sent,future=exercise(True,fail=True);assert not sent and future is None
value={'pass':True,'base_sha256':hashlib.sha256(BASE.read_bytes()).hexdigest(),'new_sha256':hashlib.sha256(NEW.read_bytes()).hexdigest(),'request_assignment_AST_identical':True,'flag_off_matches_original_order_and_future':True,'same_single_request_args_and_count':True,'failed_original_binding_leaves_no_deferred_future':True,'original_wait_observe_and_loaded_binding_calls_identical':True,'no_actual_physics_or_GPU_test_run':True,'branch_cases':rows,'failure_case':log}
(Path(__file__).parent/'lifecycle-results.json').write_text(json.dumps(value,indent=2)+'\n');print(json.dumps({k:v for k,v in value.items()if k not in ['branch_cases','failure_case']},indent=2))
