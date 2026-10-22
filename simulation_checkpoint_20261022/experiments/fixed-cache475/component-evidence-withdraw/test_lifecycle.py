import sys,types,ast,json
from pathlib import Path
from contextlib import contextmanager
import numpy as np
BASE=Path(__file__).resolve().parent
sys.path.insert(0,str(BASE/'overrides'))
sys.modules['isaac_kinematics']=types.SimpleNamespace(log=lambda R:np.zeros(3))
import empty_right_release_withdraw as m
class Premotion(RuntimeError):pass
q=np.arange(7,dtype=float)*.1;target=np.eye(4);target[2,3]=-.08
class Art:
 def get_dof_positions(self):return np.r_[q,.04,.04][None].copy()
class Arm:
 def solve(self,*a):return q.copy()
 def fk(self,_):return target.copy()
class Checker:
 def clear(self,*a):return True,None
 def route(self,*a,**kw):return [q.copy(),q.copy()]
@contextmanager
def context():yield
original_scope=m._released_part_scope
@contextmanager
def scope(c,part,T,reserve):
 c['scope_calls']+=1;assert np.array_equal(T,np.eye(4));assert abs(reserve-.0012)<1e-10
 yield types.SimpleNamespace(stats={'queries':1})
m._released_part_scope=scope
sys.modules['fr3_gpu_ik']=types.SimpleNamespace(propose_ik=lambda *a:[q.copy()])
def run_case(line_error=None,move_error=None,no_route=False):
 c=dict(arm=Arm(),checker=Checker(),art=Art(),cmd=np.r_[q,.04,.04][None].copy(),ids=np.arange(7),fi=np.array([7,8]),config={},planning_diagnostics=[],scope_calls=0);called={'line':0,'move':0}
 if no_route:c['checker'].route=lambda *a,**kw:None
 def line(*a,**kw):
  called['line']+=1
  if line_error:raise line_error
 def move(*a,**kw):
  called['move']+=1
  if move_error:raise move_error
 result=m.execute(c,'ring1',target,.08,dict(withdraw_q=[q],pre_q=q),np.eye(4),dict(rms_m=.0001,lateral_CAD_registration_rms_m=.0004),line,move,context,lambda *a,**kw:None,Premotion)
 return c,called,result
c,n,r=run_case();assert n=={'line':1,'move':0}and c['scope_calls']==0
c,n,r=run_case(Premotion('original guarded rejection'));assert n['move']==1 and c['scope_calls']==1 and r['same_original_target']
try:run_case(RuntimeError('actuator or unexpected error'));raise AssertionError('Unexpected line error swallowed')
except RuntimeError as e:assert str(e)=='actuator or unexpected error'
try:run_case(Premotion('original'),RuntimeError('partial actuator failure'));raise AssertionError('Partial movement retried')
except RuntimeError as e:assert str(e)=='partial actuator failure'
try:run_case(Premotion('original'),Premotion('retimed blocked'));raise AssertionError('Blocked field accepted')
except RuntimeError as e:assert str(e).startswith('No full-current-context empty RIGHT withdrawal')
try:run_case(Premotion('original'),no_route=True);raise AssertionError('Missing route accepted')
except RuntimeError as e:assert str(e).startswith('No full-current-context empty RIGHT withdrawal')
# Static guarantee: the ONLY typed preflight rejection precedes the complete
# original command-issuing loop, and never wraps step()/actuation failures.
source=(BASE/'overrides/native_right_ring_insert.py').read_text();t=ast.parse(source)
move=next(n for n in ast.walk(t)if isinstance(n,ast.FunctionDef)and n.name=='move')
raises=[n for n in ast.walk(move)if isinstance(n,ast.Raise)and isinstance(n.exc,ast.Call)and isinstance(n.exc.func,ast.Name)and n.exc.func.id=='ExecutedCurvePreflightBlocked']
issued=next(n for n in ast.walk(move)if isinstance(n,ast.For)and isinstance(n.target,ast.Tuple)and[n.id for n in n.target.elts]==['row','u'])
assert len(raises)==1 and raises[0].lineno<issued.lineno
result=dict(original_success_no_fallback_no_extra_obstacle=True,typed_no_motion_rejection_replanned=True,unchanged_exact_world_target=True,unexpected_and_partial_actuation_errors_propagate=True,rejected_retimed_field_no_motion=True,no_route_remains_failed=True,only_typed_raise_before_first_command=True,fresh_released_part_scope_only_on_fallback=True)
(BASE/'lifecycle-tests.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
# Scoped newly released obstacle never reduces a previous completed reserve.
m._released_part_scope=original_scope
class Placed:
 def __init__(self):self.parts={};self.generation=0
 def register(self,checker,label,template,T,uncertainty_m):self.parts[label]={'uncertainty_m':uncertainty_m};self.generation+=1
sys.modules['placed_part_clearance']=types.SimpleNamespace(PlacedPartClearance=Placed)
old=Placed();old.parts={'ring2':{'uncertainty_m':.0035}};ck=types.SimpleNamespace(placed_part_clearance=old);c=dict(checker=ck,part_collision_templates={'ring1':'fixed_class_CAD'})
with original_scope(c,'ring1',np.eye(4),.0012)as new:
 assert new.parts['ring2']['uncertainty_m']==.0035 and new.parts['ring1']['uncertainty_m']==.0012
assert ck.placed_part_clearance is old and 'ring1'not in old.parts
old.parts['ring1']={'uncertainty_m':.0035}
try:
 with original_scope(c,'ring1',np.eye(4),.0012):raise AssertionError('Existing reserve overwritten')
except RuntimeError as e:assert str(e).startswith('Released ring already registered')
assert ck.placed_part_clearance is old and old.parts['ring1']['uncertainty_m']==.0035
result['existing_completed_reserves_preserved_and_duplicate_ring_fails_closed']=True
(BASE/'lifecycle-tests.json').write_text(json.dumps(result,indent=2)+'\n')
