from pathlib import Path
import sys,types,numpy as np,json
from contextlib import contextmanager
sys.path[:0]=['/tmp/fixed463-live-entry-bridge/overrides','/tmp/fixed461-full/overrides','/home/gpuadmin/workspace/jake/mujoco/isaac_port/native_hand_layout/roas-table']
import fixed_saved_executor as m
rows=np.zeros((2,7));rows[:,0]=.1;m.right_carry=lambda:rows;m.eligible=lambda *a:True;state={'in_context':False,'route':None,'route_calls':0,'moves':0}
class Checker:
 def route(self,*a,**kw):
  assert state['in_context'];state['route_calls']+=1;return state['route']
cmd=np.array([[0.]*7+[.035,.035]],np.float32);art=types.SimpleNamespace(get_dof_positions=lambda:cmd.copy());c=dict(checkers=dict(right=Checker()),cmd=cmd,ids=np.arange(7),fi=np.array([7,8]),art=art,planning_diagnostics=[])
@contextmanager
def context(G):
 assert np.array_equal(G,np.eye(4));state['in_context']=True
 try:yield
 finally:state['in_context']=False
class StopAfterPlanner(RuntimeError):pass
class Route(list):verified_cubic=True
state['route']=None
assert m.execute_carry(c,np.eye(4),context,lambda *a,**kw:None,lambda *a:None)is False
assert not state['moves'] and c['planning_diagnostics'][-1]['no_motion_issued']
state['route']=Route([np.zeros(7),np.ones(7)*.05,rows[0]])
def move(*a,**kw):
 state['moves']+=1;assert a[0]is state['route']and kw['smooth']is True;raise StopAfterPlanner('original exact-curve admission remains authoritative')
try:m.execute_carry(c,np.eye(4),context,move,lambda *a:None)
except StopAfterPlanner:pass
else:raise AssertionError('Original move error swallowed')
assert state['moves']==1 and not state['in_context']
def partial(*a,**kw):
 cmd[0,0]=.02;raise StopAfterPlanner('partial-actuation error')
try:m.execute_carry(c,np.eye(4),context,partial,lambda *a:None)
except StopAfterPlanner:pass
else:raise AssertionError('Partial actuation error swallowed')
assert cmd[0,0]==np.float32(.02)
result=dict(obstacle_aware_route_runs_in_fresh_payload_other_context=True,no_route_returns_before_any_move=True,original_exact_issued_move_guard_receives_cubic_marker=True,no_guard_or_partial_actuation_errors_caught=True,AST_compile_pass=True)
Path('/tmp/fixed463-live-entry-bridge/tests.json').write_text(json.dumps(result,indent=2));print(result)
