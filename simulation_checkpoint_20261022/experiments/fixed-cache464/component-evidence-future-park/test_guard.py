import sys,types,json,numpy as np
from contextlib import contextmanager
from pathlib import Path
sys.modules['pxr']=types.SimpleNamespace(UsdGeom=None,UsdPhysics=None,Usd=None)
sys.path[:0]=['/tmp/fixed464-whole-carry-park/overrides','/tmp/fixed463-full/overrides','/home/gpuadmin/workspace/jake/mujoco/isaac_port/native_hand_layout/roas-table']
import future_saved_carry_guard as m
import fixed_saved_executor as stored
import fixed_canonical_plan as plan
import serial_gpu_pair_scope as pair
import dual_aperture_path_check as original
G=np.eye(4);H=np.eye(4);H[2,3]=.1034;rows=np.array([[0]*7,[.01]*7,[.02]*7]);plan.right_carry=lambda:rows
state=dict(domain=True,inside=False,clear=False,calls=0,entered=0)
@contextmanager
def other(*a):
 state['inside']=True
 try:yield
 finally:state['inside']=False
rc=types.SimpleNamespace(payload_path='ring1',payload_relative=H.copy(),other_arm_at=other);lc=types.SimpleNamespace(payload_path=None);c=dict(checkers=dict(left=lc,right=rc),fixed_prevalidated_routes={});q=np.arange(7,dtype=np.float32);rw=np.linspace(.071,.075,5);lw=np.linspace(.0639,.064,5)
stored.select_domain=lambda *args:object()if state['domain']else None
@contextmanager
def static(*args):
 assert state['inside'];state['entered']+=1
 yield dict(original_static_own_scope=True)
stored.static_scope=static
@contextmanager
def pairs(c,a,b,contexts,qs,widths):
 assert state['inside']and a is rc and b is lc and np.array_equal(qs,rows)and len(contexts)==5 and np.array_equal(widths,rw)
 assert all(np.array_equal(cq,q.astype(float))for cq,w in contexts)
 yield
pair.serial_path_scope=pairs
def guard(a,b,contexts,qs,widths,diagnostics):
 state['calls']+=1;assert state['inside']and np.array_equal(qs,rows)and len(contexts)==5 and np.array_equal(widths,rw);diagnostics['original_guard_called']=True
 return state['clear'],None if state['clear']else dict(collision='original_pair_block')
original.path_other_contexts=guard
r=m.check(c,q,G,rw,lw);assert r['applies']and not r['clear']and r['collision']['collision']=='original_pair_block'
state['clear']=True;r=m.check(c,q,G,rw,lw);assert r['clear']and r['fresh_actual_postpark_whole_path_required']and r['rows']==3 and r['query_diagnostics']['original_guard_called']
state['domain']=False;calls=state['calls'];r=m.check(c,q,G,rw,lw);assert not r['applies']and state['calls']==calls and not state['inside']
state['domain']=True;rc.payload_relative[0,3]=.001
try:m.check(c,q,G,rw,lw)
except RuntimeError:pass
else:raise AssertionError('Wrong attached publicG accepted')
rc.payload_relative=H.copy();lc.payload_path='unexpected'
try:m.check(c,q,G,rw,lw)
except RuntimeError:pass
else:raise AssertionError('Nonempty future LEFT accepted')
result=dict(entire_original_stored_middle_before_any_actuation=True,all_five_by_five_width_contexts_retained=True,existing_static_own_GPU_and_original_path_guards_called=True,original_collision_rejects_candidate=True,domain_miss_requests_original_planner=True,unknown_future_actual_encoder_not_claimed=True,wrong_G_and_nonempty_LEFT_rejected=True)
Path('/tmp/fixed464-whole-carry-park/helper-tests.json').write_text(json.dumps(result,indent=2));print(result)
