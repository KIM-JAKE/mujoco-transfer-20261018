import sys,types,json
from pathlib import Path
from contextlib import contextmanager
import numpy as np
BASE=Path(__file__).resolve().parent
sys.modules['pxr']=types.SimpleNamespace(UsdGeom=None,UsdPhysics=None,Usd=None)
sys.path[:0]=[str(BASE/'overrides'),'/tmp/fixed476-full/overrides','/home/gpuadmin/workspace/jake/mujoco/isaac_port/native_hand_layout/roas-table']
import future_saved_carry_guard as m
import fixed_saved_executor as stored, fixed_canonical_plan as plan
import serial_gpu_pair_scope as serial, dual_aperture_path_check as original
import gpu_pair_schedule as gpu
G=np.eye(4);H=G.copy();H[2,3]=.1034;rows=np.array([[0.]*7,[.01]*7,[.02]*7]);plan.right_carry=lambda:rows
state=dict(domain=True,mode='clear',original_clear=True,calls=0,static_calls=0,inside=False)
def shape():return(np.zeros(3),np.eye(3),np.ones(3)*.01,np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])*.01,np.eye(3))
@contextmanager
def other(*a):
 state['inside']=True
 try:yield
 finally:state['inside']=False
rc=types.SimpleNamespace(arm=types.SimpleNamespace(base=np.eye(4)),pair_boxes=[('R','r',shape())],payload_path='ring1',payload_relative=H.copy(),payload_boxes=[('ring1','part',shape())],other_arm_at=other,resolution_rad=.005)
lc=types.SimpleNamespace(arm=types.SimpleNamespace(base=np.eye(4)),pair_boxes=[('L','l',shape())],payload_path=None,payload_relative=None,payload_boxes=[])
c=dict(checkers={'left':lc,'right':rc},fixed_prevalidated_routes={},cmd=np.zeros((1,18)),_collision_physics_epoch=5,config=dict(gpu_pair_schedule_certificate=True,gpu_pair_schedule_socket='/model/pair-binary.sock'))
stored.select_domain=lambda *a:object()if state['domain']else None
@contextmanager
def static(*a):state['static_calls']+=1;yield dict(original_static_own_scope=True)
stored.static_scope=static
@contextmanager
def serialscope(*a):yield
serial.serial_path_scope=serialscope
def guard(a,b,contexts,qs,widths,diagnostics):
 state['calls']+=1;assert state['inside']and len(contexts)==5 and len(widths)==5 and np.array_equal(qs,rows)
 return state['original_clear'],None if state['original_clear']else dict(original_collision=True)
original.path_other_contexts=guard
rw=np.linspace(.071,.075,5);lw=np.linspace(.0639,.064,5);q=np.arange(7,dtype=np.float32)
def prove(left,right,pairs,cfg):
 from aperture_path_check import _samples
 assert len(pairs)==len(list(_samples(rc,rows)))*25
 cfg['diagnostics'].update(geometry_sha256=m._geometry_sha(lc,rc),certificate='original_proxy_separation_only',proposal_only=False,float64=True,outward_reserve_m=1e-9,original_pair_margin_m=.001)
 mask=np.ones(len(pairs),bool);mode=state['mode']
 if mode=='unknown':mask[0]=False
 if mode=='wrong_sha':cfg['diagnostics']['geometry_sha256']='0'*64
 if mode=='bad_policy':cfg['diagnostics']['original_pair_margin_m']=.0001
 if mode=='mutate_geometry':rc.pair_boxes[0][2][3][0,0]+=1e-6
 if mode=='mutate_model':rc.arm.base[0,3]+=1e-6
 if mode=='mutate_command':c['cmd'][0,0]+=1e-6
 if mode=='mutate_epoch':c['_collision_physics_epoch']+=1
 if mode=='bad_dtype':return mask.astype(np.int8)
 if mode=='exception':raise TimeoutError('model timeout')
 return mask
gpu.prove=prove
r=m.check(c,q,G,rw,lw);assert r['clear']and r['candidate_pair_filter_only']and state['calls']==0 and state['static_calls']==0 and r['fresh_actual_postpark_whole_path_required']
results={'ALL_CLEAR_pair_only_no_hypothetical_static_own_placed_scan':True}
for mode in ['unknown','wrong_sha','bad_policy','mutate_geometry','mutate_model','mutate_command','mutate_epoch','bad_dtype','exception']:
 state['mode']=mode;before=state['calls'];r=m.check(c,q,G,rw,lw);assert r['clear']and state['calls']==before+1 and r['query_diagnostics']['original_static_own_scope'];results[mode+'_uses_original_full_future_guard']=True
state['mode']='unknown';state['original_clear']=False;r=m.check(c,q,G,rw,lw);assert not r['clear']and r['collision']['original_collision'];results['UNKNOWN_is_not_collision_original_guard_decides']=True
state['domain']=False;before=state['calls'];r=m.check(c,q,G,rw,lw);assert not r['applies']and r['clear']is None and state['calls']==before;results['domain_miss_original_planner_policy_unchanged']=True
# Only helper changed: originalactualpostpark full path guard still precedes
# issuance of ALLsavedRIGHTrows in the unchanged production executor.
s=(Path('/tmp/fixed476-full/overrides')/'fixed_saved_executor.py').read_text();start=s.index('def execute_carry(');s=s[start:];assert s.index('clear, why = path_openings')<s.index('if not clear:')<s.index('for q in rows[1:]:');results['unchanged_full_actual_postpark_guard_before_first_saved_RIGHT_row']=True
(BASE/'unit-tests.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(results,indent=2))
