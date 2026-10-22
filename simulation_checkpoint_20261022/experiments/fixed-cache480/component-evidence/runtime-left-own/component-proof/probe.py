"""Intrinsic original-native model qualification; no application/physics or actors."""
import sys,time,json,types,numpy as np
from pathlib import Path
from pxr import Usd
R=Path('/workspace/project/mujoco');H=R/'isaac_port/native_hand_layout/roas-table';O=R/'.local/runtime-left-own477';sys.path[:0]=[str(O),str(H)]
from predictive_pair_clearance import proxy
from isaac_kinematics import Arm,frame
from same_arm_clearance import SameArmClearance
from fixed_own_route_certificate import compile_rows,bound_own_rows
from fixed_small_left_plan import META,left_carry
from fixed_small_own_domains import DOMAINS,select
import fast_numeric_fk
fast_numeric_fk.enable(Arm)
from exact_collision_proof_cache import own_model_fingerprint
stage=Usd.Stage.Open(str(R/'isaac_port/runs/native-roas-table-bin-retimed14/scene-authored.usda'));local=np.load(R/'.local/render-current/pair432-exact-locals.npz');model=json.loads((R/'.local/fixed-own-model450/left-native-kinematics.json').read_text());arm=Arm.__new__(Arm)
for key in['base','tip','axes','frames','bounds']:setattr(arm,key,np.asarray(model[key],float))
rows=[(str(b),str(s),proxy(local['v'+str(i)]))for i,(b,s)in enumerate(zip(local['bodies'],local['sources']))if'/left_arm_base/'in b and not str(b).endswith('_arm_fr3v2_link0')];arm.link_paths=[next(b for b,_,_ in rows if b.endswith('_arm_fr3v2_link'+str(i)))for i in range(1,8)];arm.hand_path=next(b for b,_,_ in rows if b.endswith('_NativeHand'))
ck=types.SimpleNamespace(stage=stage,arm=arm,side='left',pair_boxes=rows,payload_path='/World/Cell/Geometry/workpiece_ring2',payload_rows=[],payload_boxes=[],payload_relative=None)
p=np.load(R/'.local/collision-gpu2/binary/small-class-local.npz');ck.payload_boxes=[(ck.payload_path,str(s),proxy(p['v'+str(i)]))for i,s in enumerate(p['sources'])];guard=SameArmClearance(ck);ck.same_arm_clearance=guard;identity=own_model_fingerprint(guard);q=left_carry();state=dict(own_certificate=None,additional_own_domains=[]);compiled=[];began=time.perf_counter()

from fixed_small_own_domains import select_or_compile,prepare_current_for_carry
from fixed_small_left_executor import static_scope
from fixed_static_route_certificate import StaticRows,exact_row_key,payload_geometry_sha256
from fixed_own_route_certificate import OwnRows
# Intrinsic-model metadata only: no static query is skipped by this harness.
metadata=dict(world_sha256='MODEL_intrinsic_only',model_sha256=identity,payload_geometry_sha256=payload_geometry_sha256(ck),width_interval_m=META['width_interval_m'],nominal_hand_object_T=(frame([0,0,.1034],np.eye(3))@np.asarray(META['nominal_TCP_observation_T'])).tolist(),translation_m=.003,rotation_rad=.06)
state=dict(certificate=StaticRows(metadata,{exact_row_key(v)for v in q}),own_certificate=None,additional_own_domains=[])
for d in DOMAINS:
 cert=OwnRows.load(R/'.local/left-own472b'/(d['name']+'-own-MODEL.json'));state['additional_own_domains'].append(dict(name=d['name'],certificate=cert))
public=json.loads((O/'current-public-G.json').read_text());G=frame([0,0,.1034],np.eye(3))@np.asarray(public['measured_grasp']);ck.payload_relative=G
before,name=select(ck,state);assert before is None and name=='original_broad_domain'
c=dict(config=dict(fixed_route_cache_dir=str(O/'cache')),planning_diagnostics=[],fixed_prevalidated_small_left_routes=state)
state.update(world_sha256='MODEL_intrinsic_only',model_sha256=identity)
# Earlier speculative future-payload scopes do not consume the actual carry attempt.
ck.payload_relative=frame([0,0,.1034],np.eye(3))@np.asarray(DOMAINS[0]['TCP_grasp'])
with static_scope(c,ck):pass
assert '_runtime_own_domain_attempted'not in state and not c['planning_diagnostics']
ck.payload_relative=G
wrong=np.asarray(public['measured_grasp']).copy();wrong[0,3]+=.001
assert prepare_current_for_carry(c,ck,wrong)is None and '_runtime_own_domain_attempted'not in state
started=time.perf_counter();cert,name=prepare_current_for_carry(c,ck,np.asarray(public['measured_grasp']));runtime_s=time.perf_counter()-started;assert name=='current_public_RGBD_SMALL_LEFT_own'and cert is not None and len(cert.loaded)>0
# Reentering with the same current G must not compile again.
again,_=select_or_compile(c,ck,state);assert again is cert and len(c['planning_diagnostics'])==1
samples=q[np.unique(np.r_[np.arange(0,len(q),40),len(q)-1])];widths=np.linspace(.058,.064,5)
original=[guard.clear(v,float(w))[0]for v in samples for w in widths]
with bound_own_rows(ck,cert,current_world_sha256='MODEL_intrinsic_only',current_model_sha256=identity)as stats:accelerated=[guard.clear(v,float(w))[0]for v in samples for w in widths]
assert original==accelerated
# A later public G outside the tiny domain cannot trigger a second compile,
# and original physical palm collision is retained at an exact cached row.
bad=G.copy();bad[2,3]-=.06;ck.payload_relative=bad;fallback,fname=select_or_compile(c,ck,state);assert fallback is None and len(c['planning_diagnostics'])==1
b=guard.clear(q[0],.061)[0]
with bound_own_rows(ck,cert,current_world_sha256='MODEL_intrinsic_only',current_model_sha256=identity):a=guard.clear(q[0],.061)[0]
assert b is False and a is False
ck.payload_relative=G
# Model/world mismatch must miss every acceleration bit; original authority.
with bound_own_rows(ck,cert,current_world_sha256='changed_world',current_model_sha256=identity)as mismatch:
 assert guard.clear(q[0],.061)[0]==original[0]
assert mismatch['native_self_certificate_hits']==0 and mismatch['own_payload_certificate_hits']==0
# Failed/zero-bit runtime preparation consumes at most one attempt and retains
# the original broad own proof; no new static proof is installed.
import fixed_own_route_certificate as own_module
original_prepare=own_module.prepare_like_static
faults={}
for case in ['zero_bits','compiler_exception']:
 fresh=dict(state);fresh.pop('_runtime_own_domain_attempted',None);fresh['additional_own_domains']=state['additional_own_domains'][:4]
 cc=dict(config=c['config'],planning_diagnostics=[],fixed_prevalidated_small_left_routes=fresh)
 def failed_prepare(*args,**kwargs):
  if case=='compiler_exception':raise RuntimeError('Injected compiler failure')
  return dict(certificate=OwnRows(metadata,[],[]),summary=dict(ready=True))
 own_module.prepare_like_static=failed_prepare
 for _ in range(2):
  fallback,_=prepare_current_for_carry(cc,ck,np.asarray(public['measured_grasp']));assert fallback is None
 assert len(cc['planning_diagnostics'])==1 and not cc['planning_diagnostics'][0]['admitted']
 faults[case]=True
own_module.prepare_like_static=original_prepare
result=dict(MODEL_only=True,world_static_proof_not_claimed=True,current_G_source='public474 current carry stable-hold held LEFT RGB-D+SDK frame18',prior_four_domains_miss=True,earlier_future_payload_scope_does_not_consume_attempt=True,mismatched_attached_candidate_G_does_not_consume_attempt=True,failure_and_zero_bits_original_fallback=faults,native_certified_unique_rows=len(cert.native),loaded_certified_unique_rows=len(cert.loaded),runtime_preparation_s=runtime_s,runtime_preparation_count=len(c['planning_diagnostics']),original_queries=len(original),all_original_verdicts_equal=True,component_reuse=stats,later_out_of_domain_palm_collision_preserved=True,changed_world_certificate_miss=True,physical_commands_issued=0,physics_steps=0,uses_free_body_poses=False,task_clock_includes_runtime_preparation=True,diagnostic=c['planning_diagnostics'][0])
(O/'qualification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
