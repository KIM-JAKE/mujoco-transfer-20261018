"""Intrinsic original-native model qualification; no application/physics or actors."""
import sys,time,json,types,numpy as np
from pathlib import Path
from pxr import Usd
R=Path('/workspace/project/mujoco');H=R/'isaac_port/native_hand_layout/roas-table';O=R/'.local/co-moving484-model';sys.path[:0]=[str(O),str(H)]
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


from co_moving_own_clearance import co_moving_scope
from baseline_own_payload_clearance import OwnPayloadClearance as BaselineOwn
from fixed_static_route_certificate import payload_geometry_sha256
public=json.loads((O/'actual480-current-G-metadata.json').read_text());m=public['metadata'];G=np.asarray(m['nominal_hand_object_T']);ck.payload_relative=G
assert payload_geometry_sha256(ck)==m['payload_geometry_sha256']
# Original native-self certificate only. ZERO loaded bits are deliberately
# retained to reproduce the actual480 full-own fallback hotspot.
cert,why=compile_rows(ck,q,META['width_interval_m'],world_sha256='MODEL_intrinsic_only',model_sha256=identity,nominal_hand_object_T=G,translation_m=.00025,rotation_rad=.005)
assert not cert.loaded and len(cert.native)==715
class OriginalOwn(BaselineOwn):
 def clear(self,checker,transforms,*,changed_bodies=None,requested_opening=None):return super().clear(checker,transforms,changed_bodies=changed_bodies)
original=OriginalOwn(guard);accelerator=guard.own_payload_clearance
samples=q[np.unique(np.r_[np.arange(0,len(q),7),len(q)-1])];widths=np.linspace(.059,.062,5)
queries=[(v,float(w))for v in samples for w in widths]
c=dict(planning_diagnostics=[])
def run(own,memo=False):
 guard.own_payload_clearance=own
 from contextlib import ExitStack
 with ExitStack()as stack:
  stack.enter_context(bound_own_rows(ck,cert,current_world_sha256='MODEL_intrinsic_only',current_model_sha256=identity))
  if memo:stack.enter_context(co_moving_scope(c,ck))
  started=time.perf_counter();verdicts=[guard.clear(v,w)[0]for v,w in queries];elapsed=time.perf_counter()-started
 return verdicts,elapsed
baseline,bt=run(original);faster,ft=run(accelerator,True);assert baseline==faster and all(baseline)
stats=c['planning_diagnostics'][-1];assert stats['clear_queries']>0 and stats['skipped_candidate_pairs']>0
# Changed public G: exact key must miss and preserve a real palm collision.
guard.own_payload_clearance=accelerator;bad=G.copy();bad[2,3]-=.06;ck.payload_relative=bad
with co_moving_scope(c,ck):
 for repeat in range(3):assert guard.clear(q[0],.061)[0]is False
assert c['planning_diagnostics'][-1]['clear_queries']==0
# Within one scope, changed width and changed G get distinct first-seen keys.
ck.payload_relative=G
with co_moving_scope(c,ck):
 guard.clear(q[0],.061);guard.clear(q[1],.061)
 memo=accelerator.scoped_co_moving_proof;first=memo.stats['first_seen']
 guard.clear(q[2],.0611);assert memo.stats['first_seen']==first+1
 changed=G.copy();changed[0,3]+=.000001;ck.payload_relative=changed;guard.clear(q[3],.0611);assert memo.stats['first_seen']==first+2
# An unexpected actual Hand-relative finger displacement invalidates a prior
# co-moving proof, even if requested width and publicG are unchanged.
ck.payload_relative=G
with co_moving_scope(c,ck):
 guard.clear(q[0],.061);guard.clear(q[1],.061)
 before=c['planning_diagnostics'][-1]if c['planning_diagnostics']else{}
 memo=accelerator.scoped_co_moving_proof
 Ts=ck.arm.bodies(q[2],.061);Ts[guard.base_path]=ck.arm.base;finger=ck.arm.hand_path+'/left_left_finger';Ts[finger]=Ts[finger].copy();Ts[finger][0,3]+=.002
 assert accelerator.clear(ck,Ts,requested_opening=.061)[0]in [True,False]
 assert memo.stats['relative_misses']>0
# A freshly changed NON-co-moving own link remains an obstacle. Its current
# AABB overlap must prevent fast return and preserve original full FCL BLOCKED.
ck.payload_relative=G
with co_moving_scope(c,ck):
 guard.clear(q[0],.061);guard.clear(q[1],.061)
 Ts=ck.arm.bodies(q[2],.061);Ts[guard.base_path]=ck.arm.base
 target=(Ts[ck.arm.hand_path]@G);point=np.mean(ck.payload_boxes[0][2][3],axis=0)@target[:3,:3].T+target[:3,3]
 body,_,shape=accelerator.robot_rows[2];Ts[body]=np.eye(4);Ts[body][:3,3]=point-np.mean(shape[3],axis=0)
 expected=original.clear(ck,Ts)[0];obtained=accelerator.clear(ck,Ts,requested_opening=.061)[0]
 assert expected is False and obtained is False
result=dict(MODEL_only=True,EVALUATION_only_geometry_source='public480 current RGB-D runtime metadata + original class CAD/encoder FK',native_self_only_rows=len(cert.native),own_envelope_rows=0,original_full_native_queries=len(queries),all_original_verdicts_equal=True,baseline_s=bt,memo_s=ft,speedup=bt/ft,memo_stats=stats,changed_G_palm_collision_preserved=True,current_noncomoving_link_collision_preserved=True,changed_G_width_keys_separate=True,unexpected_relative_finger_transform_misses=True,physical_commands_issued=0,physics_steps=0)
(O/'qualification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
