import sys,json,time,types,hashlib,numpy as np
from pathlib import Path
from pxr import Usd
R=Path('/workspace/project/mujoco');O=R/'.local/fixed490-width-model';H=R/'isaac_port/native_hand_layout/roas-table';sys.path[:0]=[str(O),str(H),str(R/'isaac_port')]
from predictive_pair_clearance import proxy
from isaac_kinematics import Arm,frame
from same_arm_clearance import SameArmClearance
from loaded_contact_aperture import LoadedContactAperture
from fixed_small_left_plan import left_carry
from exact_collision_proof_cache import own_model_fingerprint
import fast_numeric_fk
fast_numeric_fk.enable(Arm)
from gpu_pair_binary_protocol import KEYS,geometry_sha256
import gpu_pair_schedule,baseline_gpu_pair_width_scope,gpu_pair_width_scope
run=R/'isaac_port/runs/native-roas-table-fixed-front-v488-small-binding-split-gpu-profile-nominal';report=json.loads((run/'pipeline-results.json').read_text());record=next(x for x in report['planning_diagnostics']if x.get('event')=='small_loaded_receiver_encoder_geometry_bound');stage=Usd.Stage.Open(str(R/'isaac_port/runs/native-roas-table-bin-retimed14/scene-authored.usda'));local=np.load(R/'.local/render-current/pair432-exact-locals.npz');model=json.loads((R/'.local/fixed-own-model450/left-native-kinematics.json').read_text());arm=Arm.__new__(Arm)
for key in ['base','tip','axes','frames','bounds']:setattr(arm,key,np.asarray(model[key],float))
rows=[(str(b),str(s),proxy(local['v'+str(i)]))for i,(b,s)in enumerate(zip(local['bodies'],local['sources']))if'/left_arm_base/'in b and not str(b).endswith('_arm_fr3v2_link0')];arm.link_paths=[next(b for b,_,_ in rows if b.endswith('_arm_fr3v2_link'+str(i)))for i in range(1,8)];arm.hand_path=next(b for b,_,_ in rows if b.endswith('_NativeHand'))
ck=types.SimpleNamespace(stage=stage,arm=arm,side='left',pair_boxes=rows,payload_path='/World/Cell/Geometry/workpiece_ring2',payload_rows=[],payload_boxes=[],payload_relative=frame([0,0,.1034],np.eye(3))@np.asarray(record['measured_grasp']),resolution_rad=.035)
p=np.load(R/'.local/collision-gpu2/binary/small-class-local.npz');ck.payload_boxes=[(ck.payload_path,str(s),proxy(p['v'+str(i)]))for i,s in enumerate(p['sources'])];guard=SameArmClearance(ck);ck.same_arm_clearance=guard;ck.loaded_contact_aperture=LoadedContactAperture(side='left',part='ring2',payload_path=ck.payload_path,encoder_samples=record['measured_finger_samples_m'],sample_times_s=record['sample_times_s'],contact=record['fresh_bilateral_CAD_contact'],measured_grasp=record['measured_grasp'],control_dt_s=1/480,capture_age_physics_s=record['capture_age_physics_s']);before_identity=own_model_fingerprint(guard)
other=types.SimpleNamespace(arm=types.SimpleNamespace(hand_path='/right_NativeHand'),pair_boxes=[],payload_boxes=[],payload_path=None,payload_relative=None);ck.preview_other_configuration=(other,np.zeros(7),.024);c=dict(config={'gpu_pair_schedule_certificate':True},checkers={'left':ck,'right':other},planning_diagnostics=[])
# This is an intrinsic own-geometry regression: pair bits are deliberately
# ALL UNKNOWN, so acceleration cannot depend on optimistic pair acceptance.
def unknown(left,right,pairs,cfg):
 cfg['diagnostics']['geometry_sha256']='1'*64;return np.zeros(len(pairs),np.bool_)
gpu_pair_schedule.prove=unknown
q=left_carry()[np.arange(0,len(left_carry()),3)];widths=np.linspace(.059,.062,5)
# Native-self policy is unchanged and evaluated by the original guard everyq.
def run_scope(module):
 began=time.perf_counter();verdicts=[]
 with module.certified_width_path_scope(c,ck,q,widths):
  for w in widths:
   for value in q:verdicts.append(guard.clear(value,float(w))[0])
 return verdicts,time.perf_counter()-began
baseline,old_s=run_scope(baseline_gpu_pair_width_scope);fast,new_s=run_scope(gpu_pair_width_scope);assert baseline==fast and all(baseline);after_identity=own_model_fingerprint(guard);assert before_identity==after_identity
stats=[x for x in c['planning_diagnostics']if x.get('event')=='co_moving_native_own_FCL_scope'];assert sum(x['fast_returns']for x in stats)>0
own=guard.own_payload_clearance;assert not hasattr(own,'scoped_co_moving_proof')
# Restoration on exception and nested previous memo identity, no physics calls.
sentinel=object();own.scoped_co_moving_proof=sentinel
try:
 with gpu_pair_width_scope.certified_width_path_scope(c,ck,q[:2],widths):
  assert own.scoped_co_moving_proof is not sentinel;raise RuntimeError('EXPECTED_COMPONENT_FAULT')
except RuntimeError as error:assert str(error)=='EXPECTED_COMPONENT_FAULT'
assert own.scoped_co_moving_proof is sentinel;del own.scoped_co_moving_proof
# Changed heldG must not inherit a previously clear co-moving certificate.
original_G=ck.payload_relative.copy();bad=original_G.copy();bad[2,3]-=.060
with gpu_pair_width_scope.certified_width_path_scope(c,ck,q[:3],widths):
 guard.clear(q[0],float(widths[0]));guard.clear(q[1],float(widths[0]));ck.payload_relative=bad
 guarded_rejection=guard.clear(q[2],float(widths[0]))[0]
ck.payload_relative=original_G
with baseline_gpu_pair_width_scope.certified_width_path_scope(c,ck,q[:3],widths):
 ck.payload_relative=bad;original_rejection=guard.clear(q[2],float(widths[0]))[0]
ck.payload_relative=original_G
assert original_rejection is False and guarded_rejection is False
result=dict(status='PASS',changed_G_original_palm_collision_preserved=True,scope='PUBLIC488 measuredRGBD G/stationary aperture binding +originalfull native/classCAD; original complete intrinsic self/own queries; pairbits forcedUNKNOWN; nophysics',original_native_queries=len(baseline),all_original_verdicts_identical=True,all_original_queries_clear=all(baseline),baseline_scope_s=old_s,new_scope_s=new_s,component_speedup=old_s/new_s,own_memo_scopes=stats,normal_unknown_path_covered=True,cleanup_after_return=True,cleanup_and_nested_restore_after_fault=True,model_fingerprint_unchanged=before_identity==after_identity,model_sha256=before_identity,only_gpu_pair_width_scope_changed=True,physical_commands_issued=0,physics_steps=0)
(O/'results.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
