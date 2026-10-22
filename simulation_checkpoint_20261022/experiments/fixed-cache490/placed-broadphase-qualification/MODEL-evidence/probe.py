"""Native current-q/G placed-part broadphase equivalence; no physics/actors."""
import sys,json,time,types,hashlib
from pathlib import Path
import numpy as np
from pxr import Usd
R=Path('/workspace/project/mujoco');H=R/'isaac_port/native_hand_layout/roas-table';O=R/'.local/placed490-model';RUN=R/'isaac_port/runs/native-roas-table-fixed-front-v488-small-binding-split-gpu-profile-nominal';sys.path[:0]=[str(O),str(R/'.local/co-moving484-model'),str(H),str(R)]
from placed_part_clearance import PlacedPartClearance
from baseline_placed import PlacedPartClearance as Original
from predictive_pair_clearance import proxy,transform_proxy
from isaac_kinematics import Arm,frame
from same_arm_clearance import SameArmClearance
from rgbd_geometry import axis_frame
import fast_numeric_fk,fcl
fast_numeric_fk.enable(Arm)
stage=Usd.Stage.Open(str(R/'isaac_port/runs/native-roas-table-bin-retimed14/scene-authored.usda'));local=np.load(R/'.local/render-current/pair432-exact-locals.npz');report=json.loads((RUN/'pipeline-results.json').read_text());a=np.load(RUN/'executed-actuator-commands.npz');public=json.loads((R/'.local/small486-binding-model/public-packet.json').read_text());smallG=np.asarray(public['public_frames'][0]['hand_G'])
placed=PlacedPartClearance();original=Original();last=[];timings=[];counts=dict(queries=0,clear=0,blocked=0,equal=0)
for side in ['left','right']:
 model=json.loads((R/'.local/fixed-own-model450/left-native-kinematics.json' if side=='left' else RUN/'robot-kinematics.json').read_text());arm=Arm.__new__(Arm)
 for key in ['base','tip','axes','frames','bounds']:setattr(arm,key,np.asarray(model[key],float))
 rows=[(str(b),str(s),proxy(local['v'+str(i)]))for i,(b,s)in enumerate(zip(local['bodies'],local['sources']))if'/'+side+'_arm_base/'in b and not str(b).endswith('_arm_fr3v2_link0')];arm.link_paths=[next(b for b,_,_ in rows if b.endswith('_arm_fr3v2_link'+str(i)))for i in range(1,8)];arm.hand_path=next(b for b,_,_ in rows if b.endswith('_NativeHand'))
 ck=types.SimpleNamespace(stage=stage,arm=arm,side=side,pair_boxes=rows,payload_path=None,payload_boxes=[],payload_relative=None);guard=SameArmClearance(ck);ck.same_arm_clearance=guard
 if side=='left':
  for item in report['planning_diagnostics']:
   if item.get('event')=='completed_part_added_to_both_arm_collision_worlds'and item['part']in ['ring1','ring2']:
    pose=item['pose_from_RGBD'];placed.register(ck,item['part'],'/World/Cell/Geometry/workpiece_'+item['part'],frame(pose['center_m'],axis_frame(pose['axis'])),uncertainty_m=item['uncertainty_m'])
  original.parts=placed.parts
 indices=[i for i,n in enumerate(report['dof_names'])if n.startswith(side+'_arm_fr3v2_joint')];fingers=[i for i,n in enumerate(report['dof_names'])if n.startswith(side+'_native_')]
 # Actual public encoder rows across the post-pickup cycle, three known-class
 # carried contexts. These are geometry queries, not re-executed trajectories.
 selected=np.unique(np.r_[np.arange(6000,len(a['command']),40),len(a['command'])-1]);queries=[]
 for part in [None,'ring2','ring1']:
  if part:
   p=np.load(R/'.local/collision-gpu2/binary'/('small-class-local.npz'if part=='ring2'else'large-class-local.npz'));boxes=[('/World/Cell/Geometry/workpiece_'+part,str(s),proxy(p['v'+str(i)]))for i,s in enumerate(p['sources'])]
   if part=='ring2':G=smallG
   else:
    row=next(v for v in report['planning_diagnostics']if v.get('event')=='paired_RIGHT_loaded_contact_bound_before_lift');G=frame([0,0,.1034],np.eye(3))@np.asarray(row['measured_grasp'])
  else:boxes=[];G=None
  for i in selected:
   q=a['actual_dof_position'][i,indices].astype(float);width=float(a['actual_dof_position'][i,fingers].sum());T=arm.bodies(q,width);T[guard.base_path]=arm.base;queries.append((part,boxes,G,T,int(i)))
 def run(obj):
  outcomes=[];elapsed=0
  for part,boxes,G,T,index in queries:
   ck.payload_path=None if part is None else '/World/Cell/Geometry/workpiece_'+part;ck.payload_boxes=boxes;ck.payload_relative=G
   for (body,_,_),o in zip(guard.rows,guard.objects):
    t=T[body];o.setTransform(fcl.Transform(np.ascontiguousarray(t[:3,:3]),np.ascontiguousarray(t[:3,3])))
   t=time.perf_counter();ok,why=obj.clear(ck,T);elapsed+=time.perf_counter()-t;outcomes.append((bool(ok),None if why is None else (why['kind'],why['query'],why['placed_part'])))
  return outcomes,elapsed
 before,bt=run(original);after,ft=run(placed);assert before==after
 warm_before,wbt=run(original);warm_after,wft=run(placed);assert warm_before==warm_after==before
 counts['queries']+=len(before);counts['equal']+=len(before);counts['clear']+=sum(v[0]for v in before);counts['blocked']+=sum(not v[0]for v in before);timings.append(dict(side=side,queries=len(before),original_s=bt,early_broadphase_s=ft,speedup=bt/ft,warm_original_s=wbt,warm_early_s=wft,warm_speedup=wbt/wft))
result=dict(MODEL_only=True,physics_steps=0,commands_issued=0,currentq_source='actual488 public SDK encoders',G_source='recorded public RGB-D and known class CAD only',placed_source='actual488 registered released-retention RGB-D',all_verdicts_and_rejection_identifiers_equal=True,counts=counts,timings=timings,early_stats=placed.stats,source_sha256=hashlib.sha256((O/'placed_part_clearance.py').read_bytes()).hexdigest())
(O/'qualification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
import cProfile,pstats,io
profile=cProfile.Profile();profile.enable();run(placed);profile.disable();stream=io.StringIO();pstats.Stats(profile,stream=stream).sort_stats('cumtime').print_stats(22);(O/'profile.txt').write_text(stream.getvalue());print(stream.getvalue(),flush=True)
