"""CPU-only original native attach/FCL equivalence. No simulation or GPU work."""
import sys,ast,json,time,types,hashlib
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,PhysicsSchemaTools,Sdf
R=Path('/workspace/project/mujoco');H=R/'isaac_port/native_hand_layout/roas-table';O=R/'.local/rawproxy493-model';RUN=R/'isaac_port/runs/native-roas-table-fixed-front-v490-width-memo-placed-broadphase-nominal';S=RUN/'code_snapshot';sys.path[:0]=[str(O),str(S),str(H),str(R)]
from predictive_pair_clearance import proxy
from isaac_kinematics import Arm,frame
from same_arm_clearance import SameArmClearance
from own_payload_clearance import OwnPayloadClearance
from exact_collision_proof_cache import own_model_fingerprint
import fast_numeric_fk,fcl_transport,payload_proxy_content_intern as interning
fast_numeric_fk.enable(Arm)
# Literal original native attach/detach AST; __init__'s PhysX application setup
# is intentionally absent. This executes ALL original USD class/query writes.
original_source=(S/'isaac_collision_queries.py').read_text();tree=ast.parse(original_source);cls=next(n for n in tree.body if isinstance(n,ast.ClassDef)and n.name=='Checker');functions=[n for n in cls.body if isinstance(n,ast.FunctionDef)and n.name in ['attach_payload','detach_payload']];definition=ast.ClassDef(name='NativeAttach',bases=[],keywords=[],body=functions,decorator_list=[]);namespace=dict(np=np,proxy=proxy,UsdGeom=UsdGeom,PhysicsSchemaTools=PhysicsSchemaTools,Sdf=Sdf,__name__='__native_attach_model__');exec(compile(ast.fix_missing_locations(ast.Module(body=[definition],type_ignores=[])),str(S/'isaac_collision_queries.py'),'exec'),namespace);NativeAttach=namespace['NativeAttach'];original_attach=NativeAttach.attach_payload
stage=Usd.Stage.Open(str(R/'isaac_port/runs/native-roas-table-bin-retimed14/scene-authored.usda'));local=np.load(R/'.local/render-current/pair432-exact-locals.npz');model=json.loads((R/'.local/fixed-own-model450/left-native-kinematics.json').read_text());arm=Arm.__new__(Arm)
for key in ['base','tip','axes','frames','bounds']:setattr(arm,key,np.asarray(model[key],float))
rows=[(str(b),str(s),proxy(local['v'+str(i)]))for i,(b,s)in enumerate(zip(local['bodies'],local['sources']))if'/left_arm_base/'in b and not str(b).endswith('_arm_fr3v2_link0')];arm.link_paths=[next(b for b,_,_ in rows if b.endswith('_arm_fr3v2_link'+str(i)))for i in range(1,8)];arm.hand_path=next(b for b,_,_ in rows if b.endswith('_NativeHand'))
def checker():
 ck=NativeAttach();ck.stage=stage;ck.side='left';ck.arm=arm;ck.pair_boxes=rows;ck.payload_path=None;ck.payload_boxes=[];ck.payload_rows=[];ck.payload_relative=None;return ck
baseline=checker();fast=checker();guard=SameArmClearance(fast);guard.checker=fast;fast.same_arm_clearance=guard;before_hash=own_model_fingerprint(guard);ownA=OwnPayloadClearance(guard);ownB=OwnPayloadClearance(guard)
public=json.loads((R/'.local/small486-binding-model/public-packet.json').read_text());report=json.loads((RUN/'pipeline-results.json').read_text());a=np.load(RUN/'executed-actuator-commands.npz');ids=[i for i,n in enumerate(report['dof_names'])if n.startswith('left_arm_fr3v2_joint')];cases=[]
for part in ['ring2','ring1']:
 for k in range(12):
  G=np.asarray(public['public_frames'][k%4]['hand_G']).copy()if part=='ring2'else frame([0,0,.1034],np.eye(3))@np.asarray(next(v for v in report['planning_diagnostics']if v.get('event')=='paired_RIGHT_loaded_contact_bound_before_lift')['measured_grasp'])
  if part=='ring1':G[0,3]+=(k%3-1)*.00001
  q=a['actual_dof_position'][[7000,10000,14000][k%3],ids].astype(float);width=[.05902091786265373,.061830341815948486,.064][k%3];cases.append((part,G,q,width))
count=[0];original_convex=fcl_transport.convex
def counted(v):count[0]+=1;return original_convex(v)
fcl_transport.convex=counted
baseline_results=[];started=time.perf_counter();attachA=ownTimeA=0
for part,G,q,width in cases:
 t=time.perf_counter();original_attach(baseline,'/World/Cell/Geometry/workpiece_'+part,G);attachA+=time.perf_counter()-t
 T=arm.bodies(q,width);T[guard.base_path]=arm.base;t=time.perf_counter();baseline_results.append(ownA.clear(baseline,T));ownTimeA+=time.perf_counter()-t
base_wall=time.perf_counter()-started;baseline_hulls=count[0];count[0]=0
predictive_original=__import__('predictive_pair_clearance').proxy;info=interning.enable(NativeAttach);assert __import__('predictive_pair_clearance').proxy is predictive_original;after_hash=own_model_fingerprint(guard);assert before_hash==after_hash
fast_results=[];started=time.perf_counter();attachB=ownTimeB=0;last={};freshG=True
for part,G,q,width in cases:
 t=time.perf_counter();fast.attach_payload('/World/Cell/Geometry/workpiece_'+part,G);attachB+=time.perf_counter()-t;assert fast.payload_relative is G
 # Source/template and original planning query mesh are still exact, even
 # when the proxy identity is interned and G changes between attachments.
 assert stage.GetPrimAtPath('/World/PlanningQueries/left_payload0').IsValid()
 source=stage.GetPrimAtPath(fast.payload_boxes[0][1]);query=stage.GetPrimAtPath('/World/PlanningQueries/left_payload0');localT=__import__('payload_class_geometry').class_local_transform(UsdGeom.XformCache(),source,stage.GetPrimAtPath(fast.payload_path));expected_query=np.asarray(UsdGeom.Mesh(source).GetPointsAttr().Get())@localT[:3,:3].T+localT[:3,3];assert np.array_equal(np.asarray(UsdGeom.Mesh(query).GetPointsAttr().Get()),expected_query.astype(np.float32))
 if part in last:assert fast.payload_boxes[0][2]is last[part]
 last[part]=fast.payload_boxes[0][2]
 T=arm.bodies(q,width);T[guard.base_path]=arm.base;t=time.perf_counter();fast_results.append(ownB.clear(fast,T));ownTimeB+=time.perf_counter()-t
fast_wall=time.perf_counter()-started;fast_hulls=count[0]
def verdict(v):return(v[0],None if v[1]is None else(v[1]['kind'],v[1]['query'],v[1]['collider'],v[1]['distance_m']))
assert [verdict(v)for v in baseline_results]==[verdict(v)for v in fast_results]
# Real native class CAD changed: full original source/query writes execute,
# changed local content obtains fresh identities rather than an old cache hit.
part='ring2';path='/World/Cell/Geometry/workpiece_ring2';G=cases[0][1];fast.attach_payload(path,G);old=fast.payload_boxes[0][2];mesh=UsdGeom.Mesh(stage.GetPrimAtPath(fast.payload_boxes[0][1]));points=mesh.GetPointsAttr().Get();modified=np.asarray(points).copy();modified[0,0]+=.0001;mesh.GetPointsAttr().Set(modified)
fast.attach_payload(path,G);assert fast.payload_boxes[0][2]is not old;changed_snapshot=interning._snapshot(fast.payload_boxes);original_attach(baseline,path,G);assert changed_snapshot==interning._snapshot(baseline.payload_boxes);mesh.GetPointsAttr().Set(points)
# Poison the cached canonical geometry after creation. Original new geometry
# must survive unchanged, and no existing FCL hull may be paired with poison.
fast.attach_payload(path,G);poisoned=fast.payload_boxes[0][2];poisoned[3][0,0]+=.00001;poison_before=info['stats']['cached_content_poisoned'];fast.attach_payload(path,G);assert fast.payload_boxes[0][2]is not poisoned and info['stats']['cached_content_poisoned']==poison_before+1
fast.attach_payload(path,G);poisoned=fast.payload_boxes[0][2];poisoned[0][0]=np.nan;poison_before=info['stats']['cached_content_poisoned'];fast.attach_payload(path,G);assert fast.payload_boxes[0][2]is not poisoned and info['stats']['cached_content_poisoned']==poison_before+1
assert own_model_fingerprint(guard)==before_hash
result=dict(MODEL_only=True,CPU_only=True,GPU_work=False,physics_steps=0,commands_issued=0,original_attach_AST_and_all_USD_query_writes_executed=True,native_cases=len(cases),different_current_q_and_fresh_G_preserved=True,verdicts_and_distances_bit_equal=True,baseline=dict(wall_s=base_wall,attach_s=attachA,own_FCL_s=ownTimeA,new_FCL_hulls=baseline_hulls),content_intern=dict(wall_s=fast_wall,attach_s=attachB,own_FCL_s=ownTimeB,new_FCL_hulls=fast_hulls),speedup=base_wall/fast_wall,original_protected_model_sha256=before_hash,protected_model_hash_unchanged=True,predictive_original_proxy_function_unchanged=True,changed_actual_native_CAD_uses_original_fresh_content=True,cached_valid_and_nonfinite_poison_both_rejected=True,stats=info['stats'],helper_sha256=hashlib.sha256((O/'payload_proxy_content_intern.py').read_bytes()).hexdigest(),original_checker_source_sha256=hashlib.sha256(original_source.encode()).hexdigest())
(O/'qualification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
