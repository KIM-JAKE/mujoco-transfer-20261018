from pathlib import Path
from types import SimpleNamespace,ModuleType
import sys,ast,json,numpy as np,hashlib,fcl
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
R=Path('/home/gpuadmin/workspace/jake/mujoco');P=next((R/'isaac_port/runs').glob('*v530*'));H=P/'code_snapshot';O=Path('/tmp/fixed531-small-width-proof-expansion');sys.path.insert(0,str(H))
# Literal original FCL convex constructor, without unrelated USD module imports.
node=next(n for n in ast.parse((H/'fcl_transport.py').read_text()).body if isinstance(n,ast.FunctionDef)and n.name=='convex');mod=ModuleType('fcl_transport');mod.__dict__.update(np=np,ConvexHull=ConvexHull,fcl=fcl);exec(compile(ast.Module(body=[node],type_ignores=[]),'<original convex>','exec'),mod.__dict__);sys.modules['fcl_transport']=mod
from predictive_pair_clearance import proxy
from own_payload_clearance import OwnPayloadClearance
from loaded_contact_aperture import LoadedContactAperture
r=json.load(open(P/'pipeline-results.json'));b=next(v for v in r['planning_diagnostics']if v.get('event')=='small_loaded_receiver_encoder_geometry_bound');d=np.load(P/'executed-actuator-commands.npz');local=np.load(R/'.local/render-current/pair432-exact-locals.npz');model=json.load(open(R/'.local/fixed-own-model450/left-native-kinematics.json'));model={k:np.array(v)for k,v in model.items()if k in ['base','tip','frames','axes','bounds']}
rows=[(str(body),str(s),proxy(local['v'+str(i)]))for i,(body,s)in enumerate(zip(local['bodies'],local['sources']))if'/left_arm_base/'in str(body)];hand=next(x[0]for x in rows if x[0].endswith('_NativeHand'));links=[next(x[0]for x in rows if x[0].endswith('_link'+str(i)))for i in range(1,8)];base=next(x[0]for x in rows if x[0].endswith('_link0'));arm=SimpleNamespace(hand_path=hand);guard=SimpleNamespace(rows=rows,arm=arm,margin_m=.0005);own=OwnPayloadClearance(guard)
def frame(P,R):T=np.eye(4);T[:3,:3]=R;T[:3,3]=P;return T
def bodies(q,w):
 T=model['base'].copy();out={base:T.copy()}
 for path,x,a,(pre,post)in zip(links,q,model['axes'],model['frames']):T=T@pre@frame([0,0,0],Rotation.from_rotvec(a*x).as_matrix())@post;out[path]=T.copy()
 H=T@model['tip']@frame([0,0,-.1034],np.eye(3));out[hand]=H
 for sign,label in [(1,'left'),(-1,'right')]:out[hand+'/left_'+label+'_finger']=H@frame([0,sign*w/2,.0584],np.eye(3)if sign==1 else np.diag([-1.,-1.,1.]))
 return out
cl=np.load(R/'.local/collision-gpu2/binary/small-class-local.npz');path='/World/Cell/Geometry/workpiece_ring2';payload=[(path,str(s),proxy(cl['v'+str(i)]))for i,s in enumerate(cl['sources'])];G=frame([0,0,.1034],np.eye(3))@np.array(b['measured_grasp']);ck=SimpleNamespace(payload_path=path,payload_boxes=payload,payload_relative=G,arm=arm)
ids=[r['dof_names'].index('left_arm_fr3v2_joint'+str(i))for i in range(1,8)];fi=[i for i,n in enumerate(r['dof_names'])if n.startswith('left_native_')];start=int(round(b['sample_times_s'][-1]*480));qrows=np.vstack([d['command'][start:,ids],d['actual_dof_position'][start:,ids]]);qrows=np.unique(qrows,axis=0);qrows=qrows[np.linspace(0,len(qrows)-1,min(15,len(qrows)),dtype=int)]
results=[]
for interval in [.00025,.0005]:
 bind=LoadedContactAperture(side='left',part='ring2',payload_path=path,encoder_samples=b['measured_finger_samples_m'],sample_times_s=b['sample_times_s'],contact=b['fresh_bilateral_CAD_contact'],measured_grasp=b['measured_grasp'],control_dt_s=1/480,capture_age_physics_s=b['capture_age_physics_s'],half_interval_m=interval);ck.loaded_contact_aperture=bind
 trials=[]
 for q in qrows:
  ok,why=own.clear(ck,bodies(q,float(d['actual_dof_position'][-1,fi].sum())));trials.append(dict(clear=bool(ok),rejection=why))
 actual=d['actual_dof_position'][-1,fi];bound_ok=True
 try:bind.check(actual)
 except RuntimeError:bound_ok=False
 results.append(dict(half_interval_m=interval,full_original_own_native_vs_48_class_CAD_all_probed_q_clear=all(x['clear']for x in trials),q_samples=len(trials),failures=[x for x in trials if not x['clear']],actual_failed530_next_tick_now_inside=bound_ok,lower=bind.lower.tolist(),upper=bind.upper.tolist(),same_original_1mm_required_clearance=True))
result=dict(MODEL_ONLY=True,scope='Original full native palm/proximal fingers/arm/base vs every SMALL class CAD proxy. Conservative exact endpoint-hull swept finger interval, unchanged original own guard/FCL1mm criterion; no physical or scene-wide qualification claimed. Original full live motion gates remain mandatory.',source=P.name,results=results,actual_failed530_fingers=actual.tolist(),native_geometry_sha256=hashlib.sha256((R/'.local/render-current/pair432-exact-locals.npz').read_bytes()).hexdigest(),class_geometry_sha256=hashlib.sha256((R/'.local/collision-gpu2/binary/small-class-local.npz').read_bytes()).hexdigest(),original_loaded_helper_sha256=hashlib.sha256((H/'loaded_contact_aperture.py').read_bytes()).hexdigest());(O/'native-wide-model-proof.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
