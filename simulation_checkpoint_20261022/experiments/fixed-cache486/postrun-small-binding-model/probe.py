"""Intrinsic original-native model qualification; no application/physics or actors."""
import sys,time,json,types,numpy as np
from pathlib import Path
from pxr import Usd
R=Path('/workspace/project/mujoco');H=R/'isaac_port/native_hand_layout/roas-table';O=R/'.local/co-moving484-model';sys.path[:0]=[str(O),str(H),str(R)]
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
p=np.load(R/'.local/collision-gpu2/binary/small-class-local.npz');ck.payload_boxes=[(ck.payload_path,str(s),proxy(p['v'+str(i)]))for i,s in enumerate(p['sources'])];guard=SameArmClearance(ck);ck.same_arm_clearance=guard

import fcl
from fcl_transport import convex
packet=json.loads(Path('/workspace/project/mujoco/.local/small486-binding-model/public-packet.json').read_text());out=[]
for obs in packet['public_frames']:
 ck.payload_relative=np.asarray(obs['hand_G']);q=np.asarray(packet['actual_encoder_q'])
 for label,width in [('command',packet['command_aperture_m']),('actual',packet['actual_aperture_m'])]:
  Ts=arm.bodies(q,width);Ts[guard.base_path]=arm.base
  own=guard.own_payload_clearance;ok,why=own.clear(ck,Ts)
  rb,rs,rg=next(r for r in own.robot_rows if r[1].endswith('left_left_finger/tip'));pb,ps,pg=next(r for r in ck.payload_boxes if r[1].endswith('ring2_21'))
  A=Ts[rb];B=Ts[arm.hand_path]@ck.payload_relative
  a=fcl.CollisionObject(convex(rg[3]),fcl.Transform(A[:3,:3],A[:3,3]));b=fcl.CollisionObject(convex(pg[3]),fcl.Transform(B[:3,:3],B[:3,3]))
  distance=float(fcl.distance(a,b,fcl.DistanceRequest(),fcl.DistanceResult()))
  out.append(dict(frame=obs['observation_index'],width_basis=label,width_m=width,pair_distance_m=distance,full_own_clear=bool(ok),full_own_rejection=why))
Path('/workspace/project/mujoco/.local/small486-binding-model/qualification.json').write_text(json.dumps(dict(MODEL_only=True,public_RGBD_and_SDK_only=True,physics_steps=0,queries=out),indent=2));print(json.dumps(out),flush=True)

from loaded_contact_aperture import LoadedContactAperture
from receiver_pad_contact import contact_state
from isaac_port.fieldkit.perception import CadPoseEstimator
window=json.loads((R/'.local/small486-binding-model/stable-window.json').read_text());public=packet['public_frames'][0];G=np.asarray(public['TCP_G']);ctx=dict(observed_grasp_clearance=types.SimpleNamespace(estimator=CadPoseEstimator()),config={})
contact=contact_state(ctx,'ring2',G,float(np.asarray(window['samples'])[-1].sum()),contact_tolerance_m=.0008)
binding=LoadedContactAperture(side='left',part='ring2',payload_path=ck.payload_path,encoder_samples=window['samples'],sample_times_s=window['times'],contact=contact,measured_grasp=G,control_dt_s=window['dt'],capture_age_physics_s=.0)
binding.check(packet['actual_fingers_m']);ck.loaded_contact_aperture=binding;ck.payload_relative=np.asarray(packet['public_frames'][-1]['hand_G'])
Ts=arm.bodies(np.asarray(packet['actual_encoder_q']),packet['command_aperture_m']);Ts[guard.base_path]=arm.base
clear,why=guard.own_payload_clearance.clear(ck,Ts)
qualification=json.loads((R/'.local/small486-binding-model/qualification.json').read_text());qualification.update(existing_stable_window_fresh_contact=contact,binding=binding.record,full_conservative_swept_interval_clear=bool(clear),swept_rejection=why,actual_failure_encoders_still_inside_checked_interval=True)
(R/'.local/small486-binding-model/qualification.json').write_text(json.dumps(qualification,indent=2));print(json.dumps(dict(contact_pass=contact['opposed_contact_compatible'],swept_interval_clear=bool(clear),rejection=why)),flush=True)
