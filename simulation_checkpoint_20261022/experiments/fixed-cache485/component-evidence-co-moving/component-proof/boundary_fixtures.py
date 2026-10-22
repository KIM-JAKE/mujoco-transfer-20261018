"""Original FCL near1mm differential fixtures; no simulator or actuation."""
import sys,json,types,numpy as np
from pathlib import Path
R=Path('/workspace/project/mujoco');O=R/'.local/co-moving484-model';sys.path[:0]=[str(O),str(R/'isaac_port/native_hand_layout/roas-table')]
import fcl
from co_moving_own_clearance import Memo
from predictive_pair_clearance import proxy
from fcl_transport import convex
cube=np.array([[x,y,z]for x in [-.01,.01]for y in [-.01,.01]for z in [-.01,.01]])
hand='/World/test/left_NativeHand';body='/World/test/pin';shape=proxy(cube);rows=[(hand,'native_Hand',shape)];payload=[(body,'classCAD',shape)];robot=[fcl.CollisionObject(convex(cube))];objects=[fcl.CollisionObject(convex(cube))];cases=[]
for delta in [-2e-6,-1e-9,0.,.5e-9,.5e-6,1.01e-6,2e-6,10e-6]:
 G=np.eye(4);G[0,3]=.021+delta;ck=types.SimpleNamespace(arm=types.SimpleNamespace(hand_path=hand),payload_path=body,payload_relative=G);memo=Memo(ck)
 hs=[]
 for k in range(5):
  t=.6*k;c,s=np.cos(t),np.sin(t);H=np.eye(4);H[:3,:3]=np.array([[c,-s,0],[s,c,0],[0,0,1.]])@np.diag([1+k*5e-8,1-k*3e-8,1.]);H[:3,3]=[k*.5,-k*.3,k*.2];hs.append(H)
 verdicts=[]
 for H in [hs[0],hs[0],*hs]:
  P=H@G;robot[0].setTransform(fcl.Transform(H[:3,:3],H[:3,3]));objects[0].setTransform(fcl.Transform(P[:3,:3],P[:3,3]));distance=float(fcl.distance(robot[0],objects[0],fcl.DistanceRequest(),fcl.DistanceResult()));ids=memo.clear_rows(ck,H,rows,[H],robot,payload,objects,.061,.0005)
  assert not ids or distance>.001,dict(delta=delta,distance=distance,stats=memo.stats)
  verdicts.append(dict(original_distance_m=distance,original_clear=distance>.001,memo_clear=bool(ids)))
 if delta<=1e-6:assert not any(v['memo_clear']for v in verdicts)
 cases.append(dict(separation_above_original_1mm_m=delta,verdicts=verdicts,stats=memo.stats))
# Proxy replacement after an accepted key cannot reuse old geometry IDs.
G=np.eye(4);G[0,3]=.03;ck.payload_relative=G;memo=Memo(ck);H=np.eye(4)
for _ in range(2):
 robot[0].setTransform(fcl.Transform(H[:3,:3],H[:3,3]));objects[0].setTransform(fcl.Transform(G[:3,:3],G[:3,3]));memo.clear_rows(ck,H,rows,[H],robot,payload,objects,.061,.0005)
replacement=[(body,'classCAD',proxy(cube*1.5))];assert not memo.clear_rows(ck,H,rows,[H],robot,replacement,objects,.061,.0005)
result=dict(MODEL_only=True,original_threshold_m=.001,extra_numerical_conservatism_m=1e-6,fixtures=len(cases),zero_false_clear=True,geometry_replacement_misses=True,physical_commands_issued=0,physics_steps=0,cases=cases)
(O/'boundary-qualification.json').write_text(json.dumps(result,indent=2)+'\n');print('BOUNDARY_PASS',len(cases),'zero falseCLEAR; exact replacement miss')
