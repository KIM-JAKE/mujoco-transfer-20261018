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

G=np.eye(4);G[0,3]=.03;ck=types.SimpleNamespace(arm=types.SimpleNamespace(hand_path=hand),payload_path=body,payload_relative=G);memo=Memo(ck);H=np.eye(4)
for _ in range(2):
 robot[0].setTransform(fcl.Transform(H[:3,:3],H[:3,3]));objects[0].setTransform(fcl.Transform(G[:3,:3],G[:3,3]));memo.clear_rows(ck,H,rows,[H],robot,payload,objects,.061,.0005)
far=np.eye(4);far[0,3]=2.;two=rows+[('/World/test/nonco','nonco',shape)]
assert memo.fast_clear(ck,H,two,[H,far],payload,.061,.0005)
bad=far.copy();bad[0,0]=float('nan');assert not memo.fast_clear(ck,H,two,[H,bad],payload,.061,.0005)
invalid=list(shape);invalid[1]=np.asarray(shape[1]).copy();invalid[1][0,0]=float('nan');malformed=rows+[('/World/test/nonco','nonco',tuple(invalid))];assert not memo.fast_clear(ck,H,malformed,[H,far],payload,.061,.0005)
entry=next(v for v in memo.entries.values()if v is not None);original=entry['world_radius_sum'];entry['world_radius_sum']=float('nan');assert not memo.clear_rows(ck,H,rows,[H],robot,payload,objects,.061,.0005);entry['world_radius_sum']=original
entry['relative'][0,0,0]=float('nan');assert not memo.clear_rows(ck,H,rows,[H],robot,payload,objects,.061,.0005)
result=dict(MODEL_only=True,NaN_nonco_transform_falls_back=True,NaN_proxy_axes_falls_back=True,NaN_cached_radius_falls_back=True,NaN_relative_error_falls_back=True,physical_commands_issued=0,physics_steps=0)
(O/'malformed-qualification.json').write_text(json.dumps(result,indent=2)+'\n');print('MALFORMED_PASS',result)
