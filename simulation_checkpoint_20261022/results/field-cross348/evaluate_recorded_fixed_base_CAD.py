from pathlib import Path
import json,numpy as np,fcl
from scipy.spatial.transform import Rotation
R=Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/runs/native-roas-table-scatter-stable-v348-gpu-carry-no-greedy-final-retreat-all3-446');z=np.load(R/'camera-local-models.npz');t=np.load(R/'physx-sequence-trace.npz');objects=[];details=[]
for side in ['right','left']:
 path=next(p for p in z['paths']if p.endswith('/'+side+'_arm_fr3v2_link0'));mi=z['paths'].tolist().index(path);bi=t['camera_body_paths'].tolist().index(path);p=t['camera_bodies'][0,bi].astype(float);p[3:]/=np.linalg.norm(p[3:]);verts=z['v'+str(mi)].astype(float);faces=z['f'+str(mi)].astype(int);m=fcl.BVHModel();m.beginModel(len(verts),len(faces));m.addSubModel(verts,faces);m.endModel();obj=fcl.CollisionObject(m,fcl.Transform(Rotation.from_quat(p[3:]).as_matrix(),p[:3]));objects.append(obj);details.append(dict(side=side,body_path=str(path),vertices=len(verts),faces=len(faces),fixed_pose=p.tolist()))
req=fcl.DistanceRequest(enable_nearest_points=True,enable_signed_distance=False);res=fcl.DistanceResult();dist=fcl.distance(*objects,req,res);cr=fcl.CollisionResult();count=fcl.collide(*objects,fcl.CollisionRequest(num_max_contacts=10,enable_contact=True),cr)
r=dict(scope='Original recorded Isaac camera collision meshes and fixed body poses, diagnostic evaluation only; no runtime exception/config change',distance_m=dist,nearest_points=[v.tolist()for v in res.nearest_points],triangle_contacts=count,colliding=cr.is_collision,models=details)
Path('/tmp/field-cross348/base-CAD.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
