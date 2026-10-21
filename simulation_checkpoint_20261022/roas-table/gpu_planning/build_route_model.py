"""Build approximate proposal-only spheres; final exact checker remains mandatory."""
from pathlib import Path
import hashlib,json
import numpy as np,trimesh
from scipy.cluster.vq import kmeans2
from scipy.spatial.distance import cdist
from service_paths import ROOT,SOURCE,URDF
import os
source=Path(os.environ['M80_GPU_COLLISION_NPZ']);data=np.load(source);spheres={}
def fit(index,n):
 from curobo.sphere_fit import fit_spheres_to_mesh,SphereFitType
 mesh=trimesh.Trimesh(vertices=data[f'v{index}'],faces=data[f'f{index}'],process=False)
 result=fit_spheres_to_mesh(mesh,num_spheres=n,fit_type=SphereFitType.MORPHIT,iterations=80,compute_metrics=False)
 C=result.centers.cpu().numpy();R=result.radii.cpu().numpy();rows=[dict(center=c.tolist(),radius=float(r)+.0005)for c,r in zip(C,R)]
 print('FITTED_LINK_PROPOSAL',index,len(rows),flush=True);return rows
for body,index in [(i,24-i)for i in range(1,8)]:spheres[f'body{body}']=fit(index,32)
palm=fit(14,32)
for s in palm:s['center']=(np.array(s['center'])+[0,0,-.1034]).tolist()
left=fit(15,12);right=fit(16,12);tcp=palm.copy()
for sign,rows in [(1,left),(-1,right)]:
 for s in rows:
  flip=np.array([sign,sign,1.]);tcp.append(dict(center=(np.array(s['center'])*flip+[0,sign*.04,-.045]).tolist(),radius=s['radius']))
tcp.append(dict(center=[0,0,0],radius=-1.));spheres['tcp']=tcp
robot=dict(robot_cfg=dict(kinematics=dict(urdf_path=str(URDF),base_link='world',tool_frames=['tcp'],collision_link_names=list(spheres),collision_spheres=spheres,collision_sphere_buffer=0.,self_collision_ignore={k:[]for k in spheres},self_collision_buffer={k:0. for k in spheres},cspace=dict(joint_names=[f'joint{i+1}'for i in range(7)],default_joint_position=json.loads(SOURCE.read_text())['seed'],null_space_weight=[1.]*7,cspace_distance_weight=[1.]*7,max_acceleration=9.5,max_jerk=4750.))))
keys=list(spheres)
for i,k in enumerate(keys):
 for j,l in enumerate(keys):
  if abs(i-j)<=2:robot['robot_cfg']['kinematics']['self_collision_ignore'][k].append(l)
(ROOT/'route-robot.json').write_text(json.dumps(robot,indent=2));(ROOT/'native-route-spheres.json').write_text(json.dumps(dict(palm=palm,left=left,right=right),indent=2));fixed=ROOT/'fixed-CAD.ply'
if not fixed.exists():trimesh.Trimesh(vertices=data['static_vertices'],faces=data['static_faces'],process=False).export(fixed)
manifest=dict(scope='Optional approximate MORPHIT sphere proposals; original full Isaac/PhysX fixed/pair/payload gates remain required',source=source.name,spheres_per_link={k:len(v)for k,v in spheres.items()},static_triangles=int(data['static_faces'].shape[0]),current_native_hand=True,uses_Panda_or_Robotiq=False,fixed_world_sha256=hashlib.sha256(fixed.read_bytes()).hexdigest());(ROOT/'route-model-manifest.json').write_text(json.dumps(manifest,indent=2));print(json.dumps(manifest,indent=2))
