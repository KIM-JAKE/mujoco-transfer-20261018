"""Approximate current-native sphere proposal with boundary witnesses."""
from pathlib import Path
import numpy as np,json,trimesh
from curobo.sphere_fit import fit_spheres_to_mesh,SphereFitType
from service_paths import ROOT as root
import os
d=np.load(os.environ['M80_GPU_COLLISION_NPZ'])
def fitted(index,count,witnesses):
 v=d[f'v{index}'];f=d[f'f{index}'];mesh=trimesh.Trimesh(vertices=v,faces=f,process=False);r=fit_spheres_to_mesh(mesh,num_spheres=count,fit_type=SphereFitType.MORPHIT,iterations=80,compute_metrics=False);centers=r.centers.cpu().numpy();radii=r.radii.cpu().numpy()+.0005;points=np.unique(np.vstack([v,v[f].mean(1)]),axis=0);dist=np.min(np.linalg.norm(points[:,None]-centers[None],axis=-1)-radii[None],axis=1);extra=[]
 for _ in range(witnesses):
  i=int(np.argmax(dist));p=points[i]
  if dist[i]<=0:break
  extra.append(dict(center=p.tolist(),radius=.003));dist=np.minimum(dist,np.linalg.norm(points-p,axis=1)-.003)
 return [dict(center=p.tolist(),radius=float(r))for p,r in zip(centers,radii)]+extra
native=dict(palm=fitted(14,32,48),left=fitted(15,12,12),right=fitted(16,12,12))
for s in native['palm']:s['center'][2]-=.1034
(root/'native-route-spheres.json').write_text(json.dumps(native,indent=2));robot=json.loads((root/'route-robot.json').read_text());rows=native['palm'].copy()
for sign,key in[(1,'left'),(-1,'right')]:
 for s in native[key]:rows.append(dict(center=(np.array(s['center'])*[sign,sign,1]+[0,sign*.04,-.045]).tolist(),radius=s['radius']))
rows.append(dict(center=[0,0,0],radius=-1.));robot['robot_cfg']['kinematics']['collision_spheres']['tcp']=rows;(root/'route-robot.json').write_text(json.dumps(robot,indent=2));print(json.dumps(dict(native_counts={k:len(v)for k,v in native.items()},tcp_spheres=len(rows),scope='Approximate fit plus3mm boundary witness spheres, current native geometry; fulloriginalchecker mandatory')))
