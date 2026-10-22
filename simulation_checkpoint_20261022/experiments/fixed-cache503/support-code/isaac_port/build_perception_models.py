"""Build separate fitting and validation samples from original CAD surfaces."""
from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
root=Path(__file__).resolve().parent;src=json.loads((root/'generated/contact_task/contact-source.json').read_text());arr=np.load(root/'generated/contact_task/collision.npz');precise=np.load(root/'generated/contact_task/precision-cad.npz');pin=np.load(root/'generated/contact_task/precision-pin-main.npz');rng=np.random.default_rng(1847);data={};validation={};meta={}
for name in ['ring1','ring2','pin']:
 if name.startswith('ring'):
  key='instance_0542'if name=='ring1'else'instance_0549';v=precise[key+'_vertices'];f=precise[key+'_faces']
 else:
  vv=[];ff=[];offset=0
  for row in src['parts'][name]['colliders']:
   if row['mesh']=='pin_mesh_206':v=pin['instance_0557_vertices'];f=pin['instance_0557_faces']
   else:
    R=Rotation.from_quat(np.array(row['quaternion'])[[1,2,3,0]]).as_matrix();v=arr[row['mesh']+'_vertices']@R.T+row['position'];f=arr[row['mesh']+'_faces']
   vv.append(v);ff.append(f+offset);offset+=len(v)
  v=np.vstack(vv);f=np.vstack(ff)
 tri=v[f];normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);twice=np.linalg.norm(normal,axis=1);areas=twice/2;chosen=rng.choice(len(f),size=12000,p=areas/areas.sum());b=rng.random((12000,2));fold=b.sum(axis=1)>1;b[fold]=1-b[fold];points=tri[chosen,0]+b[:,0,None]*(tri[chosen,1]-tri[chosen,0])+b[:,1,None]*(tri[chosen,2]-tri[chosen,0]);normals=normal[chosen]/np.maximum(twice[chosen,None],1e-15);data[name+'_points']=points;data[name+'_normals']=normals;chosen2=rng.choice(len(f),size=8000,p=areas/areas.sum());uv=rng.random((8000,2));flip=uv.sum(axis=1)>1;uv[flip]=1-uv[flip];validation[name+'_points']=tri[chosen2,0]+uv[:,0,None]*(tri[chosen2,1]-tri[chosen2,0])+uv[:,1,None]*(tri[chosen2,2]-tri[chosen2,0]);validation[name+'_normals']=normal[chosen2]/np.maximum(twice[chosen2,None],1e-15);meta[name]=dict(nominal_position=src['parts'][name]['position'],nominal_quaternion_wxyz=src['parts'][name]['quaternion'],axial_symmetry=name.startswith('ring'),bbox_local_m=[v.min(axis=0).tolist(),v.max(axis=0).tolist()])
np.savez_compressed(root/'fieldkit/assets/part-models.npz',**data);np.savez_compressed(root/'fieldkit/assets/part-validation-surfaces.npz',**validation);(root/'fieldkit/assets/part-models.json').write_text(json.dumps(dict(units='m',coordinate_frame='original_part_body',nominal_poses_scope='simulation examples only; not site calibration',parts=meta),indent=2)+'\n')
print(json.dumps({n:r['bbox_local_m']for n,r in meta.items()}))
