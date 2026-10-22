"""Exact nominal CAD meshes for rigid payload geometry checks."""
from pathlib import Path
import json,numpy as np,fcl
from scipy.spatial.transform import Rotation
PORT=Path(__file__).resolve().parent.parent
_CACHE={};_BOUNDS={}
def shape_for(part):
 if part in _CACHE:return _CACHE[part]
 if part.startswith('ring'):
  archive=np.load(PORT/'generated/contact_task/precision-cad.npz');key='instance_0542'if part=='ring1'else'instance_0549';vertices=archive[key+'_vertices'];faces=archive[key+'_faces']
 else:
  source=json.load(open(PORT/'generated/contact_task/contact-source.json'));archive=np.load(PORT/'generated/contact_task/collision.npz');precise=np.load(PORT/'generated/contact_task/precision-pin-main.npz');vs=[];fs=[];offset=0
  for r in source['parts']['pin']['colliders']:
   key=r['mesh']
   if key=='pin_mesh_206':v=precise['instance_0557_vertices'];f=precise['instance_0557_faces']
   else:v=archive[key+'_vertices']@Rotation.from_quat(np.array(r['quaternion'])[[1,2,3,0]]).as_matrix().T+r['position'];f=archive[key+'_faces']
   vs.append(v);fs.append(f+offset);offset+=len(v)
  vertices=np.concatenate(vs);faces=np.concatenate(fs)
 mesh=fcl.BVHModel();mesh.beginModel(len(vertices),len(faces));mesh.addSubModel(vertices,faces);mesh.endModel();_CACHE[part]=mesh;_BOUNDS[part]=(vertices.min(0),vertices.max(0));return mesh

def bounds_for(part):
 shape_for(part);return _BOUNDS[part]
