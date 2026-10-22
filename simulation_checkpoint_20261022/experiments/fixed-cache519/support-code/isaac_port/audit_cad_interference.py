"""Independent triangle-intersection audit of original and corrected rail layout.

Run with the restored host .venv (python-fcl), not the Isaac runtime.
"""
from pathlib import Path
import json,hashlib
import numpy as np
import fcl
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent

def main():
 s=json.loads((ROOT/'generated/contact_task/contact-source.json').read_text());arrays=np.load(ROOT/'generated/contact_task/collision.npz');fine=np.load(ROOT/'generated/contact_task/precision-cad.npz');pin_fine=np.load(ROOT/'generated/contact_task/precision-pin-main.npz');cache={}
 def mesh(key,store):
  if key not in cache:
   v=store[key+'_vertices'];f=store[key+'_faces'];m=fcl.BVHModel();m.beginModel(len(v),len(f));m.addSubModel(v,f);m.endModel();cache[key]=m
  return cache[key]
 cfg=json.loads((ROOT.parent/'keti_m80_mujoco_20261013/duo_fr3_2f85_20261013/cap_button_parallel_alt_20261016/config.json').read_text());result={}
 for name,i,quat in [('ring1',542,[1,0,0,0]),('ring2',549,[.8660254037844386,0,0,.5])]:
  pos=next(p['goal_origin_m']for p in cfg['parts']if p['name']==name);part=fcl.CollisionObject(mesh(f'instance_{i:04d}',fine),fcl.Transform(Rotation.from_quat(np.array(quat)[[1,2,3,0]]).as_matrix(),np.array(pos)))
  hits={mode:[]for mode in ['original','rail_corrected']}
  for row in s['geoms']:
   if row['body']!='m80' or row['kind']!=7:continue
   shape=mesh(row['mesh'],arrays);r=Rotation.from_quat(np.array(row['quaternion'])[[1,2,3,0]]).as_matrix()
   for mode in hits:
    pp=np.array(row['position']).copy()
    if mode=='rail_corrected':
     if row['name']in ['instance_0369','instance_0380']:pp[1]+=.012
     if row['name']in ['instance_0373','instance_0382']:pp[1]-=.030
     if row['name']in ['instance_0371','instance_0384']:pp[1]-=.060
    other=fcl.CollisionObject(shape,fcl.Transform(r,pp));response=fcl.CollisionResult()
    if fcl.collide(part,other,fcl.CollisionRequest(num_max_contacts=1),response):hits[mode].append(row['name'])
  result[name]=hits;print(name,hits,flush=True)
 # All four original pin components, including their local CAD transforms.
 pin_pose=next(p['goal_origin_m']for p in cfg['parts']if p['name']=='pin')
 pin_objects=[]
 for c in s['parts']['pin']['colliders']:
  r=Rotation.from_quat(np.array(c['quaternion'])[[1,2,3,0]]).as_matrix()
  if c['name']=='pin_mesh_206':pin_objects.append(fcl.CollisionObject(mesh('instance_0557',pin_fine),fcl.Transform(np.eye(3),np.array(pin_pose))))
  else:pin_objects.append(fcl.CollisionObject(mesh(c['mesh'],arrays),fcl.Transform(r,np.array(pin_pose)+c['position'])))
 hits={mode:[]for mode in ['original','rail_corrected']}
 for row in s['geoms']:
  if row['body']!='m80'or row['kind']!=7:continue
  precise=row['name']=='instance_0554'
  shape=mesh('instance_0554',pin_fine)if precise else mesh(row['mesh'],arrays);r=np.eye(3)if precise else Rotation.from_quat(np.array(row['quaternion'])[[1,2,3,0]]).as_matrix()
  for mode in hits:
   pp=np.zeros(3)if precise else np.array(row['position']).copy()
   if mode=='rail_corrected':
    if row['name']in ['instance_0369','instance_0380']:pp[1]+=.012
    if row['name']in ['instance_0373','instance_0382']:pp[1]-=.030
    if row['name']in ['instance_0371','instance_0384']:pp[1]-=.060
   other=fcl.CollisionObject(shape,fcl.Transform(r,pp))
   if any(fcl.collide(part,other,fcl.CollisionRequest(),fcl.CollisionResult())for part in pin_objects):hits[mode].append(row['name'])
 result['pin']=hits;print('pin',hits,flush=True)
 report=dict(method='FCL triangle mesh intersection at nominal final pose',original_cad_assembly_pass=False,corrected_all_part_targets_clear=all(not row['rail_corrected']for row in result.values()),parts=result,mesh_sha256={n:hashlib.sha256((ROOT/'generated/contact_task'/n).read_bytes()).hexdigest()for n in ['precision-cad.npz','collision.npz']},scope='No tolerance, actuator, robot sweep or hardware certification')
 (ROOT/'generated/contact_task/cad-clearance-audit.json').write_text(json.dumps(report,indent=2)+'\n')
 return report
if __name__=='__main__':main()
