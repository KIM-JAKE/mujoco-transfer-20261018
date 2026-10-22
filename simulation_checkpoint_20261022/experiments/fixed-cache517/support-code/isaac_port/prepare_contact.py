"""Export exact source collision geometry and control samples for PhysX task development."""
from pathlib import Path
import json,hashlib,sys
import numpy as np
import mujoco
from prepare import HERE,SOURCES

DUO=SOURCES['cap_button'].parent.parent
OUT=HERE/'generated/contact_task';OUT.mkdir(parents=True,exist_ok=True)
RUN=DUO/'cap_button_parallel_alt_20261016'
model=mujoco.MjModel.from_xml_path(str(SOURCES['cap_button']));data=mujoco.MjData(model);mujoco.mj_forward(model,data)
arrays={};geoms=[]
for i in range(model.ngeom):
    g=model.geom(i);body=model.body(int(g.bodyid[0])).name;kind=int(g.type[0])
    # Preserve actual static CAD and source robot collision geometry. Dynamic
    # ring collisions are exported below as CAD sectors so their bores stay open.
    if body.startswith('workpiece_'):continue
    if body.startswith('cap_stock_ring'):continue
    active=(body=='m80' or body=='cap_supply_fixture' or body.startswith('cap_stock_pin') or bool(g.contype[0]|g.conaffinity[0]))
    if not active:continue
    static=body in ['m80','cap_supply_fixture','world'] or body.startswith('cap_stock_')
    row=dict(name=g.name,body=body,kind=kind,static=static,position=(data.geom_xpos[i] if static else g.pos).tolist(),quaternion=(data.geom_xmat[i] if False else g.quat).tolist(),size=g.size.tolist(),source_group=int(g.group[0]))
    if static:
        quat=np.empty(4);mujoco.mju_mat2Quat(quat,data.geom_xmat[i]);row['quaternion']=quat.tolist()
    if kind==7:
        j=int(g.dataid[0]);key='mesh_'+str(j)
        if key+'_vertices' not in arrays:
            va,vc=model.mesh_vertadr[j],model.mesh_vertnum[j];fa,fc=model.mesh_faceadr[j],model.mesh_facenum[j]
            arrays[key+'_vertices']=model.mesh_vert[va:va+vc].copy();arrays[key+'_faces']=model.mesh_face[fa:fa+fc].copy()
        row['mesh']=key
    geoms.append(row)
cm=json.loads((DUO/'ring_test_1m_20261013/contact_model.json').read_text());parts={}
for name in ['ring1','ring2','pin']:
    b=model.body('workpiece_'+name);colliders=[]
    if name!='pin':
        entry=next(r for r in cm['rows'] if r['name']==name)
        for k in range(64):
            path=DUO/'ring_test_1m_20261013/contact_meshes'/f'{name}_{k:02d}.obj'
            points=[];faces=[]
            for line in path.read_text().splitlines():
                if line.startswith('v '):points.append([float(x)for x in line.split()[1:]])
                if line.startswith('f '):faces.append([int(x.split('/')[0])-1 for x in line.split()[1:]])
            key=f'{name}_sector_{k}';arrays[key+'_vertices']=np.array(points);arrays[key+'_faces']=np.array(faces)
            colliders.append(dict(name=key,kind=7,mesh=key,position=[0,0,0],quaternion=[1,0,0,0]))
        mass=entry['mass_kg'];inertia=entry['inertia_kg_m2']
    else:
        volume=0
        for i in range(int(b.geomadr[0]),int(b.geomadr[0]+b.geomnum[0])):
            g=model.geom(i);j=int(g.dataid[0]);key=f'pin_mesh_{j}'
            va,vc=model.mesh_vertadr[j],model.mesh_vertnum[j];fa,fc=model.mesh_faceadr[j],model.mesh_facenum[j]
            v=model.mesh_vert[va:va+vc].copy();f=model.mesh_face[fa:fa+fc].copy()
            arrays[key+'_vertices']=v;arrays[key+'_faces']=f
            volume+=abs(np.einsum('ij,ij->i',v[f[:,0]],np.cross(v[f[:,1]],v[f[:,2]])).sum()/6)
            colliders.append(dict(name=key,kind=7,mesh=key,position=g.pos.tolist(),quaternion=g.quat.tolist()))
        mass=float(volume*7800);inertia=[mass*.035**2/12,mass*.035**2/12,mass*.0065**2/2]
    parts[name]=dict(mass=mass,inertia=inertia,position=b.pos.tolist(),quaternion=b.quat.tolist(),colliders=colliders)
# Use the same sector geometry for each fixed lower stock ring, preserving support.
for name in ['ring1','ring2']:
    for b in [model.body(i) for i in range(model.nbody) if model.body(i).name.startswith('cap_stock_'+name)]:
        for shape in parts[name]['colliders']:
            R=data.xmat[b.id].reshape(3,3);q=np.empty(4);mujoco.mju_mulQuat(q,data.xquat[b.id],np.array(shape['quaternion']))
            geoms.append(dict(shape,name=b.name+'_'+shape['name'],body=b.name,static=True,position=(data.xpos[b.id]+R@shape['position']).tolist(),quaternion=q.tolist()))
connect=[]
for i in range(model.neq):
    if model.eq_type[i]==mujoco.mjtEq.mjEQ_CONNECT:
        connect.append(dict(body0=model.body(int(model.eq_obj1id[i])).name,body1=model.body(int(model.eq_obj2id[i])).name,anchor0=model.eq_data[i,:3].tolist(),anchor1=model.eq_data[i,3:6].tolist()))
meta=dict(source_scene=str(SOURCES['cap_button']),source_scene_sha256=hashlib.sha256(SOURCES['cap_button'].read_bytes()).hexdigest(),
          geoms=geoms,parts=parts,connect=connect,body_names=[model.body(i).name for i in range(model.nbody)],joints=[dict(name=model.joint(i).name,qpos_address=int(model.jnt_qposadr[i])) for i in range(model.njnt)],
          bodies=[dict(name=model.body(i).name,mass=float(model.body_mass[i]),inertia=model.body_inertia[i].tolist(),com=model.body_ipos[i].tolist(),principal=model.body_iquat[i].tolist())for i in range(1,model.nbody)])
np.savez_compressed(OUT/'collision.npz',**arrays)
(OUT/'contact-source.json').write_text(json.dumps(meta,indent=2)+'\n')
print(json.dumps(dict(colliders=len(geoms),meshes=len(arrays)//2,parts={n:p['mass']for n,p in parts.items()},connect=len(connect))))
