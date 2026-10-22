"""Contact-only lift test on the archived supply, without arm or insertion.

Real CAD, free workpiece, force-limited native fingers. The palm follows a
mocap target; no full-cell dynamic assembly or task time is measured here.
"""
from pathlib import Path
import json,copy,xml.etree.ElementTree as E
import numpy as np,mujoco,trimesh
from scipy.spatial.transform import Rotation
from grasp_bench import build,vec,quat

HERE=Path(__file__).resolve().parent;PORT=HERE.parent;OUT=HERE/'original-9s-native-table'
source=json.load(open(PORT/'generated/contact_task/contact-source.json'))
arrays=np.load(PORT/'generated/contact_task/collision.npz')

def add_mesh(asset,world,out,name,v,f,P,R,tube_axis=None):
    groups=[f]
    if tube_axis is not None:
        a,b=[i for i in range(3)if i!=tube_axis];c=v[f].mean(1)
        bins=np.floor((np.arctan2(c[:,b],c[:,a])+np.pi)/(2*np.pi)*64).astype(int)%64
        groups=[f[bins==i]for i in range(64)if np.any(bins==i)]
    for k,faces in enumerate(groups):
        points=v[np.unique(faces)]
        try:h=trimesh.convex.convex_hull(points)
        except Exception:continue
        p=out/(name+'-'+str(k)+'.obj');h.export(p)
        key=name+'_'+str(k);E.SubElement(asset,'mesh',name=key,file=str(p))
        E.SubElement(world,'geom',name=key,type='mesh',mesh=key,pos=vec(P),quat=quat(R),contype='4',conaffinity='3',rgba='.1 .6 .55 1')

def run(part,recipe):
    out=OUT/('contact-'+part);out.mkdir(exist_ok=True)
    tcp=np.array(recipe['tcp']);R=np.array(recipe['R']);lp=np.array(recipe['localP']);L=np.array(recipe['localR'])
    path,P,Rp=build(part,tcp,R,lp,L,recipe['gap'],out,holder_specs=[])
    root=E.parse(path).getroot();world=root.find('worldbody');asset=root.find('asset');act=root.find('actuator')
    root.find('option').set('timestep','.00005' if part!='pin' else '.000025')
    for b in list(world):
        if b.tag=='body'and b.get('name','').startswith('clip_'):world.remove(b)
    for a in list(act):
        if a.get('joint','').startswith('clip_'):act.remove(a)
    for row in source['geoms']:
        if row['body']!='cap_supply_fixture' and not row['body'].startswith('cap_stock_'):continue
        name=row['name'].replace('-','minus_');rot=Rotation.from_quat(np.array(row['quaternion'])[[1,2,3,0]]).as_matrix()
        if row['kind']==7:
            v=arrays[row['mesh']+'_vertices'];f=arrays[row['mesh']+'_faces']
            add_mesh(asset,world,out,name,v,f,row['position'],rot,0 if row['name'].startswith('cap_supply_pin_socket')else None)
        else:
            kind='box' if row['kind']==6 else 'cylinder'
            size=row['size'] if kind=='box'else row['size'][:2]
            E.SubElement(world,'geom',name=name,type=kind,size=vec(size),pos=vec(row['position']),quat=quat(rot),contype='4',conaffinity='3',rgba='.1 .6 .55 1')
    E.SubElement(world,'geom',name='original_pin_socket_floor',type='box',pos='-.555 -.16 1.28025',size='.014 .014 .002',contype='4',conaffinity='3')
    from stock_fixture_geometry import pin_stock_sleeve
    v,f,center=pin_stock_sleeve();add_mesh(asset,world,out,'original_tip_sleeve',v,f,center,np.eye(3),2)
    E.ElementTree(root).write(path,encoding='unicode')
    m=mujoco.MjModel.from_xml_path(str(path));d=mujoco.MjData(m)
    mid=m.body('hand_target').mocapid[0];palm=m.joint('hand_root').qposadr[0];obj=m.body('part').id
    fi=[m.joint(x).qposadr[0]for x in ['left','right']];origin=tcp-R@np.array([0,0,.1034]);pre=np.array([0,0,.04])
    d.qpos[palm:palm+3]=origin+pre;d.qpos[fi]=.04 if part!='pin'else .01;d.mocap_pos[mid]=origin+pre
    d.ctrl[:]=0;mujoco.mj_forward(m,d)
    dt=.001;substeps=round(dt/m.opt.timestep);contacts=[];bad=[];states=[];before=None
    def smooth(u):u=np.clip(u,0,1);return u**3*(10-15*u+6*u*u)
    for k in range(2200):
        t=k*dt
        approach=pre*(1-smooth((t-.3)/.3));lift=.04*smooth((t-1.)/.5)
        d.mocap_pos[mid]=origin+approach+[0,0,lift]
        d.ctrl[:2]=-.0475 if t>=.6 else 0
        mujoco.mj_step(m,d,nstep=substeps)
        if k==599:before=d.xpos[obj].copy()
        touched=set()
        for c in d.contact:
            i,j=int(c.geom1),int(c.geom2);bi,bj=int(m.geom_bodyid[i]),int(m.geom_bodyid[j])
            if bi==obj and m.geom(j).name in ('left_tip','right_tip'):touched.add(m.geom(j).name)
            if bj==obj and m.geom(i).name in ('left_tip','right_tip'):touched.add(m.geom(i).name)
            if (m.geom_contype[i]==2 and m.geom_contype[j]==4)or(m.geom_contype[j]==2 and m.geom_contype[i]==4):
                if len(bad)<20:bad.append(dict(t_s=t,geoms=[m.geom(i).name,m.geom(j).name],depth_m=float(c.dist)))
        if t>=1.6:contacts.append(len(touched)==2)
        if k%25==0:states.append(dict(t_s=t,qpos=d.qpos.tolist(),mocap_pos=d.mocap_pos.tolist(),mocap_quat=d.mocap_quat.tolist(),position=d.xpos[obj].tolist(),rotation=d.xmat[obj].reshape(3,3).tolist()))
    error=float(np.linalg.norm(d.xpos[obj]-(P+[0,0,.04])));lift=float(d.xpos[obj,2]-before[2]);fraction=float(np.mean(contacts))
    report=dict(part=part,passed=error<.0015 and lift>.035 and fraction>.95 and not bad,position_error_m=error,lift_height_m=lift,both_tip_contact_fraction=fraction,hand_fixture_or_unselected_contacts=bad,final_rotation_error_deg=float(Rotation.from_matrix(d.xmat[obj].reshape(3,3)@Rp.T).magnitude()*180/np.pi),scene=str(path),timestep_s=float(m.opt.timestep),force_limit_per_finger_n=30,friction=.8,free_workpiece=True,arm_dynamics_simulated=False,full_task_timing_measured=False,scope='Single pickup from unchanged archived supply; prescribed palm motion; no transport, M80 insertion or PLC',states=states)
    (out/'result.json').write_text(json.dumps(report,indent=2));print('ORIGINAL_CONTACT',part,{k:v for k,v in report.items()if k!='states'},flush=True)
    return report

if __name__=='__main__':
    d=json.load(open(OUT/'endpoint-screen.json'));summary=[]
    for part in ['ring1','ring2','pin']:
        if part not in d['parts']:continue
        summary.append({k:v for k,v in run(part,d['parts'][part]['recipe']).items()if k!='states'})
        (OUT/'contact-summary.json').write_text(json.dumps(summary,indent=2))
