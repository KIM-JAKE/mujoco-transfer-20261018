"""Isolated MuJoCo contact test of a fork cradle and standard Franka fingertips.
The moving palm is mocap driven; this is NOT a full arm or insertion simulation.
"""
from pathlib import Path
import os,sys,json,math
import numpy as np,mujoco,trimesh
from scipy.spatial.transform import Rotation
import xml.etree.ElementTree as E
HERE=Path(__file__).resolve().parent;PORT=HERE.parent
sys.path.insert(0,str(PORT/'native_hand_feasibility'))
from probe import hand_shapes

def vec(x):return ' '.join(map(str,np.asarray(x).ravel()))
def quat(R):return vec(Rotation.from_matrix(R).as_quat()[[3,0,1,2]])
def build(part,pick,R,localP,localR,gap,out,angle_deg=0,spacing=.52,extraction=False,head_motion=None,holder_specs=None):
 out.mkdir(exist_ok=True);Rp=R@localR.T;P=pick-Rp@localP
 root=E.Element('mujoco',model='native hand fork cradle contact bench');E.SubElement(root,'compiler',angle='radian');E.SubElement(root,'option',timestep='.001',gravity='0 0 -9.81',integrator='implicitfast',iterations='80',cone='elliptic')
 default=E.SubElement(root,'default');E.SubElement(default,'geom',friction='.8 .005 .0002',solref='.005 1',solimp='.95 .99 .001',condim='4')
 asset=E.SubElement(root,'asset');world=E.SubElement(root,'worldbody');act=E.SubElement(root,'actuator')
 origin=pick-R@np.array([0,0,.1034])
 target=E.SubElement(world,'body',name='hand_target',mocap='true',pos=vec(origin),quat=quat(R))
 palm=E.SubElement(world,'body',name='hand_base',pos=vec(origin),quat=quat(R))
 E.SubElement(palm,'freejoint',name='hand_root');E.SubElement(palm,'inertial',mass='.7',pos='0 .01 .03',diaginertia='.001 .0025 .0017')
 equality=E.SubElement(root,'equality');E.SubElement(equality,'weld',body1='hand_target',body2='hand_base',solref='.002 1')
 E.SubElement(asset,'mesh',name='palm',file=str(PORT/'native_hand_feasibility/hand.stl'))
 E.SubElement(palm,'geom',type='mesh',mesh='palm',rgba='.8 .8 .8 1',contype='2',conaffinity='1')
 for sign,label in [(1,'left'),(-1,'right')]:
  B=Rotation.from_euler('z',0 if sign==1 else np.pi).as_matrix();finger=E.SubElement(palm,'body',name=label,pos='0 0 .0584',quat=quat(B));E.SubElement(finger,'inertial',mass='.015',pos='0 .01 .025',diaginertia='.00001 .00001 .00001');E.SubElement(finger,'joint',name=label,type='slide',axis='0 1 0',range='0 .04',damping='2',solreflimit='.002 1',solimplimit='.999 .999 .00001')
  for name,pos,size,angle in [('mount',[0,.0185,.011],[.022,.015,.020],[0,0,0]),('sledge',[0,.0068,.0022],[.022,.0088,.0038],[0,0,0]),('diagonal',[0,.0159,.02835],[.0175,.007,.0235],[np.pi/6 if sign==1 else -np.pi/6,0,0 if sign==1 else np.pi]),('tip',[0,.00758,.04525],[.0175,.0152,.0185],[0,0,0])]:
   E.SubElement(finger,'geom',name=label+'_'+name,type='box',pos=vec(pos),size=vec(np.array(size)/2),quat=quat(Rotation.from_euler('xyz',angle).as_matrix()),contype='2',conaffinity='1',rgba='.12 .12 .14 1')
  E.SubElement(act,'velocity',joint=label,kv='600',forcerange='-30 30',ctrllimited='true',ctrlrange='-.05 .05')
 E.SubElement(equality,'joint',joint1='left',joint2='right',polycoef='0 1 0 0 0',solref='.002 1',solimp='.999 .999 .00001')
 # Open fork leaves a clear channel for the fingers; no rod through the ring.
 axis=R[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis);axis=Rotation.from_euler('z',angle_deg,degrees=True).apply(axis);cross=np.cross([0,0,1],axis);B=np.column_stack([axis,cross,[0,0,1]])
 radius=.0375 if part=='ring1'else .031
 if part.startswith('ring'):
  half=.00775 if part=='ring1'else .007
  holder=world;origin_holder=np.zeros(3)
  if head_motion:
   E.SubElement(world,'body',name='fork_target',mocap='true',pos=vec(P))
   holder=E.SubElement(world,'body',name='fork_support',pos=vec(P));origin_holder=P
   E.SubElement(holder,'freejoint',name='fork_root');E.SubElement(holder,'inertial',mass='.08',pos='0 0 0',diaginertia='.0001 .0001 .0001')
   E.SubElement(equality,'weld',name='fork_drive',body1='fork_target',body2='fork_support',solref='.002 1')
  for sign in [-1,1]:
   pos=P+axis*(-.02)+cross*(sign*radius*spacing)+np.array([0,0,-half-.001]);E.SubElement(holder,'geom',name='fork_'+str(sign),type='box',pos=vec(pos-origin_holder),quat=quat(B),size=vec([radius+.025,.003,.001]),rgba='.1 .65 .6 1',contype='4',conaffinity='3')
  if head_motion:E.SubElement(holder,'geom',name='fork_spine',type='box',pos=vec(P-axis*(radius+.045)+[0,0,-half-.001]-origin_holder),quat=quat(B),size=vec([.004,radius*.7+.004,.002]),rgba='.1 .65 .6 1',contype='4',conaffinity='3')
  fine=np.load(PORT/'generated/contact_task/precision-cad.npz');key='instance_0542'if part=='ring1'else'instance_0549';v=fine[key+'_vertices'];faces=fine[key+'_faces'];cent=v[faces].mean(1);bins=((np.arctan2(cent[:,1],cent[:,0])+np.pi)/(2*np.pi)*48).astype(int)%48
  body=E.SubElement(world,'body',name='part',pos=vec(P),quat=quat(Rp));E.SubElement(body,'freejoint',name='object');part_spec=json.load(open(PORT/'generated/contact_task/contact-source.json'))['parts'][part];E.SubElement(body,'inertial',mass=str(part_spec['mass']),pos='0 0 0',diaginertia=vec(part_spec['inertia']))
  for i in range(48):
   sub=faces[bins==i]
   if not len(sub):continue
   h=trimesh.convex.convex_hull(v[np.unique(sub)]);path=out/(f'{part}-sector-{i}.obj');h.export(path);E.SubElement(asset,'mesh',name='sector'+str(i),file=str(path));E.SubElement(body,'geom',type='mesh',mesh='sector'+str(i),contype='1',conaffinity='6',rgba='.95 .48 .05 1'if part=='ring1'else'.05 .5 .95 1')
 else:
  source=json.load(open(PORT/'generated/contact_task/contact-source.json'));archive=np.load(PORT/'generated/contact_task/collision.npz');precision=np.load(PORT/'generated/contact_task/precision-pin-main.npz');spec=source['parts']['pin']
  body=E.SubElement(world,'body',name='part',pos=vec(P),quat=quat(Rp));E.SubElement(body,'freejoint',name='object');E.SubElement(body,'inertial',mass=str(spec['mass']),pos='0 0 0',diaginertia=vec(spec['inertia']))
  pieces=[]
  for row in spec['colliders']:
   key=row['mesh']
   if key=='pin_mesh_206':v=precision['instance_0557_vertices'];faces=precision['instance_0557_faces']
   else:
    v=archive[key+'_vertices'];faces=archive[key+'_faces'];r=Rotation.from_quat(np.array(row['quaternion'])[[1,2,3,0]]).as_matrix();v=v@r.T+row['position']
   if key=='pin_mesh_205':pieces.append(v);continue
   c=v[faces].mean(1);angles=((np.arctan2(c[:,1]+.00025,c[:,0])+np.pi)/(2*np.pi)*24).astype(int)%24;zs=np.clip(((c[:,2]-v[:,2].min())/max(np.ptp(v[:,2]),1e-9)*8).astype(int),0,7)
   for k in np.unique(angles+24*zs):
    subset=faces[angles+24*zs==k];points=v[np.unique(subset)]
    if len(points)>=4:pieces.append(points)
  for k,points in enumerate(pieces):
   try:h=trimesh.convex.convex_hull(points)
   except Exception:continue
   if abs(h.volume)<1e-15:continue
   path=out/(f'pin-contact-{k}.obj');h.export(path);E.SubElement(asset,'mesh',name='pinpart'+str(k),file=str(path));E.SubElement(body,'geom',type='mesh',mesh='pinpart'+str(k),contype='1',conaffinity='6',rgba='.9 .24 .06 1')
  # Separate head clamp plus axial datum. Not attached to the M80.
  E.SubElement(world,'geom',name='pin_axial_datum',type='cylinder',pos=vec(P+Rp@np.array([0,-.00025,-.01875])),quat=quat(Rp),size='.0024 .001',contype='4',conaffinity='3',rgba='.1 .65 .6 1')
  for sign,label in ([(1,'clip_left'),(-1,'clip_right')] if localP[2]<-.008 else []):
   axis=Rp[:,0]*sign;pos=P+Rp@np.array([sign*.0048,-.00025,.004]);jaw=E.SubElement(world,'body',name=label,pos=vec(pos),quat=quat(Rp));E.SubElement(jaw,'inertial',mass='.01',pos='0 0 0',diaginertia='.000001 .000001 .000001');E.SubElement(jaw,'joint',name=label,type='slide',axis=vec([sign,0,0]),range='-.004 .010',stiffness='0',damping='1',solreflimit='.002 1',solimplimit='.999 .999 .00001');E.SubElement(jaw,'geom',type='box',size='.001 .004 .004',contype='4',conaffinity='3',rgba='.1 .65 .6 1');E.SubElement(act,'position',joint=label,kp='5000',kv='5',forcerange='-20 20',ctrllimited='true',ctrlrange='-.001 .008')

 if holder_specs is not None:
  for node in list(world):
   if node.tag=='geom'and(node.get('name','').startswith('fork_')or node.get('name')=='pin_axial_datum'):world.remove(node)
  for spec in holder_specs:
   if 'clip_sweep'in spec['name']:continue  # The moving jaws above are physical.
   kw=dict(name='holder_'+spec['name'],type=spec['type'],contype='4',conaffinity='3',rgba='.1 .65 .6 1')
   if spec['type']=='capsule':kw.update(fromto=vec(np.r_[spec['a'],spec['b']]),size=str(spec['radius']))
   else:
    kw.update(pos=vec(spec['pos']),quat=quat(np.array(spec.get('R',np.eye(3)))))
    kw['size']=vec(np.array(spec['size'])/2)if spec['type']=='box'else vec([spec['radius'],spec['length']/2])
   E.SubElement(world,'geom',**kw)
 path=out/(part+'-bench.xml');E.ElementTree(root).write(path,encoding='unicode');return path,P,Rp

def run(part,pick,R,localP,localR,gap,out,angle_deg=0,spacing=.52,extraction=False,pull_angle_deg=None,head_motion=None,holder_specs=None,approach_delta_m=None):
 path,P,Rp=build(part,pick,R,localP,localR,gap,out,angle_deg,spacing,head_motion=head_motion,holder_specs=holder_specs);m=mujoco.MjModel.from_xml_path(str(path));d=mujoco.MjData(m);mid=m.body('hand_target').mocapid[0];obj=m.body('part').id;finger_indices=[m.joint(x).qposadr[0]for x in ('left','right')];d.qpos[finger_indices]=.04 if part.startswith('ring')else .01;d.ctrl[:]=0;d.ctrl[2:]=-.001;origin=pick-R@np.array([0,0,.1034]);d.mocap_pos[mid]=origin+(np.array(approach_delta_m)if approach_delta_m is not None else np.array([0,0,-.025]));d.qpos[:3]=d.mocap_pos[mid];d.mocap_quat[mid]=Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
 initial=None;lift=None;contacts=[];support_hand_contacts=[];states=[]
 axis=R[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis);axis=Rotation.from_euler('z',angle_deg,degrees=True).apply(axis)
 if pull_angle_deg is not None:
  axis=R[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis);axis=Rotation.from_euler('z',pull_angle_deg,degrees=True).apply(axis)
 if axis[1]>0:axis=-axis
 pull=axis*.10 if extraction and not head_motion else np.zeros(3)
 if head_motion:
  hm=m.body('fork_target').mocapid[0];drop=head_motion['drop_m'];horizontal=np.array(head_motion['horizontal_delta_m']);Td=max(.08,1.875*drop/.15);Th=max(.10,1.875*np.linalg.norm(horizontal)/.5)
 for k in range(2500 if extraction else 2000):
  t=k*.001
  if t<.35:offset=-.025
  elif t<.65:offset=-.025+.025*(t-.35)/.3
  elif t<1.:offset=0
  elif t<1.4:offset=.04*(t-1.)/.4
  else:offset=.04
  if extraction:
   offset=0 if t>=.65 else offset
   shift_begin=1.15 if part=='pin' else 1.0
   if t>=shift_begin+.4:offset=.04*min(1.,(t-shift_begin-.4)/.4)
   shift=pull*min(1.,max(0.,(t-shift_begin)/.4))
  else:shift=np.zeros(3)
  if head_motion:
   def smooth(u):
    u=np.clip(u,0,1);return u**3*(10-15*u+6*u*u)
   d.mocap_pos[hm]=P+np.array([0,0,-drop])*smooth((t-1.)/Td)+horizontal*smooth((t-1.-Td)/Th)
   offset=0 if t>=.65 else offset
   if t>=1.+Td+Th+.04:offset=.04*smooth((t-1.-Td-Th-.04)/.4)
   shift=np.zeros(3)
  if approach_delta_m is not None and t<.65:d.mocap_pos[mid]=origin+np.array(approach_delta_m)*np.clip((.65-t)/.3,0,1)
  else:d.mocap_pos[mid]=origin+shift+[0,0,offset]
  if t>=.65:
   d.ctrl[:2]=-.05
  if part=='pin' and t>=1.0:d.ctrl[2:]=.008
  mujoco.mj_step(m,d)
  if k==349:initial=d.xpos[obj].copy()
  if k==999:lift=d.xpos[obj].copy()
  if k>=(1800 if extraction else 1400):
   sides=set()
   for c in d.contact:
    names=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i)or'' for i in (c.geom1,c.geom2)]
    for label in ('left_tip','right_tip'):
     if label in names:sides.add(label)
   contacts.append(len(sides)==2)
  for c in d.contact:
   if (m.geom_contype[c.geom1]==4 and m.geom_contype[c.geom2]==2)or(m.geom_contype[c.geom2]==4 and m.geom_contype[c.geom1]==2):support_hand_contacts.append(t)
  if k%20==0:states.append(dict(t=t,object=d.xpos[obj].tolist(),hand=d.mocap_pos[mid].tolist(),qpos=d.qpos.tolist()))
 final=d.xpos[obj].copy();result=dict(part=part,custom_holder_geometry=holder_specs,approach_delta_m=approach_delta_m,retracting_support_head=head_motion,lateral_extraction=extraction and not head_motion,fork_angle_deg=angle_deg,fork_spacing_radius_fraction=spacing,physics='MuJoCo 3.11, dt=1ms, free workpiece, contact-only grasp',arm_dynamics_simulated=False,gripper_mocap_driven=True,part_geometry='actual CAD sector hulls'if part.startswith('ring')else'CAD local convex sectors, including precision slotted shell',friction=.8,assumed_mass_kg=json.load(open(PORT/'generated/contact_task/contact-source.json'))['parts'][part]['mass'],force_limit_per_finger_n=30,settled_displacement_m=float(np.linalg.norm(initial-P)),lift_height_m=float(final[2]-lift[2]),final_position_error_m=float(np.linalg.norm(final-(P+pull+[0,0,.04]))),both_tip_contact_fraction=float(np.mean(contacts)),hand_support_contact_steps=len(set(support_hand_contacts)),first_support_hand_contact_s=min(support_hand_contacts)if support_hand_contacts else None,actual_final_aperture_m=float(sum(d.qpos[finger_indices])),passed=bool(final[2]-lift[2]>.03 and np.linalg.norm(final-(P+pull+[0,0,.04]))<.006 and np.mean(contacts)>.8 and not support_hand_contacts),scene=str(path),states=states)
 (out/(part+'-result.json')).write_text(json.dumps(result,indent=2));return result

if __name__=='__main__':
 cfg=json.load(open(HERE/'layout-search.json'));parts=json.load(open(PORT/'native_hand_feasibility/consistent-grasp.json'))['cases'];results=[]
 for p,report in zip(parts,cfg['parts']):
  best=report['best'];r=run(p['part'],np.array(best['tcp']),np.array(best['R']),np.array(p['constant_hand_to_part_contact_m']),np.array(p['constant_hand_to_part_rotation']),p['endpoints'][1]['gap_m'],HERE/'grasp-bench');results.append({k:v for k,v in r.items()if k!='states'});print(results[-1],flush=True)
 (HERE/'grasp-bench-summary.json').write_text(json.dumps(results,indent=2))
