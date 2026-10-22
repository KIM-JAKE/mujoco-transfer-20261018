"""Screen all native-hand pieces against adjacent stored parts and fixture.
Static geometry sweep, 1 mm contact reserve; does not certify dynamic pickup.
"""
from pathlib import Path
import json,copy,xml.etree.ElementTree as E
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from five_cycle_storage import pin_specs,part_pose,HERE
from grasp_bench import vec,quat
j=json.load(open(HERE/'five-cycle-symmetric-pin/pin-five-picks.json'));recipe=json.load(open(HERE/'compact-path-candidates.json'))['pin'][0]['recipe'];P,Rp=part_pose(recipe);R=np.array(recipe['R']);lp=np.array(recipe['localP']);source=E.parse(j['scene']).getroot();out=HERE/'pin-pitch-screen';out.mkdir(exist_ok=True);results=[]
for pitch in [.025,.030,.035,.040,.045,.050,.055,.060,.065]:
 root=copy.deepcopy(source);world=root.find('worldbody');spec,centers,meta=pin_specs(P,R,pitch=pitch);_,oldcenters,_=pin_specs(P,R,pitch=.065)
 for i,C in enumerate(centers):
  world.find("body[@name='stored_"+str(i)+"']").set('pos',vec(C))
  for side in ['left','right']:
   b=world.find("body[@name='clip_"+side+'_'+str(i)+"']");b.set('pos',vec(np.fromstring(b.get('pos'),sep=' ')+np.array(C)-oldcenters[i]))
 for node in list(world):
  if node.tag=='geom'and node.get('name','').startswith('storage_'):world.remove(node)
 for n,s in enumerate(spec):
  if s.get('collision_only')or s.get('render_only'):continue
  kw=dict(name='storage_'+s['name']+'_'+str(n),type=s['type'],contype='4',conaffinity='3')
  if s['type']=='capsule':kw.update(fromto=vec(np.r_[s['a'],s['b']]),size=str(s['radius']))
  else:kw.update(pos=vec(s['pos']),quat=quat(np.array(s.get('R',np.eye(3)))),size=vec(np.array(s['size'])/2)if s['type']=='box'else str(s['radius'])if s['type']=='sphere'else vec([s['radius'],s['length']/2]))
  E.SubElement(world,'geom',**kw)
 path=out/('pitch-'+str(round(pitch*1000))+'.xml');E.ElementTree(root).write(path,encoding='unicode');m=mujoco.MjModel.from_xml_path(str(path));d=mujoco.MjData(m);hand=np.flatnonzero(m.geom_contype==2);m.geom_margin[hand]=.001;qa=m.joint('hand_root').qposadr[0];fj=[m.joint(x).qposadr[0]for x in ['left','right']];body=[m.body('stored_'+str(i)).id for i in range(5)];fail=[]
 for i,C in enumerate(centers):
  for u in np.linspace(0,1,29):
   if u<.2:offset=np.array([0,0,-.025*(1-u/.2)]);width=.02;opened=False;carrying=False
   elif u<.3:offset=np.zeros(3);width=.02-(.02-.0097)*(u-.2)/.1;opened=False;carrying=False
   elif u<.4:offset=np.array([0,0,.003*(u-.3)/.1]);width=.0097;opened=True;carrying=True
   elif u<.7:offset=np.array(meta['extraction_axis'])*.04*(u-.4)/.3+[0,0,.003];width=.0097;opened=True;carrying=True
   else:offset=np.array(meta['extraction_axis'])*.04+[0,0,.003+.037*(u-.7)/.3];width=.0097;opened=True;carrying=True
   d.qpos[qa:qa+3]=np.array(C)+Rp@lp-R@np.array([0,0,.1034])+offset;d.qpos[qa+3:qa+7]=Rotation.from_matrix(R).as_quat()[[3,0,1,2]];d.qpos[fj]=width/2
   pa=m.joint('stored_joint_'+str(i)).qposadr[0];d.qpos[pa:pa+3]=np.array(C)+(offset if carrying else 0)
   for slot in range(5):
    for side in ['left','right']:d.qpos[m.joint('clip_'+side+'_'+str(slot)).qposadr[0]]=.008 if slot==i and opened else-.001
   mujoco.mj_kinematics(m,d);mujoco.mj_collision(m,d)
   for c in d.contact:
    for h,other in [(c.geom1,c.geom2),(c.geom2,c.geom1)]:
     if m.geom_contype[h]!=2:continue
     b=int(m.geom_bodyid[other]);kind='support'if m.geom_contype[other]==4 else'other_part'if b in body[i+1:]else'non_tip_selected'if b==body[i]and m.geom(h).name not in ['left_tip','right_tip']else None
     if kind and c.dist<.001:
      event=dict(slot=i+1,phase=float(u),kind=kind,hand=m.geom(h).name,other=m.geom(other).name,distance_m=float(c.dist))
      if len(fail)<6:fail.append(event)
  # Earlier parts have been removed before the next cell is selected.
  for gi in np.flatnonzero(m.geom_bodyid==body[i]):m.geom_contype[gi]=m.geom_conaffinity[gi]=0
 row=dict(pitch_mm=round(pitch*1000),passed=not fail,first_failures=fail);results.append(row);print('PIN_PITCH',json.dumps(row),flush=True)
(out/'result.json').write_text(json.dumps(results,indent=2))
