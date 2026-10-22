"""Five consecutive contact-only picks with real CAD and standard native fingers.
No full robot dynamics, no M80 insertion: this validates storage/presentation only.
"""
from five_cycle_storage import *
import os,mujoco,xml.etree.ElementTree as E
from grasp_bench import build,vec,quat

def make_scene(part,recipe,out,qty=5):
 R=np.array(recipe['R']);P,Rp=part_pose(recipe);path,_,_=build(part,np.array(recipe['tcp']),R,np.array(recipe['localP']),np.array(recipe['localR']),recipe['gap'],out,holder_specs=[]);root=E.parse(path).getroot();world=root.find('worldbody');act=root.find('actuator')
 # The passive pin fork replaces the former active head clip.
 if part=='pin':
  specs,centers,meta=pin_specs(P,R)
  oldclips=[b for b in list(world)if b.tag=='body'and b.get('name','').startswith('clip_')];oldact=[a for a in list(act)if a.get('joint','').startswith('clip_')]
  for b in oldclips:world.remove(b)
  for a in oldact:act.remove(a)
  if meta.get('active_stock_clips'):
   # The pin neck is slightly tapered. A jaw-mounted axial stop prevents the
   # waiting pins walking upward, and retracts with the same opening command.
   left=next(b for b in oldclips if b.get('name')=='clip_left')
   right=next(b for b in oldclips if b.get('name')=='clip_right')
   for jaw,sign in [(left,1),(right,-1)]:
    # Two opposing half stops retract symmetrically, avoiding the lateral
    # impulse from the earlier single jaw-mounted disk touching the pin head.
    E.SubElement(jaw,'geom',name='top_stop',type='box',pos=vec([-sign*.0028,0,.01475]),size='.001 .0024 .001',contype='4',conaffinity='3',rgba='.08 .65 .58 1')
    for corner in [-1,1]:
     E.SubElement(jaw,'geom',name='neck_locating_'+str(corner),type='sphere',pos=vec([-sign*.0008,corner*.004,0]),size='.0015',contype='4',conaffinity='3',rgba='.08 .65 .58 1')
   for i,C in enumerate(centers):
    for template in oldclips:
     b=copy.deepcopy(template);name=template.get('name')+'_'+str(i);b.set('name',name);b.set('pos',vec(np.fromstring(template.get('pos'),sep=' ')+np.array(C)-P));b.find('joint').set('name',name)
     for geom in b.findall('geom'):
      if geom.get('name'):geom.set('name',geom.get('name')+'_'+name)
     world.append(b)
    for template in oldact:
     a=copy.deepcopy(template);name=template.get('joint')+'_'+str(i);a.set('joint',name);a.set('name','drive_'+name);act.append(a)
 else:
  specs,_,meta=ring_specs(part,P,R);centers=[(P+[0,0,-i*meta['pitch_m']]).tolist()for i in range(qty)]
 original=world.find("body[@name='part']");original.set('name','stored_0');original.set('pos',vec(centers[0]));original.find('freejoint').set('name','stored_joint_0')
 for geom in original.findall('geom'):geom.set('conaffinity','7');geom.set('condim','3')
 for i in range(1,qty):
  clone=copy.deepcopy(original);clone.set('name','stored_'+str(i));clone.set('pos',vec(centers[i]));clone.find('freejoint').set('name','stored_joint_'+str(i));world.append(clone)
 for i,s in enumerate(specs):
  if s.get('collision_only')or s.get('render_only'):continue
  if s.get('passive_gate'):
   b=E.SubElement(world,'body',name=s['name'],pos=vec(s['pos']));E.SubElement(b,'inertial',mass='.002',pos='0 0 0',diaginertia='.00000001 .00000001 .00000001');E.SubElement(b,'joint',name=s['name'],type='slide',axis=vec(s['slide_axis']),range='0 .003',stiffness=str(s['spring_n_m']),springref='0',damping='.2',solreflimit='.002 1');E.SubElement(b,'geom',name=s['name'],type='sphere',size=str(s['radius']),contype='4',conaffinity='3',rgba='.1 .65 .58 1');continue
  kw=dict(name='storage_'+s['name']+'_'+str(i),type=s['type'],contype='4',conaffinity='3',rgba='.08 .65 .58 1')
  if 'friction'in s:kw.update(friction=vec([s['friction'],.005,.0002]),priority='1')
  if s['type']=='capsule':kw.update(fromto=vec(np.r_[s['a'],s['b']]),size=str(s['radius']))
  else:kw.update(pos=vec(s['pos']),quat=quat(np.array(s.get('R',np.eye(3)))),size=vec(np.array(s['size'])/2)if s['type']=='box'else str(s['radius'])if s['type']=='sphere'else vec([s['radius'],s['length']/2]))
  E.SubElement(world,'geom',**kw)
 if part!='pin':
  plate=E.SubElement(world,'body',name='elevator_plate',pos=vec(meta['plate_initial_center']));E.SubElement(plate,'inertial',mass='.05',pos='0 0 0',diaginertia='.00002 .00002 .00004');E.SubElement(plate,'joint',name='elevator',type='slide',axis='0 0 1',range=vec([0,meta['lift_stroke_m']]),damping='10',solreflimit='.002 1');E.SubElement(plate,'geom',name='elevator_plate',type='cylinder',size=vec([meta['plate_radius_m'],meta['plate_thickness_m']/2]),contype='4',conaffinity='3',rgba='.15 .7 .6 1');E.SubElement(act,'position',name='elevator_drive',joint='elevator',kp='50000',kv='100',forcerange='-20 20')
 # Force-limited velocity servos on 15 g fingers can chatter at a 1 ms step
 # after contact release. Use a converged contact/servo integration step.
 root.find('option').set('timestep',os.environ.get('M80_STORAGE_DT','.000025'))
 E.ElementTree(root).write(path,encoding='unicode');return path,meta,centers

def run(part,recipe,out):
 out.mkdir(exist_ok=True);path,meta,centers=make_scene(part,recipe,out);m=mujoco.MjModel.from_xml_path(str(path));d=mujoco.MjData(m);R=np.array(recipe['R']);Rp=R@np.array(recipe['localR']).T;lp=np.array(recipe['localP']);mid=m.body('hand_target').mocapid[0];palm=m.joint('hand_root').qposadr[0];pvel=m.joint('hand_root').dofadr[0];fidx=[m.joint(x).qposadr[0]for x in ['left','right']];fvel=[m.joint(x).dofadr[0]for x in ['left','right']];ids=[m.body('stored_'+str(i)).id for i in range(5)];rows=[];motor_force=0.;max_speed=0.
 physics_dt=m.opt.timestep
 dt=.001
 substeps=round(dt/physics_dt)
 assert abs(substeps*physics_dt-dt)<1e-12
 def smooth(u):u=np.clip(u,0,1);return u*u*u*(10-15*u+6*u*u)
 if part=='pin'and meta.get('active_stock_clips'):d.ctrl[2:]=-.001
 for index in range(5):
  # Present the next ring by moving the real supporting plate. Remaining rings
  # stay free bodies and rise through physical contacts; none are teleported.
  if part!='pin'and index:
   for k in range(round(.6/dt)):
    u=k*dt/.4;d.ctrl[2]=(index-1+smooth(u))*meta['pitch_m'];mujoco.mj_step(m,d,nstep=substeps);motor_force=max(motor_force,abs(float(d.actuator_force[2])));max_speed=max(max_speed,abs(float(d.qvel[m.joint('elevator').dofadr[0]])))
  C=np.array(centers[index])if part=='pin'else np.array(centers[0]);pick=C+Rp@lp;origin=pick-R@np.array([0,0,.1034]);pre=np.array(recipe.get('pregrasp_delta_m',[0,0,-.025])if part=='pin'else[0,0,.04]);d.qpos[palm:palm+3]=origin+pre;d.qpos[palm+3:palm+7]=Rotation.from_matrix(R).as_quat()[[3,0,1,2]];d.qvel[pvel:pvel+6]=0;d.qpos[fidx]=.01 if part=='pin'else .04;d.qvel[fvel]=0;d.mocap_quat[mid]=Rotation.from_matrix(R).as_quat()[[3,0,1,2]];d.ctrl[:2]=0
  d.mocap_pos[mid]=origin+pre;d.qacc_warmstart[:]=0;d.qacc[:]=0;mujoco.mj_forward(m,d)
  selected=ids[index];before=None;contacts=[];support_hits=0;unpicked_hits=0;unpicked_move=0.;peakforce=0.;states=[];axis=np.array(meta.get('extraction_axis',[0,0,0]));pull=axis*meta.get('extraction_distance_m',.03)if part=='pin'else np.zeros(3)
  for k in range(round((2.9 if part=='pin'else 2.)/dt)):
   t=k*dt;approach=pre*np.clip((.65-t)/.3,0,1)if t<.65 else np.zeros(3);shift=pull*smooth((t-1.3)/.4)if part=='pin'else np.zeros(3)
   if part=='pin'and meta.get('active_stock_clips')and t>=1.:
    for label in ['clip_left','clip_right']:d.ctrl[m.actuator('drive_'+label+'_'+str(index)).id]=.008
   first_lift=meta.get('pre_extraction_lift_m',.003)
   lift=first_lift*smooth((t-1.15)/.15)+(.04-first_lift)*smooth((t-1.7)/.4)if part=='pin'else .04*smooth((t-1.)/.4)
   d.mocap_pos[mid]=origin+approach+shift+[0,0,lift];d.ctrl[:2]=-.0475 if t>=.65 else 0;mujoco.mj_step(m,d,nstep=substeps)
   if os.environ.get('M80_STORAGE_DEBUG'):
    for c in d.contact:
     if(m.geom_contype[c.geom1]==2 and m.geom_contype[c.geom2]==4)or(m.geom_contype[c.geom2]==2 and m.geom_contype[c.geom1]==4):
      print('FIRST_HAND_STORAGE_CONTACT',index+1,t,m.geom(c.geom1).name,m.geom(c.geom2).name,float(c.dist),d.qpos[palm:palm+3].tolist(),d.mocap_pos[mid].tolist(),flush=True);raise RuntimeError('storage diagnostic stop')
   if k==round(.65/dt)-1:before={b:d.xpos[b].copy()for b in ids[index:]}
   touched=set()
   for c in d.contact:
    g1,g2=c.geom1,c.geom2;b1,b2=int(m.geom_bodyid[g1]),int(m.geom_bodyid[g2]);n1,n2=m.geom(g1).name or'',m.geom(g2).name or''
    for name,bother,handgeom in [(n1,b2,g1),(n2,b1,g2)]:
     if name in('left_tip','right_tip')and bother==selected:touched.add(name)
     if m.geom_contype[handgeom]==2 and bother in ids[index+1:]:unpicked_hits+=1
    if(m.geom_contype[g1]==2 and m.geom_contype[g2]==4)or(m.geom_contype[g2]==2 and m.geom_contype[g1]==4):support_hits+=1
   if t>=.65:
    if before:unpicked_move=max(unpicked_move,max([np.linalg.norm(d.xpos[b]-before[b])for b in ids[index+1:]]or[0]))
    if t>=(1.8 if part=='pin'else 1.5):contacts.append(len(touched)==2)
   if k%round(.025/dt)==0:states.append(dict(t=t,selected_position=d.xpos[selected].tolist(),qpos=d.qpos.tolist(),ctrl=d.ctrl.tolist(),actuator_force=d.actuator_force.tolist(),qvel=d.qvel.tolist()))
  final=d.xpos[selected].copy();error=np.linalg.norm(final-(C+pull+[0,0,.04]));row=dict(pick_number=index+1,final_position_error_m=float(error),lift_height_m=float(final[2]-before[selected][2]),both_tip_contact_fraction=float(np.mean(contacts)),hand_support_contacts=support_hits,hand_unselected_part_contacts=unpicked_hits,unpicked_max_displacement_m=float(unpicked_move),final_aperture_m=float(sum(d.qpos[fidx])),passed=bool(error<.0015 and final[2]-before[selected][2]>.035 and np.mean(contacts)>.95 and support_hits==0 and unpicked_hits==0 and unpicked_move<.0015),states=states);rows.append(row);print('STORAGE_PICK',part,index+1,{k:v for k,v in row.items()if k!='states'},flush=True)
  # The picked part has left the storage bench. Park it between subtests; this
  # is explicitly not a simulated insertion into M80 or a timed full cycle.
  joint=m.joint('stored_joint_'+str(index));qa=joint.qposadr[0];va=joint.dofadr[0];d.qpos[qa:qa+3]=[2+index,2,2];d.qvel[va:va+6]=0;m.body_gravcomp[selected]=1
  for gi in np.flatnonzero(m.geom_bodyid==selected):m.geom_contype[gi]=m.geom_conaffinity[gi]=0
 result=dict(part=part,capacity=5,all_five_passed=all(r['passed']for r in rows),physics='MuJoCo contact-only storage/pick benchmark',timestep_s=float(physics_dt),control_and_contact_observation_dt_s=dt,arm_dynamics_simulated=False,M80_insertion_simulated=False,elevator_force_cap_n=20 if part!='pin'else None,elevator_peak_force_n=motor_force,elevator_peak_speed_m_s=max_speed,friction_assumption=.8,meta=meta,scene=str(path),picks=rows);(out/(part+'-five-picks.json')).write_text(json.dumps(result,indent=2));return result

if __name__=='__main__':
 cs=json.load(open(HERE/'compact-path-candidates.json'));recipes={p:copy.deepcopy(cs[p][0]['recipe'])for p in ['ring1','ring2','pin']};summary=[]
 for part in ['ring1','ring2','pin']:
  r=run(part,recipes[part],HERE/('five-cycle-contact-'+part));summary.append({k:v for k,v in r.items()if k!='picks'});(HERE/'five-cycle-contact-summary.json').write_text(json.dumps(summary,indent=2))
