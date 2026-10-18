"""Use FR3 redundancy to preserve hand poses while clearing the button plate.

Offline MuJoCo FK/Jacobians + FCL distances only; no MuJoCo dynamics.
"""
from pathlib import Path
import json,time,sys,argparse
import numpy as np
import mujoco,fcl,trimesh
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation,Slerp
from prepare import HERE,SOURCES
from task_path_offsets import pin_clearance_z,empty_right_offset,pin_grasp_weight
args_parser=argparse.ArgumentParser();args_parser.add_argument('--retimed',action='store_true');args_parser.add_argument('--fast',action='store_true');args=args_parser.parse_args()if __name__=='__main__'else args_parser.parse_args([])
if args.fast:
 from task_timing_fast import source_time
else:
 from task_timing import source_time
clock=source_time if(args.retimed or args.fast)else lambda side,t:t
ROOT=HERE;RUN=ROOT/'generated/contact_task/fast'if args.fast else ROOT/'generated/contact_task/retimed'if args.retimed else SOURCES['cap_button'].parent.parent/'cap_button_parallel_alt_20261016'
class Checker:
 def __init__(self,run=None,layout="physical",source_clock=None):
  self.layout=layout;run=RUN if run is None else Path(run);self.source_clock=source_clock or clock
  self.source=json.loads((ROOT/'generated/contact_task/contact-source.json').read_text());self.arr=np.load(ROOT/'generated/contact_task/collision.npz');self.cache={};self.static=[];self.robot=[];self.static_names={};self.side='right'
  for row in self.source['geoms']:
   if row['body']=='m80'and row['name']!='instance_0159':
    pos=np.array(row['position']).copy()
    if layout=='trial'and row['name']in ['instance_0369','instance_0380']:pos[1]+=.012
    if layout=='trial'and row['name']in ['instance_0373','instance_0382']:pos[1]-=.030
    if layout=='trial'and row['name']in ['instance_0371','instance_0384']:pos[1]-=.060
    self.static.append(fcl.CollisionObject(self.shape(row),fcl.Transform(self.rot(row['quaternion']),pos)));self.static_names[tuple(np.round(pos,7))]=row['name']
   elif row['body'].startswith(('left_','right_','official_FR3_Duo')):self.robot.append((row,fcl.CollisionObject(self.shape(row,True))))
  self.static.append(fcl.CollisionObject(fcl.Cylinder(.018,.004),fcl.Transform(Rotation.from_euler('x',90,degrees=True).as_matrix(),np.array([.355969,-.2395,1.527]))))
  from stock_fixture_geometry import pin_stock_sleeve
  vertices,faces,center=pin_stock_sleeve();sleeve=fcl.BVHModel();sleeve.beginModel(len(vertices),len(faces));sleeve.addSubModel(vertices,faces);sleeve.endModel();self.stock_sleeve=fcl.CollisionObject(sleeve,fcl.Transform(np.eye(3),center));self.static.append(self.stock_sleeve)
  precise=np.load(ROOT/'generated/contact_task/precision-cad.npz');cfg=json.loads((run/'config.json').read_text());targets=np.load(run/'trajectory.npz');self.installed=[];self.current_t=0.
  for name,identifier,side,end,index in [('ring1',542,'right',5.750333333333333,0),('ring2',549,'left',3.244333333333333,1)]:
   key=f'instance_{identifier:04d}';v=precise[key+'_vertices'];f=precise[key+'_faces'];pos=next(part['goal_origin_m']for part in cfg['parts']if part['name']==name);q=targets['mocap_quat'][-1,index];centroids=v[f].mean(axis=1);bins=np.floor((np.arctan2(centroids[:,1],centroids[:,0])+np.pi)/(2*np.pi)*64).astype(int)%64;objects=[]
   # Every original triangle belongs to a hull containing all of its vertices.
   # The union is conservative and retains the central opening.
   for sector in range(64):
    subset=f[bins==sector]
    if not len(subset):continue
    hull=trimesh.convex.convex_hull(v[np.unique(subset)]);hv=np.asarray(hull.vertices);hf=np.asarray(hull.faces);shape=fcl.BVHModel();shape.beginModel(len(hv),len(hf));shape.addSubModel(hv,hf);shape.endModel();objects.append(fcl.CollisionObject(shape,fcl.Transform(self.rot(q),np.array(pos))))
   manager=fcl.DynamicAABBTreeCollisionManager();manager.registerObjects(objects);manager.setup();self.installed.append((name,side,end,manager,objects))
  self.manager=fcl.DynamicAABBTreeCollisionManager();self.manager.registerObjects(self.static);self.manager.setup();pairs=json.loads((ROOT/'generated/contact_task/robot-filters.json').read_text())['pairs'];self.excludes={tuple(sorted((p['body0'],p['body1'])))for p in pairs}
  print('FCL_READY',len(self.static),len(self.robot),flush=True)
 def rot(self,q):return Rotation.from_quat(np.array(q)[[1,2,3,0]]).as_matrix()
 def shape(self,row,convex=False):
  kind=row['kind']
  if kind==7:
   from sensor_geometry import SENSOR_NAMES,physical_mesh
   optical=self.layout=='physical'and row['name']in SENSOR_NAMES
   key=row['mesh']+str(convex)+str(optical)
   if key not in self.cache:
    v=self.arr[row['mesh']+'_vertices'];f=self.arr[row['mesh']+'_faces']
    if optical:v,f,_=physical_mesh(v,f)
    if convex:
     hull=trimesh.Trimesh(v,f,process=False).convex_hull;v=np.array(hull.vertices);f=np.array(hull.faces)
    b=fcl.BVHModel();b.beginModel(len(v),len(f));b.addSubModel(v,f);b.endModel();self.cache[key]=b
   return self.cache[key]
  size=row['size']
  if kind==6:return fcl.Box(*[2*x for x in size])
  if kind in [3,5]:return fcl.Cylinder(size[0],2*size[1])
  if kind==2:return fcl.Sphere(size[0])
  raise ValueError(row)
 def check(self,model,data,detail=False):
  objects=[];worst=1e3;reason=None
  for row,obj in self.robot:
   b=model.body(row['body']).id;r=data.xmat[b].reshape(3,3);p=data.xpos[b]+r@row['position'];obj.setTransform(fcl.Transform(r@self.rot(row['quaternion']),p));objects.append((row,obj))
   if (self.side!='both'and not row['body'].startswith(self.side+'_'))or row['body'].endswith(('base','link0')):continue
   # Arm links need a clearance reserve; fingers approach the mating fixture.
   threshold=.003 if '_arm_'in row['body']else .0001
   distance=fcl.DistanceData();distance.result.min_distance=threshold;self.manager.distance(obj,distance,fcl.defaultDistanceCallback);d=distance.result.min_distance;worst=min(worst,d)
   for part,side,released,manager,installed_shapes in self.installed:
    reference=self.current_t[side]if isinstance(self.current_t,dict)else self.current_t
    if self.source_clock(side,reference)<released:continue
    required=.003 if '_arm_'in row['body']else .001;query=fcl.DistanceData();query.result.min_distance=required;manager.distance(obj,query,fcl.defaultDistanceCallback);separation=query.result.min_distance
    if separation<required:
     reason=(row['body'],float(separation),'installed_'+part)
     if not detail:return False,reason
   if d<threshold:
    reason=(row['body'],float(d),'machine');
    if not detail:return False,reason
  for i,(ra,oa)in enumerate(objects):
   for rb,ob in objects[i+1:]:
    if self.side!='both'and not(ra['body'].startswith(self.side+'_')or rb['body'].startswith(self.side+'_')):continue
    if ra['body']==rb['body']or tuple(sorted((ra['body'],rb['body'])))in self.excludes:continue
    if fcl.collide(oa,ob,fcl.CollisionRequest(),fcl.CollisionResult()):
     reason=(ra['body'],rb['body'],'robot');
     if not detail:return False,reason
  return reason is None,reason

if __name__=='__main__':
 model=mujoco.MjModel.from_xml_path(str(SOURCES['cap_button']));data=mujoco.MjData(model);record=np.load(RUN/'trajectory.npz');times=record['time_s'];raw=record['qpos'];pin_rotation=Slerp(times,Rotation.from_quat(record['mocap_quat'][:,2][:,[1,2,3,0]]));checker=Checker()
 props={}
 for side in ['left','right']:
  joints=[model.joint(side+'_arm_fr3v2_joint'+str(i)).id for i in range(1,8)]
  props[side]=dict(qadr=np.array([model.jnt_qposadr[j]for j in joints]),bounds=model.jnt_range[joints],hand=model.body(side+'_grip_base').id)
 # Calibrate the real pair of inner pad faces, rather than a virtual tool site.
 pin_schedule=json.loads((RUN/'independent_schedule.json').read_text());pin_pick=next(r['end_s']for r in pin_schedule['lanes']['left']if r['stage']['label']=='grasp_last_pin')+pin_schedule['delays']['left']
 data.qpos[:]=np.array([np.interp(pin_pick,times,raw[:,j])for j in range(30)]);mujoco.mj_kinematics(model,data)
 pad_local=np.array([0,-.0066,.035]);real_tcp=np.mean([data.xpos[model.body('left_grip_'+jaw+'_pad').id]+data.xmat[model.body('left_grip_'+jaw+'_pad').id].reshape(3,3)@pad_local for jaw in ['left','right']],axis=0)
 pin_pos=np.array([np.interp(pin_pick,times,record['mocap_pos'][:,2,j])for j in range(3)]);grasp_band_local=np.array([0,-.00025,-.015]);desired_tcp=pin_pos+pin_rotation(pin_pick).apply(grasp_band_local)
 hand=props['left']['hand'];pin_tcp_delta=data.xmat[hand].reshape(3,3).T@(desired_tcp-real_tcp)
 print('REAL_PIN_TCP_CALIBRATION',pin_tcp_delta.tolist(),'world_delta',(desired_tcp-real_tcp).tolist(),flush=True)

 # The old trajectory targeted a point beyond the real pad. Bake the tested
 # physical pad retarget into the planned button approach as well.
 data.qpos[:]=raw[-1];mujoco.mj_kinematics(model,data);tip=model.body('right_grip_right_pad').id;g=model.geom('right_grip_right_pad1');tip_local=g.pos+np.array([0,g.size[1],g.size[2]])
 button_delta=np.array([.3559690421575,-.24925+.0021,1.527])-(data.xpos[tip]+data.xmat[tip].reshape(3,3)@tip_local)
 def strength(t,side):
  t=clock(side,t)
  if side=='left':
   u=float(np.clip(t/.5,0,1));return 1-u*u*(3-2*u)
  u=float(np.clip((t-3.85)/.35,0,1));return u*u*(3-2*u)
 def solve(t,bias,side,seed=None):
  checker.current_t=float(t)
  qadr=props[side]['qadr'];bounds=props[side]['bounds'];hand=props[side]['hand']
  nominal=np.array([np.interp(t,times,raw[:,i])for i in range(30)]);data.qpos[:]=nominal;mujoco.mj_kinematics(model,data);P=data.xpos[hand].copy()
  if side=='left':
   P[2]+=pin_clearance_z(clock('left',t));P+=data.xmat[hand].reshape(3,3)@pin_tcp_delta*pin_grasp_weight(clock('left',t))
  if side=='right':
   P+=np.array(empty_right_offset(clock('right',t)))
   u=float(np.clip((clock('right',t)-7.4)/.35,0,1));P+=u*u*(3-2*u)*button_delta
  R=Rotation.from_matrix(data.xmat[hand].reshape(3,3));q=nominal[qadr].copy();q[0]+=bias*strength(t,side)
  if q[0]<bounds[0,0]or q[0]>bounds[0,1]:return None,None
  def fun(x):
   q[1:]=x;data.qpos[:]=nominal;data.qpos[qadr]=q;mujoco.mj_kinematics(model,data)
   return np.r_[10*(data.xpos[hand]-P),(Rotation.from_matrix(data.xmat[hand].reshape(3,3))*R.inv()).as_rotvec()]
  fit=least_squares(fun,np.clip(nominal[qadr][1:]if seed is None else seed[1:],bounds[1:,0]+1e-7,bounds[1:,1]-1e-7),bounds=(bounds[1:,0]+1e-7,bounds[1:,1]-1e-7),max_nfev=160,ftol=1e-10,xtol=1e-10,gtol=1e-10)
  error=fun(fit.x)
  if np.linalg.norm(error[:3])>1e-4 or np.linalg.norm(error[3:])>1e-4:return None,dict(ik_error=error.tolist())
  return q.copy(),None
 coarse={'right':[3.2,3.6,3.6166666667,4,4.27,4.4,4.6,4.9,5.5,5.85,6.4,7.4,8.060333333333332], 'left':[0,.15,.4,.8,1.5,2.5,3.125,3.4,4.2,4.7,5,6.5,7.15,7.4,8.060333333333332]}
 selected={}
 for side in ['right','left']:
  checker.side=side
  candidates=[.20,.15,.25,.10,0,-.10]if side=='right'else[.20,.25,.30,.40,.15,.10,-.10]
  for bias in candidates:
   failures=[]
   for t in coarse[side]:
    q,error=solve(t,bias,side)
    if q is None:failures.append((t,'IK',error));continue
    valid,why=checker.check(model,data)
    if not valid:failures.append((t,why))
   print('POSTURE_CANDIDATE',side,bias,'failures',failures,flush=True)
   if not failures:selected[side]=bias;break
  if side not in selected:raise RuntimeError('No constant elbow bias is clear for '+side)
 (ROOT/'generated/contact_task/posture-candidates.json').write_text(json.dumps(dict(selected=selected),indent=2))
 rows={side:[]for side in props};previous={side:None for side in props};checker.side='both'
 for f,t in enumerate(times):
  for side in props:
   q,error=solve(float(t),selected[side],side,previous[side])
   if q is None:raise RuntimeError(('Dense IK failed',side,float(t),error))
   if previous[side]is not None and max(abs(q-previous[side]))>.18:raise RuntimeError(('Joint jump',side,float(t),q-previous[side]))
   rows[side].append(q);previous[side]=q.copy()
  data.qpos[:]=raw[f]
  for side in props:data.qpos[props[side]['qadr']]=rows[side][-1]
  mujoco.mj_kinematics(model,data);valid,why=checker.check(model,data)
  if not valid:raise RuntimeError(('Dense collision failed',float(t),why))
  if f%60==0:print('DENSE',f,len(times),float(t),flush=True)
 np.savez_compressed(ROOT/'generated/contact_task/arm-posture.npz',time_s=times,left_qpos=np.array(rows['left']),right_qpos=np.array(rows['right']))
 report=dict(state='passed',joint1_bias_rad=selected,right_blend_start_s=3.85,right_blend_duration_s=.35,assembly_hand_pose_preserved_except_declared_offsets=True,pin_tcp_hand_local_delta_m=pin_tcp_delta.tolist(),pin_grasp_band_local_m=grasp_band_local.tolist(),pin_real_pad_contact_local_m=pad_local.tolist(),empty_right_hand_detour_m=[0,-.05,-.06],right_vertical_detour_source_interval_s=[5.95,6.5],button_real_pad_retarget_m=button_delta.tolist(),left_bias_fades_by_s=.5,dense_samples=len(times),arm_machine_clearance_min_m=.003,gripper_machine_clearance_min_m=.0001,retimed=args.retimed,fast=args.fast,installed_ring_clearance_min_m=.001,passive_supply_guide_checked=True,scope='Native robot hulls vs corrected M80, passive pin supply sleeve, installed ring obstacles; robot self and cross-arm collisions active; workpiece dynamics verified separately')
 (ROOT/'generated/contact_task/arm-posture.json').write_text(json.dumps(report,indent=2));print('POSTURE_PLAN_PASS',report,flush=True)
