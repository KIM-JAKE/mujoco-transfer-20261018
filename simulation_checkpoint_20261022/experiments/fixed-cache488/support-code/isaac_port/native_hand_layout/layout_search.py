"""Native Hand supply layout search: measured CAD, IK, sampled collision and timing.
Does not claim force-based assembly or hardware execution.
"""
from pathlib import Path
import sys,json,time,math,os
import numpy as np,mujoco,fcl
from scipy.spatial.transform import Rotation,Slerp
from scipy.optimize import least_squares
HERE=Path(__file__).resolve().parent;PORT=HERE.parent
sys.path.insert(0,str(PORT/'native_hand_feasibility'));sys.path.insert(0,str(PORT))
import search as g
from fr3_motion_control import position_velocity_bounds,LOWER,UPPER
model=g.model;data=g.data
parts=json.load(open(PORT/'native_hand_feasibility/consistent-grasp.json'))['cases']
AD={s:np.array([model.joint(f'{s}_arm_fr3v2_joint{j}').qposadr[0]for j in range(1,8)])for s in ('left','right')}
READY={s:g.plan[s+'_qpos'][0].copy()for s in AD}
FL={s:model.body(s+'_arm_fr3v2_link8').id for s in AD}
ROBOT={s:[(row,obj)for row,obj in g.checker.robot if row['body'].startswith(s+'_arm_')]for s in AD}
OBJECT_BOUNDS={};OBJECT_OBBS={};_ROW_BOUNDS={};_HAND_BOUNDS={}
BBOX_SCREEN_ENABLED=os.environ.get('M80_BBOX_SCREEN','0')=='1'

def register_bounds(obj,R,P,bounds):
 low,high=bounds;center=(np.asarray(low)+high)/2;half=(np.asarray(high)-low)/2;world=np.asarray(P)+R@center;extent=np.abs(R)@half
 OBJECT_BOUNDS[id(obj)]=(world-extent,world+extent)
 OBJECT_OBBS[id(obj)]=(world,np.asarray(R).copy(),half)

def threshold_distance(manager,obj,reserve):
 if reserve<=0:
  query=fcl.CollisionData();manager.collide(obj,query,fcl.defaultCollisionCallback);return-1. if query.result.is_collision else 0.
 bounds=OBJECT_BOUNDS.get(id(obj))
 if not BBOX_SCREEN_ENABLED or bounds is None:
  collision=fcl.CollisionData();manager.collide(obj,collision,fcl.defaultCollisionCallback)
  if collision.result.is_collision:return-1.
  query=fcl.DistanceData();query.result.min_distance=reserve;manager.distance(obj,query,fcl.defaultDistanceCallback);return query.result.min_distance
 # Only broad-phase AABB overlaps are collected here. Every candidate still
 # uses the original exact FCL shapes and distance query, with the same margin.
 low,high=bounds;center=(low+high)/2;size=np.maximum(high-low+2*reserve+2e-9,1e-9);box=fcl.CollisionObject(fcl.Box(*size),fcl.Transform(np.eye(3),center));best=reserve
 def candidate(other,query,_):
  nonlocal best
  distance=-1. if fcl.collide(other,obj,COLREQ,fcl.CollisionResult())else fcl.distance(other,obj,fcl.DistanceRequest(),fcl.DistanceResult())
  if distance<best:best=distance
  return best<reserve
 manager.collide(box,None,candidate)
 return best

def clearance_margin(name,margin):
 if getattr(g,'layout_collision_search_mode',False):return 0.
 if name.endswith('_hand_payload')and getattr(g,'layout_payload_collision_only',False):return 0.
 return .003 if '_arm_'in name else margin
# Old assumed pin supply sleeve is replaced by independent holders. Keep M80
# geometry and fixed real sensors, preserve all actual mating geometry.
statics=[o for o in g.statics if np.linalg.norm(o.getTranslation()-[-.55475,-.16,1.28525])>1e-6]
g.manager=fcl.DynamicAABBTreeCollisionManager();g.manager.registerObjects(statics);g.manager.setup()
for s in AD:data.qpos[AD[s]]=READY[s]
mujoco.mj_kinematics(model,data)

def fk(side,q):
 data.qpos[AD[side]]=q;mujoco.mj_kinematics(model,data);b=FL[side]
 R=data.xmat[b].reshape(3,3)@Rotation.from_euler('z',-45,degrees=True).as_matrix()
 return data.xpos[b]+R@np.array([0,0,.1034]),R

def ik(side,tcp,R,seed):
 goal=tcp-R@np.array([0,0,.1034]);rotation=Rotation.from_matrix(R@Rotation.from_euler('z',45,degrees=True).as_matrix())
 target_fixed=float(seed[0])
 def fun(q):
  data.qpos[AD[side]]=q;mujoco.mj_kinematics(model,data)
  return np.r_[10*(data.xpos[FL[side]]-goal),(Rotation.from_matrix(data.xmat[FL[side]].reshape(3,3))*rotation.inv()).as_rotvec(),.015*(q[0]-target_fixed)]
 fit=least_squares(fun,np.clip(seed,LOWER+1e-5,UPPER-1e-5),bounds=(LOWER+1e-5,UPPER-1e-5),max_nfev=65,ftol=1e-7,xtol=1e-7,gtol=1e-7)
 r=fun(fit.x)
 if np.linalg.norm(r[:3])>3e-4 or np.linalg.norm(r[3:6])>3e-4:return None
 return fit.x

def objects(side,q,gap):
 tcp,R=fk(side,q);out=[];OBJECT_BOUNDS.clear();OBJECT_OBBS.clear()
 for row,obj in ROBOT[side]:
  b=model.body(row['body']).id;r=data.xmat[b].reshape(3,3);p=data.xpos[b]+r@row['position'];rotation=r@g.checker.rot(row['quaternion']);obj.setTransform(fcl.Transform(rotation,p));out.append((row['body'],obj))
  key=row['name']
  if key not in _ROW_BOUNDS:
   if row['kind']==7:
    v=g.checker.arr[row['mesh']+'_vertices'];_ROW_BOUNDS[key]=(v.min(0),v.max(0))
   else:
    s=row['size'];half=np.asarray(s)if row['kind']==6 else np.repeat(s[0],3)if row['kind']==2 else np.array([s[0],s[0],s[1]]);_ROW_BOUNDS[key]=(-half,half)
  register_bounds(obj,rotation,p,_ROW_BOUNDS[key])
 origin=tcp-R@np.array([0,0,.1034])
 for name,shape,r,p in g.shapes:
  p=p.copy()
  if name not in ('palm','payload'):p[1]+=(1 if name.startswith('left')else-1)*gap/2
  rotation=R@r;position=origin+R@p;obj=fcl.CollisionObject(shape,fcl.Transform(rotation,position));out.append((side+'_hand_'+name,obj))
  if name=='payload':bounds=getattr(g,'layout_payload_bounds',None)
  else:
   if name not in _HAND_BOUNDS:
    if name=='palm':mesh=g.trimesh.load(PORT/'native_hand_feasibility/hand.stl');_HAND_BOUNDS[name]=tuple(mesh.bounds)
    else:half=np.asarray(shape.side)/2;_HAND_BOUNDS[name]=(-half,half)
   bounds=_HAND_BOUNDS[name]
  if bounds is not None:register_bounds(obj,rotation,position,bounds)
 return out,tcp,R

COLREQ=fcl.CollisionRequest()
def pairhit(a,b):
 if BBOX_SCREEN_ENABLED:
  aa=OBJECT_BOUNDS.get(id(a));bb=OBJECT_BOUNDS.get(id(b))
  if aa is not None and bb is not None and(np.any(aa[1]+1e-9<bb[0])or np.any(bb[1]+1e-9<aa[0])):return False
 return bool(fcl.collide(a,b,COLREQ,fcl.CollisionResult()))
def clear(side,q,gap,extra=None,margin=.0001):
 obs,tcp,R=objects(side,q,gap)
 for name,obj in obs:
  if name.endswith('link0'):continue
  threshold=clearance_margin(name,margin);distance=threshold_distance(g.manager,obj,threshold)
  if distance<threshold:return False,[name,'machine',float(distance)]
  if extra:
   for label,other in extra:
    if pairhit(obj,other):return False,[name,label]
 for i,(a,oa)in enumerate(obs):
  for b,ob in obs[i+1:]:
   if a==b or tuple(sorted((a,b)))in g.checker.excludes:continue
   if '_hand_'in a and '_hand_'in b:continue
   if ('_hand_'in a and b.endswith(('link7','link8')))or('_hand_'in b and a.endswith(('link7','link8'))):continue
   if pairhit(oa,ob):return False,[a,b,'self']
 return True,None

def segment_time(a,b):
 a=np.asarray(a);b=np.asarray(b)
 dq=np.abs(b-a);u=np.linspace(0,1,201);h=u**3*(10-15*u+6*u*u)
 q=a[None,:]+h[:,None]*(b-a)[None,:];v=30*u*u*(1-u)**2
 lo,hi=position_velocity_bounds(q);cap=.95*np.where((b-a)[None,:]>=0,hi,-lo)
 T=max(float(np.max(v[:,None]*dq[None,:]/np.maximum(cap,1e-6))),float(np.sqrt(np.max(dq)*5.773503/9.5)),float(np.cbrt(np.max(dq)*60/4750)),.04)*1.02
 return T

def path_clear(side,qs,gap,steps=13,extra=None):
 for si,(a,b)in enumerate(zip(qs[:-1],qs[1:])):
  for u in np.linspace(0,1,steps):
   okay,why=clear(side,(1-u)*a+u*b,gap,extra)
   if not okay:return False,dict(segment=si,fraction=float(u),why=why)
 return True,None

def cartesian(side,a,tcp,R,steps=5):
 start,Ra=fk(side,a);qs=[a]
 if Rotation.from_matrix(Ra@R.T).magnitude()>.001:return None
 for u in np.linspace(0,1,steps+1)[1:]:
  q=ik(side,(1-u)*start+u*tcp,R,qs[-1])
  if q is None:return None
  qs.append(q)
 return qs

def prepare_part(p):
 name=p['part'];side='right'if name=='ring1'else'left';insert=p['endpoints'][1];q=np.array(insert['arm_clear_ik'][0]['qpos']);tcp=np.array(insert['tcp_world_m']);R=np.array(insert['hand_rotation'])
 return dict(name=name,side=side,q_insert=q,tcp=tcp,R=R,gap=insert['gap_m'],localP=np.array(p['constant_hand_to_part_contact_m']),localR=np.array(p['constant_hand_to_part_rotation']))


def main():
 start=time.monotonic();results=[];winning={}
 previous=HERE/'search-progress.json'
 if previous.exists():
  results=json.load(open(previous))['parts'];winning={r['part']:r['best']for r in results}
  for row in results:
   for c in row['candidates']:
    if c.get('total_lane_s',0)>30:c['valid']=False;c['reject']='near_bound_velocity_or_timing'
   row['valid_count']=sum(c['valid']for c in row['candidates'])
 for p0 in parts:
  if p0['part']in winning:continue
  p=prepare_part(p0);side=p['side'];tcp=p['tcp'];R=p['R'];qi=p['q_insert'];gap=p['gap']
  # Insertion normal is world Z in the original fixed M80. Approach below the overhead insertion jig.
  pre=ik(side,tcp+[0,0,-.055],R,qi)
  if pre is None:raise RuntimeError('No preinsert IK '+p['name'])
  okay,why=path_clear(side,[pre,qi],gap,15)
  print('INSERT',p['name'],okay,why,flush=True)
  p['q_preinsert']=pre
  candidates=[]
  # Baseline old stock geometry and identical initial joint states.
  row=p0['endpoints'][0];candidates.append(dict(label='original_stock',tcp=np.array(row['tcp_world_m']),R=np.array(row['hand_rotation']),seed=np.array(row['arm_clear_ik'][0]['qpos']),family='baseline'))
  for tilt in (0,45,90,135,180):
   # Rotate around hand closing axis, preserving grasp band on the workpiece.
   RR=R@Rotation.from_euler('y',tilt,degrees=True).as_matrix()
   for y in (-.22,-.12,-.02,.08):
    for dz in (-.18,-.08,.03):
     P=tcp+np.array([0,y-tcp[1],dz]);candidates.append(dict(label=f'tilt{tilt}_y{y}_z{dz}',tcp=P,R=RR,seed=pre,family='independent'))
  tested=[]
  for ci,c in enumerate(candidates):
   q=ik(side,c['tcp'],c['R'],c['seed']);out=dict(label=c['label'],family=c['family'],tcp=c['tcp'].tolist(),R=c['R'].tolist(),valid=False)
   if q is not None:out['q_pick']=q.tolist()
   if q is None:out['reject']='IK';tested.append(out);continue
   ok,why=clear(side,q,.08 if p['name'].startswith('ring')else .02)
   if not ok:out.update(reject='pickup_collision',why=why);tested.append(out);continue
   lift=ik(side,c['tcp']+[0,0,.035],c['R'],q)
   if lift is None:out['reject']='lift_IK';tested.append(out);continue
   # Direct transfer is screened, with an elevated waypoint if needed.
   qs=[q,lift,pre,qi];ok,why=path_clear(side,qs,gap,9)
   if not ok:
    midtcp=(c['tcp']+tcp)/2+[0,-.045,.16];midR=Rotation.from_quat(Slerp([0,1],Rotation.from_matrix([c['R'],R]))(.5).as_quat());mid=ik(side,midtcp,midR.as_matrix(),(lift+pre)/2)
    if mid is not None:
     qs=[q,lift,mid,pre,qi];ok,why=path_clear(side,qs,gap,9)
   if not ok:out.update(reject='path_collision',why=why);tested.append(out);continue
   # Initial motion included equally in comparisons. Pin starts after ring2.
   startq=READY[side] if p['name']!='pin' else winning['ring2']['q_exit']
   entry=[np.array(startq),q];ok,why=path_clear(side,entry,.08 if p['name'].startswith('ring')else .02,9)
   if not ok:out.update(reject='entry_collision',why=why);tested.append(out);continue
   duration=sum(segment_time(a,b)for a,b in zip(qs[:-1],qs[1:]))+segment_time(startq,q)
   if duration>30:
    out.update(reject='near_bound_velocity_or_timing');tested.append(out);continue
   # 100 mm/s nominal native gripper speed, 50 mm/s open; these are
   # comparison assumptions, not measured device response times.
   close=max(.12,((.08 if p['name'].startswith('ring')else .02)-gap)/.1+.08)
   release=max(.12,((.08 if p['name'].startswith('ring')else .02)-gap)/.05+.05)
   total=duration+close+.18+release+segment_time(qi,pre)
   out.update(valid=True,q_pick=q.tolist(),q_path=[a.tolist()for a in qs],q_exit=pre.tolist(),motion_s=duration,grasp_s=close,clamp_s=.18,release_s=release,total_lane_s=total)
   tested.append(out)
   print('VALID',p['name'],c['label'],round(total,3),flush=True)
  valids=sorted([x for x in tested if x['valid']],key=lambda x:x['total_lane_s'])
  if not valids:
   (HERE/(p['name']+'-failures.json')).write_text(json.dumps(tested,indent=2));raise RuntimeError('No valid layout '+p['name'])
  best=valids[0];winning[p['name']]=best
  results.append(dict(part=p['name'],valid_count=len(valids),trials=len(tested),insert_path_valid=okay,insert_path_failure=why,best=best,candidates=tested))
  (HERE/'search-progress.json').write_text(json.dumps(dict(parts=results,seconds=time.monotonic()-start),indent=2))
 result=dict(state='sampled_collision_and_kinematic_timing_search',hardware_ready=False,physical_grasp_and_insertion_tested=False,original_fixed_M80=True,common_start_q={s:READY[s].tolist()for s in READY},planning_fraction=.95,parts=results,seconds=time.monotonic()-start)
 (HERE/'layout-search.json').write_text(json.dumps(result,indent=2)+'\n');print('SEARCH_DONE',result['seconds'],flush=True)
if __name__=='__main__':main()
