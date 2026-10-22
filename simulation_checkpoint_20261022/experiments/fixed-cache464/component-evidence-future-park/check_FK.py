import sys,json,time,hashlib
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path[:0]=['/tmp/fixed463-full/overrides','/tmp/fixed445-binary/overrides','/home/gpuadmin/workspace/jake/mujoco/isaac_port/native_hand_layout/roas-table']
import gpu_pair_schedule_json as old
import gpu_pair_schedule_binary as new
from predictive_pair_clearance import proxy,at_pose,shape_overlap
ROOT=Path('/home/gpuadmin/workspace/jake/mujoco');run=ROOT/'isaac_port/runs/native-roas-table-fixed-front-v432-phase-offset-paired-native-all3';local=np.load(ROOT/'.local/render-current/pair432-exact-locals.npz');z=np.load(run/'executed-actuator-commands.npz');report=json.loads((run/'pipeline-results.json').read_text())
class Arm:
 def __init__(self,side,model,rows):
  for key in ['base','tip','axes','frames']:setattr(self,key,np.asarray(model[key],float))
  self.base_path=next(b for b,_,_ in rows if b.endswith('_arm_fr3v2_link1')).rsplit('/',1)[0];self.side=side;self.hand_path=next(b for b,_,_ in rows if b.endswith('_NativeHand'));self.link_paths=[next(b for b,_,_ in rows if b.endswith('_arm_fr3v2_link'+str(i)))for i in range(1,8)]
 def bodies(self,q,opening):
  T=self.base.copy();out={self.base_path:self.base.copy()}
  for p,x,axis,(a,b)in zip(self.link_paths,q,self.axes,self.frames):
   v=axis*x;t=np.linalg.norm(v);R=np.eye(3)
   if t>=1e-12:
    xx,yy,zz=v/t;K=np.array([[0,-zz,yy],[zz,0,-xx],[-yy,xx,0]]);R+=np.sin(t)*K+(1-np.cos(t))*(K@K)
   E=np.eye(4);E[:3,:3]=R;T=T@a@E@b;out[p]=T.copy()
  F=np.eye(4);F[2,3]=-.1034;H=T@self.tip@F;out[self.hand_path]=H
  for sign,label in [(1,'left'),(-1,'right')]:
   F=np.eye(4);F[:3,3]=[0,sign*opening/2,.0584];F[:3,:3]=np.eye(3)if sign==1 else np.diag([-1.,-1.,1.]);out[self.hand_path+'/'+self.side+'_'+label+'_finger']=H@F
  return out
checkers={}
for side in ('left','right'):
 rows=[(str(b),str(s),proxy(local['v'+str(i)]))for i,(b,s)in enumerate(zip(local['bodies'],local['sources']))if'/'+side+'_arm_base/'in b and not str(b).endswith('_arm_fr3v2_link0')]
 model=json.loads((Path('/tmp/gpu_planning_publish_ready/left-native-kinematics.json')if side=='left'else run/'robot-kinematics.json').read_text());arm=Arm(side,model,rows);checkers[side]=SimpleNamespace(arm=arm,pair_boxes=rows,payload_boxes=[],payload_path=None,payload_relative=None)

from fixed_canonical_plan import right_carry
from aperture_path_check import _samples
run=ROOT/'isaac_port/runs/native-roas-table-fixed-front-v463-live-entry-cached-variation-plus';report=json.loads((run/'pipeline-results.json').read_text());z=np.load(run/'executed-actuator-commands.npz');park=next(x for x in report['planning_diagnostics']if x.get('event')=='large_goal_aware_empty_LEFT_park_admitted');p=np.load(ROOT/'.local/collision-gpu2/binary/large-class-local.npz');rc=checkers['right'];lc=checkers['left'];rc.payload_path='/World/Cell/Geometry/workpiece_ring1';rc.payload_boxes=[(rc.payload_path,str(s),proxy(p['v'+str(i)]))for i,s in enumerate(p['sources'])];G=np.asarray(park['fresh_RGBD_held_RIGHT_G']);F=np.eye(4);F[2,3]=.1034;rc.payload_relative=F@G;rc.resolution_rad=.005
rows=right_carry();samples=[q for _,_,q in _samples(rc,np.asarray(rows,float))];lw=park['left_openings_m'];rw=park['right_openings_m'];choices=[('actual463_bad_local2',np.asarray(park['future_left_q'])),('public444',np.array([-.13557329773902893,-.14186294376850128,-.01783299632370472,-2.220045566558838,-2.794273853302002,1.8495267629623413,-1.3566551208496094]))]
# Bounded original local-offset CPU IK candidates from PUBLIC encoder state.
li=[0,2,4,6,8,10,12];sel=np.where(z['pin_phase']=='ring1_right_goal_aware_empty_left_park')[0];ql=z['actual_dof_position'][sel[0]-1,li].astype(float)
import types
sys.modules['pxr']=types.SimpleNamespace(UsdGeom=None,UsdPhysics=None,Usd=None)
from isaac_kinematics import Arm as KArm
la=KArm.__new__(KArm)
for key in ['base','tip','axes','frames']:setattr(la,key,getattr(lc.arm,key))
la.bounds=np.asarray(json.loads(Path('/tmp/gpu_planning_publish_ready/left-native-kinematics.json').read_text())['bounds']);la.hand_path=lc.arm.hand_path;la.link_paths=lc.arm.link_paths
H=la.fk(ql)
for i,delta in enumerate([[-.10,-.10,-.08],[-.10,0.,-.10],[0.,-.12,-.08],[-.18,-.10,-.10],[-.20,0.,-.10],[0.,-.20,-.10]]):
 P=H[:3,3]+delta;q=la.solve(P,H[:3,:3],ql)
 if q is not None:choices.append(('original_local_'+str(i)+'_CPU_IK',q.astype(np.float32).astype(float)))

print('GEOMtarget',park['geometric_target_world_m'],'MODEL',la.fk(np.asarray(park['future_left_q']))[:3,3].tolist())
