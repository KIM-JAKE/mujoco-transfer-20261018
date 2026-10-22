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

from aperture_path_check import _samples
run=ROOT/'isaac_port/runs/native-roas-table-fixed-front-v464-whole-carry-aware-park-plus';report=json.loads((run/'pipeline-results.json').read_text());z=np.load(run/'executed-actuator-commands.npz');park=next(x for x in report['planning_diagnostics']if x.get('event')=='large_goal_aware_empty_LEFT_park_admitted');p=np.load(ROOT/'.local/collision-gpu2/binary/large-class-local.npz');rc=checkers['right'];lc=checkers['left'];rc.payload_path='/World/Cell/Geometry/workpiece_ring1';rc.payload_boxes=[(rc.payload_path,str(s),proxy(p['v'+str(i)]))for i,s in enumerate(p['sources'])];G=np.asarray(park['fresh_RGBD_held_RIGHT_G']);F=np.eye(4);F[2,3]=.1034;rc.payload_relative=F@G;lc.resolution_rad=.035
li=[0,2,4,6,8,10,12];sel=np.where(z['pin_phase']=='ring1_right_goal_aware_empty_left_park')[0];first=sel[0]-1;ql=z['actual_dof_position'][first,li].astype(float);cl=z['command'][first,li].astype(float);rows=np.vstack([cl,z['command'][sel][:,li].astype(float)]);curve=np.vstack([ql,cl,rows]);samples=[q for _,_,q in _samples(lc,curve)];contexts=[(np.asarray(rq,float),float(gap))for rq in[park['actual_RIGHT_q'],park['command_RIGHT_q']]for gap in park['right_openings_m']];pairs=[(q,float(w),oq,ow)for q in samples for w in park['left_openings_m']for oq,ow in contexts];diag={};began=time.perf_counter();mask=new.prove(lc,rc,pairs,dict(socket=str(ROOT/'.local/collision-gpu2/pair-binary.sock'),timeout_s=10,diagnostics=diag));result=dict(MODEL_only=True,source='464 public encoder/commands and fresh RGBD heldG; original fixed native CAD',sample_count=len(samples),contexts=len(contexts),states=len(pairs),GPU_CLEAR=int(mask.sum()),uncertain=int((~mask).sum()),wall_s=time.perf_counter()-began,service=diag);print(json.dumps(result),flush=True);Path('/tmp/fixed470-dense-park/actual464-park-pair-MODEL.json').write_text(json.dumps(result,indent=2))
