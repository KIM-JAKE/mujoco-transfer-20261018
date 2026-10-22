import sys,json,time,hashlib
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path[:0]=['/tmp/future-pair-candidate-filter/overrides','/tmp/fixed476-full/overrides','/home/gpuadmin/workspace/jake/mujoco/isaac_port/native_hand_layout/roas-table']
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


sys.modules['pxr']=SimpleNamespace(UsdGeom=None,UsdPhysics=None,Usd=None)
from fixed_canonical_plan import right_carry
from future_saved_carry_guard import _pair_candidate_filter
run=ROOT/'isaac_port/runs/native-roas-table-fixed-front-v463-live-entry-cached-variation-plus';report=json.loads((run/'pipeline-results.json').read_text());park=next(x for x in report['planning_diagnostics']if x.get('event')=='large_goal_aware_empty_LEFT_park_admitted');p=np.load(ROOT/'.local/collision-gpu2/binary/large-class-local.npz');rc=checkers['right'];lc=checkers['left'];rc.payload_path='/World/Cell/Geometry/workpiece_ring1';rc.payload_boxes=[(rc.payload_path,str(s),proxy(p['v'+str(i)]))for i,s in enumerate(p['sources'])];G=np.asarray(park['fresh_RGBD_held_RIGHT_G']);F=np.eye(4);F[2,3]=.1034;rc.payload_relative=F@G;rc.resolution_rad=.005
c=dict(config=dict(gpu_pair_schedule_certificate=True,gpu_pair_schedule_socket=str(ROOT/'.local/collision-gpu2/pair-binary.sock'),gpu_pair_schedule_timeout_s=10),_collision_physics_epoch=0)
qs=right_carry();rw=np.asarray(park['right_openings_m']);lw=np.asarray(park['left_openings_m']);choices=[('bad463_local2',np.asarray(park['future_left_q'])),('public444',np.array([-.13557329773902893,-.14186294376850128,-.01783299632370472,-2.220045566558838,-2.794273853302002,1.8495267629623413,-1.3566551208496094]))];out=[]
def cpu_pair(ql,qr):
 left=at_pose(lc,ql,float(lw[0]),lazy=True);right=at_pose(rc,qr,float(rw[0]),lazy=True)
 def bounds(rows):
  centers=np.array([r[2][0]for r in rows]);extents=np.array([abs(r[2][1])@(r[2][2]+.0005)for r in rows]);return centers-extents,centers+extents
 al,ah=bounds(left);bl,bh=bounds(right);active=np.argwhere(np.all(ah[:,None,:]>=bl[None,:,:],2)&np.all(bh[None,:,:]>=al[:,None,:],2))
 for i,j in active:
  if shape_overlap(left[i][2],right[j][2],.0005):return dict(clear=False,left=left[i][1],right=right[j][1])
 return dict(clear=True)
for label,q in choices:
 accepted,stats=_pair_candidate_filter(c,lc,rc,[(q,float(w))for w in lw],np.asarray(qs,float),rw);cpu=cpu_pair(q,np.asarray(qs[1171],float));out.append(dict(label=label,candidate_pair_ALL_CLEAR=accepted,original_CPU_pair_at_known_critical_row1171=cpu,stats=stats));print(json.dumps(dict(label=label,ALL_CLEAR=accepted,wall_s=stats.get('wall_s'),states=stats.get('pair_states'),unknown=stats.get('GPU_unknown_states'),original_CPU=cpu)),flush=True)
assert not out[0]['candidate_pair_ALL_CLEAR']and not out[0]['original_CPU_pair_at_known_critical_row1171']['clear'];assert out[1]['candidate_pair_ALL_CLEAR']and out[1]['original_CPU_pair_at_known_critical_row1171']['clear']
Path('/tmp/future-pair-candidate-filter/native-MODEL.json').write_text(json.dumps(dict(scope='Exact native+heldCAD original463 public RGBD/robot FK. GPU CLEAR candidatefilter only; unknown retains original full future guard. Actual motion gates unchanged.',results=out),indent=2))
