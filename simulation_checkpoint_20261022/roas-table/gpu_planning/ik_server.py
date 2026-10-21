"""Persistent exact-native-FR3 GPU IK proposal service; never commands robots."""
from pathlib import Path
import hashlib,json,socket,time,sys
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from curobo.inverse_kinematics import InverseKinematics,InverseKinematicsCfg
from curobo._src.types.robot import RobotCfg
from curobo.types import DeviceCfg,Pose,JointState
from curobo._src.types.tool_pose import GoalToolPose
from service_paths import ROOT,SOURCE,URDF
sys.stdout=open(ROOT/'ik-server.log','a',buffering=1);sys.stderr=sys.stdout
raw=json.loads(SOURCE.read_text());fields=['base','tip','frames','axes','bounds'];canonical={k:np.round(np.asarray(raw[k],float),12).tolist()for k in fields};MODEL_SHA=hashlib.sha256(json.dumps(canonical,sort_keys=True,separators=(',',':')).encode()).hexdigest();bounds=np.asarray(raw['bounds']);SOCKET=ROOT/'ik.sock'
if SOCKET.exists():
 try:
  test=socket.socket(socket.AF_UNIX);test.settimeout(.2);test.connect(str(SOCKET));test.close();raise RuntimeError('Existing IK service is alive; refusing replacement')
 except (ConnectionRefusedError,FileNotFoundError):SOCKET.unlink()
DEV=DeviceCfg(device=torch.device('cuda:0'),dtype=torch.float32);robot=RobotCfg.from_basic(str(URDF),'world',['tcp'],DEV);solver=InverseKinematics(InverseKinematicsCfg.create(robot=robot,device_cfg=DEV,num_seeds=32,self_collision_check=False,load_collision_spheres=False,use_cuda_graph=True,position_tolerance=1e-5,orientation_tolerance=1e-4));JOINT_NAMES=[f'joint{i+1}'for i in range(7)]
def fk(q):
 T=np.asarray(raw['base']).copy()
 for x,axis,(a,b)in zip(q,raw['axes'],raw['frames']):
  R=np.eye(4);R[:3,:3]=Rotation.from_rotvec(np.asarray(axis)*x).as_matrix();T=T@a@R@b
 return T@raw['tip']
def solve(target,current):
 quat=Rotation.from_matrix(target[:3,:3]).as_quat()[[3,0,1,2]];goal=GoalToolPose.from_poses({'tcp':Pose(position=torch.tensor(target[:3,3],device='cuda:0',dtype=torch.float32)[None,None],quaternion=torch.tensor(quat,device='cuda:0',dtype=torch.float32)[None,None])},ordered_tool_frames=['tcp']);state=JointState.from_position(torch.tensor(current,device='cuda:0',dtype=torch.float32)[None],joint_names=JOINT_NAMES);r=solver.solve_pose(goal_tool_poses=goal,current_state=state,return_seeds=16);torch.cuda.synchronize();rows=[]
 for q,ok in zip(r.solution.cpu().numpy().reshape(-1,7),r.success.cpu().numpy().reshape(-1)):
  if not ok or not np.isfinite(q).all()or np.any(q<bounds[:,0])or np.any(q>bounds[:,1]):continue
  H=fk(q);pe=float(np.linalg.norm(H[:3,3]-target[:3,3]));ae=float(np.linalg.norm(Rotation.from_matrix(H[:3,:3]@target[:3,:3].T).as_rotvec()))
  if pe>1e-5 or ae>1e-4 or any(np.linalg.norm(q-v['q'])<.025 for v in rows):continue
  rows.append(dict(q=q.tolist(),position_error_m=pe,rotation_error_rad=ae))
 rows.sort(key=lambda v:np.linalg.norm(np.asarray(v['q'])-current));return rows
seed=np.asarray(raw['seed']);solve(fk(seed),seed);print(json.dumps(dict(event='GPU_IK_RPC_READY',model_sha256=MODEL_SHA,socket=str(SOCKET),device=torch.cuda.get_device_name(0),CUDA_graph=True)),flush=True);server=socket.socket(socket.AF_UNIX);server.bind(str(SOCKET));SOCKET.chmod(0o660);server.listen(4)
while True:
 conn,_=server.accept();conn.settimeout(3.);start=time.perf_counter();request={}
 try:
  buf=b''
  while b'\n'not in buf:
   part=conn.recv(8192)
   if not part:raise ValueError('Incomplete request')
   buf+=part
   if len(buf)>65536:raise ValueError('Oversized request')
  request=json.loads(buf.split(b'\n',1)[0]);target=np.asarray(request['target_TCP_world'],float);current=np.asarray(request['current_q'],float)
  if request['model_sha256']!=MODEL_SHA:raise ValueError('Production native FR3 model hash mismatch')
  if target.shape!=(4,4)or current.shape!=(7,)or not np.isfinite(target).all()or not np.isfinite(current).all()or not np.allclose(target[3],[0,0,0,1],atol=1e-10)or np.max(abs(target[:3,:3].T@target[:3,:3]-np.eye(3)))>1e-5 or np.linalg.det(target[:3,:3])<.99999:raise ValueError('Invalid finite TCP/current encoder request')
  if np.any(current<bounds[:,0]-.02)or np.any(current>bounds[:,1]+.02):raise ValueError('Current joints outside production model envelope')
  rows=solve(target,np.clip(current,bounds[:,0],bounds[:,1]));response=dict(id=request['id'],model_sha256=MODEL_SHA,target_TCP_world=target.tolist(),proposals=rows,solve_s=time.perf_counter()-start,proposal_only=True,physical_motion_executed=False)
 except Exception as e:response=dict(id=request.get('id'),error=str(e),proposal_only=True)
 try:conn.sendall(json.dumps(response).encode()+b'\n')
 except (BrokenPipeError,ConnectionResetError):pass
 finally:conn.close()
 print(json.dumps(dict(event='GPU_IK_RPC',id=response.get('id'),solutions=len(response.get('proposals',[])),error=response.get('error'),wall_s=time.perf_counter()-start)),flush=True)
