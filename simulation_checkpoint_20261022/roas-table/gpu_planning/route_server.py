"""Optional collision-aware GPU route proposals; original Isaac checks mandatory."""
from pathlib import Path
import hashlib,json,socket,time,sys
import numpy as np
import torch,yaml
from curobo.motion_planner import MotionPlanner,MotionPlannerCfg
from curobo.types import DeviceCfg,JointState,Pose
from curobo._src.geom.collision.buffer_collision import CollisionBuffer
from service_paths import ROOT,SOURCE,VENDOR
sys.stdout=open(ROOT/'route-server.log','a',buffering=1);sys.stderr=sys.stdout;SOCKET=ROOT/'route.sock';raw=json.loads(SOURCE.read_text());bounds=np.asarray(raw['bounds']);canonical={k:np.round(np.asarray(raw[k],float),12).tolist()for k in ['base','tip','frames','axes','bounds']};MODEL_SHA=hashlib.sha256(json.dumps(canonical,sort_keys=True,separators=(',',':')).encode()).hexdigest();NBOX=192
if SOCKET.exists():
 try:
  test=socket.socket(socket.AF_UNIX);test.settimeout(.2);test.connect(str(SOCKET));test.close();raise RuntimeError('Route service already alive; refusing replacement')
 except (ConnectionRefusedError,FileNotFoundError):SOCKET.unlink()
DEV=DeviceCfg(device=torch.device('cuda:0'),dtype=torch.float32);robot=json.loads((ROOT/'route-robot.json').read_text());native=json.loads((ROOT/'native-route-spheres.json').read_text());esdf=json.loads((ROOT/'fixed-proposal-esdf.json').read_text());from curobo._src.geom.types import SceneCfg,VoxelGrid,Cuboid
world=SceneCfg(voxel=[VoxelGrid(name='fixed_CAD_ESDF',dims=esdf['dims_m'],pose=esdf['center_world_m']+[1,0,0,0],voxel_size=esdf['voxel_size_m'],feature_tensor=torch.as_tensor(np.load(ROOT/'fixed-proposal-esdf.npy'),device='cuda:0'))],cuboid=[Cuboid(name=f'dynamic{i}',dims=[.001]*3,pose=[1000+i,1000,1000,1,0,0,0])for i in range(NBOX)]);graph=yaml.safe_load((VENDOR/'curobo/content/configs/task/graph_planner/exact_graph_planner.yml').read_text());graph['graph_planner'].update(max_nodes=4000,max_path_finding_iterations=4,min_finetune_iterations=1);started=time.perf_counter();planner=MotionPlanner(MotionPlannerCfg.create(robot=robot,scene_model=world,graph_planner_config=graph,device_cfg=DEV,num_ik_seeds=16,num_trajopt_seeds=4,use_cuda_graph=True,self_collision_check=True,collision_cache={'cuboid':NBOX},optimizer_collision_activation_distance=.001,interpolation_dt=.025));planner.scene_collision_checker.data.voxels.params[0,0,:3].copy_(torch.tensor(esdf['shape'],device='cuda:0',dtype=torch.float32));print('GPU_ROUTE_MODEL_LOADED',time.perf_counter()-started,flush=True)
def state(q):return JointState.from_position(torch.tensor(q,device='cuda:0',dtype=torch.float32)[None],joint_names=[f'joint{i+1}'for i in range(7)])
def set_native(opening,payload):
 rows=[list(s['center'])+[s['radius']]for s in native['palm']]
 for sign,key in [(1,'left'),(-1,'right')]:
  for s in native[key]:rows.append((np.asarray(s['center'])*[sign,sign,1]+[0,sign*opening/2,-.045]).tolist()+[s['radius']])
 rows.append([0,0,0,-1]if payload is None else list(payload['center_TCP_m'])+[float(payload['radius_m'])]);tensor=torch.tensor(rows,device='cuda:0',dtype=torch.float32)
 for kin in [planner.kinematics,planner.ik_solver.kinematics]:kin.config.kinematics_config.update_link_spheres('tcp',tensor)
def set_obstacles(boxes):
 if len(boxes)>NBOX:raise ValueError('Too many observed/encoder obstacle envelopes')
 from scipy.spatial.transform import Rotation
 dims=np.zeros((NBOX,4),np.float32);invpose=np.zeros((NBOX,8),np.float32);invpose[:,3]=1;enabled=np.zeros(NBOX,np.uint8)
 if boxes:
  D=np.asarray([box['dims_m']for box in boxes],float);P=np.asarray([box['pose_world_wxyz']for box in boxes],float)
  if D.shape!=(len(boxes),3)or P.shape!=(len(boxes),7)or not np.isfinite(D).all()or not np.isfinite(P).all()or np.any(D<=0)or np.any(abs(np.linalg.norm(P[:,3:],axis=1)-1)>1e-5):raise ValueError('Invalid current encoder/RGBD obstacle boxes')
  R=Rotation.from_quat(P[:,[4,5,6,3]])
  invpose[:len(boxes),:3]=-R.inv().apply(P[:,:3]);invpose[:len(boxes),3:7]=R.inv().as_quat()[:,[3,0,1,2]];dims[:len(boxes),:3]=D;enabled[:len(boxes)]=1
 # One batched copy per public CuboidData tensor; identical per-box dimensions,
 # inverse transforms and enable flags to its update_dims/update_pose methods.
 cuboids=planner.scene_collision_checker.data.cuboids
 cuboids.dims[0].copy_(torch.as_tensor(dims,device='cuda:0'))
 cuboids.inv_pose[0].copy_(torch.as_tensor(invpose,device='cuda:0'))
 cuboids.enable[0].copy_(torch.as_tensor(enabled,device='cuda:0'))
 planner.graph_planner.reset_buffer()
def collision_hint(q):
 s=planner.compute_kinematics(state(q));buf=CollisionBuffer.from_shape(s.robot_spheres.shape,DEV);v=planner.scene_collision_checker.get_sphere_distance(s,buf,torch.ones(1,device='cuda:0'),torch.zeros(1,device='cuda:0')).detach().cpu().numpy().reshape(-1);idx=np.argsort(v)[-5:][::-1];spheres=s.robot_spheres.detach().cpu().numpy().reshape(-1,4);vox=planner.scene_collision_checker.data.voxels
 rows=[]
 for i in idx:
  if v[i]<=1e-6:continue
  point=spheres[i];xyz=(point[:3]-esdf['center_world_m'])/esdf['voxel_size_m']+np.asarray(esdf['shape'])/2-.5;ijk=np.clip(np.round(xyz).astype(int),0,np.array(esdf['shape'])-1);flat=int(np.ravel_multi_index(ijk,esdf['shape']));rows.append(dict(sphere=int(i),penetration_cost=float(v[i]),world_xyzr=point.tolist(),grid_nearest_feature=float(vox.features.reshape(-1)[flat].item()),voxel_params=vox.params.cpu().numpy().tolist(),voxel_inv_pose=vox.inv_pose.cpu().numpy().tolist()))
 return rows
set_obstacles([]);set_native(.08,None);q=np.asarray(raw['seed']);g=q.copy();g[0]=np.clip(g[0]+.015,bounds[0,0],bounds[0,1]);planner.plan_cspace(state(g),state(q),max_attempts=1,enable_graph_attempt=1);torch.cuda.synchronize();print(json.dumps(dict(event='GPU_ROUTE_RPC_READY',model_sha256=MODEL_SHA,socket=str(SOCKET),startup_s=time.perf_counter()-started,fixed_triangles=8808756,original_PhysX_fixed_pair_payload_and_FR3_retime_mandatory=True)),flush=True);server=socket.socket(socket.AF_UNIX);server.bind(str(SOCKET));SOCKET.chmod(0o660);server.listen(4)
while True:
 conn,_=server.accept();conn.settimeout(3.);start=time.perf_counter();request={}
 try:
  data=b''
  while b'\n'not in data:
   part=conn.recv(65536)
   if not part:raise ValueError('Incomplete request')
   data+=part
   if len(data)>2097152:raise ValueError('Oversized request')
  request=json.loads(data.split(b'\n',1)[0]);a=np.asarray(request['start_q'],float);b=np.asarray(request['goal_q'],float);opening=float(request['opening_m'])
  if request['model_sha256']!=MODEL_SHA:raise ValueError('Production native FR3 model hash mismatch')
  if a.shape!=(7,)or b.shape!=(7,)or not np.isfinite(np.r_[a,b,opening]).all()or np.any(a<bounds[:,0])or np.any(a>bounds[:,1])or np.any(b<bounds[:,0])or np.any(b>bounds[:,1])or not 0<=opening<=.08:raise ValueError('Invalid joint bounds/native aperture')
  update_start=time.perf_counter();set_native(opening,request.get('payload'));torch.cuda.synchronize();native_s=time.perf_counter()-update_start;update_start=time.perf_counter();set_obstacles(request.get('obstacle_boxes',[]));torch.cuda.synchronize();obstacle_s=time.perf_counter()-update_start;plan_start=time.perf_counter();r=planner.plan_cspace(state(b),state(a),max_attempts=1,enable_graph_attempt=1);torch.cuda.synchronize();route=None
  if r is not None and bool(r.success.any()):
   curve=r.js_solution.position.detach().cpu().numpy().reshape(-1,7);curve[0]=a;curve[-1]=b
   if len(curve)>=2 and np.isfinite(curve).all()and np.all(curve>=bounds[:,0])and np.all(curve<=bounds[:,1]):route=curve.tolist()
  response=dict(id=request['id'],model_sha256=MODEL_SHA,start_q=a.tolist(),goal_q=b.tolist(),opening_m=opening,route=route,solve_s=time.perf_counter()-start,native_update_s=native_s,obstacle_update_s=obstacle_s,plan_s=time.perf_counter()-plan_start,proposal_only=True,collision_world='8mmESDF of fixed CAD (proposal only) + current encoder/RGBD bounds; approximate native/FR3 spheres; original checker mandatory',physical_motion_executed=False)
  if route is None:response['start_sphere_collisions']=collision_hint(a);response['goal_sphere_collisions']=collision_hint(b)
 except Exception as e:response=dict(id=request.get('id'),error=type(e).__name__+': '+str(e),proposal_only=True)
 try:conn.sendall(json.dumps(response).encode()+b'\n')
 except (BrokenPipeError,ConnectionResetError):pass
 finally:conn.close()
 print(json.dumps(dict(event='GPU_ROUTE_RPC',id=response.get('id'),knots=len(response.get('route')or[]),error=response.get('error'),start_sphere_collisions=response.get('start_sphere_collisions'),goal_sphere_collisions=response.get('goal_sphere_collisions'),wall_s=time.perf_counter()-start,native_update_s=response.get('native_update_s'),obstacle_update_s=response.get('obstacle_update_s'),plan_s=response.get('plan_s'))),flush=True)
