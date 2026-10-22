from pathlib import Path
import json,numpy as np,importlib.util,hashlib
from types import SimpleNamespace
from scipy.spatial.transform import Rotation
R=Path('/home/gpuadmin/workspace/jake/mujoco'); H=R/'isaac_port/native_hand_layout/roas-table';OUT=Path(__file__).parent
spec=importlib.util.spec_from_file_location('numeric_public',H/'fast_numeric_ik.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
seeds=json.loads((H/'bin-picking/direct-pin-robot-seeds.json').read_text())['seeds'];seed=np.asarray(seeds['pin_tail_approach'])
catalog=json.loads((R/'isaac_port/runs/native-roas-table-scatter-stable-v251-corrected-pin-catalog/pin-native-grasp-catalog.json').read_text());cand=catalog['candidates'][0]
spec=importlib.util.spec_from_file_location('collar_public',OUT.parent/'fixed504-pin-collar-flat/overrides/pin_collar_contact_target.py');collar=importlib.util.module_from_spec(spec);spec.loader.exec_module(collar);cand,_=collar.retarget(cand,-.001)
rows=[]
for n in (507,508,509):
 p=next((R/'isaac_port/runs').glob(f'*v{n}-*'));j=json.loads((p/'pipeline-results.json').read_text());m=json.loads((p/'robot-kinematics.json').read_text());arm=SimpleNamespace(**{k:np.array(m[k],float)for k in ('base','tip','frames','axes','bounds')});ik=mod.NumericIK(arm)
 assert hashlib.sha256(mod.LIB.read_bytes()).hexdigest()==j['CPU_fast_kinematics']['library_sha256']
 pose=j['pin_skill_progress']['events'][0]['result']['pose'];P=np.array(pose['center_m']);T=np.array(cand['pickup_H']);T[:3,3]+=P-np.array(catalog['pickup_center_from_RGBD']);T[2,3]+=.07
 q=ik.solve(T[:3,3],T[:3,:3],seed);assert q is not None
 def fk(q):
  F=arm.base.copy()
  for x,a,(pre,post)in zip(q,arm.axes,arm.frames):
   X=np.eye(4);X[:3,:3]=Rotation.from_rotvec(a*x).as_matrix();F=F@pre@X@post
  return F@arm.tip
 F=fk(q);assert np.all(q>=arm.bounds[:,0]) and np.all(q<=arm.bounds[:,1]);assert abs(q[5])<2 and q[6]<0
 # Original descend algorithm: same7knots, seeded current approved prepose.
 knots=[q];D=T.copy();D[2,3]-=.07
 for u in np.linspace(0,1,7)[1:]:
  K=T.copy();K[:3,3]=T[:3,3]*(1-u)+D[:3,3]*u;x=ik.solve(K[:3,3],K[:3,:3],knots[-1]);assert x is not None;knots.append(x)
 rows.append(dict(run=n,source_run=p.name,public_initial_center=P.tolist(),same_fresh_RGBD_pre_target=T.tolist(),existing_seed=seed.tolist(),solved_pre_q=q.tolist(),solved_pickup_q=knots[-1].tolist(),position_residual_m=float(np.linalg.norm(F[:3,3]-T[:3,3])),rotation_residual_rad=float(Rotation.from_matrix(F[:3,:3]@T[:3,:3].T).magnitude()),all_original_bounds=True,original_line_seven_IK_knots_solved=True,whole_scene_collision_proof=False))
s=dict(scope='Original sealed numericIK+public samecalibratedmodel, initialRGBD+original catalogue fixed−1mm target. No actorposes or physics/scene collision admission.',library_sha256=hashlib.sha256(mod.LIB.read_bytes()).hexdigest(),rows=rows)
(OUT/'public-ik-probe.json').write_text(json.dumps(s,indent=2)+'\n');print(json.dumps(s,indent=2))
