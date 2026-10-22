from pathlib import Path
import json,hashlib,importlib.util,time
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path('/home/gpuadmin/workspace/jake/mujoco');H=ROOT/'isaac_port/native_hand_layout/roas-table';O=Path(__file__).parent
spec=importlib.util.spec_from_file_location('numeric_public',H/'fast_numeric_ik.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
seeds=json.load(open(H/'bin-picking/direct-pin-robot-seeds.json'))['seeds'];cat=json.load(open(ROOT/'isaac_port/runs/native-roas-table-scatter-stable-v251-corrected-pin-catalog/pin-native-grasp-catalog.json'))['candidates'][0]
def frame(P,R):
 F=np.eye(4);F[:3,3]=P;F[:3,:3]=R;return F
out=[]
for num in [534]:
 d=next((ROOT/'isaac_port/runs').glob(f'*v{num}-*'));r=json.load(open(d/'pipeline-results.json'));p=r['pin_skill_progress'];obs=json.load(open(d/'rgb-observations.json'));a=np.load(d/'executed-actuator-commands.npz');m=json.load(open(d/'robot-kinematics.json'));arm=SimpleNamespace(**{k:np.array(m[k],float)for k in ['base','tip','frames','axes','bounds']});ik=mod.NumericIK(arm)
 assert hashlib.sha256(mod.LIB.read_bytes()).hexdigest()==r['CPU_fast_kinematics']['library_sha256']
 ids=[r['dof_names'].index('right_arm_fr3v2_joint'+str(i))for i in range(1,8)]
 G=np.array(p['measured_grasp']);events=p['events'];post=[e for e in events if e['event']=='pin_lift_confirm_RGBD'];used=[]
 for e in post:
  ob=min((x for x in obs if x.get('part')=='pin'),key=lambda x:abs(x['delivered_wall_s']-e['wall_s']));assert abs(ob['delivered_wall_s']-e['wall_s'])<1e-6
  T=np.array(ob['hand_TCP_world']);pose=e['result']['pose'];inv=np.linalg.inv(T);v=inv[:3,:3]@np.array(pose['axis']);cross=np.cross(G[:3,2],v);R=Rotation.from_rotvec(cross/max(np.linalg.norm(cross),1e-12)*np.arctan2(np.linalg.norm(cross),G[:3,2]@v)).as_matrix();G=frame((inv@np.r_[pose['center_m'],1])[:3],R@G[:3,:3]);used.append(ob['work'])
 R=np.array(cat['goal_part_R']);goal=np.array(p['physical_axis_midpoint_goal_world_m']);target=frame(goal-[0,0,.045],R)@np.linalg.inv(G)
 label='pin_tail_direct_carry_to_preinsert';ix=np.flatnonzero(a['pin_phase']==label);first,last=int(ix[0]),int(ix[-1]);prior=first-1;current=a['actual_dof_position'][prior,ids];entry=a['command'][first,ids];endpoint=a['command'][last,ids]
 def fk(q):
  F=arm.base.copy()
  for v,ax,(pre,post)in zip(q,arm.axes,arm.frames):F=F@pre@frame([0,0,0],Rotation.from_rotvec(ax*v).as_matrix())@post
  return F@arm.tip
 solutions=[]
 for name,seed in [('robot_seeds.pin_direct_preinsert',seeds['pin_direct_preinsert']),('robot_seeds.pin_tail_carry_to_preinsert',seeds['pin_tail_carry_to_preinsert']),('current_encoder',current)]:
  t=time.perf_counter();q=ik.solve(target[:3,3],target[:3,:3],seed);elapsed=time.perf_counter()-t
  row={'source':name,'solve_wall_s':elapsed,'solved':q is not None}
  if q is not None:
   T=fk(q);row.update(q=q.tolist(),translation_residual_m=float(np.linalg.norm(T[:3,3]-target[:3,3])),rotation_residual_rad=float(Rotation.from_matrix(T[:3,:3]@target[:3,:3].T).magnitude()),joint_distance_from_entry_rad=float(np.linalg.norm(q-entry)),joint_distance_from_actual_endpoint_rad=float(np.linalg.norm(q-endpoint)))
  solutions.append(row)
 term=p.get('pin_terminal_branch_preflight',[]);lastlift=post[-1]['wall_s'];out.append(dict(run=num,source_run=d.name,whole_task_pass=r['full_task_pass'],terminal_candidates_checked=len(term),terminal_acceptance=[x['accepted']for x in term],terminal_individual_timers_recorded=False,GPU_preinsert=[x for x in p['GPU_IK_queries']if x['label']=='pin_preinsert_carry'],shortcut_nodes=p.get('pin_shortcut_nodes'),physical_carry_s=len(ix)/480,recorded_carry_wall_s=float(a['time_s'][last]-a['time_s'][first]),lift_confirm_to_carry_first_command_wall_s=float(a['time_s'][first]-lastlift),first_carry_command_gap_s=float(a['time_s'][first]-a['time_s'][prior]),postlift_RGBD_packets=used,public_G=G.tolist(),same_public_hand_target=target.tolist(),original_CPU_seed_IK=solutions,actual_endpoint_residual_m=float(np.linalg.norm(fk(endpoint)[:3,3]-target[:3,3])),native_collision_or_route_admission_proven=False))
result={'scope':'Read-only public RGB-D+encoder calibratedFK seed solvability. No collision/native/terminal checks or actuation. No runtime/source changes. Individual native terminal timers not instrumented.','numeric_library_sha256':hashlib.sha256(mod.LIB.read_bytes()).hexdigest(),'runs':out}
(O/'public-evidence.json').write_text(json.dumps(result,indent=2)+'\n')
for x in out:print(x['run'],'terminal',x['terminal_candidates_checked'],'carry',x['physical_carry_s'],x['recorded_carry_wall_s'],'planning',x['lift_confirm_to_carry_first_command_wall_s'],'seed',[(s['source'],s['solved'],s.get('joint_distance_from_actual_endpoint_rad'))for s in x['original_CPU_seed_IK']])
