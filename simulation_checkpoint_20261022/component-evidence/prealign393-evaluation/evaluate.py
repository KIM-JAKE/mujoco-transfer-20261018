from pathlib import Path
import numpy as np,json,hashlib
from scipy.spatial.transform import Rotation
REPO=Path('/home/gpuadmin/workspace/jake/mujoco');R=REPO/'isaac_port/runs/native-roas-table-scatter-stable-v393-current-pair-visual-descent-all3-450'
obs=json.loads((R/'rgb-observations.json').read_text());report=json.loads((R/'pipeline-results.json').read_text());trace=np.load(R/'physx-sequence-trace.npz');paths=list(trace['camera_body_paths']);actor=report['selected_actor_ids_for_validation_only']['ring1'];ai=paths.index(actor);hi=next(i for i,p in enumerate(paths)if p.endswith('/left_NativeHand'));goal=np.asarray(json.loads((R.parent/'native-roas-table-bin-retimed14/arm-plan.json').read_text())['goals']['ring1']['P'])
def T(row):
 t=np.eye(4);t[:3,:3]=Rotation.from_quat(row[3:]).as_matrix();t[:3,3]=row[:3];return t
def angle(a,b):
 a=np.asarray(a,float);b=np.asarray(b,float);return float(np.arccos(np.clip((a@b)/(np.linalg.norm(a)*np.linalg.norm(b)),-1,1)))
rows=[]
for oi,o in list(enumerate(obs))[-6:]:
 pose=o.get('result',{}).get('pose')
 if not pose:continue
 t=o['captured_wall_s'];near=int(np.argmin(abs(trace['time_s']-t)));before=max(0,int(np.searchsorted(trace['time_s'],t,side='right')-1));A=T(trace['camera_bodies'][near,ai]);H=T(trace['camera_bodies'][near,hi]);H[:3,3]+=.1034*H[:3,2];imageH=np.asarray(o['hand_TCP_world']);P=np.asarray(pose['center_m']);axis=np.asarray(pose['axis']);actualaxis=A[:3,2]
 if actualaxis@axis<0:actualaxis=-actualaxis
 actualG=np.linalg.inv(H)@A;imageG=np.linalg.inv(imageH);imageG[:3,3]=imageG[:3,:3]@P+imageG[:3,3];imageG[:3,2]=imageH[:3,:3].T@axis
 row=dict(observation_index=oi,capture_wall_s=t,trace_index=near,trace_wall_s=float(trace['time_s'][near]),trace_simulation_s=float(trace['simulation_time_s'][near]),nearest_trace_time_difference_s=float(trace['time_s'][near]-t),previous_trace_index=before,previous_trace_wall_s=float(trace['time_s'][before]),image_center_m=P.tolist(),physical_center_m=A[:3,3].tolist(),image_minus_physical_center_m=(P-A[:3,3]).tolist(),image_radial_center_error_m=float(np.linalg.norm((P-A[:3,3])[:2])),image_3D_center_error_m=float(np.linalg.norm(P-A[:3,3])),image_axis_error_rad=angle(axis,actualaxis),image_axis_goal_error_rad=angle(axis,[0,0,1]),physical_axis_goal_error_rad=angle(actualaxis,[0,0,1]),image_radial_goal_error_m=float(np.linalg.norm((P-goal)[:2])),physical_radial_goal_error_m=float(np.linalg.norm((A[:3,3]-goal)[:2])),physical_G=actualG.tolist(),image_G_center_m=imageG[:3,3].tolist(),image_G_axis=(imageG[:3,2]/np.linalg.norm(imageG[:3,2])).tolist(),captureHand_vs_nearest_trace_position_m=float(np.linalg.norm(imageH[:3,3]-H[:3,3])),captureHand_vs_nearest_trace_rotation_rad=float(Rotation.from_matrix(imageH[:3,:3]@H[:3,:3].T).magnitude()),pose_method=pose.get('method'),image_points=pose.get('depth_points'),image_rms_m=pose.get('rms_m'))
 if rows:
  old=np.asarray(rows[-1]['physical_G']);row['physical_G_translation_change_m']=float(np.linalg.norm(actualG[:3,3]-old[:3,3]));row['physical_G_axis_change_rad']=angle(actualG[:3,2],old[:3,2])
  oldimage=rows[-1];row['image_G_translation_change_m']=float(np.linalg.norm(imageG[:3,3]-np.asarray(oldimage['image_G_center_m'])));row['image_G_axis_change_rad']=angle(imageG[:3,2],oldimage['image_G_axis'])
 prev=T(trace['camera_bodies'][before,ai]);row['nearest_vs_previous_actor_translation_m']=float(np.linalg.norm(prev[:3,3]-A[:3,3]));row['nearest_vs_previous_actor_rotation_rad']=float(Rotation.from_matrix(prev[:3,:3]@A[:3,:3].T).magnitude())
 rows.append(row)
result=dict(scope='Evaluation ONLY: saved393RGBD pose compared against private recorded PhysX actor transforms; no controller/source edits, no Isaac job or hardware commands',hardware_tested=False,source_run=R.name,goal=goal.tolist(),trace_quaternion_convention='xyzw',timestamp_basis='Recorded393observations explicitly declare captured_wall_s; scene_sim_time_s is only legacywallalias. Match trace.time_s not trace.simulation_time_s.',rows=rows,source_hashes={n:hashlib.sha256((R/n).read_bytes()).hexdigest()for n in ['rgb-observations.json','physx-sequence-trace.npz','pipeline-results.json']})
Path('/tmp/prealign393-evaluator/results.json').write_text(json.dumps(result,indent=2));print(json.dumps(rows,indent=2))
