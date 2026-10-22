from pathlib import Path
import numpy as np,json
from scipy.spatial.transform import Rotation
R=Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/runs');p=next(R.glob('*v519-*'));O=Path(__file__).parent;j=json.loads((p/'pipeline-results.json').read_text());pin=j['pin_skill_progress'];z=np.load(p/'physx-sequence-trace.npz');idx=list(z['part_names']).index('pin');m=json.loads((p/'robot-kinematics.json').read_text());ids=[j['dof_names'].index(f'right_arm_fr3v2_joint{i}')for i in range(1,8)];goal=np.array(pin['physical_axis_midpoint_goal_world_m']);obs=json.loads((p/'rgb-observations.json').read_text());lastentry=next(e for e in reversed(pin['events'])if e['event']=='pin_entry_RGBD');v=min((x for x in obs if x.get('part')=='pin'and x['result'].get('pose')),key=lambda x:abs(x['delivered_wall_s']-lastentry['wall_s']));rows=[]
def fk(q):
 T=np.array(m['base'])
 for x,a,(pre,post)in zip(q,m['axes'],m['frames']):
  X=np.eye(4);X[:3,:3]=Rotation.from_rotvec(np.array(a)*x).as_matrix();T=T@np.array(pre)@X@np.array(post)
 return T@np.array(m['tip'])
def sample(label,t):
 k=int(np.argmin(abs(z['time_s']-t)));body=z['parts'][k,idx];A=Rotation.from_quat(body[3:]).as_matrix();P=body[:3]+A@np.array([0,-.00025,0]);axis=A[:,2];H=fk(z['q'][k,ids]);center=(np.linalg.inv(H)@np.r_[P,1])[:3];a=H[:3,:3].T@axis
 row=dict(label=label,requested_wall_s=float(t),sample_wall_s=float(z['time_s'][k]),time_offset_s=float(z['time_s'][k]-t),simulation_time_s=float(z['simulation_time_s'][k]),phase=str(z['pin_phase'][k]),physical_axis_midpoint=P.tolist(),physical_axis=axis.tolist(),physical_position_error_m=float(np.linalg.norm(P-goal)),physical_radial_error_m=float(np.linalg.norm(P[:2]-goal[:2])),physical_signed_axial_remaining_m=float(goal[2]-P[2]),physical_axis_error_rad=float(np.arccos(np.clip(axis[2],-1,1))),SDK_Hand=H.tolist(),physical_hand_relative_center=center.tolist(),physical_hand_relative_axis=a.tolist());rows.append(row);return row
pre=sample('last_camera_qualified_ENTRY',v['captured_wall_s']);end=sample('final18mm_command_finished',next(e['wall_s']for e in pin['events']if e['event']=='pin_single_axial_insert'));final=sample('after_all13_failed_full_depth_views',z['time_s'][-1])
def diff(a,b):
 H1,H2=[np.array(x['SDK_Hand'])for x in(a,b)];P1,P2=[np.array(x['physical_axis_midpoint'])for x in(a,b)];g1,g2=[np.array(x['physical_hand_relative_center'])for x in(a,b)];ax1,ax2=[np.array(x['physical_hand_relative_axis'])for x in(a,b)];return dict(Hand_translation_m=(H2[:3,3]-H1[:3,3]).tolist(),physical_PIN_translation_m=(P2-P1).tolist(),physical_Hand_relative_drift_m=float(np.linalg.norm(g2-g1)),physical_Hand_relative_axis_drift_rad=float(np.arccos(np.clip(np.dot(ax1,ax2),-1,1))))
result=dict(scope='EVALUATOR ONLY actualphysicaltracePINpose; forbidden as controller sensor. PublicSDKFK comparison used solely postrun. Not a task pass because qualifiedfull-depth measurement unavailable.',source_run=p.name,goal=goal.tolist(),physical_final_acceptance_geometry_only=bool(final['physical_position_error_m']<.0015 and final['physical_axis_error_rad']<.05),rows=rows,last18mm_delta=diff(pre,end),poststroke_camera_wait_delta=diff(end,final),public_lastentry_pose=v['result']['pose'],runtime_control_used_actor_pose=False)
(O/'qualification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
