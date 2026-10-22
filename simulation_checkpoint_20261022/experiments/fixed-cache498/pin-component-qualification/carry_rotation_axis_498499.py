"""Read-only public robot FK decomposition; no object truth/control output."""
import numpy as np,json
from scipy.spatial.transform import Rotation
from pathlib import Path
R=Path('/home/gpuadmin/workspace/jake/mujoco');out=[]
for v in[498,499]:
 r=next((R/'isaac_port/runs').glob(f'*v{v}-*'));p=json.loads((r/'pipeline-results.json').read_text());z=np.load(r/'physx-sequence-trace.npz');obs=[o for o in json.loads((r/'rgb-observations.json').read_text())if o.get('part')=='pin'and o['mode']=='held'and o.get('result',{}).get('pose')]
 # For complete494, find the last original postlift preload capture before FIRST precision.
 phases=z['pin_phase'];precision_times=z['time_s'][phases=='pin_precision_encoder_settle'];first=float(precision_times.min());previous=[o for o in obs if o['captured_wall_s']<first];following=[o for o in obs if o['captured_wall_s']>=first];o0=previous[-1];o1=following[0];t0=o0['captured_wall_s'];t1=o1['captured_wall_s'];A=np.array(o0['hand_TCP_world']);B=np.array(o1['hand_TCP_world']);rv=Rotation.from_matrix(A[:3,:3].T@B[:3,:3]).as_rotvec();axis=rv/np.linalg.norm(rv)
 # q is the recorded public robot SDK joint position; original numeric FK.
 model=json.loads((r/'robot-kinematics.json').read_text());names=p['dof_names'];ids=[names.index('right_arm_fr3v2_joint'+str(i))for i in range(1,8)]
 def fk(q):
  T=np.array(model['base'])
  for value,ax,(pre,post)in zip(q,model['axes'],model['frames']):
   X=np.eye(4);X[:3,:3]=Rotation.from_rotvec(np.array(ax)*value).as_matrix();T=T@pre@X@post
  return T@model['tip']
 idx=np.where((z['time_s']>=t0)&(z['time_s']<=t1))[0];frames=[fk(q[ids])for q in z['q'][idx]];body_omegas=np.asarray([Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).as_rotvec()for a,b in zip(frames[:-1],frames[1:])]);bodyabs=np.abs(body_omegas).sum(0);length=np.linalg.norm(body_omegas,axis=1).sum();Gaxis=A[:3,:3].T@np.array(o0['result']['pose']['axis']);P0=np.asarray(o0['result']['pose']['center_m']);G=np.linalg.inv(A);G[:3,3]=(np.linalg.inv(A)@np.r_[P0,1.])[:3];# Need original catalogue gauge, only axis direction used for candidate diagnostics.
 row=dict(run=r.name,scope='Public RGBD captures+robot SDKq/original numeric FK only; no object truth',start_work=Path(o0['work']).name,end_work=Path(o1['work']).name,t0=t0,t1=t1,endpoint_rotation_rad=float(np.linalg.norm(rv)),endpoint_rotation_axis_in_initialHand=axis.tolist(),absolute_closingY_dot_endpoint_axis=float(abs(axis[1])),integral_absolute_local_axis_rad=bodyabs.tolist(),integral_rotation_path_rad=float(length),closingY_integral_absolute_fraction=float(bodyabs[1]/length),start_public_PIN_axis_in_Hand=Gaxis.tolist());out.append(row);print(json.dumps(row,indent=2))
Path('/tmp/fixed496-pin-diagnosis/public-component-carry-rotation-axis.json').write_text(json.dumps(out,indent=2))
