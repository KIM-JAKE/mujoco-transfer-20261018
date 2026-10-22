"""Read-only runtime reports plus POST-RUN evaluator-only relative grip trace."""
import json,numpy as np,hashlib
from pathlib import Path
from scipy.spatial.transform import Rotation
root=Path('/home/gpuadmin/workspace/jake/mujoco');out=[]
for v in[502,503]:
 r=next((root/'isaac_port/runs').glob(f'*v{v}-*'));p=json.load(open(r/'pipeline-results.json'));s=p['pin_skill_progress'];tr=np.load(r/'physx-sequence-trace.npz');model=json.load(open(r/'robot-kinematics.json'));names=p['dof_names'];ids=[names.index('right_arm_fr3v2_joint'+str(i))for i in range(1,8)];pi=tr['part_names'].tolist().index('pin');record=dict(run=r.name,pin_only_component=p['pin_only_component'],full_task_pass=p['full_task_pass'],wall_s=p['wall_verified_elapsed_s'],error=p['error'].splitlines()[-1],pin_progress=s,contact_records=len(p['pin_contact_evaluator_only']['records']),EVALUATION_ONLY=True,used_for_control=False)
 def fk(q):
  H=np.array(model['base'])
  for val,ax,(pre,post)in zip(q,model['axes'],model['frames']):
   X=np.eye(4);X[:3,:3]=Rotation.from_rotvec(np.asarray(ax)*val).as_matrix();H=H@pre@X@post
  return H@model['tip']
 traces=[]
 for k,phase in enumerate(tr['pin_phase']):
  if str(phase)not in['pin_confirmed_postlift_preload_ramp','pin_tail_orient_free','pin_tail_carry_to_preinsert','pin_precision_encoder_settle']:continue
  H=fk(tr['q'][k,ids]);x=tr['parts'][k,pi];R=Rotation.from_quat(x[3:]).as_matrix();P=x[:3]+R@np.array([0,-.00025,0]);g=(np.linalg.inv(H)@np.r_[P,1])[:3];axis=H[:3,:3].T@R[:,2];traces.append(dict(phase=str(phase),t=float(tr['time_s'][k]),Gcenter=g,Gaxis=axis,H=H))
 phases=[]
 for phase in['pin_tail_orient_free','pin_tail_carry_to_preinsert','pin_precision_encoder_settle']:
  rr=[x for x in traces if x['phase']==phase]
  if not rr:continue
  first,last=rr[0],rr[-1];rot=Rotation.from_matrix(first['H'][:3,:3].T@last['H'][:3,:3]).as_rotvec();angular=np.asarray([Rotation.from_matrix(a['H'][:3,:3].T@b['H'][:3,:3]).as_rotvec()for a,b in zip(rr[:-1],rr[1:])]);length=np.linalg.norm(angular,axis=1).sum();a=np.asarray(first['Gaxis']);b=np.asarray(last['Gaxis']);phases.append(dict(phase=phase,wall_start_s=first['t'],wall_end_s=last['t'],grip_center_step_mm=float(np.linalg.norm(last['Gcenter']-first['Gcenter'])*1000),grip_axis_step_mrad=float(np.arccos(np.clip(a@b/np.linalg.norm(a)/np.linalg.norm(b),-1,1))*1000),Hand_endpoint_rotation_rad=float(np.linalg.norm(rot)),Hand_endpoint_rotation_axis_local=(rot/max(np.linalg.norm(rot),1e-12)).tolist(),Hand_local_closingY_integral_fraction=float(np.abs(angular[:,1]).sum()/length)if length>0 else 0.))
 record['EVALUATOR_relative_grip_by_actual_segment']=phases;out.append(record);print(v,p['wall_verified_elapsed_s'],record['error']);print(json.dumps(phases,indent=2))
Path('/tmp/fixed502-pin-air-tangent/actual502503-failed-EVALUATION.json').write_text(json.dumps(out,indent=2))
