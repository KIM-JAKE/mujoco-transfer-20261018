from pathlib import Path
import numpy as np,json
from scipy.spatial.transform import Rotation
p=next(Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/runs').glob('*v526*'));r=json.load(open(p/'pipeline-results.json'));pin=r['pin_skill_progress'];d=np.load(p/'executed-actuator-commands.npz');t=np.load(p/'physx-sequence-trace.npz');k=json.load(open(p/'robot-kinematics.json'));names=r['dof_names'];ids=[names.index('right_arm_fr3v2_joint'+str(j))for j in range(1,8)];fids=[i for i,n in enumerate(names)if n.startswith('right_native_')];goal=np.array(pin['physical_axis_midpoint_goal_world_m'])
def fk(q):
 T=np.array(k['base'])
 for x,a,(pre,post)in zip(q,k['axes'],k['frames']):
  X=np.eye(4);X[:3,:3]=Rotation.from_rotvec(np.asarray(a)*x).as_matrix();T=T@np.array(pre)@X@np.array(post)
 return T@np.array(k['tip'])
def truth_pose(row):
 tr=t['parts'][row,list(t['part_names']).index('pin')];R=Rotation.from_quat(tr[3:]).as_matrix();return tr[:3]+R@np.array([0,-.00025,0]),R[:,2]
ix=np.flatnonzero(d['pin_phase']=='pin_single_axial_insert');before=ix[0]-1;after=ix[-1];time0,time1=d['time_s'][before],d['time_s'][after]
rows=np.flatnonzero((t['time_s']>=time0-.02)&(t['time_s']<=time1+.03));out=[]
for n in rows:
 wall=float(t['time_s'][n]);j=max(0,np.searchsorted(d['time_s'],wall,side='right')-1);A=fk(d['actual_dof_position'][j,ids]);C=fk(d['command'][j,ids]);P,a=truth_pose(n);G=np.linalg.inv(A)@np.r_[P,1.];out.append(dict(wall_s=wall,physical_s=float(t['simulation_time_s'][n]),phase=str(d['pin_phase'][j]),command_H=C.tolist(),actual_H=A.tolist(),physical_PIN_center_EVAL_ONLY=P.tolist(),physical_PIN_axis_EVAL_ONLY=a.tolist(),nose_z_EVAL_ONLY=float(P[2]+.01775*a[2]),nose_XY_error_m_EVAL_ONLY=(P[:2]+.01775*a[:2]-goal[:2]).tolist(),center_XY_error_m_EVAL_ONLY=(P[:2]-goal[:2]).tolist(),physical_G_translation_EVAL_ONLY=G[:3].tolist(),actual_finger_gap_m=float(d['actual_dof_position'][j,fids].sum()),command_finger_gap_m=float(d['command'][j,fids].sum())))
obs=json.load(open(p/'rgb-observations.json'));public=[]
for e in pin['events']:
 if e['event']not in ['pin_entry_RGBD','pin_full_depth_RGBD']:continue
 ob=min([o for o in obs if o.get('part')=='pin'],key=lambda o:abs(o['delivered_wall_s']-e['wall_s']));P=np.array(ob['result']['pose']['center_m']);a=np.array(ob['result']['pose']['axis']);A=np.array(ob['hand_TCP_world']);j=max(0,np.searchsorted(d['time_s'],ob['captured_wall_s'],side='right')-1);C=fk(d['command'][j,ids]);G=np.linalg.inv(A)@np.r_[P,1.];public.append(dict(label=e['event'],capture_wall_s=ob['captured_wall_s'],P=P.tolist(),axis=a.tolist(),actual_H=A.tolist(),command_H=C.tolist(),G_translation=G[:3].tolist(),G_axis=(A[:3,:3].T@a).tolist(),actual_gap_m=float(d['actual_dof_position'][j,fids].sum()),command_gap_m=float(d['command'][j,fids].sum()),position_error_m=float(np.linalg.norm(P-goal))))
pre=public[-2];post=public[-1];P0,P1=np.array(pre['P']),np.array(post['P']);A0,A1=np.array(pre['actual_H']),np.array(post['actual_H']);C0,C1=np.array(pre['command_H']),np.array(post['command_H']);
summary=dict(scope='Recorded public RGB-D+SDK evidence, with clearly separated evaluator-only physical trace samples; never control input',source=p.name,source_final_error=pin['RGBD_full_depth_check'],stroke_rows=len(ix),command_endpoint_rise_m=float(fk(d['command'][after,ids])[2,3]-fk(d['command'][before,ids])[2,3]),actual_endpoint_rise_m=float(fk(d['actual_dof_position'][after,ids])[2,3]-fk(d['actual_dof_position'][before,ids])[2,3]),public_before_after=dict(command_Hand_rise_m=float(C1[2,3]-C0[2,3]),actual_Hand_rise_m=float(A1[2,3]-A0[2,3]),PIN_measured_rise_m=float(P1[2]-P0[2]),PIN_minus_Hand_vertical_motion_m=float((P1-P0)[2]-(A1-A0)[2,3]),command_actual_bias_before_m=(C0[:3,3]-A0[:3,3]).tolist(),command_actual_bias_after_m=(C1[:3,3]-A1[:3,3]).tolist(),measured_hand_relative_translation_delta_m=(np.array(post['G_translation'])-pre['G_translation']).tolist(),measured_hand_relative_axis_rotation_rad=float(np.arccos(np.clip(np.dot(pre['G_axis'],post['G_axis']),-1,1))),actual_gap_before_m=pre['actual_gap_m'],actual_gap_after_m=post['actual_gap_m']),public_observations=public)
contact=r['pin_contact_evaluator_only']['records'];sims=[o['physical_s']for o in out];active=[v for v in contact if min(sims)<=v['physics_s']<=max(sims)]
summary['evaluator_hand_contacts_during_stroke']=dict(records=len(active),normal_force_total_per_header_min_N=min([v['normal_force_magnitude_n']for v in active],default=None),normal_force_total_per_header_max_N=max([v['normal_force_magnitude_n']for v in active],default=None),records_with_zero_points=sum(not v['contacts']for v in active),maximum_points=max([len(v['contacts'])for v in active],default=None),not_fixture_contact_sensor=True,used_for_control=False)
summary['EVAL_ONLY_stroke_trace']=out
Path('/tmp/fixed526-pin-undertravel/qualification.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items()if k not in ['public_observations','EVAL_ONLY_stroke_trace']},indent=2))
for x in out[::max(1,len(out)//12)]:print('sample',x['phase'],round(x['physical_s'],5),'noseZ',round(x['nose_z_EVAL_ONLY'],6),'noseXYmm',np.round(np.array(x['nose_XY_error_m_EVAL_ONLY'])*1000,3),'Gmm',np.round(np.array(x['physical_G_translation_EVAL_ONLY'])*1000,3),'gap',round(x['actual_finger_gap_m']*1000,3))
