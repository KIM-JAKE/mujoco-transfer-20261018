from pathlib import Path
import json,hashlib,datetime,numpy as np
from scipy.spatial.transform import Rotation
B=Path('/home/gpuadmin/workspace/jake/mujoco');O=B.parent/'isaac-roas-column248-20261022/pin-fixed-grip-cohort592-596';O.mkdir(exist_ok=True)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def agg(x):return hashlib.sha256(json.dumps(x,sort_keys=True,separators=(',',':')).encode()).hexdigest()
rows=[];common_source=None;common_cfg=None;referenceG=None
for v in range(592,597):
 p=next((B/'isaac_port/runs').glob(f'*v{v}-*'),None)
 if p is None or not(p/'pipeline-results.json').exists():rows.append(dict(version=v,status='pending'));continue
 r=json.loads((p/'pipeline-results.json').read_text());s=r.get('pin_skill_progress',{});c=json.loads((p/'trial-config.json').read_text());f=json.loads((p/'frozen-source-manifest.json').read_text());f=f.get('files',f);assert len(f)==426
 ch=dict(c)
 for k in ['experiment_note','run','output_dir']:ch.pop(k,None)
 csha=agg(ch);fsha=agg(f)
 if common_source is None:common_source=fsha;common_cfg=csha
 assert fsha==common_source and csha==common_cfg,(v,'Frozen policy changed')
 G=np.asarray(s.get('pin_grip_transport_reference_G',[]));samples=s.get('pin_grip_transport_samples',[])
 if G.shape==(4,4) and referenceG is None:referenceG=G
 xdrift=None
 if G.shape==(4,4):xdrift=dict(translation_m=float(np.linalg.norm(G[:3,3]-referenceG[:3,3])),axis_rad=float(np.arccos(np.clip(G[:3,2]@referenceG[:3,2],-1,1))))
 row=dict(version=v,run=p.name,status=r.get('state'),component_pass=bool(r.get('pin_grip_transport_component_pass')),whole3_pass=bool(r.get('full_task_pass')),insertion_executed=bool(s.get('insertion_executed',False)),visual_micro_alignment_executed=bool(s.get('visual_micro_alignment_executed',False)),fixed_preclose=s.get('normal_fixed_preclose_verification'),lift_evidence=s.get('pin_RGBD_SDK_postlift'),preload_evidence=s.get('pin_qualified_postlift_preload'),postcarry_samples=samples,actual_q=s.get('pin_transport_actual_q'),command_q=s.get('pin_transport_command_q'),postlift_reference_G=s.get('pin_grip_transport_reference_G'),repeat_reference_drift=xdrift,wall_s=r.get('wall_verified_elapsed_s'),error=r.get('error'),source_426_Python_SHA256=fsha,config_SHA256_excluding_note=csha,result_SHA256=sha(p/'pipeline-results.json'),config_raw_SHA256=sha(p/'trial-config.json'),encoder_speed_audit=r.get('encoder_motor_speed_audit'))
 assert not row['whole3_pass'] and not row['insertion_executed'] and not row['visual_micro_alignment_executed']
 rows.append(row)
finished=[r for r in rows if r['status']!='pending'];passed=sum(r['component_pass']for r in finished)
out=dict(updated_KST=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat(),scope='Fresh physical PIN grasp/lift/carry only; NOT insertion/full3/hardware',planned=5,finished=len(finished),component_pass=passed,whole3_pass=0,normal_micro_corrections=0,source_426_Python_SHA256=common_source,config_SHA256_excluding_note=common_cfg,environment_limitations='Stock rings, empty ring jig, open clamps; initial both empty robot encoder postures copied from575 before PIN; fresh ungrasped PIN on original holder, no PIN attachment or carried-state restoration. Actual left parking and carry executed, not frozen q_preinsert qualified.',trials=rows)
(O/'cohort-results.json').write_text(json.dumps(out,indent=2)+'\n')
last=next((r for r in reversed(finished)if r['component_pass']),None)
if last:
 handoff=dict(scope='Candidate starting state for independent path qualification. Not fixed insertion certificate.',source_run=last['run'],source_result_SHA256=last['result_SHA256'],actual_q=last['actual_q'],command_q=last['command_q'],public_measured_hand_to_pin_G=last['postcarry_samples'][-1]['current_axis_TCP_G'],actual_gripper_opening_m=last['postcarry_samples'][-1]['original_hold']['actual_encoder_aperture_m'],frozen_target_G_is_not_actual_grasp=True,source_426_Python_SHA256=common_source,environment_limitations=out['environment_limitations'],hardware_tested=False)
 run=B/'isaac_port/runs'/last['run'];rr=json.loads((run/'pipeline-results.json').read_text());commands=np.load(run/'executed-actuator-commands.npz')
 handoff.update(actual_all_joint_names=rr['dof_names'],actual_all_joint_positions=commands['actual_dof_position'][-1].tolist(),actual_all_joint_velocities=commands['actual_dof_velocity'][-1].tolist() if 'actual_dof_velocity'in commands.files else None,source_robot_kinematics=str(run/'robot-kinematics.json'),source_trial_configuration=str(run/'trial-config.json'),source_static_geometry=str(run/'geometry-input.usdc'),source_outputs_are_read_only_inputs=True)
 (O/'CURRENT_GRASP_AND_Q_HANDOFF.json').write_text(json.dumps(handoff,indent=2)+'\n')
print(json.dumps({k:out[k]for k in ['planned','finished','component_pass','source_426_Python_SHA256','config_SHA256_excluding_note']}))
