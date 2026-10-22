import json,pathlib,hashlib,numpy as np
ROOT=pathlib.Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/runs')
OUT=pathlib.Path(__file__).parent
for n in (506,507,509):
 p=next(ROOT.glob(f'*v{n}-*')); result=p/'pipeline-results.json';j=json.loads(result.read_text()); pin=j['pin_skill_progress'];f=p/'frozen-source-manifest.json';m=json.loads(f.read_text()); a=pin['preinsert_visual_alignment'];z=np.load(p/'executed-actuator-commands.npz')
 arrayproof={}
 for k in ('command','actual_dof_position','actual_dof_velocity'):
  v=z[k];arrayproof[k]={'shape':list(v.shape),'dtype':str(v.dtype),'all_finite':bool(np.isfinite(v).all())}
 assert all(x['all_finite'] for x in arrayproof.values())
 assert j['FR3_command_limit_audit']['complete_issued_field_checked_at_end']
 assert j['FR3_command_limit_audit']['validated_sample_count']==len(z['command'])
 assert j['encoder_motor_speed_audit']['enforced']
 assert j['part_attachments'] is False and j['workpiece_teleports_after_initialization']==0
 resets=pin.get('pin_feedback_resets_after_rebranch',[])
 last=a[-1]; assert last['radial_error_m']<=last['radial_limit_m'] and last['axis_error_rad']<=last['axis_limit_rad'] and last['maximum_shaft_axis_radial_error_m']<=last['maximum_shaft_axis_radial_limit_m']
 retention=pin.get('pin_RGBD_SDK_postcarry',[])
 assert all(x['confirmed'] and x['relative_center_drift_m']<=x['center_drift_limit_m'] and x['relative_axis_drift_rad']<=x['axis_drift_limit_rad'] for x in retention)
 files=m['files']; relevant={k:v for k,v in files.items() if any(x in k for x in ('direct_pin_skill.py','pin_collar_contact_target.py','pin_terminal','run_realtime_recipe.py','loose_bin_task.py'))}
 s={'run':p.name,'run_path':str(p),'result_sha256':hashlib.sha256(result.read_bytes()).hexdigest(),'frozen_manifest_sha256':hashlib.sha256(f.read_bytes()).hexdigest(),'relevant_frozen_source_files':relevant,'full_task_pass':j['full_task_pass'],'pin_component_pass':j.get('pin_component_pass'),'pin_only_component':j.get('pin_only_component',False),'wall_s':j['wall_verified_elapsed_s'],'physics_s':j['simulation_elapsed_s'],'reset_event_count':len(resets),'reset_efficacy_qualified':bool(resets),'precision_image_count':len(a),'original_total_image_budget':16,'precision_sequence':[ {k:x.get(k) for k in ('iteration','radial_error_m','axis_error_rad','maximum_shaft_axis_radial_error_m','selected_feedback_gain')} for x in a],'qualified_final_precision':{k:last[k] for k in ('radial_error_m','radial_limit_m','axis_error_rad','axis_limit_rad','maximum_shaft_axis_radial_error_m','maximum_shaft_axis_radial_limit_m')},'postcarry_max_center_drift_m':max(x['relative_center_drift_m'] for x in retention),'postcarry_max_axis_drift_rad':max(x['relative_axis_drift_rad'] for x in retention),'postcarry_confirmations':len(retention),'command_limit_audit':j['FR3_command_limit_audit'],'actual_encoder_motor_audit':j['encoder_motor_speed_audit'],'independent_saved_arrays':arrayproof,'part_attachments':j['part_attachments'],'workpiece_teleports_after_initialization':j['workpiece_teleports_after_initialization'],'native_finger_target_speed_cap_m_s':j['native_finger_target_speed_cap_m_s'],'finger_effort_caps_n':j['finger_effort_caps_n'],'controller_hz':j['controller_hz'],'physics_substeps_per_command':j['physics_substeps_per_command'],'final_acceptance_checks':j.get('final_checks'),'caveats':['Simulation only; no physical-robot or repeatability qualification.','Reset branch did not execute: no evidence of actual reset efficacy.','Component506 has stock rings and cannot qualify the whole three-part task.']}
 (OUT/f'actual{n}-audit.json').write_text(json.dumps(s,indent=2)+'\n')
 print(n,'PASS audits, result',s['full_task_pass'],s['pin_component_pass'],'samples',len(z['command']),'source',s['frozen_manifest_sha256'])
