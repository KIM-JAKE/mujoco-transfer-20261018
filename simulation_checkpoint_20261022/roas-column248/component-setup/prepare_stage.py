from pathlib import Path
import shutil,py_compile
src=Path('/tmp/roas248-views-v4/overrides');out=Path('/tmp/pin-fixed-normal-20261022/overrides');out.mkdir(exist_ok=True)
for p in src.glob('*.py'):shutil.copy2(p,out/p.name)
p=out/'run_realtime_recipe.py';s=p.read_text();old="prepared_initial_command=cmd.copy();art.set_dof_positions(cmd,idx)";new="""prepared_initial_command=cmd.copy()
    fixed_component=startup_config.get('pin_normal_fixed_grasp')
    if fixed_component is not None:
        if not(startup_config.get('pin_only_component') and startup_config.get('pin_grip_transport_only')):raise ValueError('Fixed reference posture is restricted to independent grip/transport tests')
        if fixed_component['initial_joint_names']!=names:raise ValueError('Public SDK initial joint order changed')
        initial_q=np.asarray(fixed_component['initial_actual_joint_positions'],float)
        if initial_q.shape!=(len(names),) or not np.isfinite(initial_q).all():raise ValueError('Invalid public empty-robot initial pose')
        cmd[:]=initial_q;prepared_initial_command=cmd.copy()
        report['pin_component_initial_conditions']=dict(public_reference=fixed_component,ring_environment='Original stock, no installed-ring surrogate; ring clamps open',PIN_initialization='Fresh free PIN on holder; no attachment or carried-state restoration',robots_initially_empty=True,actual_SDK_initial_pose_recorded_after_settle=True,full_assembly_qualification=False)
    art.set_dof_positions(cmd,idx)""";assert s.count(old)==2
# Only replace the first initial setup occurrence; a later standby update is unrelated.
s=s.replace(old,new,1)
s=s.replace("'pin_only_component','pin_visual_servo_gain'","'pin_only_component','pin_grip_transport_only','pin_normal_fixed_grasp','pin_visual_servo_gain'")
old="if result.get('state')!='pin_only_component_executed' or result.get('full_task_pass') is not False:raise RuntimeError('Invalid PIN-only component result boundary')";new="""expected_component_state='pin_grip_transport_component_executed' if startup_config.get('pin_grip_transport_only') else 'pin_only_component_executed'
            if result.get('state')!=expected_component_state or result.get('full_task_pass') is not False:raise RuntimeError('Invalid PIN-only component result boundary')""";assert old in s;s=s.replace(old,new)
anchor="        if startup_config.get('scattered_full_task') and not startup_config.get('pin_only_component',False):";addition="""        if startup_config.get('pin_grip_transport_only'):
            passed=bool(result['pin'].get('pin_grip_transport_pass'))
            report.update(state='pin_grip_transport_component_pass' if passed else 'pin_grip_transport_component_failed',pin_grip_transport_component_pass=passed,pin_component_pass=False,pin_only_component=True,component_part='pin_grip_transport',full_task_pass=False,vision_end_to_end_pass=False,normal_fixed_execution=True,visual_micro_alignment_executed=False,insertion_executed=False,ring_tasks_executed=0,requires_whole_three_part_trial=True)
""";assert s.count(anchor)==1;s=s.replace(anchor,addition+anchor);p.write_text(s)
p=out/'loose_bin_task.py';s=p.read_text();anchor="        seated=next((e['result']['pose']for e in reversed(pin_result['events'])";ix=s.index(anchor);s=s[:ix]+"        if c['config'].get('pin_grip_transport_only'):return pin_result\n"+s[ix:]
old="        return dict(state='pin_only_component_executed',component_only=True";new="        return dict(state='pin_grip_transport_component_executed'if c['config'].get('pin_grip_transport_only')else'pin_only_component_executed',component_only=True";assert old in s;s=s.replace(old,new);p.write_text(s)
p=out/'direct_pin_skill.py';s=p.read_text();anchor="    visual_preinsert=bool(pin_config.get('pin_visual_preinsert_alignment'))";addition="""    fixed_normal=pin_config.get('pin_normal_fixed_grasp')
    if fixed_normal is not None and not(pin_config.get('pin_grip_transport_only') and pin_config.get('pin_only_component')):raise ValueError('Fixed-grasp stage must not be counted as full PIN insertion')
""";assert s.count(anchor)==1;s=s.replace(anchor,addition+anchor)
anchor="    newpre=pickupH.copy();newpre[:3,3]+=[0,0,.07]";addition="""    if fixed_normal is not None:
        reference_P=np.asarray(fixed_normal['nominal_initial_pin_center_world_m'],float);reference_axis=np.asarray(fixed_normal['nominal_initial_pin_axis_world'],float);observed_axis=np.asarray(v['result']['pose']['axis'],float)
        initial_error=float(np.linalg.norm(P-reference_P));initial_axis_error=float(np.arccos(np.clip(reference_axis@observed_axis,-1,1)))
        if initial_error>.0003 or initial_axis_error>.01:raise RuntimeError('Fixed normal PIN placement outside original300um/10mrad preclose envelope; recovery required')
        fixed_H=np.asarray(fixed_normal['fixed_pickup_command_H_world'],float);fixed_G=np.asarray(fixed_normal['reference_axis_TCP_G'],float)
        for T in [fixed_H,fixed_G]:
            if T.shape!=(4,4)or not np.isfinite(T).all()or np.max(abs(T[:3,:3].T@T[:3,:3]-np.eye(3)))>1e-6 or abs(np.linalg.det(T[:3,:3])-1)>1e-6:raise ValueError('Invalid calibrated fixed-grasp transform')
        delta=float(np.linalg.norm(fixed_H[:3,3]-pickupH[:3,3]));angle=float(np.linalg.norm(log(fixed_H[:3,:3]@pickupH[:3,:3].T)))
        if delta>.002 or angle>.05:raise RuntimeError('Fixed normal pickup exceeds original2mm/50mrad target envelope')
        pickupH=fixed_H.copy();G=fixed_G.copy()
        report['normal_fixed_pickup']=dict(fixed_command_H=pickupH.tolist(),fixed_relative_G=G.tolist(),current_initial_position_error_m=initial_error,current_initial_axis_error_rad=initial_axis_error,online_target_updates=False,preclose_servo_allowed=False,recovery_used=False,uses_free_body_poses=False)
""";assert s.count(anchor)==1;s=s.replace(anchor,addition+anchor)
old="reference=np.asarray(robot_seeds.get('pin_tail_approach',cand['pickup_q']),float)";new="reference=np.asarray(fixed_normal['fixed_pickup_seed_q'] if fixed_normal is not None else robot_seeds.get('pin_tail_approach',cand['pickup_q']),float)";assert old in s;s=s.replace(old,new)
old="for source,seed in [('robot_seeds.pin_tail_approach',robot_seeds.get('pin_tail_approach'))";new="for source,seed in ([('fixed_public_reference.pickup_seed',fixed_normal['fixed_pickup_seed_q'])] if fixed_normal is not None else [])+[('robot_seeds.pin_tail_approach',robot_seeds.get('pin_tail_approach'))";assert old in s;s=s.replace(old,new)
anchor="    if pin_axis_geometry and pin_config.get('pin_preclose_visual_alignment'):";addition="""    if fixed_normal is not None:
        value=observe('pin_holder_fixed_target_verification_RGBD',planned_G,require_circle=True);actual_G=measured_grasp(value,planned_G)
        desired=np.asarray(value['H'])@actual_G@np.linalg.inv(planned_G);A=arm.fk(art.get_dof_positions()[0,ids]);translation=float(np.linalg.norm(desired[:3,3]-A[:3,3]));angle=float(np.linalg.norm(log(desired[:3,:3]@A[:3,:3].T)))
        report['normal_fixed_preclose_verification']=dict(translation_m=translation,axis_rotation_rad=angle,original_translation_limit_m=.0003,original_rotation_limit_rad=.01,target_corrected=False,uses_free_body_poses=False)
        if translation>.0003 or angle>.01:raise RuntimeError('Fixed normal contact target not reached inside original preclose gates; recovery required')
""";assert s.count(anchor)==1;s=s.replace(anchor,addition+anchor)
s=s.replace('for grasp_attempt in range(2 if pin_axis_geometry else 1):','for grasp_attempt in range(1 if fixed_normal is not None else 2 if pin_axis_geometry else 1):');s=s.replace('if grasp_attempt==1:raise RuntimeError(',"if grasp_attempt==(0 if fixed_normal is not None else 1):raise RuntimeError(")
anchor="    adaptive_pin_gain=bool(pin_config.get('pin_visual_adaptive_gain',False))";addition="""    if pin_config.get('pin_grip_transport_only'):
        if fixed_normal is None:raise ValueError('Grip/transport test requires frozen normal pickup parameters')
        await_precision_encoder_window('pin_component_postcarry_encoder_confirmation')
        samples=[]
        for observation_index in range(2):
            if observation_index:wait(.05,'pin_component_independent_hold_confirmation')
            value=observe('pin_component_postcarry_RGBD',G,require_circle=True);current_G=measured_grasp(value,G);gap=float(art.get_dof_positions()[0,fi].sum());held=_pin_rgbd_sdk_hold_evidence(value['result']['pose'],pin_hold_reference_G,current_G,gap)
            eligible=bool(held['confirmed'] and held['relative_center_drift_m']<=.000075 and held['relative_axis_drift_rad']<=.003)
            samples.append(dict(observation=observation_index,original_hold=held,fixed_path_grasp_repeatability_qualified=eligible,repeatability_translation_limit_m=.000075,repeatability_axis_limit_rad=.003,current_axis_TCP_G=current_G.tolist(),uses_free_body_poses=False))
        passed=all(x['fixed_path_grasp_repeatability_qualified']for x in samples)
        report.update(pin_grip_transport_pass=passed,pin_grip_transport_samples=samples,pin_grip_transport_reference_G=pin_hold_reference_G.tolist(),pin_transport_actual_q=art.get_dof_positions()[0,ids].tolist(),pin_transport_command_q=cmd[0,ids].tolist(),normal_fixed_execution=True,visual_micro_alignment_executed=False,insertion_executed=False,pin_single_stroke_pass=False,recovery_required=not passed)
        return dict(report,state='pin_grip_transport_pass'if passed else'pin_grip_transport_failed',component_only=True,full_task_pass=False)
""";assert s.count(anchor)==1;s=s.replace(anchor,addition+anchor);p.write_text(s)
for p in out.glob('*.py'):py_compile.compile(str(p),doraise=True)
print(out)
