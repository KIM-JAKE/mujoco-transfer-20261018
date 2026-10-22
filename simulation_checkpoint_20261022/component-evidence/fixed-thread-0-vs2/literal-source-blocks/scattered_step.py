def scattered_step(label):
    global physical,active_physics_substeps
    tick=time.perf_counter();old=actuators[-1]['q'][None,:] if actuators else raw['command'][:1].copy()
    for part,at in list(scheduled_clamps.items()):
        if left_queue_clock()>=at:
            for attribute,limit in clamp_targets[part]:attribute.Set(limit)
            sim.flush_changes();del scheduled_clamps[part]
    if not actuators:old[:]=cmd
    velocity_target=(cmd-old)/DT;encoder_q=art.get_dof_positions()[0]
    restore_shared_gains=False
    for side,enabled in native_grip_hold.items():
        finger_dofs=finger_ids[side];speed=.015 if enabled else .04
        if shared_motor:
            if abs(float(np.diff(cmd[0,finger_dofs])[0]))>1e-6:raise RuntimeError('Independent finger references are forbidden in single-motor mode')
            if np.any(stiffness[0,finger_dofs]!=0) or np.any(damping[0,finger_dofs]!=2000.):
                stiffness[0,finger_dofs]=0.;damping[0,finger_dofs]=2000.;restore_shared_gains=True
            velocity_target[0,finger_dofs]=np.clip(60*(cmd[0,finger_dofs].mean()-encoder_q[finger_dofs].mean()),-speed,speed)
            difference=float(abs(np.diff(encoder_q[finger_dofs])[0]));encoder_speed_audit['max_coupled_finger_difference_m']=max(encoder_speed_audit.get('max_coupled_finger_difference_m',0.),difference)
            if difference>.0002:raise RuntimeError('Mechanically coupled finger position drift exceeded 0.2 mm')
        else:velocity_target[0,finger_dofs]=np.clip(60*(cmd[0,finger_dofs]-encoder_q[finger_dofs]),-speed,speed)
        if enabled and startup_config.get('continuous_force_hold'):
            velocity_target[0,finger_dofs]=native_grip_direction[side]*loaded_preload(side)
    if restore_shared_gains:art.set_dof_stiffnesses(stiffness,idx);art.set_dof_dampings(damping,idx)
    update_motor_speed_limits(velocity_target[0],cmd[0]);art.set_dof_position_targets(cmd,idx);art.set_dof_velocity_targets(velocity_target,idx);audit.update(cmd);active_physics_substeps=int(startup_config.get('scattered_physics_substeps',2))
    control_finished=time.perf_counter();profile['control_s']+=control_finished-tick
    for substep in range(active_physics_substeps):sim.simulate(DT/active_physics_substeps,physical+substep*DT/active_physics_substeps);sim.fetch_results()
    profile['physics_s']+=time.perf_counter()-control_finished
    physical+=DT
    if empty_donor_payload_peak['force_n']>12.:
        report['empty_donor_payload_contact_abort']=dict(empty_donor_payload_peak,phase=label,source='Phase-armed raw PhysX contact safety monitor; no free-body pose or selected-actor feedback')
        raise RuntimeError('Transferred ring contacted the withdrawn empty donor above 12 N')
    if peak>12:
        report['cross_arm_abort']=dict(peak_force_n=float(peak),phase=label,command=cmd[0].tolist(),encoder=art.get_dof_positions()[0].tolist())
        raise RuntimeError('Cross-arm contact exceeded 12 N')
    record=round(physical/DT)%4==0
    if record:snapshot=dict(sim=physical,phase=physical,q=art.get_dof_positions()[0].copy(),links=art.get_link_transforms()[0].copy(),camera_bodies=camera.all_bodies.get_transforms().copy(),parts=np.array([v.get_transforms()[0] for v in truth.values()]),pin_phase=label)
    remain=tick+DT-time.perf_counter()
    if remain>.0003:time.sleep(remain-.0003)
    while time.perf_counter()<tick+DT:pass
    wall=time.perf_counter()-started;actuators.append(dict(t=wall,q=cmd[0].copy(),phase=physical,pin_phase=label,physics_substeps=active_physics_substeps,actual_dof_velocity=art.get_dof_velocities()[0].copy(),actual_dof_position=art.get_dof_positions()[0].copy()))
    if record:snapshot['t']=wall;tape.append(snapshot)
