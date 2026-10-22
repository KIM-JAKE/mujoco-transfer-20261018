native_grip_hold={'left':False,'right':False};native_grip_direction={'left':-1.,'right':-1.};native_grip_started={'left':None,'right':None};report['native_gripper_control']='Single motor + mechanically coupled passive follower; encoder velocity servo 40 mm/s free and 15 mm/s loaded; motor effort capped 60 N total (30 N per coupled pad); follower drive inactive'
report['native_single_motor_coupling_validated']=True
shared_motor=bool(startup_config.get('native_shared_motor_drive'));pad_force=float(startup_config.get('native_grip_pad_force_n',30.));assert 0<pad_force<=30.
report['finger_effort_caps_n'].update(left_per_finger=pad_force,right_per_finger=pad_force)
report['native_gripper_control']='Single motor + passive coupled follower; 40 mm/s free, 15 mm/s loaded; finite pad force '+str(pad_force)+' N each'
report['native_shared_motor_drive']=shared_motor
if shared_motor:report['native_gripper_control']='One scalar width/velocity command + hard mechanical coupling; same reference on both pads; total effort cap '+str(2*pad_force)+' N and total damping 4000 N s/m, distributed equally; no additional motor capability'
for side in native_grip_hold:
    finger_dofs=finger_ids[side];stiffness[0,finger_dofs]=0.;damping[0,finger_dofs]=[2000.,2000.] if shared_motor else [4000.,.001];forces[0,finger_dofs]=[pad_force,pad_force] if shared_motor else [2*pad_force,.0001]
art.set_dof_stiffnesses(stiffness,idx);art.set_dof_dampings(damping,idx);art.set_dof_max_forces(forces,idx)
if startup_config.get('continuous_force_hold'):
    report['native_gripper_control']+='; loaded grasp maintains closing force at 15 mm/s velocity preload instead of relaxing at a fixed minimum width'
loaded_ramp_s=float(startup_config.get('native_loaded_preload_ramp_s',0.))
loaded_final_speed=float(startup_config.get('native_loaded_preload_m_s',.015))
loaded_initial_speed=float(startup_config.get('native_loaded_initial_preload_m_s',loaded_final_speed))
if not 0<=loaded_ramp_s<=2. or not 0<loaded_initial_speed<=loaded_final_speed<=.015:raise ValueError('Invalid native grasp preload ramp')
report['native_loaded_preload_ramp']=dict(duration_s=loaded_ramp_s,initial_m_s=loaded_initial_speed,final_m_s=loaded_final_speed,motor_force_cap_n=2*pad_force,arm_timing_unchanged=True)
report['native_gripper_control']+='; configured loaded preload '+str(loaded_final_speed*1000)+' mm/s with '+str(loaded_ramp_s)+' s force ramp'
native_preload_target={side:loaded_final_speed for side in native_grip_hold}
native_preload_initial={side:loaded_initial_speed for side in native_grip_hold}
def loaded_preload(side):
    onset=native_grip_started[side]
    progress=1. if loaded_ramp_s<=0 or onset is None else np.clip((physical-onset)/loaded_ramp_s,0.,1.)
    blend=progress*progress*(3-2*progress)
    return native_preload_initial[side]+(native_preload_target[side]-native_preload_initial[side])*blend
def set_native_grip_hold(side,enabled,direction=-1.,preload_m_s=None):
    if preload_m_s is not None and not 0<float(preload_m_s)<=.015:raise ValueError('Invalid native holding preload')
    if enabled and not native_grip_hold[side]:
        native_grip_started[side]=physical;native_preload_initial[side]=loaded_initial_speed
        native_preload_target[side]=loaded_final_speed if preload_m_s is None else float(preload_m_s)
    elif enabled and preload_m_s is not None and float(preload_m_s)!=native_preload_target[side]:
        native_preload_initial[side]=loaded_preload(side);native_preload_target[side]=float(preload_m_s);native_grip_started[side]=physical
        report.setdefault('native_contact_force_stages',[]).append(dict(side=side,physics_s=physical,initial_preload_m_s=native_preload_initial[side],target_preload_m_s=float(preload_m_s),force_cap_n=2*pad_force))
    if not enabled:native_grip_started[side]=None
    native_grip_hold[side]=bool(enabled);native_grip_direction[side]=1. if direction>0 else -1.;ids=finger_ids[side];stiffness[0,ids]=0.;damping[0,ids]=[2000.,2000.] if shared_motor else [4000.,.001];art.set_dof_stiffnesses(stiffness,idx);art.set_dof_dampings(damping,idx)
