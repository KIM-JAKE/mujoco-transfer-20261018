"""Fresh CPU PhysX trial: first capture through retention on one wall clock.

Camera preparation and taught-path compilation precede operation. Every live
capture, detection, correction, scheduling delay and physics step thereafter
is timed. Motion is paced; no fast-forward/camera wait removal is permitted.
"""
from pathlib import Path
import os,sys,time,json,traceback,numpy as np
from pxr import Usd,Sdf,UsdUtils,UsdPhysics,PhysxSchema,PhysicsSchemaTools,UsdGeom,Gf
import carb,omni.kit.app,omni.physx,omni.physics.tensors as tensors
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE/'bin-picking/realtime-deps-python312'),str(HERE),str(HERE.parents[1]),str(HERE.parents[2])];OUT=Path(os.environ['M80_COMPONENT_OUT']);OUT.mkdir(exist_ok=True)
from isaac_kinematics import Arm,frame,exp,log
from isaac_motion import Audit
# Kit's cached extension finder does not discover files added after startup.
# Load the CPU kernel wrapper explicitly, as with the other live task modules.
import types
fast_path=HERE/'fast_numeric_ik.py';fast_module=types.ModuleType('fast_numeric_ik');fast_module.__file__=str(fast_path);sys.modules['fast_numeric_ik']=fast_module;exec(compile(fast_path.read_text(),str(fast_path),'exec'),fast_module.__dict__);fast_ik_info=fast_module.enable(Arm)
fast_fk_path=HERE/'fast_numeric_fk.py';fast_fk_module=types.ModuleType('fast_numeric_fk');fast_fk_module.__file__=str(fast_fk_path);sys.modules['fast_numeric_fk']=fast_fk_module;exec(compile(fast_fk_path.read_text(),str(fast_fk_path),'exec'),fast_fk_module.__dict__);fast_fk_info=fast_fk_module.enable(Arm)
from rgbd_geometry import axis_frame
from native_pickup_probe import rotation
# Load current source explicitly: Kit's extension import cache can retain a
# previous class after importlib.reload in a persistent interpreter.
import types,hashlib
fresh_modules={}
for module_name in ['fr3_path_timing','path_local_timing','gpu_pair_binary_protocol','gpu_pair_schedule_json','gpu_pair_schedule_binary','gpu_pair_schedule','predictive_pair_clearance','loaded_contact_aperture','co_moving_own_clearance','own_payload_clearance','placed_part_clearance','same_arm_clearance','aperture_path_check','dual_aperture_path_check','gpu_pair_schedule_scope','serial_gpu_pair_scope','gpu_pair_width_scope','gpu_pair_path_scope','production_width_probe','exact_loaded_left_proof','left_loaded_path_guard','local_rim_grasp','inner_bore_grasp','receiver_pad_contact','fr3_gpu_reference','fr3_gpu_ik','fr3_gpu_route','fr3_gpu_route_dual','gpu_route_curve_validation','pin_native_contact','pin_terminal_geometry','axial_rim_receiver','grasp_transfer_feasibility','balanced_small_receiver','two_location_native_handover','probe_native_right_ring','empty_right_release_withdraw','native_right_ring_insert','fcl_transport','fixed_canonical_plan','fixed_small_left_plan','fixed_small_donor_plan','fixed_small_donor_executor','fixed_small_own_domains','fixed_static_route_certificate','fixed_own_route_certificate','exact_collision_proof_cache','fixed_public_grasp_domains','fixed_saved_executor','fixed_small_left_executor','isaac_collision_queries','payload_proxy_content_intern','rgbd_pick_clearance','rgbd_fast_tracker','rgbd_annular_cap_gate','rgbd_scattered_rings','rgbd_initial_fast','rgbd_batch_sensor','realtime_recipe','realtime_vision','process_vision','randomize_bin_trial','fixed_supply_setup','clamp_tensor_drives']:
    module_path=HERE/(module_name+'.py');module=types.ModuleType(module_name);module.__file__=str(module_path);module.__package__='';sys.modules[module_name]=module;exec(compile(module_path.read_text(),str(module_path),'exec'),module.__dict__);fresh_modules[module_name]=module
payload_CAD_content_intern=fresh_modules['payload_proxy_content_intern'].enable(fresh_modules['isaac_collision_queries'].Checker)
fresh_modules['realtime_vision'].detect=fresh_modules['rgbd_initial_fast'].detect
fresh_modules['realtime_vision'].FastRGBDTracker=fresh_modules['rgbd_fast_tracker'].FastRGBDTracker
BatchRGBDCamera=fresh_modules['rgbd_batch_sensor'].BatchRGBDCamera;compile_recipe=fresh_modules['realtime_recipe'].compile_recipe;Vision=fresh_modules['process_vision'].Vision
print('REALTIME_CODE_VERSION',[(n,hashlib.sha256((HERE/(n+'.py')).read_bytes()).hexdigest()[:12])for n in fresh_modules],flush=True)
assert 'local_scenes'in BatchRGBDCamera.capture.__code__.co_names and 'all_bodies'in BatchRGBDCamera.snapshot.__code__.co_names
assert 'Image'not in Vision.save.__code__.co_names
# Release callback-owned contexts from older trials in this private service.
import gc,types
old_contexts={}
for function in gc.get_objects():
    if isinstance(function,types.FunctionType)and function.__name__=='contacts':
        context=function.__globals__
        if context is not globals()and context.get('__file__')==str(HERE/'run_realtime_recipe.py'):old_contexts[id(context)]=context
for context in old_contexts.values():
    context['subscription']=None
    old_vision=context.get('vision')
    if old_vision is not None:old_vision.close()if hasattr(old_vision,'close')else old_vision.pool.shutdown(wait=True)
    for key in ['vision','camera','art','view','truth','namespace']:context[key]=None
old_contexts.clear();old_vision=None;context=None;function=None;gc.collect()
sys.setswitchinterval(.005)
class RecipeAudit:
    def __init__(self,cmd,indices,dt):
        self.ids=np.concatenate(list(indices.values()));self.initial=cmd[0,self.ids].copy();self.dt=dt;self.samples=0;self.checked_samples=0;self.peaks=np.zeros(3)
    def validate(self,commands):
        from fr3_motion_control import position_velocity_bounds
        q=commands.astype(np.float32)[:,self.ids].astype(float);v=np.diff(q,axis=0,prepend=self.initial[None,:])/self.dt;a=np.diff(v,axis=0,prepend=np.zeros((1,14)))/self.dt;j=np.diff(a,axis=0,prepend=np.zeros((1,14)))/self.dt
        lo,hi=position_velocity_bounds(q.reshape(-1,2,7));cap=.95*np.where(v.reshape(-1,2,7)>=0,hi,-lo);peak=np.array([np.max(abs(v.reshape(-1,2,7))/np.maximum(cap,1e-8)),np.max(abs(a)),np.max(abs(j))])
        if peak[0]>1. or peak[1]>9.5 or peak[2]>4750:
            slot,joint=np.unravel_index(np.argmax(abs(v)/np.maximum(cap.reshape(-1,14),1e-8)),v.shape);raise RuntimeError('Corrected command field exceeds FR3 limits '+str(peak)+' phase '+str(phase[slot])+' joint '+str(joint))
        self.peaks=np.maximum(self.peaks,peak);self.checked_samples=len(commands)
    def update(self,cmd):self.samples+=1
    def report(self):return dict(samples=self.samples,velocity_fraction_of_95_percent_PBV=float(self.peaks[0]),max_acceleration_rad_s2=float(self.peaks[1]),max_jerk_rad_s3=float(self.peaks[2]),acceleration_limit_rad_s2=9.5,jerk_limit_rad_s3=4750,command_limit_checks=True,complete_issued_field_checked_at_end=bool(self.samples and self.checked_samples==self.samples),validated_sample_count=self.checked_samples)
class SceneProbeComplete(Exception):pass
DT=1/480
report=dict(full_task_pass=False,vision_end_to_end_pass=False,end_to_end_under30=False,hardware_tested=False,part_attachments=False,workpiece_teleports_after_initialization=0,physics_hz=int(round(1/DT)),python_thread_switch_interval_s=sys.getswitchinterval(),timing_basis='Monotonic wall time from first camera snapshot through >=0.5s verified retention',CPU_camera_waits_omitted=False,finger_effort_caps_n={'left_per_finger':30.,'right_per_finger':30.,'left_final_insertion_per_finger':15.},clamp_target_update_hz=30,wall_pacing='Each pair of 480Hz physics steps takes at least 1/240s; never catch up by fast-forward',native_finger_target_speed_cap_m_s=.0475,native_force_hold='Scattered controller: single motor + coupled passive finger, velocity preload, finite force; no attachment',initial_big_ring_aperture_m=.076,neural_mask_cache_used=False)
report['payload_CAD_content_intern']=payload_CAD_content_intern
try:
    report['CPU_fast_kinematics']=fast_ik_info;report['CPU_fast_body_kinematics']=fast_fk_info
    preparation_started=time.perf_counter();carb.settings.get_settings().set_int('/persistent/physics/numThreads',int(os.environ['M80_COMPONENT_THREADS']));carb.settings.get_settings().set_bool('/physics/physxDispatcher',True);carb.settings.get_settings().set_bool('/physics/updateToUsd',True);carb.settings.get_settings().set_bool('/physics/updateVelocitiesToUsd',True);source=HERE.parents[1]/'runs/native-roas-table-bin-retimed14';Sdf.Layer.FindOrOpen(str(source/'scene-authored.usda')).Reload();stage=Usd.Stage.Open(str(source/'scene-authored.usda'));paths=json.load(open(source/'body-paths.json'));hands=paths.pop('native_hands');options=json.load(open(HERE/'bin-picking/pipeline-options.json'));drives={p.GetName():UsdPhysics.DriveAPI(p,'angular')for p in stage.Traverse()if p.IsA(UsdPhysics.RevoluteJoint)and '_arm_fr3v2_joint'in p.GetName()}
    startup_config=json.loads((OUT/'trial-config.json').read_text())if(OUT/'trial-config.json').exists()else{}
    fresh_modules['fr3_path_timing'].PATH_LOCAL_OPTIMIZATION=bool(startup_config.get('path_local_timing'))
    fresh_modules['isaac_collision_queries'].Checker.batch_query_transform_updates=bool(startup_config.get('batch_query_transform_updates',False));report['USD_planning_query_transform_batching']=bool(startup_config.get('batch_query_transform_updates',False))
    fresh_modules['isaac_collision_queries'].Checker.gpu_right_partitioned_route=bool(startup_config.get('gpu_right_partitioned_route',False));report['GPU_RIGHT_annular_payload_proposal']=dict(enabled=bool(startup_config.get('gpu_right_partitioned_route',False)),class_partitions=True,proposal_only=True,original_native_geometry_and_FR3_checks_required=True)
    if startup_config.get('gpu_right_ring_route_socket'):fresh_modules['isaac_collision_queries'].Checker.gpu_right_ring_route_socket=str(startup_config['gpu_right_ring_route_socket'])
    # Preserve the actual static control inputs together with the Python snapshot.
    manifest_path=OUT/'code-manifest.json';input_manifest=json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    for relative in ['bin-picking/direct-pin-robot-seeds.json','bin-picking/live-robot-route-hints.json','bin-picking/pipeline-options.json','bin-picking/asset-audit.json','bin-picking/scattered-home-robot-q.json','bin-picking/right-native-insertion-ik-hints.json','bin-picking/double-handover-public-priors.json']:
        input_path=HERE/relative
        if input_path.exists():
            data=input_path.read_bytes();target=OUT/'code_snapshot'/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data);input_manifest[relative]=hashlib.sha256(data).hexdigest()
    manifest_path.write_text(json.dumps(input_manifest,indent=2)+'\n')
    if startup_config.get('native_finger_convex_decomposition'):
        for side in ['left','right']:
            for label in ['left','right']:
                prim=stage.GetPrimAtPath(hands[side]+'/'+side+'_'+label+'_finger/tip')
                if prim and prim.IsA(UsdGeom.Mesh):UsdPhysics.MeshCollisionAPI(prim).CreateApproximationAttr('convexDecomposition')
        report['native_finger_collision_model']='Convex decomposition of unchanged official finger CAD; not a modified finger'
    if startup_config.get('native_pad_stiffness_n_m'):
        material=stage.GetPrimAtPath('/World/Materials/rubber');pad=PhysxSchema.PhysxMaterialAPI.Apply(material);pad.CreateCompliantContactStiffnessAttr(float(startup_config['native_pad_stiffness_n_m']));pad.CreateCompliantContactDampingAttr(float(startup_config.get('native_pad_damping_n_s_m',50.)));pad.CreateCompliantContactAccelerationSpringAttr(False);report['native_pad_compliance_assumed']=dict(stiffness_n_m=float(startup_config['native_pad_stiffness_n_m']),damping_n_s_m=float(startup_config.get('native_pad_damping_n_s_m',50.)),hardware_calibrated=False,friction_unchanged=True)
    physics_substeps=int(startup_config.get('physics_substeps',1));assert physics_substeps in [1,2,4,8]
    direct_pin=bool(startup_config.get('direct_pin'))
    if direct_pin:
        for sideidx in range(2):
            pth='/World/Clamps/pin/jaw_'+str(sideidx);body=stage.GetPrimAtPath(pth);joint=UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(pth+'/Actuator'));v=joint.GetLocalPos0Attr().Get();joint.GetLocalPos0Attr().Set(Gf.Vec3f(v[0],v[1],1.515))
            for op in UsdGeom.Xformable(body).GetOrderedXformOps():
                if op.GetOpType()==UsdGeom.XformOp.TypeTransform:
                    M=op.Get();v=M.ExtractTranslation();M.SetTranslateOnly(Gf.Vec3d(v[0],v[1],1.515));op.Set(M)
        report['pin_clamp_assumed_center_z_m']=1.515;report['pin_clamp_hardware_calibrated']=False;report['pin_method']='Tail grasp, uninterrupted axial insertion, release only after fresh full-depth RGB-D confirmation'
    if startup_config.get('ring_clamp_force_n'):
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.PrismaticJoint)and str(prim.GetPath()).startswith(('/World/Clamps/ring1/','/World/Clamps/ring2/')):UsdPhysics.DriveAPI(prim,'linear').CreateMaxForceAttr(float(startup_config['ring_clamp_force_n']))
        report['ring_pneumatic_clamp_force_per_jaw_n']=float(startup_config['ring_clamp_force_n']);report['pneumatic_force_hardware_calibrated']=False
    use_tensor_clamps=bool(json.loads((OUT/'trial-config.json').read_text()).get('tensor_clamps'))if(OUT/'trial-config.json').exists()else False
    clamp_spec=fresh_modules['clamp_tensor_drives'].author(stage)if use_tensor_clamps else None;report['clamp_tensor_drives']=use_tensor_clamps
    # Retain 480 Hz contacts; reduce excessive solver iterations instead of
    # coarse stepping. Fresh success gates still validate contact assembly.
    for prim in stage.Traverse():
        if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
            if (OUT/'trial-config.json').exists()and('/BinStock/'in str(prim.GetPath())or str(prim.GetPath()).endswith(('workpiece_ring1','workpiece_ring2'))):PhysxSchema.PhysxRigidBodyAPI(prim).CreateMaxDepenetrationVelocityAttr(.2)
            api=PhysxSchema.PhysxRigidBodyAPI(prim);api.CreateSolverPositionIterationCountAttr(int(startup_config.get('solver_position_iterations',16)));api.CreateSolverVelocityIterationCountAttr(int(startup_config.get('solver_velocity_iterations',4)))
        if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
            api=PhysxSchema.PhysxArticulationAPI(prim);api.CreateSolverPositionIterationCountAttr(int(startup_config.get('solver_position_iterations',16)));api.CreateSolverVelocityIterationCountAttr(int(startup_config.get('solver_velocity_iterations',4)))
    for prim in stage.Traverse():
        if prim.HasAPI(PhysxSchema.PhysxSceneAPI):PhysxSchema.PhysxSceneAPI(prim).CreateEnableCCDAttr(True);PhysxSchema.PhysxSceneAPI(prim).CreateEnableSceneQuerySupportAttr(direct_pin)
    report['solver_iterations']=dict(position=int(startup_config.get('solver_position_iterations',16)),velocity=int(startup_config.get('solver_velocity_iterations',4)));report['physics_hz_range']=[480,480];report['physics_hz']=480;report['physics_worker_threads']=0;report['physics_cpu_dispatcher']=True;report['continuous_collision_detection']=True;report['physics_hz']=480*physics_substeps;report['physics_hz_range']=[480,480*physics_substeps] if startup_config.get('physics_substep_window_s') else [480*physics_substeps]*2;report['controller_hz']=480;report['physics_substeps_per_command']=physics_substeps
    for side in ['left','right']:
        master=stage.GetPrimAtPath('/World/NativeJoints/'+side+'_native_left');follower=stage.GetPrimAtPath('/World/NativeJoints/'+side+'_native_right');gear=PhysxSchema.PhysxMimicJointAPI.Apply(follower,'rotX');gear.CreateReferenceJointRel().SetTargets([master.GetPath()]);gear.CreateReferenceJointAxisAttr('rotX');gear.CreateGearingAttr(-1.);gear.CreateOffsetAttr(0.);gear.CreateNaturalFrequencyAttr(float(startup_config.get('native_mimic_frequency_hz',1000.)));gear.CreateDampingRatioAttr(1.2)
    if startup_config.get('fixed_supply_mode'):
        report['fixed_supply_setup']=fresh_modules['fixed_supply_setup'].author(stage,paths,OUT,startup_config)
    ident=UsdUtils.StageCache.Get().Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();omni.physx.get_physx_interface().set_thread_count(int(os.environ['M80_COMPONENT_THREADS']));sim.attach_stage(ident);sim.simulate(1/480,0);sim.fetch_results();view=tensors.create_simulation_view('numpy',ident);clamp_driver=fresh_modules['clamp_tensor_drives'].TensorClamps(view,clamp_spec)if use_tensor_clamps else None;art=view.create_articulation_view('/World/RobotMountJoint');names=list(art.shared_metatype.dof_names);links=list(art.shared_metatype.link_names);idx=np.array([0],np.uint32);aidx={s:[names.index(f'{s}_arm_fr3v2_joint{j}')for j in range(1,8)]for s in ['left','right']};fi=[i for i,n in enumerate(names)if '_native_'in n];finger_ids={side:[i for i,n in enumerate(names)if n.startswith(side+'_native_')]for side in ['left','right']};raw=np.load(HERE.parents[1]/'runs/native-roas-table-rgbd-control05/executed-actuator-commands.npz');cmd=raw['command'][:1].copy();cmd[:]=np.array(json.load(open(HERE/'bin-picking/scattered-home-robot-q.json'))['q']) if startup_config.get('randomization_mode')=='full_bin_drop' or startup_config.get('fixed_supply_mode') else cmd;prepared_initial_command=cmd.copy();art.set_dof_positions(cmd,idx);art.set_dof_velocities(np.zeros_like(cmd),idx);art.set_dof_position_targets(cmd,idx);art.set_dof_velocity_targets(np.zeros_like(cmd),idx)
    stiffness=np.array([[12000 if n in drives else 5000 for n in names]],np.float32);damping=np.array([[600 if n in drives else 100 for n in names]],np.float32);forces=np.array([[(87 if int(n[-1])<=4 else 12)*.95 if n in drives else 30 for n in names]],np.float32);stiffness[0,fi]=0. if startup_config.get('scattered_control') else stiffness[0,fi];damping[0,fi]=2000. if startup_config.get('scattered_control') else damping[0,fi];art.set_dof_stiffnesses(stiffness,idx);art.set_dof_dampings(damping,idx);forces[0,finger_ids['left']]=30.;art.set_dof_max_forces(forces,idx);sim.simulate(1/480,0);sim.fetch_results();arm=Arm(stage,'right',drives,art.get_dof_positions()[0,aidx['right']]);left_arm=Arm(stage,'left',drives,art.get_dof_positions()[0,aidx['left']]);random_config=json.loads((OUT/'trial-config.json').read_text())if(OUT/'trial-config.json').exists()else None
    if random_config and not startup_config.get('fixed_supply_mode'):
        calibration=fresh_modules['fixed_supply_setup'].calibration() if startup_config.get('fixed_supply_mode') else json.load(open(HERE/'bin-picking/asset-audit.json'))['baskets'][0];random_records,random_views=fresh_modules['randomize_bin_trial'].apply(view,HERE.parents[1]/'runs/native-roas-table-wall-live13/camera-local-models.npz',calibration,random_config,settle_step=lambda:(sim.simulate(1/480,0),sim.fetch_results()))
        for settle in range(int(480*float(random_config.get('randomization_settle_s',2.)))):sim.simulate(1/480,settle/480);sim.fetch_results()
        fresh_modules['randomize_bin_trial'].save(OUT,random_config,random_records,random_views,calibration);report['randomized_bin_config']=random_config;report['randomized_ring_count']=10
        for _,body,old in random_views:
            prim=stage.GetPrimAtPath(body.prim_paths[0]);PhysxSchema.PhysxRigidBodyAPI(prim).CreateMaxDepenetrationVelocityAttr(1000.)
        sim.flush_changes();report['setup_depenetration_cap_m_s']=.2;report['operation_depenetration_cap_m_s']=1000.
    if startup_config.get('fixed_supply_mode'):
        calibration=fresh_modules['fixed_supply_setup'].calibration()
        random_views=[];random_records=[]
        for part in ['ring1','ring2','pin']:
            body=view.create_rigid_body_view(paths['workpiece_'+part]);random_views.append((paths['workpiece_'+part],body,body.get_transforms().copy()))
        report['randomized_ring_count']=2;report['randomized_bin_config']=None
        for settle in range(240):sim.simulate(1/480,settle/480);sim.fetch_results()
    if startup_config.get('start_above_bin'):
        from isaac_collision_queries import Checker
        os.environ['M80_ROAS_RGB_ONLY']='1';standby_checker=Checker(stage,arm,'right');standby_q=None;standby_P=None;normal=np.array(calibration['basket_to_world_R'])[:,2];Rbase=axis_frame(-normal);standby_seeds=[raw['command'][0,aidx['right']],cmd[0,aidx['right']]]
        for position in [[.515,-.715,1.255],[.49,-.725,1.24],[.48,-.725,1.24],[.49,-.70,1.25],[.46,-.73,1.26]]:
            for yaw in [0,np.pi/2,-np.pi/2,np.pi]:
                R=Rbase@exp(np.array([0.,0,yaw]))
                for seed in standby_seeds:
                    q=arm.solve(position,R,seed)
                    if q is not None and standby_checker.clear(q,.08)[0]:standby_q=q;standby_P=position;break
                if standby_q is not None:break
            if standby_q is not None:break
        if standby_q is None:raise RuntimeError('No fixed CAD-only pre-bin standby pose found')
        cmd[0,aidx['right']]=standby_q;prepared_initial_command=cmd.copy();art.set_dof_positions(cmd,idx);art.set_dof_velocities(np.zeros_like(cmd),idx);art.set_dof_position_targets(cmd,idx);art.set_dof_velocity_targets(np.zeros_like(cmd),idx)
        for _ in range(96):sim.simulate(1/480,0);sim.fetch_results()
        report['fixed_standby_TCP_world_m']=standby_P;report['standby_independent_of_images_or_object_poses']=True
        if random_config:fresh_modules['randomize_bin_trial'].save(OUT,random_config,random_records,random_views,calibration)
    report['contact_material_audit']={}
    for material_name in ['steel','rubber']:
        material_prim=stage.GetPrimAtPath('/World/Materials/'+material_name)
        material_api=UsdPhysics.MaterialAPI(material_prim)
        mode_attr=material_prim.GetAttribute('physxMaterial:frictionCombineMode')
        mode=mode_attr.Get() if mode_attr else None
        report['contact_material_audit'][material_name]=dict(static_friction=material_api.GetStaticFrictionAttr().Get(),dynamic_friction=material_api.GetDynamicFrictionAttr().Get(),authored_friction_combine_mode=mode,default_combine_mode='average',hardware_friction_measured=False)
    motor_speed_caps=np.asarray(art.get_dof_max_velocities(),dtype=np.float32).copy();motor_speed_last=np.full_like(motor_speed_caps,-1.);encoder_speed_audit=dict(samples=0,max_fraction_of_95_percent_PBV=0.,max_finger_fraction_of_47p5_mm_s=0.)
    def update_motor_speed_limits(desired_velocity=None,position_target=None):
        if not startup_config.get('enforce_motor_speed_limits',True):return
        from fr3_motion_control import position_velocity_bounds,VMAX
        robot_ids=aidx['left']+aidx['right'];actual_q=art.get_dof_positions()[0].astype(float);actual_v=art.get_dof_velocities()[0].astype(float);qa=actual_q[robot_ids].reshape(2,7);va=actual_v[robot_ids].reshape(2,7);lo,hi=position_velocity_bounds(qa);ratio=float(np.max(abs(va)/np.maximum(.95*np.where(va>=0,hi,-lo),1e-8)));finger_ratio=float(np.max(abs(actual_v[fi]))/.0475);encoder_speed_audit['samples']+=1;encoder_speed_audit['max_fraction_of_95_percent_PBV']=max(encoder_speed_audit['max_fraction_of_95_percent_PBV'],ratio);encoder_speed_audit['max_finger_fraction_of_47p5_mm_s']=max(encoder_speed_audit['max_finger_fraction_of_47p5_mm_s'],finger_ratio)
        if ratio>1.001 or finger_ratio>1.01:
            report['motor_limit_abort']=dict(names=names,q=actual_q.tolist(),velocity=actual_v.tolist(),caps=motor_speed_caps[0].tolist(),physics_reported_caps=np.asarray(art.get_dof_max_velocities()).tolist(),physical_s=globals().get('physical'),previous_q=tape[-1]['q'].tolist() if globals().get('tape') else None)
            raise RuntimeError('Actual robot motor velocity exceeded reserved limit '+str((ratio,finger_ratio)))
        margin=VMAX*DT+.5*9.5*DT*DT;vlo,_=position_velocity_bounds(qa-margin);_,vhi=position_velocity_bounds(qa+margin)
        if desired_velocity is None:cap=.93*np.minimum(-vlo,vhi)
        else:
            drive=np.asarray(desired_velocity)[robot_ids].reshape(2,7)
            if position_target is not None:drive=drive+20*(np.asarray(position_target)[robot_ids].reshape(2,7)-qa)
            cap=.93*np.where(drive>=0,vhi,-vlo)
            # Brake a residual motion in the opposite direction conservatively.
            # Actual signed encoder velocities remain checked every control step.
            reversing=(va*drive<0)&(abs(va)>.01)
            cap=np.where(reversing,.93*np.minimum(-vlo,vhi),cap)
        if np.min(cap)<=0:raise RuntimeError('Robot encoder entered the zero-speed boundary band')
        motor_speed_caps[0,robot_ids]=cap.ravel();motor_speed_caps[0,fi]=float(startup_config.get('native_motor_speed_cap_m_s',.04))
        art.set_dof_max_velocities(motor_speed_caps,idx);motor_speed_last[:]=motor_speed_caps
except BaseException:
    raise
