"""Fresh CPU PhysX trial: first capture through retention on one wall clock.

Camera preparation and taught-path compilation precede operation. Every live
capture, detection, correction, scheduling delay and physics step thereafter
is timed. Motion is paced; no fast-forward/camera wait removal is permitted.
"""
from pathlib import Path
import os,sys,time,json,traceback,numpy as np
from pxr import Usd,Sdf,UsdUtils,UsdPhysics,PhysxSchema,PhysicsSchemaTools,UsdGeom,Gf
import carb,omni.kit.app,omni.physx,omni.physics.tensors as tensors
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE/'bin-picking/realtime-deps-python312'),str(HERE),str(HERE.parents[1]),str(HERE.parents[2])];OUT=HERE.parents[1]/'runs'/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True)
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
for module_name in ['fr3_path_timing','path_local_timing','gpu_pair_binary_protocol','gpu_pair_schedule_json','gpu_pair_schedule_binary','gpu_pair_schedule','predictive_pair_clearance','loaded_contact_aperture','co_moving_own_clearance','own_payload_clearance','placed_part_clearance','same_arm_clearance','aperture_path_check','dual_aperture_path_check','gpu_pair_schedule_scope','serial_gpu_pair_scope','gpu_pair_width_scope','gpu_pair_path_scope','production_width_probe','exact_loaded_left_proof','left_loaded_path_guard','local_rim_grasp','inner_bore_grasp','receiver_pad_contact','fr3_gpu_reference','fr3_gpu_ik','fr3_gpu_route','fr3_gpu_route_dual','gpu_route_curve_validation','pin_native_contact','pin_terminal_geometry','axial_rim_receiver','grasp_transfer_feasibility','pin_contact_evaluator','pin_collar_contact_target','pin_parallel_tail_target','pin_tail_contact_target','balanced_small_receiver','two_location_native_handover','probe_native_right_ring','empty_right_release_withdraw','native_right_ring_insert','fcl_transport','fixed_canonical_plan','fixed_small_left_plan','fixed_small_donor_plan','fixed_small_donor_executor','fixed_small_own_domains','fixed_static_route_certificate','fixed_own_route_certificate','exact_collision_proof_cache','fixed_public_grasp_domains','fixed_saved_executor','fixed_small_left_executor','isaac_collision_queries','payload_proxy_content_intern','rgbd_pick_clearance','rgbd_fast_tracker','rgbd_annular_cap_gate','rgbd_scattered_rings','rgbd_initial_fast','rgbd_batch_sensor','realtime_recipe','realtime_vision','process_vision','randomize_bin_trial','fixed_supply_setup','clamp_tensor_drives']:
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
    preparation_started=time.perf_counter();carb.settings.get_settings().set_int('/persistent/physics/numThreads',0);carb.settings.get_settings().set_bool('/physics/physxDispatcher',True);carb.settings.get_settings().set_bool('/physics/updateToUsd',True);carb.settings.get_settings().set_bool('/physics/updateVelocitiesToUsd',True);source=HERE.parents[1]/'runs/native-roas-table-bin-retimed14';Sdf.Layer.FindOrOpen(str(source/'scene-authored.usda')).Reload();stage=Usd.Stage.Open(str(source/'scene-authored.usda'));paths=json.load(open(source/'body-paths.json'));hands=paths.pop('native_hands');options=json.load(open(HERE/'bin-picking/pipeline-options.json'));drives={p.GetName():UsdPhysics.DriveAPI(p,'angular')for p in stage.Traverse()if p.IsA(UsdPhysics.RevoluteJoint)and '_arm_fr3v2_joint'in p.GetName()}
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
    ident=UsdUtils.StageCache.Get().Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ident);sim.simulate(1/480,0);sim.fetch_results();view=tensors.create_simulation_view('numpy',ident);clamp_driver=fresh_modules['clamp_tensor_drives'].TensorClamps(view,clamp_spec)if use_tensor_clamps else None;art=view.create_articulation_view('/World/RobotMountJoint');names=list(art.shared_metatype.dof_names);links=list(art.shared_metatype.link_names);idx=np.array([0],np.uint32);aidx={s:[names.index(f'{s}_arm_fr3v2_joint{j}')for j in range(1,8)]for s in ['left','right']};fi=[i for i,n in enumerate(names)if '_native_'in n];finger_ids={side:[i for i,n in enumerate(names)if n.startswith(side+'_native_')]for side in ['left','right']};raw=np.load(HERE.parents[1]/'runs/native-roas-table-rgbd-control05/executed-actuator-commands.npz');cmd=raw['command'][:1].copy();cmd[:]=np.array(json.load(open(HERE/'bin-picking/scattered-home-robot-q.json'))['q']) if startup_config.get('randomization_mode')=='full_bin_drop' or startup_config.get('fixed_supply_mode') else cmd;prepared_initial_command=cmd.copy();art.set_dof_positions(cmd,idx);art.set_dof_velocities(np.zeros_like(cmd),idx);art.set_dof_position_targets(cmd,idx);art.set_dof_velocity_targets(np.zeros_like(cmd),idx)
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
    camera=BatchRGBDCamera(stage,view,build_embree=False) if startup_config.get('fixed_supply_mode') else fresh_modules['process_vision'].CachedGeometryCamera(view,HERE.parents[1]/'runs/native-roas-table-wall-live13/camera-local-models.npz');vision=Vision(camera,dict(fresh_modules['fixed_supply_setup'].calibration() if startup_config.get('fixed_supply_mode') else json.load(open(HERE/'bin-picking/asset-audit.json'))['baskets'][0],ray_image_backend=startup_config.get('ray_image_backend','embree_cpu')),OUT);vision.calibration['tracking_image_width_px']=int(startup_config.get('tracking_image_width_px',384));vision.calibration['loose_bin_mode']=startup_config.get('randomization_mode')=='full_bin_drop' or bool(startup_config.get('fixed_supply_mode'));vision.calibration.update({k:startup_config[k] for k in ['initial_camera_width_px','initial_camera_height_px','initial_camera_span_m','rgbd_gpu_socket','rgbd_gpu_source_sha256'] if k in startup_config});vision.calibration['initial_eye_offset_m']=startup_config.get('initial_eye_offset_m',[.40,-.12,.65]);report['initial_RGBD_camera_eye_offset_m']=vision.calibration['initial_eye_offset_m'];recipe=compile_recipe(HERE,arm,aidx['right'],DT,left_arm,aidx['left']) if not (startup_config.get('scatter_probe_only') or startup_config.get('scattered_control')) else dict(command=np.repeat(cmd,2,axis=0),phase=np.array([0.,DT]),controls=np.zeros((2,3)),events=[],source=np.zeros(2),right_source=np.zeros(2),at_old=lambda t:t,rot=np.tile(np.eye(3),(2,1,1)),Jinv=np.zeros((2,7,6)),held_Jinv=np.zeros((2,7,6)),left_Jinv=np.zeros((2,7,6)),left_rot=np.tile(np.eye(3),(2,1,1)),source_run='live RGB-D scattered controller; no taught pickup path',trimmed_stationary_s=0.,joint_headroom_retiming_saved_s=0.,retimed_free_motion_windows=[]);commands=recipe['command'].copy();base=commands.copy();phase=recipe['phase'];controls=recipe['controls'];events=[e for e in recipe['events']if e['mode']!='held_left'or(random_config and(random_config.get('left_monitor')or(random_config.get('left_feedback')and(e['part']=='ring2'or not random_config.get('left_small_only')))))];commands[recipe['right_source']<10.68,finger_ids['right'][0]]=np.minimum(commands[recipe['right_source']<10.68,finger_ids['right'][0]],.038);commands[recipe['right_source']<10.68,finger_ids['right'][1]]=np.minimum(commands[recipe['right_source']<10.68,finger_ids['right'][1]],.038)
    if random_config and random_config.get('digital_clamps'):controls=(controls>=float(random_config.get('digital_clamp_threshold',.5))).astype(float);report['clamp_control']='Finite-force open/close commands, same jaw strokes and force caps'
    truth={p:view.create_rigid_body_view(paths['workpiece_'+p])for p in ['ring1','ring2','pin']};joints={p:[UsdPhysics.PrismaticJoint(pr)for pr in stage.Traverse()if str(pr.GetPath()).startswith('/World/Clamps/'+p+'/')and pr.IsA(UsdPhysics.PrismaticJoint)]for p in truth};goals=json.load(open(source/'arm-plan.json'))['goals'];roots=[links.index(s+'_NativeHand')for s in ['left','right']];last_clamp=np.full(3,-1.);audit=RecipeAudit(cmd,aidx,DT);audit.validate(commands) if not (startup_config.get('scatter_probe_only') or startup_config.get('scattered_control')) else None
    smooth_setup=lambda u:np.clip(u,0,1)**3*(10-15*np.clip(u,0,1)+6*np.clip(u,0,1)**2)
    big_weight=smooth_setup((phase-.1)/.55)*(1-smooth_setup((phase-recipe['at_old'](1.8))/1.1));small_weight=smooth_setup((phase-recipe['at_old'](10.6))/.5)*(1-smooth_setup((phase-recipe['at_old'](12.2))/1.0))
    depth_twist=np.tile([0,0,0.,0,0,0.],(len(phase),1));depth_dq=np.einsum('nij,nj->ni',recipe['Jinv'],depth_twist)*big_weight[:,None]
    original=commands.copy()
    for depth_fraction in [1.,.9,.8,.7,.6,.5,.4,.3,.2,.1,0.]:
        commands[:]=original;commands[:,aidx['right']]+=depth_fraction*depth_dq
        try:
            if not (startup_config.get('scatter_probe_only') or startup_config.get('scattered_control')):audit.validate(commands)
            break
        except RuntimeError:continue
    report['pickup_depth_increase_m']={'ring1':0.,'ring2':0.}
    rings_only=bool(startup_config.get('rings_only'));report['diagnostic_rings_only']=rings_only
    if rings_only or direct_pin:events=[e for e in events if e['part']!='pin']
    if direct_pin:
        os.environ['M80_ROAS_RGB_ONLY']='1'
        from isaac_collision_queries import Checker
        pin_checker=Checker(stage,arm,'right');pin_checker.ignored_future_actors={paths['workpiece_pin']};pin_candidate=json.load(open(HERE.parents[1]/'runs/native-roas-table-pin-tail01/pin-direct-probe.json'))['candidates'][int(startup_config.get('pin_tail_candidate',0))]
        if startup_config.get('pin_grasp_catalog_run'):
            catalog_run=str(startup_config['pin_grasp_catalog_run'])
            if Path(catalog_run).name!=catalog_run:raise ValueError('Pin catalog source must be one run directory name')
            catalog_path=HERE.parents[1]/'runs'/catalog_run/'pin-native-grasp-catalog.json';catalog=json.loads(catalog_path.read_text());candidate_index=int(startup_config.get('pin_grasp_catalog_candidate',0))
            if catalog.get('uses_free_body_poses') is not False or catalog.get('robot_motion_executed') is not False:raise ValueError('Pin catalog requires camera/encoder-only geometry provenance')
            pin_candidate=dict(catalog['candidates'][candidate_index]);origin=np.asarray(pin_candidate['pickup_center_from_RGBD'],float)
            if origin.shape!=(3,) or not np.isfinite(origin).all() or not pin_candidate['native_pad_support']['admitted']:raise ValueError('Pin catalog candidate lacks finite own RGB-D origin or bilateral native support')
            report['pin_grasp_catalog_selection']=dict(run=catalog_run,candidate_index=candidate_index,tilt_deg=pin_candidate['tilt_deg'],contact_z_m=pin_candidate['contact_z_m'],pickup_center_from_RGBD=origin.tolist(),physical_grasp_previously_tested=bool(pin_candidate.get('physical_grasp_tested',False)))
    tape=[];actuators=[];peak=0.;physical=0.;active_physics_substeps=1;corrections=[];floor_offset=np.zeros(3);live_pin_grasp=None
    clamp_targets={p:[(UsdPhysics.DriveAPI(joint.GetPrim(),'linear').GetTargetPositionAttr(),float(joint.GetLowerLimitAttr().Get()if p=='pin'else joint.GetUpperLimitAttr().Get()))for joint in joints[p]]for p in truth}
    report['usd_clamp_targets_batched']=True
    for j,p in enumerate(truth):
        last_clamp[j]=controls[0,j]
        for joint in joints[p]:
            limit=joint.GetLowerLimitAttr().Get()if p=='pin'else joint.GetUpperLimitAttr().Get();drive=UsdPhysics.DriveAPI(joint.GetPrim(),'linear');value=float(limit*last_clamp[j])
            if drive.GetTargetPositionAttr().Get()!=value:drive.GetTargetPositionAttr().Set(value)
    sim.flush_changes()
    if clamp_driver is not None:clamp_driver.command(controls[0])
    settings=carb.settings.get_settings();settings.set_bool('/physics/updateToUsd',False);settings.set_bool('/physics/updateVelocitiesToUsd',False)
    # Raw contact safety telemetry only: no free-body pose or selected
    # simulator actor identity enters a target or perception decision.
    empty_donor_contact_guard={'left':False,'right':False};empty_donor_carried_part={}
    empty_donor_payload_peak={'force_n':0.}
    def set_empty_donor_contact_guard(side,enabled,part=None):
        if side not in empty_donor_contact_guard:raise ValueError('Unknown empty donor side')
        empty_donor_contact_guard[side]=bool(enabled) and bool(startup_config.get('empty_donor_payload_contact_guard',True))
        empty_donor_carried_part[side]=part;empty_donor_payload_peak.clear();empty_donor_payload_peak['force_n']=0.
        report.setdefault('empty_donor_contact_guard_events',[]).append(dict(side=side,armed=empty_donor_contact_guard[side],carried_part=part,physics_s=globals().get('physical'),force_threshold_n=12.))
    contact_side_cache={};ring_contact_debug=[]
    pin_contact_evaluator=None
    if startup_config.get('pin_contact_debug_evaluator',False):
        from pin_contact_evaluator import PinContactEvaluator
        pin_contact_evaluator=PinContactEvaluator(paths['workpiece_pin'],lambda encoded:str(PhysicsSchemaTools.intToSdfPath(encoded)))
        report['pin_contact_evaluator_only']=pin_contact_evaluator.report
    def actor_side(encoded):
        if encoded not in contact_side_cache:
            path=str(PhysicsSchemaTools.intToSdfPath(encoded));contact_side_cache[encoded]=0 if('left_arm_'in path or 'left_NativeHand'in path)else 1 if('right_arm_'in path or 'right_NativeHand'in path)else -1
        return contact_side_cache[encoded]
    def contacts(headers,data):
        global peak
        for h in headers:
            if startup_config.get('ring_contact_debug') and round(physical/DT)%4==0:
                p0=str(PhysicsSchemaTools.intToSdfPath(h.actor0));p1=str(PhysicsSchemaTools.intToSdfPath(h.actor1))
                if any(paths['workpiece_'+part] in [p0,p1] for part in ['ring1','ring2']):
                    # Evaluator-only manifold evidence, never a policy input.
                    rows=[data[i] for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data)]
                    force=sum(float(np.linalg.norm(row.impulse))/(DT/active_physics_substeps) for row in rows)
                    if force>.1:
                        points=[]
                        for row in rows:
                            item={}
                            for field in ['position','normal','impulse']:
                                value=getattr(row,field,None)
                                if value is not None:item[field]=np.asarray(value,float).tolist()
                            points.append(item)
                        ring_contact_debug.append(dict(physics_s=physical,actor0=p0,actor1=p1,force_n=force,contacts=points,evaluator_only=True))
            a=actor_side(h.actor0);b=actor_side(h.actor1)
            if (a==0 and b==1)or(a==1 and b==0):peak=max(peak,sum(float(np.linalg.norm(data[i].impulse))/(DT/active_physics_substeps) for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data)))
            if a in [0,1] and b==-1 or b in [0,1] and a==-1:
                robot_side='left' if (a if a in [0,1] else b)==0 else 'right'
                if empty_donor_contact_guard[robot_side]:
                    other_encoded=h.actor1 if a in [0,1] else h.actor0
                    other_path=str(PhysicsSchemaTools.intToSdfPath(other_encoded))
                    # All ring-stock bodies are prohibited after a verified
                    # transfer and empty-donor withdrawal. The selected actor
                    # from the evaluator is deliberately never consulted.
                    is_ring=('/World/BinStock/' in other_path and 'ring' in other_path) or other_path.endswith(('workpiece_ring1','workpiece_ring2'))
                    if is_ring:
                        force=sum(float(np.linalg.norm(data[i].impulse))/(DT/active_physics_substeps) for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data))
                        if force>empty_donor_payload_peak['force_n']:
                            empty_donor_payload_peak.update(force_n=force,empty_robot_side=robot_side,carried_part=empty_donor_carried_part.get(robot_side),actor0=str(PhysicsSchemaTools.intToSdfPath(h.actor0)),actor1=str(PhysicsSchemaTools.intToSdfPath(h.actor1)),physics_s=physical)
        if pin_contact_evaluator is not None:pin_contact_evaluator.capture(headers,data,physical,DT,active_physics_substeps)
    contact_callback=contacts
    if startup_config.get('profile_contact_callback',False):
        contact_callback_profile=dict(enabled=True,scope='All original callback invocations since subscription; inclusive timers, no contact filtering/sampling changes',callback_calls=0,header_records=0,data_records=0,empty_header_calls=0,max_headers=0,max_data_records=0,inclusive_wall_s=0.,inclusive_thread_cpu_s=0.,max_wall_s=0.,original_exceptions=0,instrumentation_errors=0,unique_actor_side_cache_entries=0)
        report['contact_callback_profile']=contact_callback_profile
        def profiled_contacts(headers,data):
            wall_started=time.perf_counter();thread_started=time.thread_time()
            contact_callback_profile['callback_calls']+=1
            try:
                nh=len(headers);nd=len(data)
                contact_callback_profile['header_records']+=nh;contact_callback_profile['data_records']+=nd
                contact_callback_profile['empty_header_calls']+=int(nh==0)
                contact_callback_profile['max_headers']=max(contact_callback_profile['max_headers'],nh)
                contact_callback_profile['max_data_records']=max(contact_callback_profile['max_data_records'],nd)
            except Exception:
                # Diagnostic failure cannot suppress or alter the safety callback.
                contact_callback_profile['instrumentation_errors']+=1
            try:return contacts(headers,data)
            except BaseException:
                contact_callback_profile['original_exceptions']+=1
                raise
            finally:
                thread_elapsed=time.thread_time()-thread_started;wall_elapsed=time.perf_counter()-wall_started
                contact_callback_profile['inclusive_wall_s']+=wall_elapsed;contact_callback_profile['inclusive_thread_cpu_s']+=thread_elapsed
                contact_callback_profile['max_wall_s']=max(contact_callback_profile['max_wall_s'],wall_elapsed)
                contact_callback_profile['unique_actor_side_cache_entries']=len(contact_side_cache)
        contact_callback=profiled_contacts
    subscription=sim.subscribe_contact_report_events(contact_callback)
    def smooth(u):u=np.clip(u,0,1);return u**3*(10-15*u+6*u*u)
    def add_correction(k,delta,rotvec,end,fade,label,ramp=.3,side='right'):
        w=smooth((phase-phase[k])/ramp)*(1-smooth((phase-end)/fade));R=recipe['rot']if side=='right'else recipe['left_rot'];Ji=(recipe['held_Jinv']if random_config else recipe['Jinv'])if side=='right'else recipe['left_Jinv'];twist=np.c_[np.einsum('nij,j->ni',R,delta),np.einsum('nij,j->ni',R,rotvec)];dq=np.einsum('nij,nj->ni',Ji,twist);commands[:,aidx[side]]+=dq*w[:,None];corrections.append(dict(label=label,translation_m=delta.tolist(),rotation_rad=rotvec.tolist(),at_phase_s=float(phase[k])))
    def consume(e,value,k):
        global floor_offset,live_pin_grasp
        r=value['result'];mode=e['mode'];part=e['part'];before=commands.copy();previous_floor=floor_offset.copy();before_corrections=len(corrections)
        if mode=='held_left'and random_config and random_config.get('left_monitor'):
            e['used']=True;e['delivered_wall_s']=time.perf_counter()-started;print('LEFT_VISION_MONITOR',part,r.get('error')or r.get('pose'),flush=True);return
        if r.get('error'):raise RuntimeError(r['error'])
        if mode=='initial':
            for p in r['poses']:
                ref=next(x for x in recipe['reference_image_initial']['poses']if x['part']==p['part']);delta=np.array(p['center_m'])-ref['center_m']
                if np.linalg.norm(delta)>(.04 if random_config else .002):raise RuntimeError('Fresh detection outside supported visual approach range')
                if p['part']=='ring1'and np.linalg.norm(delta)>.00025:
                    end=recipe['at_old'](3.329);w=smooth((phase-phase[k])/(.65 if random_config else .4))*(1-smooth((phase-end)/.6));twist=np.tile(np.r_[delta,[0,0,0]],(len(phase),1));dq=np.einsum('nij,nj->ni',recipe['Jinv'],twist);commands[:,aidx['right']]+=dq*w[:,None];corrections.append(dict(label='initial_ring1',world_translation_m=delta.tolist()))
            report['size_classification']=r['poses']
        elif mode=='pin_initial':
            delta=np.array(r['pose']['center_m'])-recipe['reference_pin_initial']['center_m']
            if np.linalg.norm(delta)>.002:raise RuntimeError('Pin source changed beyond current taught holder range')
            if np.linalg.norm(delta)>.00025:
                begin=float(np.interp(55.1,recipe['right_source'],phase));end=recipe['at_old'](20.416);w=smooth((phase-begin)/.4)*(1-smooth((phase-end)/.6));twist=np.tile(np.r_[delta,[0,0,0]],(len(phase),1));dq=np.einsum('nij,nj->ni',recipe['Jinv'],twist);commands[:,aidx['right']]+=dq*w[:,None];corrections.append(dict(label='initial_pin',world_translation_m=delta.tolist()))
        elif mode=='floor':
            delta=np.array(r['pose']['center_m'])-e['reference']['pose']['center_m'];floor_normal=np.array(vision.calibration['basket_to_world_R'])[:,2];delta[2]=float(-floor_normal[:2]@delta[:2]/floor_normal[2])if random_config else 0.;extra=delta-floor_offset;floor_offset=delta;stop=recipe['at_old'](12.9);fade=1.3166666667
            if np.linalg.norm(extra)>.08:raise RuntimeError('Observed floor displacement outside local adjustment range')
            if np.linalg.norm(extra)>.00025:
                # Floor displacement is a world translation; convert to the
                # hand-local chart expected by add_correction at this event.
                first=e['source_time_s']<9;ramp=1.4 if first else .35;w=smooth((phase-phase[k])/ramp)*(1-smooth((phase-stop)/fade));translation=w[:,None]*extra
                if first:
                    a=recipe['at_old'](8.3);b=recipe['at_old'](10.4);translation[:,1]*=1-smooth((phase-a)/(b-a))
                twist=np.c_[translation,np.zeros((len(phase),3))];dq=np.einsum('nij,nj->ni',recipe['Jinv'],twist);commands[:,aidx['right']]+=dq;corrections.append(dict(label='floor_'+str(e['source_time_s']),world_translation_m=extra.tolist(),drag_endpoint_y_kept_at_taught_bin_lip=first,floor_height_kept_at_calibrated_fixture=True,floor_slope_z_correction=bool(random_config)))
        else:
            p=r['pose'];G=np.linalg.inv(value['H'])@frame(p['center_m'],axis_frame(p['axis']));ref=e['reference_grasp'];a=ref[:3,2];b=G[:3,2];cross=np.cross(a,b);s=float(np.linalg.norm(cross));rv=cross/max(s,1e-12)*np.arctan2(s,float(a@b));G[:3,:3]=exp(rv)@ref[:3,:3];D=ref@np.linalg.inv(G)
            if mode=='held'and part=='pin':live_pin_grasp=G.copy()
            if mode=='held_left'and(not(random_config and random_config.get('left_feedback'))or random_config.get('left_translation_only')):D=frame(ref[:3,3]-G[:3,3],np.eye(3))
            distance=float(np.linalg.norm(D[:3,3]));angle=float(np.linalg.norm(log(D[:3,:3])))
            if distance>(.04 if random_config else .02)or angle>(.35 if random_config else .25):raise RuntimeError('Friction grasp changed beyond bounded local correction '+part+str((distance,angle)))
            if distance>.00035 or angle>.006:
                side='left'if mode=='held_left'else'right';end_clock=({'ring1':39.99,'ring2':67.7}[part]if side=='left'else {'ring1':27.9,'ring2':52.2,'pin':84.18}[part]);end=float(np.interp(end_clock,recipe['source'if side=='left'else'right_source'],phase));add_correction(k,D[:3,3],log(D[:3,:3]),end,1.2 if random_config and part!='pin'else .75,mode+'_'+part,float(startup_config.get('held_correction_ramp_by_part',{}).get(part,startup_config.get('held_correction_ramp_s',1.0))) if random_config and part!='pin'else .4,side)
        proposed=commands.copy();fraction=1.
        try:audit.validate(commands)
        except RuntimeError:
            for fraction in [.95,.9,.8,.7,.6,.5,.4,.3,.2,.1,0.]:
                commands[:]=before+fraction*(proposed-before)
                try:audit.validate(commands);break
                except RuntimeError:continue
            else:raise RuntimeError('No FR3-safe correction field')
            if mode=='floor':floor_offset=previous_floor+fraction*(floor_offset-previous_floor)
        for correction in corrections[before_corrections:]:correction['fraction_applied_within_motion_limits']=fraction
        e['used']=True;e['delivered_wall_s']=time.perf_counter()-started;print('REALTIME_VISION',mode,part,e['delivered_wall_s'],value['capture_s'],value['processing_s'],flush=True)
    report.update(dof_names=names,link_names=links,camera_preparation_s=camera.preparation_s,reference_motion=recipe['source_run'],stationary_trim_s=recipe['trimmed_stationary_s'],joint_headroom_retiming_saved_s=recipe['joint_headroom_retiming_saved_s'],retimed_free_motion_windows=recipe['retimed_free_motion_windows'],starts_with_rings_already_installed=False,scene_ring_color='same steel material',pin_collision_approximation=json.load(open(source/'pipeline-results.json'))['pin_collision_approximation'])
    scattered_refs={}
    if startup_config.get('scattered_full_task') or startup_config.get('handover_only') or startup_config.get('one_ring_only'):
        reference_root=HERE.parents[1]/'runs/native-roas-table-rgbd-control05';reference_observations=json.load(open(reference_root/'rgb-observations.json'));reference_commands=np.load(reference_root/'executed-actuator-commands.npz')
        for part,receive_t,insert_t in [('ring1',4.55,8.25),('ring2',15.25,19.75)]:
            ob=next(o for o in reference_observations if o['part']==part and o['mode']=='held');at=ob['scene_sim_time_s'];sample=lambda t,ids:np.array([np.interp(t,reference_commands['time_s'],reference_commands['command'][:,j]) for j in ids]);pose=ob['result']['pose'];scattered_refs[part]=dict(part_H=frame(pose['center_m'],axis_frame(pose['axis'])),right_seed=sample(at,aidx['right']),left_receive_q=sample(receive_t,aidx['left']),left_insert_seed=sample(insert_t,aidx['left']))
    if scattered_refs:
        for part,t0,t1 in [('ring1',4.9,8.315),('ring2',15.4,19.80)]:
            reference_commands=np.load(HERE.parents[1]/'runs/native-roas-table-rgbd-control05/executed-actuator-commands.npz');times=np.linspace(t0,t1,19);qs=np.array([[np.interp(t,reference_commands['time_s'],reference_commands['command'][:,j]) for j in aidx['left']]for t in times]);Gref=np.linalg.inv(left_arm.fk(qs[-1]))@frame(goals[part]['P'],np.eye(3));poses=np.array([left_arm.fk(q)@Gref for q in qs]);pre_index=int(np.argmin(np.linalg.norm(poses[:,:3,3]-(np.array(goals[part]['P'])-[0,0,.045]),axis=1)));pre_index=min(max(pre_index,2),len(qs)-3);scattered_refs[part].update(approach_ring_poses=poses,approach_robot_seeds=qs,pre_index=pre_index)
    pre_scattered_checkers={}
    if startup_config.get('scattered_control'):
        os.environ['M80_ROAS_RGB_ONLY']='1'
        from isaac_collision_queries import Checker
        pre_scattered_checkers['right']=Checker(stage,arm,'right')
        if scattered_refs:pre_scattered_checkers['left']=Checker(stage,left_arm,'left')
        for checked in pre_scattered_checkers.values():
            checked.execute_checked_edges=True;checked.gpu_route_enabled=bool(startup_config.get('gpu_route_proposals'));checked.gpu_route_timeout_s=float(startup_config.get('gpu_route_timeout_s',.4))
            if startup_config.get('gpu_route_socket'):checked.gpu_route_socket=str(startup_config['gpu_route_socket'])
            checked.gpu_left_route_proposals=bool(startup_config.get('gpu_left_route_proposals'));checked.gpu_dual_arm_reference=bool(startup_config.get('gpu_dual_arm_reference'));checked.gpu_payload_witness_proposal=bool(startup_config.get('gpu_left_route_proposals')and checked.side=='left')
            if startup_config.get('gpu_left_route_socket'):checked.gpu_left_route_socket=str(startup_config['gpu_left_route_socket'])
            if startup_config.get('gpu_reference_model_json'):checked.gpu_reference_model_json=str(startup_config['gpu_reference_model_json'])
    if all(side in pre_scattered_checkers for side in ['left','right']):
        # Actual-pair checks cannot depend on optional FCL proposal startup.
        for side,checked in pre_scattered_checkers.items():
            other_side='left' if side=='right' else 'right'
            checked.other_configuration=lambda side=other_side:(pre_scattered_checkers[side],art.get_dof_positions()[0,aidx[side]].astype(float),float(art.get_dof_positions()[0,finger_ids[side]].sum()))
        report['actual_pair_context_for_unpreviewed_queries']=True
    observed_grasp_clearance=None;empty_return_waypoints=[]
    if startup_config.get('scattered_control'):
        from rgbd_pick_clearance import ObservedGraspClearance
        observed_grasp_clearance=ObservedGraspClearance(HERE)
        if scattered_refs:
            ref=np.load(HERE.parents[1]/'runs/native-roas-table-rgbd-control05/executed-actuator-commands.npz')
            for t in [3.2,3.4,3.6,6.5,6.8,7.0,7.3]:
                q=np.array([np.interp(t,ref['time_s'],ref['command'][:,j])for j in aidx['right']]);H=arm.fk(q)
                if H[2,3]>1.25 and H[1,3]<-.30 and pre_scattered_checkers['right'].clear(q,.08)[0]:empty_return_waypoints.append(q)
        report['grasp_obstacles_from_RGBD_cloud']=True;report['loose_stock_contact_policy']='Fingertip nudging allowed; RGB-D palm sweep, fixed-structure queries and all physical contacts retained' if startup_config.get('allow_fingertip_nudging') else 'Reject observed loose-stock fingertip approach contacts'
    prepared_receiver_angles={};prepared_pad_contacts={}
    if scattered_refs and observed_grasp_clearance is not None and not (startup_config.get('empty_park_camera_probe')or startup_config.get('gpu_route_static_probe')or startup_config.get('gpu_left_route_static_probe')or startup_config.get('completed_part_static_probe')):
        # Fixed workspace/CAD compilation, independent of loose-stock poses.
        # Live image inference and all grasp/transport path planning remain
        # inside the operation clock below.
        import builtins
        catalog_start=time.perf_counter()
        catalog_key=hashlib.sha256(str((bool(startup_config.get('stable_small_grasp')),bool(startup_config.get('ring_yaw_free')),bool(startup_config.get('balanced_large_receiver_grasp')),bool(startup_config.get('exact_axis_large_receiver_grasp')),startup_config.get('receiver_depth_options_m'),startup_config.get('large_receiver_depth_options_m'))).encode()+(HERE/'grasp_transfer_feasibility.py').read_bytes()+(HERE/'isaac_collision_queries.py').read_bytes()+(HERE/'predictive_pair_clearance.py').read_bytes()+(source/'scene-authored.usda').read_bytes()+left_arm.base.tobytes()+left_arm.bounds.tobytes()+cmd.tobytes()+json.dumps(goals,sort_keys=True).encode()).hexdigest()
        saved_catalog=getattr(builtins,'_m80_fixed_receiver_catalog',{})
        if catalog_key in saved_catalog:
            prepared_receiver_angles,prepared_pad_contacts=saved_catalog[catalog_key];catalog_hit=True
        else:
            catalog_path=HERE/'grasp_transfer_feasibility.py';catalog_module=types.ModuleType('grasp_transfer_feasibility');catalog_module.__file__=str(catalog_path);exec(compile(catalog_path.read_text(),str(catalog_path),'exec'),catalog_module.__dict__)
            catalog_context=dict(config=dict(stable_small_grasp=bool(startup_config.get('stable_small_grasp')),ring_yaw_free=bool(startup_config.get('ring_yaw_free')),reliability_handover_hint=bool(startup_config.get('reliability_handover_hint')),balanced_large_receiver_grasp=bool(startup_config.get('balanced_large_receiver_grasp')),exact_axis_large_receiver_grasp=bool(startup_config.get('exact_axis_large_receiver_grasp')),receiver_depth_options_m=startup_config.get('receiver_depth_options_m'),large_receiver_depth_options_m=startup_config.get('large_receiver_depth_options_m')),left_arm=left_arm,checkers=pre_scattered_checkers,handoff_reference=scattered_refs,observed_grasp_clearance=observed_grasp_clearance,_receiver_angle_cache=prepared_receiver_angles,_pad_contact_cache=prepared_pad_contacts)
            for catalog_part,entry in scattered_refs.items():list(catalog_module.receiving_poses(catalog_context,catalog_part,entry['part_H']))
            saved_catalog[catalog_key]=(prepared_receiver_angles,prepared_pad_contacts);builtins._m80_fixed_receiver_catalog=saved_catalog;catalog_hit=False
        report['fixed_receiver_catalog']=dict(key=catalog_key,cached=catalog_hit,balanced_large_receiver_grasp=bool(startup_config.get('balanced_large_receiver_grasp')),exact_axis_large_receiver_grasp=bool(startup_config.get('exact_axis_large_receiver_grasp')),preparation_s=time.perf_counter()-catalog_start,counts={part:len(rows) for part,rows in prepared_receiver_angles.items()},uses_free_body_poses=False,preparation_excluded_from_operation_clock=True)
        (OUT/'fixed-receiver-catalog.json').write_text(json.dumps(dict(key=catalog_key,angles=prepared_receiver_angles,balanced_large_receiver_grasp=bool(startup_config.get('balanced_large_receiver_grasp')),exact_axis_large_receiver_grasp=bool(startup_config.get('exact_axis_large_receiver_grasp')),source='Fixed CAD/kinematics only; no random scene or image inputs'),indent=2))
    if startup_config.get('fast_transport'):
        try:
            report['fast_transport_planner']=fresh_modules['fcl_transport'].prepare(stage,pre_scattered_checkers,hashlib.sha256((source/'scene-authored.usda').read_bytes()).hexdigest(),[paths['workpiece_'+part] for part in ['ring1','ring2','pin']])
            for planning_side,checker in pre_scattered_checkers.items():
                other_side='left' if planning_side=='right' else 'right'
                if other_side in pre_scattered_checkers:
                    checker.other_configuration=lambda side=other_side:(pre_scattered_checkers[side],art.get_dof_positions()[0,aidx[side]].astype(float),float(art.get_dof_positions()[0,finger_ids[side]].sum()))
        except Exception:
            report['fast_transport_planner']=dict(enabled=False,error=traceback.format_exc(),fallback='Original live PhysX checker')
    if startup_config.get('native_contact_offset_m'):
        offset=float(startup_config['native_contact_offset_m']);assert .00005<=offset<=.001
        changed=0
        for prim in stage.Traverse():
            if '_finger/' in str(prim.GetPath()) and '_NativeHand/' in str(prim.GetPath()) and prim.HasAPI(UsdPhysics.CollisionAPI):
                api=PhysxSchema.PhysxCollisionAPI.Apply(prim);api.CreateContactOffsetAttr(offset);api.CreateRestOffsetAttr(0.);changed+=1
        sim.flush_changes();report['native_contact_generation']=dict(contact_offset_m=offset,rest_offset_m=0.,colliders=changed,applied_after_random_setup=True,collision_geometry_unchanged=True,friction_unchanged=True)
    if startup_config.get('operation_solver_position_iterations'):
        operation_position=int(startup_config['operation_solver_position_iterations']);operation_velocity=int(startup_config.get('operation_solver_velocity_iterations',32))
        assert 16<=operation_position<=256 and 4<=operation_velocity<=128
        for prim in stage.Traverse():
            if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                api=PhysxSchema.PhysxRigidBodyAPI(prim);api.CreateSolverPositionIterationCountAttr(operation_position);api.CreateSolverVelocityIterationCountAttr(operation_velocity)
            if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
                api=PhysxSchema.PhysxArticulationAPI(prim);api.CreateSolverPositionIterationCountAttr(operation_position);api.CreateSolverVelocityIterationCountAttr(operation_velocity)
        sim.flush_changes();report['operation_solver_iterations']=dict(position=operation_position,velocity=operation_velocity,applied_after_random_setup=True)
    prepared_fixed_routes=fresh_modules['fixed_saved_executor'].prepare(pre_scattered_checkers,art,aidx,finger_ids,paths,goals,startup_config,Path(startup_config.get('fixed_route_cache_dir',str(HERE.parents[2]/'.local/fixed-route-cache')))) if startup_config.get('fixed_prevalidated_routes') else None
    if prepared_fixed_routes:report['fixed_saved_route_preparation']=prepared_fixed_routes.get('summary',prepared_fixed_routes)
    prepared_small_left_routes=fresh_modules['fixed_small_left_executor'].prepare(pre_scattered_checkers,art,aidx,finger_ids,paths,goals,startup_config,Path(startup_config.get('fixed_route_cache_dir',str(HERE.parents[2]/'.local/fixed-route-cache')))) if startup_config.get('fixed_prevalidated_routes') else None
    if prepared_small_left_routes:report['fixed_saved_small_left_preparation']=prepared_small_left_routes.get('summary',prepared_small_left_routes)
    prepared_small_donor_routes=fresh_modules['fixed_small_donor_executor'].prepare(pre_scattered_checkers,art,aidx,finger_ids,paths,goals,startup_config,Path(startup_config.get('fixed_route_cache_dir',str(HERE.parents[2]/'.local/fixed-route-cache')))) if startup_config.get('fixed_prevalidated_routes') else None
    if prepared_small_donor_routes:report['fixed_saved_small_donor_preparation']=prepared_small_donor_routes.get('summary',prepared_small_donor_routes)
    if startup_config.get('gpu_pair_native_width_sweep'):report['GPU_native_width_full_model_qualification']=fresh_modules['production_width_probe'].qualify(pre_scattered_checkers,startup_config,paths,aidx,finger_ids,art)
    good_since=None;good_since_physical=None;completed=None;last_checks={};last_k=0;last_trace=0.;floor_y_reset=False;profile={'control_s':0.,'physics_s':0.,'clamp_flush_s':0.};soft_left=False;force_state={'left':False,'right':False};air_grasp_active=False;report['standby_preparation_wall_s']=time.perf_counter()-preparation_started;gc_was_enabled=gc.isenabled();gc.disable();report['automatic_gc_deferred_during_cycle']=True;started=time.perf_counter();vision.start=started;report['first_capture_wall_epoch_s']=time.time();tape.append(dict(t=0.,sim=0.,phase=0.,q=art.get_dof_positions()[0].copy(),links=art.get_link_transforms()[0].copy(),camera_bodies=camera.all_bodies.get_transforms().copy(),parts=np.array([v.get_transforms()[0]for v in truth.values()])))
    if startup_config.get('scattered_control'):
        import types
        plugin_path=HERE/('probe_empty_park_observe.py' if startup_config.get('empty_park_camera_probe') else 'probe_double_handover_endpoints.py' if startup_config.get('double_handover_endpoint_probe') else 'probe_completed_part.py' if startup_config.get('completed_part_static_probe') else 'probe_gpu_left_route.py' if startup_config.get('gpu_left_route_static_probe') else 'probe_gpu_native_route.py' if startup_config.get('gpu_route_static_probe') else 'probe_pin_native_catalog.py' if startup_config.get('pin_grasp_catalog_probe') else 'probe_left_carry_pair.py' if startup_config.get('left_carry_pair_probe_source_run') else 'probe_bin_axial_grasp.py' if startup_config.get('bin_axial_pick_probe') else 'probe_tilted_ring_hook.py' if startup_config.get('tilted_ring_hook_probe') else 'probe_aperture_path_check.py' if startup_config.get('aperture_path_check_probe') else 'probe_small_right_carry.py' if startup_config.get('small_right_carry_probe') else 'probe_small_right_left_park.py' if startup_config.get('right_left_park_fixture_probe') else 'native_right_ring_insert.py' if startup_config.get('right_ring_insert_probe') else 'probe_native_right_ring.py' if startup_config.get('right_insertion_fixture_probe') else 'loose_bin_rearrange.py' if startup_config.get('rearrange_probe') else 'probe_axial_receiver.py' if startup_config.get('axial_receiver_probe') else 'probe_handover_timing.py' if startup_config.get('handover_probe_source_run') else 'probe_pin_transport.py' if startup_config.get('pin_transport_probe_source_run') else 'probe_pin_pick_route.py' if startup_config.get('pin_approach_probe_source_run') else 'probe_native_ring_insertion.py' if startup_config.get('insertion_probe_source_run') else 'probe_stable_native_transfer.py' if startup_config.get('stable_pair_probe') else 'probe_scatter_clearance.py' if startup_config.get('clearance_diagnostic_only') else 'probe_native_coupling.py' if startup_config.get('native_coupling_probe_only') else 'probe_native_direct_ring.py' if startup_config.get('direct_goal_probe') else 'loose_bin_task.py' if startup_config.get('scattered_full_task') or startup_config.get('handover_only') or startup_config.get('one_ring_only') else 'loose_ring_pick.py');plugin=types.ModuleType('loose_ring_pick');plugin.__file__=str(plugin_path);exec(compile(plugin_path.read_text(),str(plugin_path),'exec'),plugin.__dict__)
        os.environ['M80_ROAS_RGB_ONLY']='1'
        from isaac_collision_queries import Checker
        scattered_checker=pre_scattered_checkers['right']
        (OUT/'robot-kinematics.json').write_text(json.dumps(dict(base=arm.base.tolist(),tip=arm.tip.tolist(),frames=[[a.tolist(),b.tolist()] for a,b in arm.frames],axes=[a.tolist() for a in arm.axes],bounds=arm.bounds.tolist(),seed=raw['command'][0,aidx['right']].tolist()),indent=2))
        scheduled_clamps={};left_queue_clock=lambda:0
        report['motor_velocity_cap_policy']='Signed position-dependent FR3 bounds at encoder position plus one-step margin; 93% drive cap and unchanged 95% actual velocity acceptance'
        report['wall_pacing']='Each 480 Hz control step advances two 960 Hz PhysX substeps and takes at least 1/480 s wall time; no catch-up fast-forward'
        report['CPU_planning_advances_physics']=False;report['hard_realtime_controller_validated']=False
        report.update(physics_hz=480*int(startup_config.get('scattered_physics_substeps',2)),physics_hz_range=[480*int(startup_config.get('scattered_physics_substeps',2))]*2,physics_substeps_per_command=int(startup_config.get('scattered_physics_substeps',2)))
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
        def bind_validation_target(part,image_pose):
            # Evaluator only. The selected actor ID or its pose is never
            # returned to the control policy; RGB-D drives every target.
            options=[(path,b) for path,b,_ in random_views if (part in path)]
            path,b=min(options,key=lambda row:np.linalg.norm(row[1].get_transforms()[0,:3]-np.array(image_pose['center_m'])))
            truth[part]=b;paths['workpiece_'+part]=path;slot=list(truth).index(part);body_slot=list(camera.all_bodies.prim_paths).index(path)
            for sample in tape:sample['parts'][slot]=sample['camera_bodies'][body_slot]
            report.setdefault('selected_actor_ids_for_validation_only',{})[part]=path
        def schedule_clamp(part,at):scheduled_clamps[part]=at
        def set_left_clock(clock):
            globals()['left_queue_clock']=clock
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
        task_checkers=pre_scattered_checkers
        def close_scattered_clamp(part):
            for attribute,limit in clamp_targets[part]:attribute.Set(limit)
            sim.flush_changes()
        def set_scattered_left_force(force):
            report['finger_effort_caps_n']['left_final_insertion_per_finger']=float(force)
            forces[0,finger_ids['left']]=[force,force] if shared_motor else [2*force,.0001];art.set_dof_max_forces(forces,idx)
        task_ctx=dict(left_hand_known_empty=True,art=art,arm=arm,ids=aidx['right'],fi=finger_ids['right'],cmd=cmd,idx=idx,vision=vision,checker=scattered_checker,started=started,stiffness=stiffness,damping=damping,dt=DT,step=scattered_step,config={k:v for k,v in startup_config.items() if k in ['pick_part','fixed_supply_mode','defer_next_initial_until_receiver_bound','fixed_large_pregrasp_preferred_views','scattered_full_task','handover_only','deadline_s','direct_goal_probe','left_pin_only_probe','allow_fingertip_nudging','allow_internal_grasp','allow_single_finger_rearrangement','one_ring_only','small_ring_first','allow_chord_grasp','handover_plan_budget_s','allow_side_entry','optimized_motion_timing','native_grip_pad_force_n','gentle_descent','stability_time_scale','grasp_settle_s','stable_small_grasp','prefer_internal_small_grasp','ring_yaw_free','reliability_handover_hint','stability_serial','stability_finish_rings_before_pin','insertion_probe_source_run','rim_grasp_probe','canonical_grasp_probe','pin_approach_probe_source_run','pin_transport_probe_source_run','pin_transport_left_standby','pin_first','descent_contact_speed_m_s','shortcut_checked_routes','parallel_transfer_approach','pregrasp_floor_m','prefer_near_handover_branch','pregrasp_floor_small_m','warm_spline_routes','pin_between_rings','center_large_receiver_grasp','balanced_large_receiver_grasp','exact_axis_large_receiver_grasp','handover_probe_source_run','handover_offsets_override','sort_donor_orientations','local_left_park','wait_for_ring_transport_clear','small_receiver_opening_m','large_receiver_opening_m','receiver_approach_standoff_m','receiver_approach_resolution_rad','receiver_approach_step_m','pickup_multiview','require_fresh_preclose_rgbd','small_ring_com_x_limit_m','small_ring_repick_on_unstable_hold','pickup_retry_limit','rim_only_pickup','allow_local_rim_grasp','prefer_local_rim_grasp','stable_receiver_grasp','initial_pickup_multiview','rim_encoder_recenter','strict_rim_clutter','native_loaded_preload_after_contact_m_s','native_loaded_preload_ramp_s','inner_only_pickup','native_inner_bore_grasp','inner_bore_friction_reserve_mu','allow_bin_release_retry','verify_receiver_pad_contact','inner_bore_x_tilt_candidates_deg','inner_bore_center_correction_limit_m','receiver_axial_only','axial_receiver_fixture_budget_s','axial_receiver_probe','all_rings_yaw_free','rearrange_probe','rearrange_then_pick','rearrange_tcp_offset_m','require_front_up_pickup','preclose_occlusion_max_age_s','verify_lifted_pad_contact','rearrange_front_up_only','right_insertion_fixture_probe','allow_partial_preclose_tracking','right_ring_insert_probe','allow_direct_right_insert','allow_direct_right_insert_by_part','fixed_supply_paired_overlap','fixed_saved_route_priors','fixed_prevalidated_routes','fixed_pickup_canonical_priors','fixed_pickup_canonical_join_q','right_direct_smooth_verified_lines','fixed_initial_precision_rgbd','fixed_initial_precision_parts','gpu_pair_native_width_sweep','gpu_pair_schedule_certificate','gpu_pair_schedule_socket','gpu_pair_schedule_timeout_s','safe_float32_receiver_endpoints','fixed_supply_resume_after_right_grasp','require_direct_right_insert_parts','large_goal_aware_empty_left_parking','large_goal_aware_park_budget_s','pin_skip_park_when_current_terminal_clear','pin_rgbd_view_order_by_label','right_left_park_fixture_probe','right_direct_empty_left_parking','small_right_carry_probe','reuse_aperture_invariant_checks','right_carry_vertical_escape','prefer_right_carry_small_posture','avoid_other_stock_finger_contact','stable_outer_donor_grasp','allow_tilted_bore_hook','rearrangement_retry_limit','broaden_tilted_bore_hook_proposals','rearrangement_pickup_extra_eye_offsets_m','donor_release_opening_cap_m','adaptive_initial_ring_order','pickup_diagonal_offsets_m','receiver_depth_options_m','right_fixture_replan_on_alignment_failure','close_empty_left_for_pin','large_receiver_depth_options_m','large_encoder_pose_bias_compensation','require_broad_large_donor_support','right_fixture_oblique_camera_views','defer_handover_until_visible_clearance','left_carry_pair_probe_source_run','verify_large_lift_grasp_retention','large_lift_grasp_translation_limit_m','large_lift_grasp_axis_limit_rad','verify_large_donor_after_carry','robust_annular_camera_view_bank','verify_selected_outer_approach','robust_pin_cap_circle','pin_precision_image_width_px','pin_visual_preinsert_alignment','pin_axis_goal_world_m','pin_grasp_catalog_probe','pin_grasp_catalog_source_run','pin_grasp_catalog_grip_z_m','pin_grasp_catalog_budget_s','pin_grasp_catalog_max_candidates','pin_grasp_catalog_run','pin_grasp_catalog_candidate','pin_axis_frame_consistent_geometry','pin_rgbd_sdk_hold_confirmation','pin_terminal_stroke_preflight','pin_axial_yaw_free','sequential_initial_ring_selection','park_before_bin_pick','empty_park_camera_probe','empty_park_known_empty','empty_park_targets_world_m','empty_park_planning_budget_s','empty_park_preshape_m','gpu_route_proposals','gpu_left_route_proposals','gpu_left_route_socket','gpu_route_socket','gpu_route_timeout_s','gpu_route_static_probe','gpu_left_route_static_probe','completed_part_static_probe','gpu_route_probe_source_run','gpu_dual_arm_reference','gpu_reference_model_json','gpu_ik_proposals','gpu_ik_socket','gpu_ik_timeout_s','verify_native_pin_hold','measured_loaded_contact_geometry','small_measured_loaded_contact_geometry','small_balanced_receiver_axis30','project_large_receiver_closing_axis','large_receiver_tool_axis_aligned','native_large_receiver_tilt25_only','large_handover_part_axis_aligned','temporary_axial_double_handover_component','temporary_axial_air_part_center_m','axial_cap_first_contact_patch','temporary_axial_preposition_seed','temporary_axial_right_receiver_seed','temporary_axial_right_via_seed','temporary_axial_right_via_center_m','reuse_dual_aperture_issued_checks','allow_guarded_full_donor_release','refresh_reverse_handover_standoff','axial_cap_contact_band_m','temporary_axial_two_location_handover','measured_initial_donor_contact_geometry','temporary_holder_preclose_pose_feedback','reuse_actual_other_context_checks','pin_goal_aware_empty_left_park','pin_preclose_visual_alignment','pin_preclose_wide_opening','pin_only_component','pin_visual_servo_gain','pin_visual_adaptive_gain','pin_visual_independent_component_feedback','pin_visual_bounded_component_recovery','pin_qualified_postlift_preload','pin_collar_flat_contact_target','pin_collar_flat_offset_m','pin_reset_feedback_after_rebranch','pin_full_depth_require_robust_circle','pin_preclose_servo_gain','pin_prefer_canonical_pickup_branch','pin_parallel_tail_contact_target','pin_tail_flat_contact_target','pin_tangent_flip_yaw_first','pin_tangent_flip_air_waypoint','pin_engine_readonly_diagnostics','small_receiver_one_shot_settled_reconfirmation','small_preinsert_one_shot_settled_reconfirmation','pin_precision_component_deadband','pin_precision_encoder_settle','large_double_handover_full_task','right_direct_skip_greedy_shortcuts','final_empty_right_clearance','right_direct_smooth_verified_carry']},calibration=vision.calibration,pickup_seed=raw['command'][0,aidx['right']].tolist(),left_arm=left_arm,left_ids=aidx['left'],left_fi=finger_ids['left'],checkers=task_checkers,handoff_reference=scattered_refs,goals=goals,bind_validation_target=bind_validation_target,schedule_clamp=schedule_clamp,set_left_queue_clock=set_left_clock,close_clamp=close_scattered_clamp,set_left_force=set_scattered_left_force,set_grip_hold=set_native_grip_hold,set_empty_donor_contact_guard=set_empty_donor_contact_guard,observed_grasp_clearance=observed_grasp_clearance,empty_return_waypoints=empty_return_waypoints,planning_diagnostics=report.setdefault('planning_diagnostics',[]),_receiver_angle_cache={part:list(rows) for part,rows in prepared_receiver_angles.items()},_pad_contact_cache=dict(prepared_pad_contacts),part_collision_templates={part:source_path for part,source_path in [(p,paths['workpiece_'+p])for p in ['ring1','ring2']]})
        task_ctx['fixed_prevalidated_routes']=prepared_fixed_routes
        task_ctx['fixed_prevalidated_small_left_routes']=prepared_small_left_routes
        task_ctx['fixed_prevalidated_small_donor_routes']=prepared_small_donor_routes
        task_ctx['pin_context']=dict(art=art,arm=arm,ids=aidx['right'],fi=finger_ids['right'],cmd=cmd,idx=idx,vision=vision,checker=scattered_checker,stage=stage,sim=sim,paths=paths,goal=goals['pin']['P'],truth=truth,started=started,stiffness=stiffness,damping=damping,dt=DT,candidate=pin_candidate,step=scattered_step,handoff_withdraw=False,report={},optimized_motion_timing=bool(startup_config.get('optimized_motion_timing')),stability_time_scale=float(startup_config.get('stability_time_scale',1.)),set_grip_hold=lambda enabled,preload_m_s=None:set_native_grip_hold('right',enabled,preload_m_s=preload_m_s))
        if startup_config.get('pin_only_component',False):report.update(pin_only_component=True,component_part='pin',full_task_pass=False,vision_end_to_end_pass=False,requires_whole_three_part_trial=True)
        report['pin_skill_progress']=task_ctx['pin_context']['report']
        issued_guard_profiler=None
        if startup_config.get('profile_issued_guards',False):
            from issued_guard_timer import install as install_issued_guard_timer
            issued_guard_profiler=install_issued_guard_timer(task_checkers)
        try:
            result=plugin.execute(task_ctx)
        finally:
            if issued_guard_profiler is not None:
                report['issued_guard_profile']=issued_guard_profiler.detach_collect()
        report.update(state='scattered_ring_pick_probe',scattered_pick=result,scene_only=False,wall_verified_elapsed_s=time.perf_counter()-started)
        if startup_config.get('pin_only_component',False):
            if result.get('state')!='pin_only_component_executed' or result.get('full_task_pass') is not False:raise RuntimeError('Invalid PIN-only component result boundary')
            report.update(state='pin_only_component_pass'if result['pin'].get('pin_single_stroke_pass')else'pin_only_component_failed',pin_only_component=True,component_part='pin',pin_component_pass=bool(result['pin'].get('pin_single_stroke_pass')),full_task_pass=False,vision_end_to_end_pass=False,end_to_end_under20=False,end_to_end_under30=False,requires_whole_three_part_trial=True,ring_tasks_executed=0,simulation_elapsed_s=physical)
        if startup_config.get('scattered_full_task') and not startup_config.get('pin_only_component',False):
            final_rows=[r for r in tape if r['sim']>=physical-.55];checks=[]
            for row in final_rows:
                part_checks={part:dict(position_error_m=float(np.linalg.norm(row['parts'][i,:3]-goals[part]['P'])),axis_error_rad=float(np.arccos(np.clip(rotation(row['parts'][i,3:])[2,2],-1,1)))) for i,part in enumerate(truth)}
                hands_clear=all(min(np.linalg.norm(row['links'][h,:3]-body[:3]) for h in roots)>.16 for body in row['parts']);checks.append(all(v['position_error_m']<.0015 and v['axis_error_rad']<.05 for v in part_checks.values()) and hands_clear)
            elapsed=time.perf_counter()-started;retention_ok=bool(len(final_rows)>1 and final_rows[-1]['sim']-final_rows[0]['sim']>=.5 and final_rows[-1]['t']-final_rows[0]['t']>=.5 and all(checks))
            report.update(state='end_to_end_under20_pass' if retention_ok and elapsed<20 else 'completed_over_budget' if retention_ok else 'physical_acceptance_failed',full_task_pass=retention_ok,vision_end_to_end_pass=retention_ok,end_to_end_under20=bool(retention_ok and elapsed<20),end_to_end_under30=bool(retention_ok and elapsed<30),verified_elapsed_s=elapsed,cycle_time_s=elapsed,simulation_elapsed_s=physical,final_checks=part_checks,final_retention_observation_s=final_rows[-1]['t']-final_rows[0]['t'],final_retention_physics_s=final_rows[-1]['sim']-final_rows[0]['sim'])
        if startup_config.get('one_ring_only'):
            final_rows=[r for r in tape if r['sim']>=physical-.55];checks=[];component_errors=[]
            for row in final_rows:
                component_part=startup_config.get('pick_part','ring1');component_slot=list(truth).index(component_part);component_body=row['parts'][component_slot];pe=float(np.linalg.norm(component_body[:3]-goals[component_part]['P']));ae=float(np.arccos(np.clip(rotation(component_body[3:])[2,2],-1,1)));clearance=min(float(np.linalg.norm(row['links'][h,:3]-component_body[:3]))for h in roots)
                checks.append(pe<.0015 and ae<.05 and clearance>.16);component_errors.append(dict(position_error_m=pe,axis_error_rad=ae,hand_clearance_m=clearance))
            passed=bool(len(final_rows)>1 and final_rows[-1]['sim']-final_rows[0]['sim']>=.5 and final_rows[-1]['t']-final_rows[0]['t']>=.5 and all(checks))
            report.update(state='one_ring_component_pass'if passed else'one_ring_component_failed',one_ring_only=True,component_part=component_part,one_ring_seating_release_retention_pass=passed,component_final_checks=component_errors,component_retention_wall_s=final_rows[-1]['t']-final_rows[0]['t'],component_retention_physical_s=final_rows[-1]['sim']-final_rows[0]['sim'],full_task_pass=False,end_to_end_under20=False)
        if actuators:audit.validate(np.array([v['q'] for v in actuators]))
        report['FR3_command_limit_audit']=audit.report()
        if startup_config.get('one_ring_only'):report['one_ring_component_accepted']=bool(report.get('one_ring_seating_release_retention_pass'))
        raise SceneProbeComplete()
    if startup_config.get('scatter_probe_only'):
        event=dict(mode='initial',part=None);future=vision.request(event,arm.fk(art.get_dof_positions()[0,aidx['right']]))
        while not future.done():
            vision.poll();time.sleep(.001)
            if time.perf_counter()-started>5:raise RuntimeError('Scattered scene RGB-D timeout')
        result=future.result()['result'];event['delivered_wall_s']=time.perf_counter()-started
        report.update(state='scattered_scene_perception_probe',scene_only=True,initial_RGBD=result,wall_verified_elapsed_s=event['delivered_wall_s'])
        raise SceneProbeComplete()
    for k in range(1,len(commands)):
        if startup_config.get('debug_stop_phase_s') and phase[k]>=float(startup_config['debug_stop_phase_s']):break
        if direct_pin and k%2==1 and phase[k]>=recipe['at_old'](float(startup_config.get('direct_pin_source_start_s',17.1))):break
        if k%2==1:batch_started=time.perf_counter()
        control_started=time.perf_counter()
        logical=float(phase[k]);critical=logical<2.7 or 11.0<logical<16.6 or 19.7<logical<21.9 or 24.0<logical<26.2;stride=1
        if k-last_k<stride:continue
        step_dt=(k-last_k)*DT;last_k=k;H=None;vision.poll()
        if logical>=recipe['at_old'](10.4)and not floor_y_reset:floor_offset[1]=0.;floor_y_reset=True
        for e in events:
            if 'future'not in e and logical>=e['request_at']:
                if H is None:H=arm.fk(art.get_dof_positions()[0,aidx['right']])
                e['floor_offset']=floor_offset.copy();event_H=left_arm.fk(art.get_dof_positions()[0,aidx['left']])if e['mode']=='held_left'else H;e['future']=vision.request(e,event_H)
            fixed_application=bool(random_config and random_config.get('fixed_visual_application'))
            deadline=(.20 if e['mode']=='initial'else .25 if e['mode']=='pin_initial'else e['deadline'])if fixed_application else(.85 if e['mode']in ['initial','pin_initial']else e['deadline'])
            if not e.get('used')and 'future'in e and e['future'].done()and(not fixed_application or logical>=deadline):
                consume(e,e['future'].result(),k)
            if not e.get('used')and logical>=deadline:raise RuntimeError('Missed live perception deadline '+e['mode']+' '+str(e['part']))
        old=cmd.copy();cmd[:]=commands[k]
        width_guard=bool(random_config and random_config.get('width_guard')and((10.4 if startup_config.get('early_width_guard') else 10.80)<=recipe['right_source'][k]<27.31 or (42.8 if startup_config.get('early_width_guard') else 44.05)<=recipe['right_source'][k]<51.63))
        if width_guard:
            diameter=.075 if recipe['right_source'][k]<30 else .062;margin=float(startup_config.get('ring2_grip_margin_m',.0008)) if diameter<.07 else .0008
            if diameter<.07:margin+=(float(startup_config.get('ring2_hold_margin_m',margin))-margin)*smooth((recipe['right_source'][k]-46.5)/1.)
            cmd[0,finger_ids['right']]=np.maximum(cmd[0,finger_ids['right']],(diameter-margin)/2)
        cmd[0,fi]=np.clip(cmd[0,fi],old[0,fi]-.0475*step_dt,old[0,fi]+.0475*step_dt);audit.update(cmd);finger_speed=float(abs(cmd[0,fi]-old[0,fi]).max()/step_dt)
        if finger_speed>.0476:raise RuntimeError('Native finger target speed exceeded')
        velocity_target=(cmd-old)/step_dt;l=recipe['source'][k];r=recipe['right_source'][k];wanted=dict(left=False,right=(10.80<=r<27.31 or 44.05<=r<51.63 or 63.30<=r<81.82));grip_changed=False
        if r>=81.82 and force_state['right']and not wanted['right']and not startup_config.get('legacy_two_stage_pin_demo',False):
            if live_pin_grasp is None:raise RuntimeError('Pin release blocked: no RGB-D grasp measurement')
            predicted=arm.fk(art.get_dof_positions()[0,aidx['right']])@live_pin_grasp;error=float(np.linalg.norm(predicted[:3,3]-goals['pin']['P']));axis_error=float(np.arccos(np.clip(predicted[2,2],-1,1)))
            report['pin_release_gate']=dict(predicted_center_from_RGBD_grasp_and_encoders=predicted[:3,3].tolist(),position_error_m=error,axis_error_rad=axis_error,ground_truth_used=False)
            if error>=.0015 or axis_error>=.05:raise RuntimeError('Pin release blocked before full axial insertion; legacy partial-release/re-approach pusher is disabled')
        for side,active in wanted.items():
            ids=finger_ids[side]
            if active!=force_state[side]:
                damping[0,ids]=(300. if width_guard and side=='right'else 1000.)if active else 100.;stiffness[0,ids]=50000. if width_guard and side=='right'else 5000.;force_state[side]=active;grip_changed=True
            if active and not(width_guard and side=='right'):velocity_target[0,ids]=-.0475
        requested_air_grasp=bool(startup_config.get('ring2_air_grasp_preload_n') and 46.5<=r<51.63)
        if requested_air_grasp!=air_grasp_active:
            air_grasp_active=requested_air_grasp;stiffness[0,finger_ids['right']]=0. if air_grasp_active else(50000. if width_guard and wanted['right'] else 5000.);damping[0,finger_ids['right']]=1000. if air_grasp_active else(300. if width_guard and wanted['right'] else 100.);grip_changed=True
        if air_grasp_active:velocity_target[0,finger_ids['right']]=-float(startup_config['ring2_air_grasp_preload_n'])/1000.
        if grip_changed:art.set_dof_dampings(damping,idx);art.set_dof_stiffnesses(stiffness,idx)
        requested_soft_left=64.7<=l<67.7
        if requested_soft_left!=soft_left:
            soft_left=requested_soft_left;forces[0,finger_ids['left']]=15. if soft_left else 30.;art.set_dof_max_forces(forces,idx)
        update_motor_speed_limits();art.set_dof_position_targets(cmd,idx);art.set_dof_velocity_targets(velocity_target,idx);changed=False;clamp_updates=[]
        for j,p in enumerate(truth):
            if k%16!=0 or abs(controls[k,j]-last_clamp[j])<1e-7:continue
            last_clamp[j]=controls[k,j];changed=True
            clamp_updates.extend((attribute,float(limit*last_clamp[j]))for attribute,limit in clamp_targets[p])
        if changed:
            if clamp_driver is not None:clamp_driver.command(controls[k])
            else:
                with Sdf.ChangeBlock():
                    for attribute,value in clamp_updates:attribute.Set(value)
                flush_started=time.perf_counter();sim.flush_changes();profile['clamp_flush_s']+=time.perf_counter()-flush_started
        physics_started=time.perf_counter();profile['control_s']+=physics_started-control_started
        physics_window=startup_config.get('physics_substep_window_s');active_physics_substeps=physics_substeps if not physics_window or physics_window[0]<=logical<physics_window[1] else 1
        for substep in range(active_physics_substeps):sim.simulate(step_dt/active_physics_substeps,physical+substep*step_dt/active_physics_substeps);sim.fetch_results()
        physical+=step_dt;profile['physics_s']+=time.perf_counter()-physics_started
        # Read validation/rendering state inside the same real-time budget.
        # Waiting occurs last; slow batches never trigger fast-forward later.
        if peak>12:raise RuntimeError('Cross-arm contact exceeded 12 N')
        actuator_q=cmd[0].copy();snapshot=None
        if physical-last_trace>=4*DT-1e-8:
            last_trace=physical
            body=np.array([v.get_transforms()[0]for v in truth.values()]);link=art.get_link_transforms()[0].copy();checks={p:dict(position_error_m=float(np.linalg.norm(body[j,:3]-goals[p]['P'])),axis_error_rad=float(np.arccos(np.clip(rotation(body[j,3:])[2,2],-1,1))))for j,p in enumerate(truth)};good=all(v['position_error_m']<.0015 and v['axis_error_rad']<.05 for p,v in checks.items() if not rings_only or p!='pin')and all(min(np.linalg.norm(link[h,:3]-tr[:3])for h in roots)>.16 for tr in (body[:2] if rings_only else body))
            snapshot=dict(sim=physical,phase=logical,q=art.get_dof_positions()[0].copy(),links=link,parts=body,camera_bodies=camera.all_bodies.get_transforms().copy());last_checks=checks
        remaining=batch_started+2*DT-time.perf_counter()if k%2==0 else 0.
        if remaining>.0005:time.sleep(remaining-.0005)
        if k%2==0:
            while time.perf_counter()<batch_started+2*DT:pass
        wall=time.perf_counter()-started;actuators.append(dict(t=wall,q=actuator_q,phase=logical,physics_substeps=active_physics_substeps))
        if snapshot is not None:
            snapshot['t']=wall;tape.append(snapshot)
            if good:
                if good_since is None:good_since=wall;good_since_physical=physical
                if wall-good_since>=.5 and physical-good_since_physical>=.5:completed=wall;break
            else:good_since=None;good_since_physical=None
        if rings_only and logical>=recipe['at_old'](21.):break
        if k%480==0:print('REALTIME_STEP',round(wall,3),round(physical,3),round(logical,3),flush=True)
    if direct_pin and not rings_only:
        import types
        pin_path=HERE/'direct_pin_skill.py';pin_module=types.ModuleType('direct_pin_skill');pin_module.__file__=str(pin_path);exec(compile(pin_path.read_text(),str(pin_path),'exec'),pin_module.__dict__)
        pin_start_physical=physical;pin_start_phase=float(phase[last_k]);pin_report={};report['direct_pin_trial']=pin_report;report['direct_pin_start_wall_s']=time.perf_counter()-started
        def direct_step(label):
            global physical,last_checks,good_since,good_since_physical,completed,soft_left,active_physics_substeps
            tick=time.perf_counter();logical=pin_start_phase+physical-pin_start_physical;slot=min(len(commands)-1,int(round(logical/DT)));old_command=actuators[-1]['q'][None,:] if actuators else raw['command'][:1];cmd[0,aidx['left']]=commands[slot,aidx['left']];cmd[0,finger_ids['left']]=commands[slot,finger_ids['left']];velocity=(cmd-old_command)/DT
            cmd[0,fi]=np.clip(cmd[0,fi],old_command[0,fi]-.0475*DT,old_command[0,fi]+.0475*DT);velocity=(cmd-old_command)/DT
            requested=64.7<=recipe['source'][slot]<67.7
            if requested!=soft_left:soft_left=requested;forces[0,finger_ids['left']]=15. if soft_left else 30.;art.set_dof_max_forces(forces,idx)
            changes=[]
            for j,p in enumerate(['ring1','ring2']):
                if abs(controls[slot,j]-last_clamp[j])>1e-7:
                    last_clamp[j]=controls[slot,j];changes.extend((attribute,float(limit*last_clamp[j])) for attribute,limit in clamp_targets[p])
            if changes:
                with Sdf.ChangeBlock():
                    for attribute,value in changes:attribute.Set(value)
                sim.flush_changes()
            update_motor_speed_limits();art.set_dof_position_targets(cmd,idx);art.set_dof_velocity_targets(velocity,idx);audit.update(cmd)
            physics_window=startup_config.get('physics_substep_window_s');active_physics_substeps=physics_substeps if not physics_window or physics_window[0]<=logical<physics_window[1] else 1
            control_finished=time.perf_counter();profile['control_s']+=control_finished-tick
            for substep in range(active_physics_substeps):sim.simulate(DT/active_physics_substeps,physical+substep*DT/active_physics_substeps);sim.fetch_results()
            profile['physics_s']+=time.perf_counter()-control_finished
            physical+=DT
            if peak>12:
                report['cross_arm_abort']=dict(peak_force_n=float(peak),phase=label,command=cmd[0].tolist(),encoder=art.get_dof_positions()[0].tolist())
                raise RuntimeError('Cross-arm contact exceeded 12 N')
            record=round(physical/DT)%4==0
            if record:
                body=np.array([v.get_transforms()[0]for v in truth.values()]);link=art.get_link_transforms()[0].copy();checks={p:dict(position_error_m=float(np.linalg.norm(body[j,:3]-goals[p]['P'])),axis_error_rad=float(np.arccos(np.clip(rotation(body[j,3:])[2,2],-1,1)))) for j,p in enumerate(truth)};good=all(v['position_error_m']<.0015 and v['axis_error_rad']<.05 for v in checks.values()) and all(min(np.linalg.norm(link[h,:3]-tr[:3]) for h in roots)>.16 for tr in body);last_checks=checks;snapshot=dict(sim=physical,phase=logical,q=art.get_dof_positions()[0].copy(),links=link,parts=body,camera_bodies=camera.all_bodies.get_transforms().copy(),pin_phase=label)
            remaining=tick+DT-time.perf_counter()
            if remaining>.0003:time.sleep(remaining-.0003)
            while time.perf_counter()<tick+DT:pass
            wall=time.perf_counter()-started;actuators.append(dict(t=wall,q=cmd[0].copy(),phase=logical,pin_phase=label,physics_substeps=active_physics_substeps))
            if record:
                snapshot['t']=wall;tape.append(snapshot)
                if good:
                    if good_since is None:good_since=wall;good_since_physical=physical
                    if wall-good_since>=.5 and physical-good_since_physical>=.5:completed=wall
                else:good_since=None;good_since_physical=None;completed=None
        pin_report=pin_module.execute(dict(art=art,arm=arm,ids=aidx['right'],fi=finger_ids['right'],cmd=cmd,idx=idx,vision=vision,checker=pin_checker,stage=stage,sim=sim,paths=paths,goal=goals['pin']['P'],truth=truth,started=started,stiffness=stiffness,damping=damping,dt=DT,candidate=pin_candidate,step=direct_step,handoff_withdraw=False,report=pin_report))
        report['direct_pin_trial']=pin_report
        for _ in range(8):direct_step('all_parts_final_retention')
        audit.validate(np.array([v['q']for v in actuators]));report['actual_issued_complete_field_audited']=True
    elapsed=time.perf_counter()-started;report['ring_pair_pass']=bool(completed is not None) if rings_only else None;physical_pass=completed is not None and not rings_only;report.update(state='end_to_end_pass'if physical_pass and elapsed<30 else'completed_over_budget'if physical_pass else'physical_acceptance_failed',full_task_pass=physical_pass,vision_end_to_end_pass=physical_pass,end_to_end_under30=bool(physical_pass and elapsed<30),cycle_time_s=completed,verified_elapsed_s=elapsed,wall_verified_elapsed_s=elapsed,simulation_elapsed_s=physical,final_retention_observation_s=(elapsed-good_since)if good_since is not None else 0,final_retention_physics_s=(physical-good_since_physical)if good_since_physical is not None else 0,final_checks=last_checks,perception_observations=len(vision.jobs),perception_body_pose_reads=0,FR3_command_limit_audit=audit.report(),cross_arm_peak_contact_n=peak,observation_driven_corrections=corrections)
except SceneProbeComplete:pass
except Exception:
    report.update(state='failed',error=traceback.format_exc(),full_task_pass=False,vision_end_to_end_pass=False,end_to_end_under20=False,end_to_end_under30=False,one_ring_component_accepted=False);print(report['error'],flush=True)
finally:
    if 'started'in globals():report['wall_verified_elapsed_s']=time.perf_counter()-started
    if 'gc_was_enabled'in globals()and gc_was_enabled:gc.enable()
    if 'corrections'in globals():report['observation_driven_corrections']=corrections
    if 'profile'in globals():report['CPU_operation_profile']=profile
    if 'pre_scattered_checkers' in globals():report['GPU_route_statistics']={side:checker.gpu_route_stats for side,checker in pre_scattered_checkers.items() if hasattr(checker,'gpu_route_stats')}
    if 'pre_scattered_checkers' in globals():report['fast_transport_statistics']={side:checker.fast_planner.stats for side,checker in pre_scattered_checkers.items() if getattr(checker,'fast_planner',None) is not None}
    if 'encoder_speed_audit'in globals():report['encoder_motor_speed_audit']=dict(encoder_speed_audit,enforced=bool(startup_config.get('enforce_motor_speed_limits',True)),velocity_cap_basis='93% of conservative position-dependent bounds with one control-step lookahead; 95% acceptance reserve')
    if 'ring_contact_debug'in globals() and ring_contact_debug:(OUT/'ring-contact-debug.json').write_text(json.dumps(ring_contact_debug,indent=2))
    if 'prepared_initial_command'in globals():np.savez(OUT/'initial-actuator-command.npz',command=prepared_initial_command[0])
    if 'commands'in globals():np.savez_compressed(OUT/'corrected-skill-field.npz',command=commands,phase=phase,indices=aidx['left']+aidx['right'])
    if 'tape'in globals()and tape:np.savez_compressed(OUT/'physx-sequence-trace.npz',camera_body_paths=camera.all_bodies.prim_paths,camera_bodies=[r['camera_bodies']for r in tape],time_s=[r['t']for r in tape],simulation_time_s=[r['sim']for r in tape],phase_time_s=[r['phase']for r in tape],pin_phase=[r.get('pin_phase','ring_skill') for r in tape],q=[r['q']for r in tape],links=[r['links']for r in tape],parts=[r['parts']for r in tape],part_names=list(truth))
    if 'actuators'in globals()and actuators:np.savez_compressed(OUT/'executed-actuator-commands.npz',time_s=[r['t']for r in actuators],command=[r['q']for r in actuators],phase_time_s=[r['phase']for r in actuators],pin_phase=[r.get('pin_phase','ring_skill') for r in actuators],physics_substeps=[r.get('physics_substeps',1) for r in actuators],actual_dof_velocity=[r.get('actual_dof_velocity',np.full(len(names),np.nan)) for r in actuators],actual_dof_position=[r.get('actual_dof_position',np.full(len(names),np.nan)) for r in actuators])
    if 'vision'in globals():
        try:vision.save(OUT)
        except Exception:report['artifact_error']=traceback.format_exc()
    (OUT/'pipeline-results.json').write_text(json.dumps(report,indent=2))
    if 'paths'in globals():(OUT/'body-paths.json').write_text(json.dumps(dict(paths,native_hands=hands),indent=2))
    if 'source'in globals():
        import shutil
        if startup_config.get('scattered_control'):
            (OUT/'arm-plan.json').write_text(json.dumps(dict(scope='Live RGB-D control from a gravity-settled random mixed 10-ring basket; see trial config and acceptance report',taught_source_metadata_not_execution=source.name,actual_motion='executed-actuator-commands.npz',free_body_poses_used_by_controller=False,hardware_tested=False),indent=2)+'\n')
        else:shutil.copyfile(source/'arm-plan.json',OUT/'arm-plan.json')
        target=OUT/'scene-authored.usda'
        if not target.exists():target.symlink_to('../'+source.name+'/scene-authored.usda')
    if 'subscription'in globals():subscription=None
    if 'sim'in globals():sim.detach_stage()
    print('REALTIME_RESULT',report.get('state'),report.get('wall_verified_elapsed_s'),flush=True)
    if not os.environ.get('M80_REALTIME_KEEPALIVE'):omni.kit.app.get_app().post_quit(0 if report.get('end_to_end_under30')else 2)
