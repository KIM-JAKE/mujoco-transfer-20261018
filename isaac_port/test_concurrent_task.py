"""Two independently clocked arms, measured contact interlocks, native PhysX audit.

Fork of test_contact_task.py for the changed simultaneous path.
"""
from pathlib import Path
import argparse,json,time,sys,traceback,math,hashlib,shutil
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--until',type=float);p.add_argument('--speed',type=float,default=4);p.add_argument('--settle',type=float,default=.5);p.add_argument('--partial-env',action='store_true');p.add_argument('--direct-gripper',action='store_true');p.add_argument('--feedback',action='store_true');p.add_argument('--precision',action='store_true');p.add_argument('--start-source',type=float,default=0);p.add_argument('--only-pin',action='store_true');p.add_argument('--posture',action='store_true');p.add_argument('--retimed',action='store_true');p.add_argument('--fast',action='store_true');p.add_argument('--control-decimation',type=int,default=4);p.add_argument('--clamps',action='store_true');p.add_argument('--cpu-threads',type=int,default=16);p.add_argument('--pin-preload',type=float,default=0);p.add_argument('--cycle-deadline',type=float);p.add_argument('--fr3-limits',action='store_true');p.add_argument('--device',default='cpu');p.add_argument('--efficient-timing',action='store_true');p.add_argument('--target-15',action='store_true');p.add_argument('--arm-damping',type=float,default=600.);p.add_argument('--right-arm-damping',type=float,default=1800.);p.add_argument('--reference-dir');p.add_argument('--plan-root');p.add_argument('--target-10',action='store_true');p.add_argument('--respect-cutoff',action='store_true');p.add_argument('--soft-clamps',action='store_true');p.add_argument('--initial-scene');p.add_argument('--physical-sensors',action='store_true',help='Exclude proven optical-axis CAD solids while preserving all sensor mounts and head/tail geometry');p.add_argument('--trial-layout',action='store_true',help='Reproduce the previous shifted-rail experiment; never a real-cell claim');a=p.parse_args()
if not(a.feedback and a.precision and a.posture and a.fast and a.clamps and a.fr3_limits and a.efficient_timing and a.physical_sensors):p.error('Concurrent test requires complete physical contact/FR3/sensor configuration')
a.target_15=True
if a.target_15 and not a.efficient_timing:p.error('--target-15 requires --efficient-timing')
if a.efficient_timing and not(a.fr3_limits and a.posture and a.fast):p.error('--efficient-timing requires --fr3-limits --posture --fast')
def arm_damping(name):return (a.right_arm_damping if name.startswith('right_')else a.arm_damping)if a.target_15 else 220
if a.respect_cutoff:
 from optimization_clock import install_cutoff_alarm
 install_cutoff_alarm()
out=a.output.resolve();out.mkdir(parents=True,exist_ok=True);started=time.monotonic()
def save(**x):
 t=out/'status.tmp';t.write_text(json.dumps(x,indent=2)+'\n');t.replace(out/'status.json')
posture_prefix='arm-posture-fr3'if a.fr3_limits else'arm-posture'
profile_prefix='fr3-concurrent-profile'
plan_root=Path(a.plan_root)if a.plan_root else Path(__file__).parent/'generated/contact_task'
recording_root=Path(a.reference_dir)if a.reference_dir else (Path(__file__).parent/'generated/contact_task/fast'if a.fast else Path(__file__).parent/'generated/contact_task/retimed'if a.retimed else Path(__file__).parent.parent/'keti_m80_mujoco_20261013/duo_fr3_2f85_20261013/cap_button_parallel_alt_20261016')
# Freeze the complete reference per run before opening any lazy NPZ reader.
frozen_recording=out/'recording';frozen_recording.mkdir(exist_ok=True)
for filename in ['trajectory.npz','independent_schedule.json','config.json','retiming.json']:
 if(recording_root/filename).exists():shutil.copy2(recording_root/filename,frozen_recording/filename)
recording_root=frozen_recording
if a.until is None:a.until=float(np.load(recording_root/'trajectory.npz')['time_s'][-1])
manifest=dict(arguments=vars(a)|{'output':str(out)},arm_drive_stiffness_nm_rad=12000,arm_drive_damping_nm_s_rad={side:arm_damping(side+'_arm_1')for side in ['left','right']},code_sha256={f:hashlib.sha256((Path(__file__).parent/f).read_bytes()).hexdigest()for f in ['test_concurrent_task.py','concurrent_profile.py','contact_scene.py','pose_feedback.py','precision_collisions.py','fixture_clamps.py','robot_collision_filters.py','button_contact_control.py','task_path_offsets.py','task_timing.py','task_timing_fast.py','stock_fixture_geometry.py','fr3_motion_control.py','fr3_motion_worker.py','fr3_reference_profile.py','prepare_fr3_timing.py','sensor_geometry.py','optimization_clock.py','source_model.py','prepare_concurrent_timing.py','prepare_shortcut_candidate.py','model_torque_screen.py','stock_scene_variation.py']},layout='trial_rail_offsets'if a.trial_layout else'original_cad',hardware_execution_certified=False,sensor_geometry='physical_head_tail'if a.physical_sensors else'full_cad_including_optical_axis',part_pose_writes='initial conditions only',part_attachments=False,physics_hz=480)
manifest['input_sha256']={f:hashlib.sha256((Path(__file__).parent/'generated/contact_task'/f).read_bytes()).hexdigest()for f in ['contact-source.json','collision.npz','precision-cad.npz','precision-pin-main.npz','robot-filters.json']}
manifest['recording_sha256']={f:hashlib.sha256((recording_root/f).read_bytes()).hexdigest()for f in ['trajectory.npz','independent_schedule.json','config.json']}
if(recording_root/'retiming.json').exists():manifest['recording_sha256']['retiming.json']=hashlib.sha256((recording_root/'retiming.json').read_bytes()).hexdigest()
if a.efficient_timing:
 for f in [profile_prefix+'.npz',profile_prefix+'.json']:manifest['input_sha256'][f]=hashlib.sha256((plan_root/f).read_bytes()).hexdigest()
if a.posture:
 for f in [posture_prefix+'.npz',posture_prefix+'.json']:manifest['input_sha256'][f]=hashlib.sha256((plan_root/f).read_bytes()).hexdigest()
if a.initial_scene:
 initial_scene=Path(a.initial_scene);shutil.copy2(initial_scene,out/'initial-scene.json');manifest['initial_scene_sha256']=hashlib.sha256(initial_scene.read_bytes()).hexdigest()
(out/'run-manifest.json').write_text(json.dumps(manifest,indent=2))
(out/'implementation').mkdir(exist_ok=True)
for filename in manifest['code_sha256']:shutil.copy2(Path(__file__).parent/filename,out/'implementation'/filename)
if a.posture:
 for filename in [posture_prefix+'.npz',posture_prefix+'.json']:shutil.copy2(plan_root/filename,out/filename)
if a.efficient_timing:
 for filename in [profile_prefix+'.npz',profile_prefix+'.json']:shutil.copy2(plan_root/filename,out/filename)
# Import task modules from the audited immutable snapshot, preserving the
# project __file__ location used to resolve the immutable assets.
import importlib.abc,importlib.util
class TaskSnapshotLoader(importlib.abc.MetaPathFinder,importlib.abc.Loader):
 def find_spec(self,fullname,path=None,target=None):
  filename=fullname+'.py'
  if filename in manifest['code_sha256']and fullname!='test_concurrent_task':return importlib.util.spec_from_loader(fullname,self,origin=str(Path(__file__).parent/filename))
 def create_module(self,spec):return None
 def exec_module(self,module):
  filename=module.__name__+'.py';module.__file__=str(Path(__file__).parent/filename)
  source=(out/'implementation'/filename).read_bytes();assert hashlib.sha256(source).hexdigest()==manifest['code_sha256'][filename]
  exec(compile(source,module.__file__,'exec'),module.__dict__)
sys.meta_path.insert(0,TaskSnapshotLoader())
sys.setswitchinterval(.00001)
save(state='running',phase='app_start')
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'multi_gpu':False,'anti_aliasing':0,'limit_cpu_threads':a.cpu_threads});code=0
try:
 import omni.usd,omni.timeline,omni.physx
 from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,PhysicsSchemaTools,Gf
 from isaacsim.core.simulation_manager import SimulationManager as SM
 from isaacsim.core.experimental.prims import Articulation,RigidPrim
 from contact_scene import build,HERE
 save(state='running',phase='build')
 source,body_paths,adapt=build(out,not a.partial_env,not a.direct_gripper)
 print('BUILT',adapt,flush=True)
 assert omni.usd.get_context().open_stage(str(out/'scene.usda'))
 stage=omni.usd.get_context().get_stage();stage.SetEditTarget(stage.GetSessionLayer())
 # Imported workpiece visuals were tagged guide and omitted by RTX.
 for name in ['ring1','ring2','pin']:
  for prim in Usd.PrimRange(stage.GetPrimAtPath(body_paths['workpiece_'+name])):
   if prim.IsA(UsdGeom.Imageable):UsdGeom.Imageable(prim).CreatePurposeAttr('default')
 if a.physical_sensors:
  if a.trial_layout:raise RuntimeError('Physical sensor correction requires original mounts')
  from sensor_geometry import apply as apply_sensors
  adapt.update(apply_sensors(stage))
 if a.precision:
  from precision_collisions import apply
  adapt.update(apply(stage,HERE,body_paths));print('PRECISION_COLLISION',adapt,flush=True)
 clamp_rows=[]
 if a.clamps:
  from fixture_clamps import apply as apply_clamps,Controller
  clamp_rows,clamp_adapt=apply_clamps(stage,body_paths,trial_layout=a.trial_layout);adapt.update(clamp_adapt);clamps=Controller(clamp_rows,soft_contact=a.soft_clamps)
  if a.target_15:adapt['clamp_close_ramp_s']={name:.08 for name in ['ring1','ring2','pin']}
  if a.soft_clamps:adapt['ring_clamp_force_control']=dict(assumed=True,initial_max_force_n=2.,final_max_force_n=5.,contact_trigger_n=.1,ramp_after_contact_s=.08,pin_unchanged=True,real_proportional_pressure_control_unverified=True)
 if a.initial_scene:
  from stock_scene_variation import apply as apply_initial_scene
  source,variation_report=apply_initial_scene(stage,source,out/'initial-scene.json');adapt['pickup_variation']=variation_report
 # Standard natural sleeping is enabled. Contact lifetimes are tracked separately
 # because sleeping islands legitimately stop emitting fresh contact impulses.
 for prim in stage.Traverse():
  if prim.HasAPI(UsdPhysics.RigidBodyAPI)and ('workpiece_'in str(prim.GetPath())or str(prim.GetPath()).startswith('/World/Clamps/')):PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSleepThresholdAttr(.00005)
 # Persist the exact task overrides separately from the immutable source.
 # Configuration is exported after all joint and contact attributes are set.
 (out/'body-paths.json').write_text(json.dumps(body_paths,indent=2))
 recording=recording_root
 tr=np.load(recording/'trajectory.npz');times=tr['time_s'];src_indices={j['name']:j['qpos_address']for j in source['joints']}
 posture=np.load(out/(posture_prefix+'.npz'))if a.posture else None
 stage.SetTimeCodesPerSecond(480);stage.SetStartTimeCode(0);stage.SetEndTimeCode(48000)
 for prim in stage.Traverse():
  if 'official_FR3_Duo'in str(prim.GetPath())and prim.HasAPI(UsdPhysics.RigidBodyAPI):PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(.1)
  if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
   PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateEnabledSelfCollisionsAttr(True)
  if '/Contact/' in str(prim.GetPath()) and '_pad' in prim.GetName() and prim.HasAPI(UsdPhysics.CollisionAPI):
   patch=PhysxSchema.PhysxCollisionAPI.Apply(prim);patch.CreateTorsionalPatchRadiusAttr(.004);patch.CreateMinTorsionalPatchRadiusAttr(.0015)
  if a.only_pin and prim.HasAPI(UsdPhysics.CollisionAPI)and ('workpiece_ring1'in str(prim.GetPath())or'workpiece_ring2'in str(prim.GetPath())or'/right_arm_base/'in str(prim.GetPath())):UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
  name=prim.GetName()
  if not prim.IsA(UsdPhysics.RevoluteJoint) or name not in src_indices:continue
  q0=float(np.interp(a.start_source,times,tr['qpos'][:,src_indices[name]]))
  if a.posture and '_arm_'in name:q0=float(posture[name.split('_')[0]+'_qpos'][0,int(name[-1])-1])
  PhysxSchema.JointStateAPI.Apply(prim,'angular').CreatePositionAttr(math.degrees(q0))
  drive=UsdPhysics.DriveAPI.Apply(prim,'angular');drive.CreateTargetPositionAttr(math.degrees(q0))
  drive.CreateStiffnessAttr(100*math.pi/180 if '_grip_' in name else 12000*math.pi/180)
  drive.CreateDampingAttr(math.pi/180 if '_grip_' in name else arm_damping(name)*math.pi/180)
  drive.CreateMaxForceAttr(2.5 if '_grip_' in name else (87 if int(name[-1])<=4 else 12))
 active_part_joints=[]
 part_paths={body_paths['workpiece_'+n]for n in ['ring1','ring2','pin']}
 for pr in stage.Traverse():
  if pr.IsA(UsdPhysics.Joint):
   joint=UsdPhysics.Joint(pr)
   if any(str(t)in part_paths for t in list(joint.GetBody0Rel().GetTargets())+list(joint.GetBody1Rel().GetTargets())):active_part_joints.append(str(pr.GetPath()))
 assert not active_part_joints,active_part_joints
 (out/'dynamics-audit.json').write_text(json.dumps(dict(engine='PhysX',diagnostic_only=bool(a.only_pin or a.partial_env or a.start_source!=0 or a.until<float(times[-1])-1e-8),disabled_collision_scope='right robot and both rings'if a.only_pin else None,dynamic_workpieces=list(part_paths),workpiece_joints=active_part_joints,gravity_m_s2=9.81,fixture_contact_enabled=True,workpiece_mass_kg={n:source['parts'][n]['mass']for n in source['parts']},object_teleportation_after_initialization=False),indent=2))
 stage.GetSessionLayer().Export(str(out/'configuration.usda'))
 SM.setup_simulation(dt=1/480,device=a.device)
 assert SM.get_active_physics_engine()=='physx'
 import carb,carb.logging
 # Preserve warnings while buffering file writes instead of flushing each
 # repeated SDF face-index warning. Physics errors still flush immediately.
 carb.settings.get_settings().set_string('/log/fileFlushLevel','error')
 carb.settings.get_settings().set_bool('/log/async',True)
 carb.logging.configure_default_logger()
 # All measurements below use PhysX tensor readback. Rendering is performed
 # from the measured tape after the experiment, so USD writeback is unnecessary.
 carb.settings.get_settings().set_bool('/physics/updateToUsd',False)
 carb.settings.get_settings().set_bool('/physics/updateVelocitiesToUsd',False)
 timeline=omni.timeline.get_timeline_interface();timeline.set_play_every_frame(True);timeline.set_target_framerate(480)
 timeline.play();app.update();timeline.pause()
 roots=[str(pr.GetPath())for pr in stage.Traverse()if pr.HasAPI(UsdPhysics.ArticulationRootAPI)]
 print('ROOTS',roots,flush=True)
 robot=Articulation(roots[0]);print('DOFS',robot.dof_names,flush=True)
 names=robot.dof_names;(out/'dof-names.json').write_text(json.dumps(names,indent=2));n=len(names);src_indices={j['name']:j['qpos_address']for j in source['joints']}
 rawq=tr['qpos'][:,[src_indices[s] for s in names]]
 if a.posture:
  assert np.allclose(posture['time_s'],times)
  for i,name in enumerate(names):
   if '_arm_'in name:rawq[:,i]=posture[name.split('_')[0]+'_qpos'][:,int(name[-1])-1]
 rawv=np.gradient(rawq,times,axis=0)/a.speed
 if a.feedback:
  from pose_feedback import PoseFeedback
  feedback=PoseFeedback(HERE,recording,names,a.speed);feedback.planned_q=rawq;feedback.motion_limited=a.fr3_limits;feedback.fast_cycle=a.target_15
  feedback.grip_targets={name:float(np.interp(window[1],times,rawq[:,names.index(window[0]+'_grip_left_driver_joint')]))for name,window in feedback.windows.items()}
  (out/'feedback-parameters.json').write_text(json.dumps(dict(clamp_gated_gripper_release=True,closed_grip_targets_rad=feedback.grip_targets,net_contact_force_limits_n=feedback.force_limits,pin_final_approach_gain=feedback.pin_insert_gain,pin_driver_preload_rad=a.pin_preload,gripper_max_effort_nm=5),indent=2))
  if a.only_pin:
   feedback.windows={n:w for n,w in feedback.windows.items()if n=='pin'};feedback.insertion={n:w for n,w in feedback.insertion.items()if n=='pin'}
 initial=np.array([np.interp(a.start_source,times,rawq[:,i])for i in range(n)])[None,:];robot.set_dof_positions(initial);robot.set_dof_velocities(np.zeros_like(initial));robot.set_dof_position_targets(initial)
 robot.set_world_poses(positions=[[-.24903096,-.6,1]],orientations=[[math.sqrt(.5),0,0,math.sqrt(.5)]])
 robot.set_velocities(linear_velocities=[0,0,0],angular_velocities=[0,0,0])
 k=[];d=[];effort=[]
 for name in names:
  if '_arm_' in name:k.append(12000);d.append(arm_damping(name));effort.append(87 if int(name[-1])<=4 else 12)
  elif a.direct_gripper:k.append(100);d.append(1);effort.append(2.5)
  elif name.endswith('left_driver_joint'):k.append(200);d.append(10);effort.append(5)
  elif name.endswith('spring_link_joint'):k.append(.05);d.append(.00125);effort.append(.5)
  else:k.append(0);d.append(.02);effort.append(0)
 robot.set_dof_gains([k],[d]);robot.set_dof_max_efforts([effort])
 limiter=None
 if a.fr3_limits:
  from concurrent_profile import ConcurrentMotionLimiter as PathMotionLimiter,ConcurrentProfile as PreparedReferenceProfile
  from fr3_reference_profile import ReferenceProfile
  profile=PreparedReferenceProfile(times,rawq,names,json.loads((recording/'independent_schedule.json').read_text()),out/(profile_prefix+'.npz'))if a.efficient_timing else ReferenceProfile(times,rawq,names,json.loads((recording/'independent_schedule.json').read_text()));(out/'reference-profile.json').write_text(json.dumps(profile.report(),indent=2)+'\n')
  limiter=PathMotionLimiter(names,initial[0],out/'implementation/fr3_motion_worker.py',profile);motion_time=[];motion_q=[];motion_v=[];command_q=[];command_v=[];command_a=[]
  if profile.metadata.get('pin_prealignment_reference') is not None:
   feedback.pin_prealignment_reference=float(profile.metadata['pin_prealignment_reference'])
  feedback.strict_clamp_alignment=bool(profile.metadata.get('strict_clamp_alignment',False))
 parts=RigidPrim([body_paths['workpiece_'+name]for name in ['ring1','ring2','pin']])
 parts.set_world_poses(positions=[source['parts'][name]['position']for name in ['ring1','ring2','pin']],orientations=[source['parts'][name]['quaternion']for name in ['ring1','ring2','pin']]);parts.set_velocities(linear_velocities=[0,0,0],angular_velocities=[0,0,0])
 if a.only_pin:
  for name in ['ring1','ring2']:PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath(body_paths['workpiece_'+name])).CreateDisableGravityAttr(True)
 button=RigidPrim('/World/Button')
 body_names=source['body_names'][1:]+['__button_plunger']+[r['path']for r in clamp_rows];body_views=RigidPrim([body_paths[n] for n in source['body_names'][1:]]+['/World/Button']+[r['path']for r in clamp_rows])
 (out/'tape-bodies.json').write_text(json.dumps(body_names,indent=2))
 clamp_forces={r['path']:0. for r in clamp_rows};pad_forces={name:{'left':0.,'right':0.}for name in ['ring1','ring2','pin']}
 active_pairs={};simulation=omni.physx.get_physx_simulation_interface();stage_id=omni.usd.get_context().get_stage_id()
 sleep_paths={name:stage.GetPrimAtPath(path).GetPath()for name,path in ({name:body_paths['workpiece_'+name]for name in ['ring1','ring2','pin']}|{r['path']:r['path']for r in clamp_rows}).items()}
 contacts={};all_contacts=[];clock=[0.];env_forces={name:0. for name in ['ring1','ring2','pin']};env_vectors={name:np.zeros(3)for name in env_forces}
 def contact(headers,points):
  for header in headers:
   aa=str(PhysicsSchemaTools.intToSdfPath(header.actor0));bb=str(PhysicsSchemaTools.intToSdfPath(header.actor1))
   pair=tuple(sorted([str(PhysicsSchemaTools.intToSdfPath(header.collider0)),str(PhysicsSchemaTools.intToSdfPath(header.collider1))]))
   if int(header.type)==1:
    active_pairs.pop(pair,None)
    continue
   if header.num_contact_data==0:continue
   if not ('workpiece_' in aa or 'workpiece_' in bb or '/World/Button' in [aa,bb] or 'official_FR3_Duo'in aa or 'official_FR3_Duo'in bb):continue
   impulses=[np.array(points[i].impulse,dtype=float)for i in range(header.contact_data_offset,header.contact_data_offset+header.num_contact_data)]
   f=sum(float(np.linalg.norm(impulse))*480 for impulse in impulses);resultant=np.sum(impulses,axis=0)*480
   if 'workpiece_'in aa or 'workpiece_'in bb:active_pairs[pair]=dict(actor0=aa,actor1=bb,last_force_n=f,last_force_time_s=clock[0])
   key=aa+' | '+bb;contacts[key]=contacts.get(key,0)+f
   for name in env_forces:
    if (aa.endswith('/workpiece_'+name) and '/Environment/m80/' in bb)or(bb.endswith('/workpiece_'+name) and '/Environment/m80/' in aa):
     env_forces[name]+=f;env_vectors[name]+=resultant if aa.endswith('/workpiece_'+name)else -resultant
   for name in pad_forces:
    if body_paths['workpiece_'+name]in [aa,bb]:
     other=bb if aa==body_paths['workpiece_'+name]else aa;side='right'if name=='ring1'else'left'
     for jaw in ['left','right']:
      if other.endswith('/'+side+'_grip_'+jaw+'_pad'):pad_forces[name][jaw]+=f
   for actor in [aa,bb]:
    if actor in clamp_forces and body_paths['workpiece_'+actor.split('/')[-2]]in [aa,bb]:clamp_forces[actor]+=f
   if f>.001:all_contacts.append(dict(time_s=clock[0],actor0=aa,actor1=bb,force_n=f,force_vector_actor0_n=resultant.tolist()))
 sub=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(contact)
 poses=[];quats=[];pose_times=[];trace=[];tape_q=[]
 save(state='running',phase='settling',adaptations=adapt)
 steps=round(((a.until-a.start_source)*a.speed+a.settle+3+24)*480) if a.feedback else round((a.until*a.speed+a.settle+.5)*480)
 if a.fr3_limits:steps=max(steps,round((a.settle+(a.cycle_deadline or 60)+3)*480))
 contact_frame=[];button_samples=[];button_travel=0.;button_pressed_steps=0;button_q_hold=None
 pin_grip_start=0.
 if a.feedback:
  pin_schedule=json.loads((recording/'independent_schedule.json').read_text());pin_grip_start=next(row['start_s']for row in pin_schedule['lanes']['left']if row['stage']['label']=='grasp_last_pin')+pin_schedule['delays']['left']
 def pin_preload(t):
  if not a.pin_preload:return 0.,0.
  end=feedback.windows['pin'][2]
  def ease(x):
   u=float(np.clip(x/.1,0,1));return u*u*(3-2*u),6*u*(1-u)/.1 if 0<x<.1 else 0.
  c,cv=ease(t-pin_grip_start);o,ov=ease(t-end)
  return a.pin_preload*c*(1-o),a.pin_preload*(cv*(1-o)-c*ov)/a.speed
 ref_clock={side:a.start_source for side in ['left','right']};held_since={side:None for side in ref_clock};done_time=None;offset=np.zeros(n);offset_v=np.zeros(n);previous_hold=None;hold=None
 from optimization_clock import require_optimization_window
 for step in range(steps):
  if a.respect_cutoff and step%a.control_decimation==0:require_optimization_window()
  physical_t=step/480;clock[0]=(step+1)/480
  if step%a.control_decimation==0:
   hold=None
   if a.feedback:
    current_p,current_q=parts.get_world_poses();current_p=current_p.numpy();current_q=current_q.numpy()
    actual=robot.get_dof_positions().numpy()[0];feedback.button_travel=button_travel;feedback.button_ready=button_pressed_steps>=49;feedback.pad_forces=pad_forces;feedback.measured_q=actual;feedback.measured_qv=robot.get_dof_velocities().numpy()[0]
    feedback.motion_following_error=limiter.following_error(actual)if limiter is not None else 0.
    motion_rate=limiter.progress_scale(actual)if limiter is not None else 1.
    motion_following=motion_rate==0
    proposed=(limiter.peek_reference(a.control_decimation/480)if physical_t>=a.settle else ref_clock)if limiter is not None else min(a.until,ref_clock+(motion_rate*a.control_decimation/(480*a.speed) if physical_t>=a.settle else 0))
    if motion_following:proposed=ref_clock
    if a.clamps:
     clamps.update(physical_t,ref_clock,feedback,current_p,current_q,clamp_forces)
     feedback.clamped=clamps.verified
    control_forces={name:float(np.linalg.norm(vector))for name,vector in env_vectors.items()}
    ref_t,hold=feedback.gate(ref_clock,proposed,current_p,current_q,control_forces,clamps.verified if a.clamps else None)
    ref_t,hold=profile.coordinate(ref_clock,ref_t,hold,actual,feedback.measured_qv,physical_t)
    for side in ['left','right']:
     if hold[side]:
      if held_since[side] is None:held_since[side]=physical_t
      if physical_t-held_since[side]>12:raise RuntimeError('Insertion gate timeout: '+str(hold))
     else:held_since[side]=None
    ref_clock=ref_t
    if motion_following and hold is None:hold='fr3_motion_following'
    if limiter is not None:limiter.set_reference(ref_t,hold,physical_t>=a.settle)
    if limiter is not None and limiter.following_error(actual)>.020:offset=feedback.offset.copy();offset_v=np.zeros(n)
    else:offset,offset_v=feedback.update(ref_t,actual,current_p,current_q,control_forces,dt=a.control_decimation/480)
    if hold!=previous_hold:print('INSERTION_GATE',physical_t,ref_t,hold,flush=True);previous_hold=hold
   else:ref_t=float(np.clip((physical_t-a.settle)/a.speed,0,a.until))
   q=np.array([np.interp(ref_t[names[i].split('_')[0]],times,rawq[:,i]) for i in range(n)])[None,:]
   if not a.direct_gripper:
    for i,name in enumerate(names):
     if name.endswith('spring_link_joint'):q[0,i]=2.62
   q+=offset[None,:]
   held_grips=[]
   if a.feedback and a.clamps:
    for part,(side,begin,end,release_end)in feedback.windows.items():
     if begin<=ref_t[side]<release_end and not clamps.verified[part]:
      j=names.index(side+'_grip_left_driver_joint');q[0,j]=max(q[0,j],feedback.grip_targets[part]);held_grips.append(j)
   preload,preload_velocity=pin_preload(ref_t['left']);q[0,names.index('left_grip_left_driver_joint')]+=preload
   button_indices=[names.index('right_arm_fr3v2_joint'+str(j))for j in range(1,8)]
   if a.feedback and feedback.button_hold:
    if button_q_hold is None:button_q_hold=q[0,button_indices].copy()
    q[0,button_indices]=button_q_hold
   else:button_q_hold=None
   if limiter is not None:
    gentle_effort=effort.copy()
    for part in ['ring1','ring2']:
     if part not in feedback.windows:continue
     side,begin,_,_=feedback.windows[part]
     if ref_t[side]<begin:gentle_effort[names.index(side+'_grip_left_driver_joint')]=.75
    robot.set_dof_max_efforts([gentle_effort])
   if limiter is None:robot.set_dof_position_targets(q)
   qv=profile.full_velocity()[None,:]+offset_v[None,:]
   for j in held_grips:qv[0,j]=0.
   if not a.direct_gripper:
    for i,name in enumerate(names):
     if name.endswith('spring_link_joint'):qv[0,i]=0
   if physical_t>=a.settle and not hold['left']:
    qv[0,names.index('left_grip_left_driver_joint')]+=preload_velocity*a.speed*profile.last_rate['left']
   if button_q_hold is not None:qv[0,button_indices]=0.
   if limiter is None:robot.set_dof_velocity_targets(qv)
   else:limiter.set_target(q[0],qv[0])
  if limiter is not None:
   cq,cv,ca=limiter.step();limited_q=q.copy();limited_v=qv.copy();limited_q[0,limiter.indices]=cq;limited_v[0,limiter.indices]=cv
   # Gripper motors use the same smooth 480 Hz phase as the arms. A 120 Hz
   # position staircase at first contact transmitted a measurable wrist impulse.
   fine_q=profile.full_path(profile.last_reference);fine_v=profile.full_velocity()
   fine_preload,fine_preload_v=pin_preload(profile.last_reference['left'])
   for driver in profile.driver_indices:
    angle=float(fine_q[driver]);velocity=float(fine_v[driver])
    if names[driver].startswith('left_'):angle+=fine_preload;velocity+=fine_preload_v*a.speed*profile.last_rate['left']
    if driver in held_grips and angle<q[0,driver]:angle=float(q[0,driver]);velocity=0.
    limited_q[0,driver]=angle;limited_v[0,driver]=velocity
   robot.set_dof_position_targets(limited_q);robot.set_dof_velocity_targets(limited_v)
  for name in env_forces:env_forces[name]=0.;env_vectors[name][:]=0.
  for name in clamp_forces:clamp_forces[name]=0.
  for forces in pad_forces.values():
   for jaw in forces:forces[jaw]=0.
  SM.step()
  recorded_t=(step+1)/480;btn=button.get_world_poses()[0].numpy()[0];button_travel=float(btn[1]+.24775);button_pressed_steps=button_pressed_steps+1 if button_travel>=.0018 else 0;button_samples.append(dict(t=recorded_t,travel_m=button_travel))
  if limiter is not None:
   motion_time.append(recorded_t);motion_q.append(robot.get_dof_positions().numpy()[0,limiter.indices].copy());motion_v.append(robot.get_dof_velocities().numpy()[0,limiter.indices].copy());command_q.append(cq.copy());command_v.append(cv.copy());command_a.append(ca.copy())
   if len(motion_v)>=3:
    actual_acceleration=(motion_v[-1].astype(float)-motion_v[-2].astype(float))*480
    actual_jerk=(motion_v[-1].astype(float)-2*motion_v[-2].astype(float)+motion_v[-3].astype(float))*480**2
    if np.max(abs(actual_acceleration))>10.0001 or np.max(abs(actual_jerk))>5000.01:raise RuntimeError('Measured FR3 kinematic limit exceeded at '+str(recorded_t)+'s: acceleration='+str(float(np.max(abs(actual_acceleration))))+', jerk='+str(float(np.max(abs(actual_jerk)))))
  if step%16==0 or step==steps-1:
   measured=robot.get_dof_positions().numpy()[0];pos,quat=parts.get_world_poses();pos=pos.numpy();quat=quat.numpy()
   velocity,angular_velocity=parts.get_velocities()
   bp,bq=body_views.get_world_poses();poses.append(bp.numpy().copy());quats.append(bq.numpy().copy());pose_times.append(recorded_t);tape_q.append(measured.copy())
   trace.append(dict(t=recorded_t,source_t=min(ref_t.values()),source_times=dict(ref_t),part_positions=pos.tolist(),part_quaternions=quat.tolist(),part_velocities=velocity.numpy().tolist(),sleeping={name:bool(simulation.is_sleeping(stage_id,PhysicsSchemaTools.sdfPathToInt(path)))for name,path in sleep_paths.items()},active_clamp_contacts=[dict(row,colliders=list(pair))for pair,row in active_pairs.items()if any(actor.startswith('/World/Clamps/')for actor in [row['actor0'],row['actor1']])],button_travel_m=button_travel,button_controller_hold=feedback.button_hold if a.feedback else False,insertion_gate=hold,fixture_forces_n=dict(env_forces),fixture_net_forces_n={name:float(np.linalg.norm(vector))for name,vector in env_vectors.items()},fixture_contact_force_vectors_n={name:vector.tolist()for name,vector in env_vectors.items()},clamp_forces_n=dict(clamp_forces),pad_forces_n={name:dict(forces)for name,forces in pad_forces.items()},clamp_verified=dict(clamps.verified)if a.clamps else {},arm_offset_rad=offset.tolist(),arm_tracking_error_rad=float(max(abs(measured[i]-q[0,i])for i,name in enumerate(names)if '_arm_' in name))))
   if step%240==0:
    print('STEP',physical_t,'ref',ref_t,'pos',pos.tolist(),'arm_error',trace[-1]['arm_tracking_error_rad'],'button',trace[-1]['button_travel_m'],flush=True)
    save(state='running',phase='physics',step=step,steps=steps,time_s=physical_t,source_t=ref_t,last=trace[-1])
   assert np.isfinite(pos).all() and np.isfinite(measured).all()
   if np.any(pos[:,2]<.5):raise RuntimeError('Workpiece dropped below 0.5 m')
  complete=all(ref_t[side]>=profile.arms[side].bounds[-1]-1e-8 for side in ['left','right'])
  if a.cycle_deadline is not None and physical_t>a.settle+a.cycle_deadline and not complete:raise RuntimeError('Physical cycle deadline exceeded before completing the motion')
  if a.feedback and complete:
   if done_time is None:done_time=physical_t
   if physical_t-done_time>=1.5:break
 if a.feedback and not complete:raise RuntimeError('Simulation step budget exhausted before completing reference')
 if limiter is not None:np.savez_compressed(out/'joint-motion.npz',time_s=motion_time,actual_position=motion_q,actual_velocity=motion_v,command_position=command_q,command_velocity=command_v,command_acceleration=command_a,dof_indices=limiter.indices)
 np.savez_compressed(out/'physics-tape.npz',time_s=pose_times,body_positions=poses,body_quaternions=quats,joint_positions=tape_q)
 (out/'button-trace.json').write_text(json.dumps(button_samples,indent=2)+'\n')
 (out/'trace.json').write_text(json.dumps(trace,indent=2)+'\n');(out/'contacts.json').write_text(json.dumps(all_contacts,indent=2)+'\n')
 (out/'tape-bodies.json').write_text(json.dumps(body_names,indent=2)+'\n')
 if a.clamps:(out/'clamp-events.json').write_text(json.dumps(clamps.events,indent=2))
 if 'profile'in globals():(out/'handoff-events.json').write_text(json.dumps(profile.handoff_events,indent=2)+'\n')
 save(state='completed_probe',seconds=time.monotonic()-started,adaptations=adapt,final=trace[-1],contact_pairs=list(contacts),physical_assembly_pass=False)
except Exception as exc:
 traceback.print_exc();save(state='failed',error=repr(exc),seconds=time.monotonic()-started);code=1
 try:
  if 'limiter'in globals()and limiter is not None and motion_time:np.savez_compressed(out/'joint-motion.npz',time_s=motion_time,actual_position=motion_q,actual_velocity=motion_v,command_position=command_q,command_velocity=command_v,command_acceleration=command_a,dof_indices=limiter.indices)
  if 'clamps' in globals():(out/'clamp-events.json').write_text(json.dumps(clamps.events,indent=2))
  if 'profile'in globals():(out/'handoff-events.json').write_text(json.dumps(profile.handoff_events,indent=2)+'\n')
  if 'button_samples'in globals():(out/'button-trace.json').write_text(json.dumps(button_samples,indent=2)+'\n')
  if 'trace' in globals():(out/'trace.json').write_text(json.dumps(trace,indent=2)+'\n')
  if 'all_contacts' in globals():(out/'contacts.json').write_text(json.dumps(all_contacts,indent=2)+'\n')
  if 'poses' in globals():np.savez_compressed(out/'physics-tape.npz',time_s=pose_times,body_positions=poses,body_quaternions=quats,joint_positions=tape_q)
 except Exception:pass
finally:
 if 'limiter'in globals()and limiter is not None:limiter.close()
 app.close(wait_for_replicator=False,exit_code=code)
sys.exit(code)
