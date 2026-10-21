"""Actual recorded SIM pixels + actual SIM encoder geometry; MOCK SDK only."""
import sys,json,copy,tempfile,hashlib,time,os
from pathlib import Path
from datetime import datetime,timezone
import numpy as np
from scipy.spatial.transform import Rotation
from field_demo import random_planner as planner,model_preset,hand_aperture
from field_demo.__main__ import init_random
from field_demo.random_cycle import LiveRuntime,select_observation
from field_demo.residual_occupancy import build_occupancy
R=Path('/home/gpuadmin/workspace/jake/field-demo-random-convex-world-20261022');REPO=R.parent/'mujoco';RUN=REPO/'isaac_port/runs/native-roas-table-scatter-stable-v278-self-clear-park-camera-446';H=REPO/'isaac_port/native_hand_layout/roas-table'
record=dict(scope='OFFLINE component: genuine recorded SIM RGB-D, SDK-shaped simulation robot states, NO actual SDK/hardware source or field installation evidence',hardware_tested=False,field_ready=False,robot_command_sent=False,uses_free_object_poses=False,events=[])
models=json.loads((RUN/'robot-kinematics.json').read_text());frames=np.asarray(models['frames']);axes=np.asarray(models['axes']);tip=np.asarray(models['tip']);trace=np.load(RUN/'physx-sequence-trace.npz');commands=np.load(RUN/'executed-actuator-commands.npz');paths=list(trace['camera_body_paths']);T=np.eye(4)
def transform(row):
 t=np.eye(4);t[:3,:3]=Rotation.from_quat(row[3:]).as_matrix();t[:3,3]=row[:3];return t
def proper(t):
 t=np.array(t,float);u,_,v=np.linalg.svd(t[:3,:3]);t[:3,:3]=u@v;return t
frames=np.array([[proper(a),proper(b)]for a,b in frames]);tip=proper(tip)
bases={s:proper(transform(trace['camera_bodies'][0,next(i for i,p in enumerate(paths)if p.endswith('/'+s+'_arm_fr3v2_link0'))]))for s in ['left','right']}
ids={'right':[1,3,5,7,9,11,13],'left':[0,2,4,6,8,10,12]};fi={'right':[16,17],'left':[14,15]}
F=np.eye(4);F[2,3]=-.107;F=F@tip

def fk(q):
 t=np.eye(4);links=[t.copy()]
 for v,ax,(a,b)in zip(q,axes,frames):
  e=np.eye(4);e[:3,:3]=Rotation.from_rotvec(ax*v).as_matrix();t=t@a@e@b;links.append(t.copy())
 tcp=t@tip;points=np.array([t[:3,3]for t in links]+[tcp[:3,3]])
 return dict(tcp_base=tcp.tolist(),link_frames_base=np.asarray(links).tolist(),link_points_base_m=points.tolist())

i0=int(np.argmin(abs(commands['time_s']-14.82)));i1=int(np.argmin(abs(commands['time_s']-14.84)));q0=commands['actual_dof_position'][i0,ids['right']].astype(float);q1=commands['actual_dof_position'][i1,ids['right']].astype(float);leftq=commands['actual_dof_position'][i1,ids['left']].astype(float)
record['native_finger_index_source']='Archived278 pipeline-results.dof_names: groupedLEFT14,15 /RIGHT16,17; old offline fixture interleaved-index mapping incorrect, productionSDKnamedHandstateunaffected'
record['robot_geometry_source']='Archived SIM robot FK/body-base calibration only, not free-body stock transforms'
record['actual_interval_s']=[float(commands['time_s'][i0]),float(commands['time_s'][i1])]
record['archived_joint_delta_rad']=float(np.max(abs(q1-q0)))
with tempfile.TemporaryDirectory(prefix='OFFLINE-recorded-closure-')as td:
 root=Path(td);site=root/'site';init_random(site)
 now=datetime.now(timezone.utc);evidence=root/'MOCK-evidence.txt';evidence.write_text('OFFLINE ONLY: no actual installation, SDK, wrench, STOP test, or hardware. All test approval records below are synthetic protocol fixtures.')
 inst=model_preset.template();inst.update(verified=False,by='OFFLINE MOCK installation contract, not field evidence',at_utc=now.isoformat(),evidence_file=str(evidence));inst['facts']=dict(inst['expected'],no_unmodeled_adapters=True,cable_radial_allowance_m=.002);ip=root/'MOCK-installation.json';ip.write_text(json.dumps(inst))
 def state(q):return dict(q=q.tolist(),q_d=q.tolist(),dq=[0.]*7,dq_d=[0.]*7,F_T_EE_column_major=F.ravel(order='F').tolist(),m_ee_kg=.73,F_x_Cee_m=[0,0,.04],I_ee_column_major_kg_m2=(np.eye(3)*.001).ravel().tolist(),m_load_kg=0.,F_x_Cload_m=[0.]*3,I_load_column_major_kg_m2=[0.]*9)
 snapshots=[]
 for s,q in [('left',leftq),('right',q0)]:
  path=root/(s+'-MOCK-sdk.json');path.write_text(json.dumps(dict(read_only=True,server_protocol_version=10,state=state(q))));snapshots.append(path)
 # CONDITIONAL NUMERICAL MODEL ONLY: no installed-model approval is invented.
 import field_demo.model_records as model_records
 def assumed_installation(path,**kwargs):return inst,evidence
 model_preset.installation=assumed_installation;model_records.installation=assumed_installation
 model_preset.author(site,ip,*snapshots)
 config=json.loads((site/'random-motion.json').read_text());meas=dict(verified=False,by='OFFLINE fixture ONLY, not field measured',at_utc=now.isoformat(),evidence_file=str(evidence))
 config.update(joint_tracking_error_limit_rad=.02,target_position_tolerance_m=.001,target_angle_tolerance_rad=.01,max_observation_age_s=2.,other_arm_q_tolerance_rad=.001)
 config['scene'].update(require_basket_observation=True,require_residual_occupancy=True,workspace=dict(T_cell_box=np.eye(4).tolist(),half_extents_m=[3]*3),obstacles=[dict(name='OFFLINE far placeholder; full machine NOT certified',T_cell_box=np.diag([1.,1.,1.,1.]).tolist(),half_extents_m=[.001]*3)])
 config['scene']['obstacles'][0]['T_cell_box'][2][3]=-2.
 for p in config['phases'].values():p.update(measurement=meas,corridors=[config['scene']['workspace']],tcp_corridor_radius_m=.01,force_monitor=dict(measurement=meas,enabled=True,force_limit_n=5,moment_limit_nm=1))
 raw=json.loads(Path('/home/gpuadmin/workspace/jake/mujoco/.local/field-floor587/fresh-observation.json').read_text());packet=dict(np.load('/home/gpuadmin/workspace/jake/mujoco/.local/field-floor587/fresh-overhead-rgbd.npz'));meta=raw['source_metadata'];meta.update(captured_at_utc=datetime.now(timezone.utc).isoformat(),cell_frame='recorded_v280_SIM_cell')
 bounds=json.loads((H/'bin-picking/asset-audit.json').read_text())['baskets'][0]['outer_bounds_world_m']
 raw.update(source_metadata=meta,captured_at_utc=meta['captured_at_utc'],residual_occupancy_metadata=meta)
 # Two authentic ray images from EXACT SAME archived render snapshot.
 clock=datetime.now(timezone.utc);meta['captured_at_utc']=clock.isoformat();raw['captured_at_utc']=clock.isoformat()
 global_path=R/'replay/recorded-closure-v280/global-rgbd.npz';global_packet=dict(np.load(global_path));global_meta=copy.deepcopy(meta);global_meta.update(capture_id='v280-same-snapshot-global',source_packet_sha256=hashlib.sha256(global_path.read_bytes()).hexdigest(),calibration_sha256=hashlib.sha256(np.asarray(global_packet['T_cell_camera']).tobytes()+np.asarray(global_packet['K']).tobytes()).hexdigest())
 original_build=build_occupancy
 global_prior=None
 overhead_only=original_build(packet,meta,bounds,voxel_size_m=.004,depth_margin_m=.003,max_age_s=2.,now_utc=clock,max_voxels=1000000)
 def fused_build(pkt,metadata,roi,**kw):
  kw.update(prior=global_prior,now_utc=clock)
  return original_build(pkt,metadata,roi,**kw)
 build_occupancy=fused_build
 import field_demo.residual_occupancy as occupancy_module
 occupancy_module.build_occupancy=fused_build
 started=time.perf_counter();snapshot=build_occupancy(packet,meta,bounds,voxel_size_m=.004,depth_margin_m=.003,max_age_s=2.,max_voxels=1000000);raw['residual_occupancy']=snapshot.to_record();record['occupancy_build_s']=time.perf_counter()-started
 selection=max([r for r in raw['instances']if r['part']=='ring1'],key=lambda r:r['center_m'][2]);raw=select_observation(raw,selection)
 runtime=LiveRuntime.__new__(LiveRuntime);runtime.recipe={'max_observation_age_s':2.};camera=dict(T_cell_camera=packet['T_cell_camera'].tolist(),residual_occupancy=dict(voxel_size_m=.004,depth_margin_m=.003),basket=dict(outer_bounds_cell_m=bounds));runtime.camera_configs={str((R/'replay/recorded-closure-v280/overhead-same-snapshot-camera.json').resolve()):camera};raw['_camera_config_path']=str(R/'replay/recorded-closure-v280/overhead-same-snapshot-camera.json')
 raw=runtime._selected_occupancy(raw)
 from field_demo.residual_occupancy import OccupancySnapshot
 final_snapshot=OccupancySnapshot.from_record(raw['residual_occupancy'],now_utc=clock)
 changed=(overhead_only.states==0)&(final_snapshot.states==1)
 old_occupied=overhead_only.states==2
 spacing=(np.asarray(bounds)[1]-np.asarray(bounds)[0])/np.asarray(final_snapshot.shape)
 unknown_indices=np.argwhere(final_snapshot.states==0);unknown_centers=np.asarray(bounds)[0]+(unknown_indices+.5)*spacing
 np.savez_compressed(Path('/tmp/field-correct-width596/unknown-voxels.npz'),unknown_centers_cell_m=unknown_centers,unknown_indices=unknown_indices,states=final_snapshot.states,bounds_cell_m=bounds,spacing_m=spacing)
 record['single_new_virtual_camera_occupancy']=dict(virtual_camera_count=1,physical_camera_proof=False,same_private_render_snapshot=True,scene_snapshot_time_s=None,overhead_target_source='sealed565 production pixel detector on ONE new overhead render of actual parked v278 scene',source_captures=final_snapshot.sources,source_age_policy_s=2.,lease_origin_utc=min(x['captured_at_utc']for x in final_snapshot.sources),source_timestamp_scope='Frozen OFFLINE replay clock, not actual fresh SDK exposure or live scene lease',overhead_unknown=int((overhead_only.states==0).sum()),global_unknown=None,fused_unknown=int((final_snapshot.states==0).sum()),genuinely_ray_cleared_unknown=int(changed.sum()),all_original_occupied_preserved=bool(np.all(final_snapshot.states[old_occupied]==2)),fused_obstacle_count=len(final_snapshot.planner_boxes()))
 raw['scope']='basket';record['selected_from_actual_pixels']=selection;record['occupancy_obstacles']=len(snapshot.planner_boxes())
 duration=.1;times=np.arange(101)*.001;delta=q1-q0;rows=[]
 for t in times:
  u=t/duration;blend=10*u**3-15*u**4+6*u**5;dq=(30*u**2-60*u**3+30*u**4)/duration*delta;q=q0+blend*delta
  rows.append(dict(time_s=float(t),q=q.tolist(),dq=dq.tolist(),**fk(q)))
 points=np.array([r['link_points_base_m']for r in rows]);lengths=np.linalg.norm(np.diff(points,axis=1),axis=-1).max(0)+1e-6;vel=1.875*abs(delta)/duration;speeds=np.zeros(9)
 for k in range(2,9):
  for j in range(7):
   if j+1<k:speeds[k]+=sum(lengths[j+1:k])*vel[j]
 native=dict(read_only=True,motion_executed=False,server_protocol_version=10,continuous_10percent_checked=True,q_start=q0.tolist(),state=state(q0),duration_s=duration,knots=[dict(time_s=0.,q=q0.tolist(),dq=[0.]*7,ddq=[0.]*7),dict(time_s=duration,q=q1.tolist(),dq=[0.]*7,ddq=[0.]*7)],samples=rows,joint_velocity_bound_rad_s=vel.tolist(),link_segment_lengths_m=lengths.tolist(),link_point_speed_bounds_m_s=speeds.tolist())
 other=dict(read_only=True,motion_executed=False,server_protocol_version=10,state=state(leftq),**fk(leftq))
 site_config=dict(robots={s:dict(ip='192.0.2.'+str(1+i),T_cell_base=bases[s].tolist())for i,s in enumerate(['left','right'])})
 now=clock;hs={s:dict(completed=True,width_m=float(commands['actual_dof_position'][i1,fi[s]].sum()),max_width_m=.08,is_grasped=False,robot_time_s=1.,captured_at_utc=now.isoformat())for s in ['left','right']};aperture=hand_aperture.binding(hs,now=now)
 # Neither lower-bound commissioning nor installation is approved here.
 config['joint_tracking_error_limit_rad']=float(os.environ['DIAG_TRACKING_RAD'])
 from field_demo import convex_self,common_duo_mount
 official=common_duo_mount.model();relative=np.linalg.inv(bases['left'])@bases['right']
 if not np.allclose(relative,official['T_left_base_right_base'],atol=1e-5):raise RuntimeError('SIM fixed common-mount model mismatch')
 def conditional_binding(*args,**kwargs):
  return common_duo_mount.CommonDuoBinding('NONEXEC_MODEL_ASSUMPTION_NOT_ACTUAL_PROOF','NONEXEC_NO_INSTALLATION_APPROVAL')
 common_duo_mount.binding=conditional_binding
 original_measured=planner.measured
 def conditional_measured(record,*args,**kwargs):
  if isinstance(record,dict)and record.get('verified')is False and record.get('kind')!='model_derived':return
  return original_measured(record,*args,**kwargs)
 planner.measured=conditional_measured
 config['convex_self_narrowphase']=dict(model_sha256=convex_self.MODEL_SHA256)
 config['convex_world_narrowphase']=dict(model_sha256=convex_self.MODEL_SHA256)
 record['conditional_model_assumptions']=dict(official_common_mount=True,installed_model_assumed=True,installation_verified=False,commissioning_profile_activated=False,proof_validation_exercised=False,geometry_and_shape_hash_checks_unchanged=True,description='Numeric sensitivity only. Installation proof precondition is explicitly ASSUMED in this isolated harness; no verified site or mount records are written.')
 record['native_fixture']=dict(mocked_SDK=True,actual_SDK_connection=False,curve_duration_s=duration,samples=len(rows),q0=q0.tolist(),q1=q1.tolist(),whole_arm_model_columns={s:len(config['robots'][s]['link_capsules_link_m'])for s in ['left','right']},gripper_widths={s:hs[s]['width_m']for s in hs},tracking_error_bound_rad=config['joint_tracking_error_limit_rad'])

 from field_demo.ik_branches import LOWER,UPPER
 from field_demo import random_cycle as policy
 from field_demo.random_cycle import offset
 start_i=int(np.argmin(abs(commands['time_s']-7.020404767)))
 startq=commands['actual_dof_position'][start_i,ids['right']].astype(float)
 # Frozen offline comparison clock allows repeated safety admission of this
 # immutable archived exposure. It is NEVER represented as live freshness.
 class FrozenDatetime:
  @staticmethod
  def now(tz=None):return clock
 policy.datetime=FrozenDatetime
 record['new_overhead_render']=json.loads(Path('/home/gpuadmin/workspace/jake/mujoco/.local/field-floor587/matched-floor-render.json').read_text())
 first_rejection_context={}
 original_collision_check=planner.collision.check_envelopes
 def capture_collision(active,others,scene,phase,envelope,**kwargs):
  try:return original_collision_check(active,others,scene,phase,envelope,**kwargs)
  except Exception as exc:
   message=str(exc)
   if not first_rejection_context:
    first_rejection_context.update(reason=message,margin_m=kwargs['margin'],world_hull_records={side:ctx.record()for side,ctx in envelope.get(convex_self.WORLD_KEY,{}).items()})
    if ' intersects measured obstacle 'in message:
     name,obname=message.split(' intersects measured obstacle ',1);cap=next(x for x in active if x['name']==name);box=next(x for x in scene['obstacles']if x['name']==obname)
     first_rejection_context.update(capsule={k:(v.tolist()if isinstance(v,np.ndarray)else v)for k,v in cap.items()},obstacle=box)
   raise
 planner.collision.check_envelopes=capture_collision
 class OfflineRuntime:
  replay=False
  recipe=dict(pickup_search_max_plans=24,pickup_search_max_captures=1,
    pickup_ik_seed_offsets_rad=[[0.]*7,[0.,0.,.8,0.,0.,0.,0.],[0.,0.,-.8,0.,0.,0.,0.]],
    target_position_tolerance_m=.002,target_angle_tolerance_rad=.03,max_observation_age_s=2.)
  def sdk_state(self,side):return dict(state=state(startq))
  def pickup_contact(self,part):return 'strict_empty_approach'
  def event(self,*args,**kwargs):record['events'].append(dict(event=args[0],**kwargs))
  def plan(self,phase,part,side,targets,observed,carried,**kwargs):
   if len(campaign)>=24:raise RuntimeError('Offline bounded diagnostic limit')
   seed=np.asarray(kwargs.get('ik_seed',startq));pre,goal=map(np.asarray,targets)
   row=dict(instance_id=observed['selected_instance']['instance_id'],part=part,
      T_cell_tcp=goal.tolist(),seed=seed.tolist())
   preq=solve(base_inv@pre,seed);q=None if preq is None else solve(base_inv@goal,preq)
   if q is None:
    row.update(accepted=False,stage='current_native_DLS_IK',reason='No bounded measured-model IK branch');campaign.append(row);raise RuntimeError(row['reason'])
   endpoint=dict(read_only=True,motion_executed=False,server_protocol_version=10,state=state(q),**fk(q))
   try:
    observed=runtime._selected_occupancy(observed)
    details,_=planner.geometry_check(endpoint,other,config,site_config,'pickup','right',part,observed,[],site,{},clock,'strict_empty_approach',aperture_binding=aperture,stopped_aperture=True)
    row.update(geometry=details,accepted=True,stage='hypothetical_endpoint_full_model_guard',q_goal=q.tolist(),q_pre=preq.tolist());campaign.append(row)
    return row
   except Exception as error:
    row.update(accepted=False,stage='hypothetical_endpoint_full_model_guard',reason=str(error));campaign.append(row);raise
 def solve(target,seed):
  q=np.asarray(seed).copy()
  for iteration in range(700):
   t=np.eye(4);origins=[];world_axes=[]
   for v,axis,(aa,bb)in zip(q,axes,frames):
    joint=t@aa;origins.append(joint[:3,3]);world_axes.append(joint[:3,:3]@axis)
    rr=np.eye(4);rr[:3,:3]=Rotation.from_rotvec(axis*v).as_matrix();t=joint@rr@bb
   t=t@tip;angular=Rotation.from_matrix(target[:3,:3]@t[:3,:3].T).as_rotvec();err=np.r_[target[:3,3]-t[:3,3],angular]
   if np.linalg.norm(err[:3])<.00005 and np.linalg.norm(angular)<.0005:return q
   ax=np.asarray(world_axes);J=np.vstack([np.cross(ax,t[:3,3]-np.asarray(origins)).T,ax.T])
   dq=J.T@np.linalg.solve(J@J.T+np.eye(6)*.00005,err);dq*=min(1.,.025/max(1e-9,np.max(abs(dq))));q=np.clip(q+dq,LOWER+.001,UPPER-.001)
  return None
 base_inv=np.linalg.inv(bases['right']);cycle=json.loads((site/'random-cycle.json').read_text());entry=next(e for e in cycle['parts']if e['part']=='ring1')
 source_run=REPO/'isaac_port/runs/native-roas-table-scatter-stable-v348-gpu-carry-no-greedy-final-retreat-all3-446'
 source_report=json.loads((source_run/'pipeline-results.json').read_text());picked=next(v['pickup']for v in source_report['scattered_pick']['parts']if v['part']=='ring1')
 source_G=np.asarray(picked['measured_grasp']);source_rotation=source_G[:3,:3].copy();u,_,v=np.linalg.svd(source_rotation);source_G[:3,:3]=u@v
 mount=np.eye(4);angle=-np.pi/4;mount[:2,:2]=[[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]
 source_tcp_hand=np.linalg.inv(F)@mount;T_hand_part=np.linalg.inv(source_tcp_hand)@source_G;T_part_hand=np.linalg.inv(T_hand_part)
 actual_tcp_hand=np.asarray(config['robots']['right']['native_hand_aperture_geometry']['T_tcp_hand']);T_part_tcp=T_part_hand@np.linalg.inv(actual_tcp_hand)
 family=copy.deepcopy(entry['grasp_families'][0]);family.update(name='SIM348_measured_stable_large_oblique_class_prior',T_part_tcp=T_part_tcp.tolist(),pregrasp_offset_part_m=(-T_part_hand[:3,2]*.06).tolist())
 entry['grasp_families']=[family]
 record['oblique_class_prior']=dict(source_run=str(source_run),source_report_sha256=hashlib.sha256((source_run/'pipeline-results.json').read_bytes()).hexdigest(),part='ring1',source='Measured postlift RGB-D + robot FK relative grasp; class-only Hand-relative prior',source_measured_G=source_G.tolist(),source_rotation_polar_adjustment=float(np.max(abs(source_rotation-source_G[:3,:3]))),T_hand_part=T_hand_part.tolist(),T_part_hand=T_part_hand.tolist(),configured_T_tcp_hand=actual_tcp_hand.tolist(),current_target_T_part_tcp=T_part_tcp.tolist(),source_native_contact=next(e for e in picked['events']if e['event']=='outer_lifted_fresh_opposed_contact'),source_lift_retention=next(e for e in picked['events']if e['event']=='large_outer_lift_grasp_retention'),source_world_object_pose_used=False,source_robot_q_replayed=False,hardware_grasp_verified=False,prior_only_new_contact_required=True)
 campaign=[];started=time.perf_counter();error=None;selected=None
 try:selected=policy.screen_pickup(OfflineRuntime(),entry,raw,[])
 except Exception as exc:error=str(exc)
 record.update(scope='CONDITIONAL NONEXEC sealed587 screen_pickup -> authentic saved SIM pixel-derived instances -> authentic saved SIM FK -> original field whole-model ENDPOINT geometry with installation precondition explicitly assumed; no proof approval. Endpoint-only preflight, not full curve or SDK execution.',
  source_package=str(R),first_rejection_context=first_rejection_context,campaign=campaign,screen_error=error,screen_selected=selected[2]if selected else None,
  reached_distinct_instances=list(dict.fromkeys(x['instance_id']for x in campaign)),
  branch_elapsed_s=time.perf_counter()-started,offline_frozen_clock=True,
  actual_sdk_connected=False,full_path_checked=False,approach_executed=False,closure_executed=False,
  tracking_bound_rad=config['joint_tracking_error_limit_rad'],unknown_voxels=int((final_snapshot.states==0).sum()))
Path('/tmp/field-correct-width596/original-'+os.environ['DIAG_TRACKING_RAD']+'.json').write_text(json.dumps(record,indent=2))
print(json.dumps(dict(plans=len(campaign),reached=record['reached_distinct_instances'],accepted=selected is not None,seconds=record['branch_elapsed_s'],reasons=list(dict.fromkeys(x.get('reason')for x in campaign))),indent=2))
