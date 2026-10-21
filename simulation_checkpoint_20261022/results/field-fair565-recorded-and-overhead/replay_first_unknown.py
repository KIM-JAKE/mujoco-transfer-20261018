"""Actual recorded SIM pixels + actual SIM encoder geometry; MOCK SDK only."""
import sys,json,copy,tempfile,hashlib,time
from pathlib import Path
from datetime import datetime,timezone
import numpy as np
from scipy.spatial.transform import Rotation
from field_demo import random_planner as planner,model_preset,hand_aperture
from field_demo.__main__ import init_random
from field_demo.random_cycle import LiveRuntime,select_observation
from field_demo.residual_occupancy import build_occupancy
R=Path('/home/gpuadmin/workspace/jake/field-demo-random-fair-search-20261022');REPO=R.parent/'mujoco';RUN=REPO/'isaac_port/runs/native-roas-table-scatter-stable-v278-self-clear-park-camera-446';H=REPO/'isaac_port/native_hand_layout/roas-table'
record=dict(scope='OFFLINE component: genuine recorded SIM RGB-D, SDK-shaped simulation robot states, NO actual SDK/hardware source or field installation evidence',hardware_tested=False,field_ready=False,robot_command_sent=False,uses_free_object_poses=False,events=[])
models=json.loads((RUN/'robot-kinematics.json').read_text());frames=np.asarray(models['frames']);axes=np.asarray(models['axes']);tip=np.asarray(models['tip']);trace=np.load(RUN/'physx-sequence-trace.npz');commands=np.load(RUN/'executed-actuator-commands.npz');paths=list(trace['camera_body_paths']);T=np.eye(4)
def transform(row):
 t=np.eye(4);t[:3,:3]=Rotation.from_quat(row[3:]).as_matrix();t[:3,3]=row[:3];return t
def proper(t):
 t=np.array(t,float);u,_,v=np.linalg.svd(t[:3,:3]);t[:3,:3]=u@v;return t
frames=np.array([[proper(a),proper(b)]for a,b in frames]);tip=proper(tip)
bases={s:proper(transform(trace['camera_bodies'][0,next(i for i,p in enumerate(paths)if p.endswith('/'+s+'_arm_fr3v2_link0'))]))for s in ['left','right']}
ids={'right':[1,3,5,7,9,11,13],'left':[0,2,4,6,8,10,12]};fi={'right':[15,17],'left':[14,16]}
F=np.eye(4);F[2,3]=-.107;F=F@tip

def fk(q):
 t=np.eye(4);links=[t.copy()]
 for v,ax,(a,b)in zip(q,axes,frames):
  e=np.eye(4);e[:3,:3]=Rotation.from_rotvec(ax*v).as_matrix();t=t@a@e@b;links.append(t.copy())
 tcp=t@tip;points=np.array([t[:3,3]for t in links]+[tcp[:3,3]])
 return dict(tcp_base=tcp.tolist(),link_frames_base=np.asarray(links).tolist(),link_points_base_m=points.tolist())

i0=int(np.argmin(abs(commands['time_s']-14.82)));i1=int(np.argmin(abs(commands['time_s']-14.84)));q0=commands['actual_dof_position'][i0,ids['right']].astype(float);q1=commands['actual_dof_position'][i1,ids['right']].astype(float);leftq=commands['actual_dof_position'][i1,ids['left']].astype(float)
record['robot_geometry_source']='Archived SIM robot FK/body-base calibration only, not free-body stock transforms'
record['actual_interval_s']=[float(commands['time_s'][i0]),float(commands['time_s'][i1])]
record['archived_joint_delta_rad']=float(np.max(abs(q1-q0)))
with tempfile.TemporaryDirectory(prefix='OFFLINE-recorded-closure-')as td:
 root=Path(td);site=root/'site';init_random(site)
 now=datetime.now(timezone.utc);evidence=root/'MOCK-evidence.txt';evidence.write_text('OFFLINE ONLY: no actual installation, SDK, wrench, STOP test, or hardware. All test approval records below are synthetic protocol fixtures.')
 inst=model_preset.template();inst.update(verified=True,by='OFFLINE MOCK installation contract, not field evidence',at_utc=now.isoformat(),evidence_file=str(evidence));inst['facts']=dict(inst['expected'],no_unmodeled_adapters=True,cable_radial_allowance_m=.002);ip=root/'MOCK-installation.json';ip.write_text(json.dumps(inst))
 def state(q):return dict(q=q.tolist(),q_d=q.tolist(),dq=[0.]*7,dq_d=[0.]*7,F_T_EE_column_major=F.ravel(order='F').tolist(),m_ee_kg=.73,F_x_Cee_m=[0,0,.04],I_ee_column_major_kg_m2=(np.eye(3)*.001).ravel().tolist(),m_load_kg=0.,F_x_Cload_m=[0.]*3,I_load_column_major_kg_m2=[0.]*9)
 snapshots=[]
 for s,q in [('left',leftq),('right',q0)]:
  path=root/(s+'-MOCK-sdk.json');path.write_text(json.dumps(dict(read_only=True,server_protocol_version=10,state=state(q))));snapshots.append(path)
 model_preset.author(site,ip,*snapshots)
 config=json.loads((site/'random-motion.json').read_text());meas=dict(verified=True,by='OFFLINE fixture ONLY, not field measured',at_utc=now.isoformat(),evidence_file=str(evidence))
 config.update(joint_tracking_error_limit_rad=.02,target_position_tolerance_m=.001,target_angle_tolerance_rad=.01,max_observation_age_s=2.,other_arm_q_tolerance_rad=.001)
 config['scene'].update(require_basket_observation=True,require_residual_occupancy=True,workspace=dict(T_cell_box=np.eye(4).tolist(),half_extents_m=[3]*3),obstacles=[dict(name='OFFLINE far placeholder; full machine NOT certified',T_cell_box=np.diag([1.,1.,1.,1.]).tolist(),half_extents_m=[.001]*3)])
 config['scene']['obstacles'][0]['T_cell_box'][2][3]=-2.
 for p in config['phases'].values():p.update(measurement=meas,corridors=[config['scene']['workspace']],tcp_corridor_radius_m=.01,force_monitor=dict(measurement=meas,enabled=True,force_limit_n=5,moment_limit_nm=1))
 raw=json.loads(Path('/home/gpuadmin/workspace/jake/mujoco/.local/field-fair-diagnostic565/fresh-observation.json').read_text());packet=dict(np.load('/home/gpuadmin/workspace/jake/mujoco/.local/field-fair-diagnostic565/fresh-overhead-rgbd.npz'));meta=raw['source_metadata'];meta.update(captured_at_utc=datetime.now(timezone.utc).isoformat(),cell_frame='recorded_v280_SIM_cell')
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
 np.savez_compressed(Path('/home/gpuadmin/workspace/jake/mujoco/.local/field-fair-diagnostic565/fresh-unknown-voxels.npz'),unknown_centers_cell_m=unknown_centers,unknown_indices=unknown_indices,states=final_snapshot.states,bounds_cell_m=bounds,spacing_m=spacing)
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
 # The SAME explicit profile supplies planner/SDK driver/Hand-monitor bound.
 from field_demo.low_speed_profile import prepare,load_tracking_profile
 main_profile=site/'random-motion.json';main_profile.write_text(json.dumps(config))
 low_profile=site/'random-motion-low-speed.json';prepare(main_profile,low_profile)
 config=json.loads(low_profile.read_text());profile_binding=load_tracking_profile(low_profile)
 record['selected_low_speed_profile']=profile_binding
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
 record['new_overhead_render']=json.loads(Path('/home/gpuadmin/workspace/jake/mujoco/.local/field-fair-diagnostic565/fresh-overhead-render.json').read_text())
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
   endpoint=dict(read_only=True,motion_executed=False,server_protocol_version=10,state=state(q),**fk(q));globals()['DIAG_ENDPOINT']=endpoint
   try:
    observed=runtime._selected_occupancy(observed)
    details,_=planner.geometry_check(endpoint,other,config,site_config,'pickup','right',part,observed,[],site,{},clock,'strict_empty_approach',aperture_binding=aperture,stopped_aperture=True)
    row.update(accepted=True,stage='hypothetical_endpoint_full_model_guard',q_goal=q.tolist(),q_pre=preq.tolist());campaign.append(row)
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

 from scipy.optimize import minimize_scalar
 from field_demo.residual_occupancy import _geometry
 captured=[];original_check=planner.collision.check_envelopes
 def capture(active,others,scene,phase,envelope,**kw):
  try:return original_check(active,others,scene,phase,envelope,**kw)
  except planner.collision.CollisionError as error:
   text=str(error)
   if ' intersects measured obstacle 'in text:
    capname,obsname=text.split(' intersects measured obstacle ',1);cap=next(x for x in active if x['name']==capname);obs=next(x for x in scene['obstacles']if x['name']==obsname)
    a=np.asarray(cap['a'])[0];b=np.asarray(cap['b'])[0];X=np.asarray(obs['T_cell_box']);half=np.asarray(obs['half_extents_m'])
    def sq(u):
     p=(a+(b-a)*u-X[:3,3])@X[:3,:3];return np.sum(np.maximum(abs(p)-half,0)**2)
    res=minimize_scalar(sq,bounds=(0,1),method='bounded',options=dict(xatol=1e-13));u=float(res.x);axispoint=a+(b-a)*u;local=(axispoint-X[:3,3])@X[:3,:3];nearest=X[:3,3]+np.clip(local,-half,half)@X[:3,:3].T
    spec=next(x for x in envelope['link_capsules_link_m']if 'right_'+x['name']==capname)
    lengths=np.linalg.norm(np.diff(np.asarray(DIAG_ENDPOINT['link_points_base_m']),axis=0),axis=1)+1e-6;i=spec['link_index'];reach=max(np.linalg.norm(spec['a']),np.linalg.norm(spec['b']))+spec['radius_m'];joint_coefficient=sum(np.sum(lengths[j+1:i])for j in range(7)if j+1<i)+i*reach
    captured.append(dict(tracking_bound_rad=config['joint_tracking_error_limit_rad'],capsule=capname,link_index=i,nominal_spec=spec,a_cell_m=a.tolist(),b_cell_m=b.tolist(),actual_checked_radius_m=float(cap['r']),margin_m=kw['margin'],centerline_to_box_m=float(planner.collision.segment_box_distance(cap['a'],cap['b'],obs)[0]),nearest_axis_point_cell_m=axispoint.tolist(),nearest_box_point_cell_m=nearest.tolist(),obstacle=obs,nominal_radius_m=spec['radius_m'],rotational_reach_m=reach,tracking_point_arm_coefficient_m=float(joint_coefficient-i*reach),tracking_local_coefficient_m=float(i*reach),tracking_coefficient_m=float(joint_coefficient),inflation_at_5mrad_m=float(.005*joint_coefficient),inflation_at_20mrad_m=float(.02*joint_coefficient),temporal_sweep_m=0.,endpoint=DIAG_ENDPOINT))
   raise
 planner.collision.check_envelopes=capture
 family=entry['grasp_families'][0];instance=next(v for v in raw['instances']if v['part']=='ring1'and v.get('geometrically_accessible'));camera_goal=np.asarray(instance['T_cell_part']);goal=camera_goal@np.asarray(family['T_part_tcp']);pre=camera_goal@offset(family['T_part_tcp'],family['pregrasp_offset_part_m'])
 campaign=[]
 try:OfflineRuntime().plan('pickup','ring1','right',[pre.tolist(),goal.tolist()],select_observation(raw,instance),[],ik_seed=startq)
 except Exception:pass
 # Same first candidate q, unchanged baseline20mrad model-only comparison.
 config['joint_tracking_error_limit_rad']=.02
 try:planner.geometry_check(DIAG_ENDPOINT,other,config,site_config,'pickup','right','ring1',runtime._selected_occupancy(select_observation(raw,instance)),[],site,{},clock,'strict_empty_approach',aperture_binding=aperture,stopped_aperture=True)
 except Exception:pass
 first=captured[0];point=np.asarray(first['nearest_box_point_cell_m']);bounds_arr=np.asarray(bounds);shape=np.asarray(snapshot.shape);spacing=(bounds_arr[1]-bounds_arr[0])/shape;index=np.floor((point-bounds_arr[0])/spacing).astype(int);index=np.clip(index,0,shape-1)
 center=bounds_arr[0]+(index+.5)*spacing;corners=center+np.asarray([[a,b,c]for a in [-1,1]for b in [-1,1]for c in [-1,1]])*spacing/2
 T=np.asarray(packet['T_cell_camera']);K=np.asarray(packet['K']);cam=(corners-T[:3,3])@T[:3,:3];proj=cam[:,:2]/cam[:,2:];proj=proj*[K[0,0],K[1,1]]+[K[0,2],K[1,2]];lo=np.floor(proj.min(0)-.5).astype(int);hi=np.ceil(proj.max(0)+.5).astype(int);depth=np.asarray(packet['depth_m']);within=bool((lo>=0).all()and(hi<[depth.shape[1],depth.shape[0]]).all());foot=depth[lo[1]:hi[1]+1,lo[0]:hi[0]+1]if within else np.array([]);valid=np.isfinite(foot)&(foot>.05)&(foot<5)
 finite_depth=foot[valid];minimum=float(finite_depth.min())if finite_depth.size else None;arg=np.argwhere(valid&(foot==minimum))[0]if finite_depth.size else None;pixel=[int(lo[0]+arg[1]),int(lo[1]+arg[0])]if arg is not None else None
 source_world=None
 if pixel:
  px,py=pixel;z=float(depth[py,px]);source_world=(np.asarray([(px-K[0,2])/K[0,0]*z,(py-K[1,2])/K[1,1]*z,z])@T[:3,:3].T+T[:3,3]).tolist()
 proof=dict(scope='Read-only root-cause of first fresh-overhead candidate; same original geometry query, no movement/UNKNOWN waiver.5mrad vs20mrad is NONEXECUTABLE offline sensitivity only, not actual commissioning evidence.',case=captured,unknown_voxel=dict(index=index.tolist(),state=int(snapshot.states[tuple(index)]),center_cell_m=center.tolist(),bounds_cell_m=[(center-spacing/2).tolist(),(center+spacing/2).tolist()],projected_pixel_low=lo.tolist(),projected_pixel_high=hi.tolist(),inside_image=within,valid_depth_pixels=int(valid.sum()),total_footprint_pixels=int(foot.size),voxel_farthest_camera_z_m=float(cam[:,2].max()),depth_margin_m=.003,nearest_observed_camera_z_m=minimum,nearest_observed_pixel=pixel,nearest_observed_surface_cell_m=source_world,free_ray_test='whole-footprint all valid AND farthest_voxel_z +3mm < nearest_depth'),hardware_tested=False,robot_commands_sent=False)
 Path('/tmp/field-fair-recorded565/root-cause.json').write_text(json.dumps(proof,indent=2));print(json.dumps(proof,indent=2))
