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
R=Path('/home/gpuadmin/workspace/jake/field-demo-random-common-duo-mount-20261022');REPO=R.parent/'mujoco';RUN=REPO/'isaac_port/runs/native-roas-table-scatter-stable-v348-gpu-carry-no-greedy-final-retreat-all3-446';H=REPO/'isaac_port/native_hand_layout/roas-table'
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

 from types import SimpleNamespace
 from field_demo.residual_occupancy import OccupancySnapshot
 from field_demo.model_geometry import robot_capsules as make_capsules
 from field_demo.random_perception import load_random_config
 from scipy.optimize import minimize_scalar
 obslist=json.loads((RUN/'rgb-observations.json').read_text());output=[]
 localmodels=np.load(RUN/'camera-local-models.npz');modelpaths=localmodels['paths'].tolist();tracepaths=trace['camera_body_paths'].tolist()
 def axis_frame(n):
  n=np.asarray(n,float);n/=np.linalg.norm(n);x=np.array([1.,0.,0.]);x-=n*(x@n);x/=np.linalg.norm(x);return np.column_stack([x,np.cross(n,x),n])
 def distance_vertices_capsules(vertices,caps):
  distances=[]
  for v in caps:
   a=np.asarray(v['a'])[0];b=np.asarray(v['b'])[0];ab=b-a;u=np.clip((vertices-a)@ab/max(ab@ab,1e-15),0,1);distances.append(np.linalg.norm(vertices-a-u[:,None]*ab,axis=1)-(v['nominal_radius_m'] if 'nominal_radius_m'in v else v['r'])+.002)
  return np.min(distances,axis=0)
 original_check=planner.collision.check_envelopes
 last={}
 def capture(active,others,scene,phase,envelope,**kw):
  last.update(active=active,other=others,scene=scene,envelope=envelope,margin=kw['margin'])
  return original_check(active,others,scene,phase,envelope,**kw)
 planner.collision.check_envelopes=capture
 for oi,label in [(7,'actual348_preclose_before_grasp'),(13,'actual348_postlift_held')]:
  observation=obslist[oi];wall=observation['captured_wall_s'];ci=int(np.argmin(abs(commands['time_s']-wall)));ti=int(np.argmin(abs(trace['time_s']-wall)));actual=commands['actual_dof_position'][ci];q=actual[ids['right']].astype(float);lq=actual[ids['left']].astype(float);source_qd=commands['command'][ci,ids['right']].astype(float)
  fk_result=fk(q);tcp=bases['right']@np.asarray(fk_result['tcp_base']);public_tcp=np.asarray(observation['hand_TCP_world']);deltaR=tcp[:3,:3]@public_tcp[:3,:3].T;reg=dict(capture_wall_s=wall,encoder_wall_s=float(commands['time_s'][ci]),encoder_age_s=float(abs(commands['time_s'][ci]-wall)),trace_wall_s=float(trace['time_s'][ti]),public_TCP_vs_saved_encoder_FK_translation_m=float(np.linalg.norm(tcp[:3,3]-public_tcp[:3,3])),public_TCP_vs_saved_encoder_FK_rotation_rad=float(np.linalg.norm(Rotation.from_matrix(deltaR).as_rotvec())),measured_actual_vs_command_joint_error_rad=float(np.max(abs(q-source_qd))))
  packet_path=RUN/'rgb-perception'/('observation-%03d-'%(oi+1)+observation['mode'])/'rgbd.npz';packet=dict(np.load(packet_path));clock=datetime.now(timezone.utc)
  cfg=dict(T_cell_camera=packet['T_cell_camera'].tolist(),residual_occupancy=dict(voxel_size_m=.004,depth_margin_m=.003),basket=dict(outer_bounds_cell_m=json.loads((H/'bin-picking/asset-audit.json').read_text())['baskets'][0]['outer_bounds_world_m']))
  bounds=cfg['basket']['outer_bounds_cell_m'];meta=dict(capture_id='actual348-observation-'+str(oi+1),captured_at_utc=clock.isoformat(),calibration_sha256=hashlib.sha256(packet['T_cell_camera'].tobytes()+packet['K'].tobytes()).hexdigest(),source_packet_sha256=hashlib.sha256(packet_path.read_bytes()).hexdigest(),cell_frame='actual348_SIM_cell',source='actual_archived_SIM_RGBD_only',replay=True,scene_epoch='actual348capture'+str(oi),rectified_registered_depth=True,projection_kind='rectified_pinhole',depth_uncertainty_m=.001)
  pose=observation['result']['pose'];P=np.eye(4);P[:3,:3]=axis_frame(pose['axis']);P[:3,3]=pose['center_m'];inst=dict(part='ring1',instance_id=meta['capture_id']+':image_target',T_cell_part=P.tolist(),center_m=pose['center_m'],axis=pose['axis'],geometrically_accessible=True)
  raw=dict(calibration_sha256=meta['calibration_sha256'],scope='basket',selected_part='ring1',selected_instance=inst,instances=[inst],T_cell_part=P.tolist(),source_packet=str(packet_path),source_packet_sha256=meta['source_packet_sha256'],source_metadata=meta,residual_occupancy_metadata=meta,capture_id=meta['capture_id'],captured_at_utc=clock.isoformat(),_camera_config_path='/tmp/field-cross348/recorded-camera.json')
  runtime=LiveRuntime.__new__(LiveRuntime);runtime.recipe=dict(max_observation_age_s=2.);runtime.camera_configs={'/tmp/field-cross348/recorded-camera.json':cfg}
  # Freeze only this immutable archived image's OFFLINE comparison clock.
  import field_demo.residual_occupancy as occ
  build=occ.build_occupancy
  def frozen_build(*args,**kw):kw['now_utc']=clock;return build(*args,**kw)
  occ.build_occupancy=frozen_build
  try:raw=runtime._selected_occupancy(raw)
  finally:occ.build_occupancy=build
  occupancy=OccupancySnapshot.from_record(raw['residual_occupancy'],now_utc=clock)
  endpoint=dict(read_only=True,motion_executed=False,server_protocol_version=10,state=state(q),**fk_result)
  other=dict(read_only=True,motion_executed=False,server_protocol_version=10,state=state(lq),**fk(lq))
  site_config=dict(robots={side:dict(ip='192.0.2.'+str(1+i),T_cell_base=bases[side].tolist())for i,side in enumerate(['left','right'])})
  hs={side:dict(completed=True,width_m=float(actual[fi[side]].sum()),max_width_m=.08,is_grasped=label.endswith('held')and side=='right',robot_time_s=wall,captured_at_utc=clock.isoformat())for side in ['left','right']};aperture=hand_aperture.binding(hs,now=clock)
  from field_demo import common_duo_mount as duo
  motion_file=site/'random-motion.json';site_file=site/'SYNTHETIC-common-site.json';motion_file.write_text(json.dumps(config));site_file.write_text(json.dumps(site_config));proof_file=site/('SYNTHETIC-common-mount-'+str(oi)+'.json');duo.prepare(motion_file,site_file,proof_file)
  proof=json.loads(proof_file.read_text());evidence=site/'SYNTHETIC-common-evidence.txt';evidence.write_text('OFFLINE SIM protocol fixture only. NOT actual common mount/SDK/site evidence.');proof.update(verified=True,actual_common_rigid_mount_installed=True,by='SYNTHETIC OFFLINE fixture only',at_utc=clock.isoformat(),evidence_file=evidence.name);proof_file.write_text(json.dumps(proof));common_spec=dict(proof_file=proof_file.name,proof_sha256=duo.digest(proof_file),evidence_sha256=duo.digest(evidence))
  cases=[];nominal_rows=None
  for tracking,mount_enabled in [(.02,False),(0.,False),(.02,True),(0.,True)]:
   if mount_enabled:config['common_duo_mount']=common_spec
   else:config.pop('common_duo_mount',None)
   config['joint_tracking_error_limit_rad']=tracking;last.clear();error=None
   try:planner.geometry_check(endpoint,other,config,site_config,'pickup','right','ring1',raw,[],site,{},clock,'strict_empty_approach',aperture_binding=aperture,stopped_aperture=True)
   except Exception as exc:error=str(exc);print('GEOMETRY_ERROR',tracking,error,flush=True)
   detail=None
   if error and ' intersects measured obstacle 'in error:
    name,obname=error.split(' intersects measured obstacle ',1);cap=next(x for x in last['active']if x['name']==name);ob=next(x for x in last['scene']['obstacles']if x['name']==obname);spec=next((v for v in config['robots']['right']['link_capsules_link_m']if 'right_'+v['name']==name),None)
    detail=dict(capsule=name,link_index=cap['link_index'],radius_m=float(cap['r']),margin_m=last['margin'],obstacle=ob,centerline_distance_m=float(planner.collision.segment_box_distance(cap['a'],cap['b'],ob)[0]),nominal_model_cable_radius_m=spec['radius_m']if spec else None)
    aa=np.asarray(cap['a'])[0];bb=np.asarray(cap['b'])[0];XX=np.asarray(ob['T_cell_box']);hh=np.asarray(ob['half_extents_m'])
    def sq(u):return np.sum(np.maximum(abs((aa+(bb-aa)*u-XX[:3,3])@XX[:3,:3])-hh,0)**2)
    u=float(minimize_scalar(sq,bounds=(0,1),method='bounded').x);axispoint=aa+(bb-aa)*u;local=(axispoint-XX[:3,3])@XX[:3,:3];point=XX[:3,3]+np.clip(local,-hh,hh)@XX[:3,:3].T;point=point*.999999+XX[:3,3]*.000001
    spacing=(np.asarray(bounds)[1]-np.asarray(bounds)[0])/np.asarray(occupancy.shape);vi=np.clip(np.floor((point-np.asarray(bounds)[0])/spacing).astype(int),0,np.asarray(occupancy.shape)-1);center=np.asarray(bounds)[0]+(vi+.5)*spacing;corners=center+np.asarray([[a,b,c]for a in [-1,1]for b in [-1,1]for c in [-1,1]])*spacing/2;CT=packet['T_cell_camera'];cam=(corners-CT[:3,3])@CT[:3,:3];projected=cam[:,:2]/np.maximum(cam[:,2:],1e-8);K=packet['K'];projected=projected*[K[0,0],K[1,1]]+[K[0,2],K[1,2]];lo=np.floor(projected.min(0)-.5).astype(int);hi=np.ceil(projected.max(0)+.5).astype(int);dep=packet['depth_m'];inside=bool((lo>=0).all()and(hi<[dep.shape[1],dep.shape[0]]).all()and(cam[:,2]>.05).all());foot=dep[lo[1]:hi[1]+1,lo[0]:hi[0]+1]if inside else np.array([]);valid=np.isfinite(foot)&(foot>.05)&(foot<5)
    detail.update(a_cell_m=aa.tolist(),b_cell_m=bb.tolist(),nearest_uncertainty_cell=dict(index=vi.tolist(),state=int(occupancy.states[tuple(vi)]),center_cell_m=center.tolist(),pixel_low=lo.tolist(),pixel_high=hi.tolist(),camera_shape=list(dep.shape),inside_FOV=inside,valid_depth_pixels=int(valid.sum()),total_pixels=int(foot.size),farthest_voxel_camera_z_m=float(cam[:,2].max()),nearest_valid_depth_m=float(foot[valid].min())if valid.any()else None,depth_margin_m=.003))
   pair_detail=None
   if error and error.startswith('Other-arm collision: '):
    an,bn=error.split(': ',1)[1].split('/');ac=next(x for x in last['active']if x['name']==an);bc=next(x for x in last['other']if x['name']==bn)
    pair_detail=dict(active={k:(v.tolist()if isinstance(v,np.ndarray)else v)for k,v in ac.items()},other={k:(v.tolist()if isinstance(v,np.ndarray)else v)for k,v in bc.items()},centerline_distance_m=float(planner.collision.segment_distance(ac['a'],ac['b'],bc['a'],bc['b'])[0]),margin_m=last['margin'])
   cases.append(dict(common_mount_enabled=mount_enabled,tracking_budget_rad=tracking,executable=False,accepted=error is None,first_rejection=error,first_obstacle=detail,first_pair=pair_detail))
   if tracking==0.:nominal_rows=last['active']
  # Private renderer body transforms are used only for this registration and
  # CAD-enclosure EVALUATION, never geometry targets or controller input.
  body_reg=[];coverage=[]
  for link in range(1,8):
   tail='/right_arm_fr3v2_link'+str(link);path=next(p for p in modelpaths if p.endswith(tail));mi=modelpaths.index(path);bi=tracepaths.index(path);actor=proper(transform(trace['camera_bodies'][ti,bi]));field=bases['right']@np.asarray(fk_result['link_frames_base'])[link];vertices=localmodels['v'+str(mi)]@actor[:3,:3].T+actor[:3,3]
   body_reg.append(dict(link=link,translation_m=float(np.linalg.norm(actor[:3,3]-field[:3,3])),rotation_rad=float(np.linalg.norm(Rotation.from_matrix(actor[:3,:3]@field[:3,:3].T).as_rotvec()))))
   caps=[r for r in nominal_rows if r.get('link_index')==link];excess=distance_vertices_capsules(vertices,caps);coverage.append(dict(link=link,Isaac_collision_mesh_vertices=len(vertices),max_vertex_outside_field_CAD_columns_without_cable_m=float(excess.max()),fraction_vertices_enclosed=float(np.mean(excess<=.0001))))
  palm=next(p for p in modelpaths if p.endswith('/right_NativeHand'));actual_palm=proper(transform(trace['camera_bodies'][ti,tracepaths.index(palm)]));field_palm=tcp@np.asarray(config['robots']['right']['native_hand_aperture_geometry']['T_tcp_hand']);reg.update(palm_root_translation_m=float(np.linalg.norm(actual_palm[:3,3]-field_palm[:3,3])),palm_root_rotation_rad=float(np.linalg.norm(Rotation.from_matrix(actual_palm[:3,:3]@field_palm[:3,:3].T).as_rotvec())),body_registrations=body_reg)
  output.append(dict(state=label,source_observation_index=oi,registration=reg,measured_G_from_public_image_and_public_TCP=(np.linalg.inv(public_tcp)@P).tolist(),native_width_m=hs['right']['width_m'],camera_FOV=dict(shape=packet['depth_m'].shape,K=packet['K'].tolist(),T_cell_camera=packet['T_cell_camera'].tolist()),occupancy=dict(unknown=int((occupancy.states==0).sum()),occupied=int((occupancy.states==2).sum()),free=int((occupancy.states==1).sum())),geometry_cases=cases,nominal_field_columns_vs_rendered_Isaac_collision_vertices=coverage))
Path('/tmp/field-common-duo-replay348.json').write_text(json.dumps(dict(scope='ONE cross-framework diagnostic at actual348 preclose+held saved states. Renderer actor transforms evaluation-only. No command, hardware, profile activation or new grasp/view/grid. Nominal0mrad is nonexecutable sensitivity. Fresh close-camera occupancy is deliberately not asserted to be a valid leased initial map; strict endpoint rejection is diagnostic, not actual held transport semantics.',cases=output,hardware_tested=False,robot_commands_sent=False),indent=2));print(json.dumps(output,indent=2))
