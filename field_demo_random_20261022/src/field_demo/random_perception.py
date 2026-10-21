"""Seed-free v119 ring RGB-D geometry with measured camera/fixture boundaries.

This module accepts camera arrays, never simulator objects, masks or scene seeds.
Observed geometric accessibility is not proof of robot reachability or contact.
"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import time
from .calibration import rigid,utc_stamp
from .observations import validate_observation,target_proposals


def _measurement(record,root,name):
    if not isinstance(record,dict) or record.get('verified') is not True:
        raise ValueError(name+': verified measured provenance required')
    for key in ('by','method','at_utc','evidence_file'):
        if not isinstance(record.get(key),str) or not record[key].strip():
            raise ValueError(name+': measured '+key+' required')
    if utc_stamp(record['at_utc'])>datetime.now(timezone.utc):
        raise ValueError(name+': future measurement timestamp')
    evidence=(root/record['evidence_file']).resolve()
    if not evidence.is_file() or not evidence.read_bytes().strip():
        raise ValueError(name+': nonempty measurement evidence file required')


def load_random_config(path):
    """Load named metric calibration without any ring position/orientation prior.

    Explicit replay_only configurations may use simulation calibration, but are
    rejected by every live detect/observe operation.
    """
    import numpy as np
    path=Path(path).resolve();c=json.loads(path.read_text())
    if c.get('schema_version')!=2 or c.get('units')!='m':
        raise ValueError('Random camera schema_version 2 and explicit units m required')
    if any(k in c for k in ('nominal_stock_poses','ring_seed','ring_seeds','scene_seed','object_poses')):
        raise ValueError('Random ring configuration cannot contain object position priors or scene seeds')
    for key in ('calibration_id','camera_frame','camera_serial','cell_frame'):
        if not isinstance(c.get(key),str) or not c[key].strip():
            raise ValueError('Named '+key+' required')
    if type(c.get('replay_only',False)) is not bool:
        raise ValueError('replay_only must be an explicit boolean')
    replay_only=c.get('replay_only',False)
    if replay_only:
        if c.get('calibration_kind')!='simulation_replay' or not c.get('replay_source'):
            raise ValueError('Replay-only calibration must name its simulation replay source')
    else:
        _measurement(c.get('measured'),path.parent,'camera calibration')
    rigid(c.get('T_cell_camera'),'T_cell_camera')
    frames={c['camera_frame'],c['cell_frame']}
    if len(frames)!=2:raise ValueError('Camera and cell frames must differ')
    for side in ('left','right'):
        row=c.get('robots',{}).get(side,{})
        if not isinstance(row.get('base_frame'),str) or not row['base_frame'] or row['base_frame'] in frames:
            raise ValueError('Distinct named robot base frame required: '+side)
        frames.add(row['base_frame']);rigid(row.get('T_cell_base'),side+'.T_cell_base')
    for part in ('ring1','ring2','pin'):
        if c.get('part_robot',{}).get(part) not in ('left','right'):
            raise ValueError('Each part_robot must name left/right')
    basket=c.get('basket',{})
    floor=np.asarray(basket.get('floor_origin_cell_m'),float)
    bounds=np.asarray(basket.get('outer_bounds_cell_m'),float)
    rotation=np.asarray(basket.get('basket_to_cell_R'),float)
    if floor.shape!=(3,) or bounds.shape!=(2,3) or not np.isfinite(floor).all() or not np.isfinite(bounds).all():
        raise ValueError('Measured basket floor and 2x3 metric bounds required')
    if np.any(bounds[0]>=bounds[1]) or np.max(np.abs(bounds))>5 or np.linalg.norm(floor)>5:
        raise ValueError('Invalid basket bounds; check mm/m units')
    if rotation.shape!=(3,3):raise ValueError('basket_to_cell_R must be 3x3')
    t=np.eye(4);t[:3,:3]=rotation;rigid(t,'basket_to_cell_R')
    if not replay_only:_measurement(basket.get('measured'),path.parent,'basket fixture')
    enclosure=basket.get('unrecognized_scene_enclosure')
    if enclosure is not None:
        from .collision import box,inside_box
        _,half=box(enclosure)
        if np.max(half)>2 or enclosure.get('includes_possible_protrusions') is not True or not enclosure.get('name'):
            raise ValueError('Named unknown-scene enclosure must explicitly include possible protruding clutter')
        corners=np.array([[bounds[i,0],bounds[j,1],bounds[k,2]] for i in (0,1) for j in (0,1) for k in (0,1)])
        if not np.all(inside_box(corners,corners,0,enclosure)):
            raise ValueError('Unknown-scene enclosure must contain the full measured basket ROI')
        if not replay_only:_measurement(enclosure.get('measurement'),path.parent,'unrecognized basket scene enclosure')
    constraint=basket.get('floor_fit_constraint')
    if constraint is not None:
        if not isinstance(constraint,dict):raise ValueError('Explicit floor-fit constraint required')
        for key in ('maximum_correction_m','reported_fit_depth_bound_m'):
            value=constraint.get(key)
            if type(value)not in (int,float)or not 0<value<=.005:raise ValueError('Explicit measured floor-fit/depth bound<=5mm required')
        uncertainty=(c.get('residual_occupancy')or {}).get('depth_uncertainty_m')
        if type(uncertainty)not in (int,float)or constraint['reported_fit_depth_bound_m']>uncertainty:raise ValueError('Floor-fit bound must be covered by explicit actual depth uncertainty')
    held=c.get('held_workspace_bounds_cell_m')
    if held is not None:
        a=np.asarray(held,float)
        if a.shape!=(2,3) or not np.isfinite(a).all() or np.any(a[0]>=a[1]) or np.max(np.abs(a))>5:
            raise ValueError('Invalid held workspace metric bounds')
    if c.get('pin_pose_backend','cad_icp')not in ('cad_icp','measured_cap_circle'):raise ValueError('Unknown pin pose backend')
    holder=c.get('pin_holder',{})
    if type(holder.get('enabled',False)) is not bool:raise ValueError('pin_holder enabled must be boolean')
    if holder.get('enabled'):
        rigid(holder.get('T_cell_pin_seed'),'measured pin holder seed')
        if not replay_only:_measurement(holder.get('measured'),path.parent,'pin holder')
    limit=c.get('max_observation_age_s',.5)
    if type(limit) not in (int,float) or not 0<limit<=2:raise ValueError('Observation age limit must be (0,2] s')
    c['_calibration_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    c['_replay_only']=replay_only
    return c


def _packet(packet,config,metadata,replay):
    import numpy as np
    allowed={'rgb','depth_m','K','T_cell_camera','points_camera_m','points_camera_image_m'}
    if set(packet)-allowed:
        raise ValueError('Forbidden/noncamera packet keys: '+','.join(sorted(set(packet)-allowed)))
    rgb=np.asarray(packet.get('rgb'));depth=np.asarray(packet.get('depth_m'),float);K=np.asarray(packet.get('K'),float)
    if rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[2]!=3 or min(rgb.shape[:2])<32:
        raise ValueError('Organized HxWx3 RGB uint8 image required')
    if depth.shape!=rgb.shape[:2] or K.shape!=(3,3) or not np.isfinite(K).all() or min(K[0,0],K[1,1])<=0:
        raise ValueError('Registered depth_m HxW and calibrated K required')
    T=rigid(config['T_cell_camera'])
    if 'T_cell_camera' in packet and not np.allclose(rigid(packet['T_cell_camera']),T,atol=1e-7):
        raise ValueError('Packet camera transform differs from named calibration')
    result=dict(rgb=rgb,depth_m=depth,K=K,T_cell_camera=T)
    organized=packet.get('points_camera_image_m')
    if organized is None and 'points_camera_m' in packet and np.asarray(packet['points_camera_m']).shape==(*depth.shape,3):
        organized=packet['points_camera_m']
    if organized is not None:
        points=np.asarray(organized,float)
        if points.shape!=(*depth.shape,3):raise ValueError('SDK points must preserve HxWx3 pixel correspondence')
        valid=np.isfinite(points).all(2)&np.isfinite(depth)&(depth>.05)&(depth<5)
        if np.count_nonzero(valid)<150:raise RuntimeError('Insufficient valid RGB-D pixels')
        if np.max(np.abs(points[:,:,2][valid]-depth[valid]))>.001:
            raise ValueError('SDK XYZ and aligned depth disagree; check frame alignment/units')
        points=points.copy();points[~valid]=np.nan;result['points_camera_m']=points
    else:
        # Authentic simulator recordings use an explicitly rectified pinhole
        # camera. Real cameras require SDK pixel deprojection even in replay.
        if not replay or config.get('calibration_kind')!='simulation_replay' or metadata.get('rectified_registered_depth') is not True:
            raise ValueError('Organized SDK pixel XYZ required; flat point clouds cannot supply image neighborhoods')
    return result


def _pose(pose,capture_id,index):
    import numpy as np
    from .stabilized.rgbd_geometry import axis_frame
    p=dict(pose);axis=np.asarray(p['axis'],float);center=np.asarray(p['center_m'],float)
    if axis.shape!=(3,) or center.shape!=(3,) or not np.isfinite(axis).all() or not np.isfinite(center).all() or abs(np.linalg.norm(axis)-1)>1e-5:
        raise RuntimeError('Detector returned invalid metric pose')
    t=np.eye(4);t[:3,:3]=axis_frame(axis);t[:3,3]=center
    p.update(instance_id=capture_id+':image:'+str(index),T_cell_part=t.tolist(),
             orientation_scope='signed axis from observed tapered face; axial yaw unobservable; canonical basis only',
             instance_identity_scope='this RGB-D capture only; no simulator actor identity',
             geometrically_accessible=bool(p.get('front_back_orientation_margin_m',0)>.0005),
             robot_reachability_verified=False)
    return p


def _previous_image(previous,config,replay,part):
    import numpy as np
    if not isinstance(previous,dict) or previous.get('state')!='random_ring_observation':
        raise ValueError('Previous pose must be a matching image observation')
    prior_config=config
    if previous.get('calibration_sha256')!=config['_calibration_sha256']:
        old_path=previous.get('_camera_config_path')
        if not old_path:raise ValueError('Cross-camera image prior needs its exact measured camera configuration')
        prior_config=load_random_config(old_path)
        if prior_config['_calibration_sha256']!=previous.get('calibration_sha256'):
            raise ValueError('Previous camera calibration file changed')
        if prior_config['cell_frame']!=config['cell_frame']:
            raise ValueError('Cross-camera images must share the same measured cell frame')
        for side in ('left','right'):
            if prior_config['robots'][side]['base_frame']!=config['robots'][side]['base_frame'] or not np.allclose(rigid(prior_config['robots'][side]['T_cell_base']),rigid(config['robots'][side]['T_cell_base']),atol=1e-8):
                raise ValueError('Cross-camera robot/cell calibration differs')
        if not replay and prior_config.get('_replay_only'):raise ValueError('Replay calibration cannot initialize a live held observation')
    if previous.get('replay') is not bool(replay):raise ValueError('Replay image hypotheses cannot enter live tracking')
    # A historical fit is only a search prior. Current exposure freshness is
    # checked independently before and after fitting current camera pixels.
    old_metadata=previous.get('source_metadata',{})
    validate_observation(old_metadata,prior_config,replay=True)
    if not replay and (old_metadata.get('source')!='realsense_live' or old_metadata.get('replay') is not False):
        raise ValueError('Held search prior must come from actual camera observation')
    candidates=[p for p in previous.get('instances',[]) if p['part']==part]
    if len(candidates)!=1:raise RuntimeError('Previous held image hypothesis is ambiguous')
    return candidates[0]


def detect_packet(packet,config,metadata,*,replay=False,scope='basket',part=None,
                  held_tcp_pose=None,held_roi_radius_m=.048,previous_observation=None,
                  held_robot=None,held_grasp_transform=None,allow_unknown_basket=False,fixture_T_cell_target=None):
    """Detect ring instances without initial object seeds; optional held/holder ROI.

    Held correspondence may use encoder TCP or an earlier IMAGE observation;
    these restrict search only. The object pose still comes from current RGB-D.
    """
    import numpy as np
    from .stabilized.rgbd_scattered_rings import detect
    from .stabilized.rgbd_geometry import cloud
    started=time.monotonic()
    if not replay and config.get('_replay_only'):raise RuntimeError('Simulation replay calibration cannot be used live')
    freshness=validate_observation(metadata,config,replay=replay)
    if scope not in ('basket','held','pin_holder','fixture'):raise ValueError('scope must be basket, held or pin_holder')
    if part is not None and part not in ('ring1','ring2','pin'):raise ValueError('Invalid part')
    camera=_packet(packet,config,metadata,replay)
    basket=config['basket'];normal=np.asarray(basket['basket_to_cell_R'])[:,2]
    calibration=dict(floor_origin_world_m=basket['floor_origin_cell_m'],basket_to_world_R=basket['basket_to_cell_R'],
                     outer_bounds_world_m=basket['outer_bounds_cell_m'],global_plane_faces=False)
    raw={};seed=None;image_candidates=[];unknown_obstacles=[]
    if allow_unknown_basket and scope!='basket':raise ValueError('Unknown basket fallback is scene-only and basket-only')
    if held_robot is not None and held_robot not in ('left','right'):raise ValueError('held_robot must be left/right')
    if scope=='pin_holder' or (scope in ('held','fixture') and part=='pin'):
        if part not in (None,'pin'):raise ValueError('pin_holder supports pin only')
        holder=config.get('pin_holder',{})
        if scope=='fixture':
            prior=rigid(fixture_T_cell_target,'measured fixture T_cell_part target');prior_scope='measured_fixture_target; current RGB-D registration required'
        elif scope=='pin_holder':
            if not holder.get('enabled'):raise RuntimeError('Measured pin holder is not configured')
            prior=rigid(holder['T_cell_pin_seed'])
            prior_scope='Independently measured physical pin holder'
        else:
            tcp=rigid(held_tcp_pose,'actual measured T_cell_tcp')
            bounds=np.asarray(config.get('held_workspace_bounds_cell_m'))
            if bounds.shape!=(2,3) or not np.all((tcp[:3,3]>=bounds[0])&(tcp[:3,3]<=bounds[1])):
                raise ValueError('Actual TCP must be inside measured held workspace')
            same_hand=(previous_observation or {}).get('held_robot') in (None,held_robot)
            if held_grasp_transform is not None and same_hand:
                if previous_observation is None:raise ValueError('Actual held pin relation requires its previous image observation')
                _previous_image(previous_observation,config,replay,'pin')
                prior=tcp@rigid(held_grasp_transform,'camera/encoder measured T_tcp_part')
                prior_scope='Actual encoder TCP times previous camera-derived held relation; current image fit required'
            else:
                previous=_previous_image(previous_observation,config,replay,'pin')
                prior=rigid(previous['T_cell_part']).copy()
                previous_tcp=previous_observation.get('measured_tcp_pose')
                if previous_tcp is not None:
                    prior=tcp@np.linalg.inv(rigid(previous_tcp))@prior
                elif np.linalg.norm(prior[:3,3]-tcp[:3,3])>.048:
                    prior[:3,3]=tcp[:3,3]
                prior_scope='Previous image pin axis near actual encoder TCP; current image fit required'
        if config.get('pin_pose_backend','cad_icp')=='measured_cap_circle':
            from .pin_cap import estimate
            fit=estimate(camera,prior)
            fit.update(part='pin',perception_scope='Current RGB-D measured cap circle/axis',search_prior_scope=prior_scope,instance_id=metadata['capture_id']+':image:pin',robot_reachability_verified=False)
        else:
            from isaac_port.fieldkit.perception import CadPoseEstimator
            estimator=CadPoseEstimator();points=cloud(camera).reshape(-1,3)
            model=estimator.models['pin']@prior[:3,:3].T+prior[:3,3]
            keep=np.isfinite(points).all(1)&np.all((points>model.min(0)-.01)&(points<model.max(0)+.01),axis=1)
            if scope=='held':
                rgb=camera['rgb'].reshape(-1,3);gray=rgb.mean(1)
                keep&=(np.ptp(rgb.astype(float),axis=1)<45)&(gray>35)&(gray<180)
            fit=estimator.fit('pin',points[keep],prior,max_shift_m=(.01,.01,.01),max_tilt_rad=.2).as_dict()
            t=np.asarray(fit['T_cell_part']);fit.update(part='pin',center_m=t[:3,3].tolist(),axis=t[:3,2].tolist(),
                      perception_scope='Current RGB-D CAD registration',search_prior_scope=prior_scope,
                      instance_id=metadata['capture_id']+':image:pin',robot_reachability_verified=False)
        instances=[fit];selected=[fit]
        raw['segmentation']=dict(backend='Current RGB-D measured cap circle/axis'if config.get('pin_pose_backend')=='measured_cap_circle'else'Current RGB-D pin CAD registration',search_prior_scope=prior_scope,simulator_input=False)
    else:
        if part=='pin':raise ValueError('Random ring detector does not infer a pin from a ring ROI')
        if scope in ('held','fixture'):
            if part not in ('ring1','ring2'):raise ValueError('Held scope requires ring1 or ring2')
            if not 0<held_roi_radius_m<=.048:raise ValueError('Held ROI radius must be (0,0.048] m; frozen detector bound is unchanged')
            if scope=='fixture':
                seed=rigid(fixture_T_cell_target,'measured fixture target ROI')[:3,3]
            elif held_tcp_pose is not None:
                seed=rigid(held_tcp_pose,'actual measured T_cell_tcp')[:3,3]
            elif previous_observation is not None:
                previous=_previous_image(previous_observation,config,replay,part)
                seed=np.asarray(previous['center_m'],float)
            else:raise ValueError('Held ROI requires actual measured TCP or previous image observation')
            bounds=np.asarray(config.get('fixture_workspace_bounds_cell_m',config.get('held_workspace_bounds_cell_m')) if scope=='fixture' else config.get('held_workspace_bounds_cell_m'))
            if bounds.shape!=(2,3):raise ValueError('Measured held/fixture workspace bounds required')
            if not np.all((seed>=bounds[0])&(seed<=bounds[1])):raise RuntimeError('Actual TCP/previous image seed outside measured held workspace')
            # A held object has no basket support floor. Build its bounded
            # local search slab along the calibrated viewing ray, allowing a
            # visible vertical annular face without a global-Z object prior.
            # The frozen detector's angular/plane tolerances remain unchanged.
            from .stabilized.rgbd_geometry import axis_frame
            view_normal=np.asarray(config['T_cell_camera'],float)[:3,3]-seed
            if np.linalg.norm(view_normal)<.05:raise RuntimeError('Held search center is too close to camera')
            view_normal/=np.linalg.norm(view_normal)
            calibration.update(floor_origin_world_m=(seed-view_normal*.07).tolist(),
                               basket_to_world_R=axis_frame(view_normal).tolist(),
                               outer_bounds_world_m=bounds.tolist(),allow_sideways_faces=True)
        # A requested pickup class must not erase the other size from collision
        # context. Initial basket images always detect all ring size instances.
        try:
            raw,masks=detect(camera,calibration,None,desired_part=part if scope in ('held','fixture') else None,seed=seed)
        except RuntimeError as error:
            if not (scope=='basket' and allow_unknown_basket and str(error)=='No visible annular surface in bin RGB-D'):
                raise
            raw=dict(poses=[],instances=[],segmentation=dict(backend='Frozen v119 RGB-D geometry',detection_rejected=str(error)))
            masks=[]
        for observed,mask in zip(raw['poses'],masks):
            yy,xx=np.nonzero(mask)
            if len(xx):
                image_candidates.append(dict(part=observed['part'],bbox_xyxy=[int(xx.min()),int(yy.min()),int(xx.max()+1),int(yy.max()+1)],
                                             visible_face_pixels=int(len(xx)),source='current RGB-D geometry, not semantic labels'))
        instances=[_pose(p,metadata['capture_id'],i) for i,p in enumerate(raw['instances'])]
        if scope=='basket' and config['basket'].get('floor_fit_constraint')is not None:
            from .floor_constraint import project_inferred_ring_above_floor
            spec=config['basket']['floor_fit_constraint'];floor=config['basket']['floor_origin_cell_m'];up=np.asarray(config['basket']['basket_to_cell_R'])[:,2]
            constrained=[]
            for item in instances:
                try:
                    T,record=project_inferred_ring_above_floor(item['T_cell_part'],item['part'],floor,up,maximum_correction_m=spec['maximum_correction_m'],reported_uncertainty_m=spec['reported_fit_depth_bound_m'])
                except ValueError:continue  # This RGB-D hypothesis contradicts measured floor beyond its own uncertainty.
                item.update(T_cell_part=T.tolist(),center_m=T[:3,3].tolist(),floor_fit_constraint=record);constrained.append(item)
            instances=constrained
        selected=[]
        if scope in ('held','fixture'):
            instances=[p for p in instances if p['part']==part and np.linalg.norm(np.asarray(p['center_m'])-seed)<=held_roi_radius_m]
            if len(instances)!=1:raise RuntimeError('Held image requires exactly one matching ring near measured TCP/previous image')
            selected=instances.copy()
        else:
            # Source selection favors the exposed highest face independently
            # of any actor identity. Keep every detected instance separately.
            for p in raw['poses']:
                options=[q for q in instances if q['part']==p['part']]
                if options:selected.append(min(options,key=lambda q:np.linalg.norm(np.asarray(q['center_m'])-p['center_m'])))
        if not instances:
            if scope!='basket' or not allow_unknown_basket:raise RuntimeError('No qualifying observed ring instances')
            enclosure=config['basket'].get('unrecognized_scene_enclosure')
            if not enclosure:raise RuntimeError('Unknown basket needs a separately measured enclosing scene OBB; absence is not empty')
            if enclosure.get('includes_possible_protrusions') is not True:
                raise ValueError('Unknown enclosure must cover possible protruding clutter')
            unknown_obstacles=[dict(enclosure,source='measured_unknown_scene_enclosure')]
        selected=[p for p in selected if p['geometrically_accessible']]
        if scope=='held' and not selected:raise RuntimeError('Held ring signed-face orientation is ambiguous')
    freshness=validate_observation(metadata,config,replay=replay)
    parts={p['part']:p for p in selected}
    target_config=config
    if scope=='held' and held_robot is not None:
        target_config=dict(config,part_robot=dict(config['part_robot'],**{part:held_robot}))
    proposal=target_proposals(parts,target_config,metadata,replay=replay)
    per_instance={p['instance_id']:target_proposals({p['part']:p},target_config,metadata,replay=replay)[p['part']] for p in instances}
    result=dict(schema_version=2,state='random_ring_observation',scope=scope,units='m',
                capture_id=metadata['capture_id'],captured_at_utc=metadata['captured_at_utc'],
                calibration_id=config['calibration_id'],calibration_sha256=config['_calibration_sha256'],
                output_frame=config['cell_frame'],source_metadata=metadata,instances=instances,poses=selected,
                parts=parts,proposals=proposal,instance_proposals=per_instance,
                both_ring_sizes_visible={'ring1','ring2'}<={p['part'] for p in instances},
                segmentation=raw['segmentation'],image_candidates=image_candidates,image_only_object_pose=True,ring_pose_seed_used=scope=='held',
                basket_scene_status='unknown_treated_fully_occupied' if unknown_obstacles else 'visible_candidates_only_not_empty_certificate',
                scene_only=bool(unknown_obstacles),conservative_scene_obstacles=unknown_obstacles,
                fixture_target_prior_used=scope=='fixture',fixture_search_prior_source='measured_fixture_target'if scope=='fixture'else None,initial_object_pose_prior_used=scope in ('pin_holder','fixture'),initial_ring_pose_prior_used=False,
                pin_fixture_prior_used=scope=='pin_holder',replay=bool(replay),execution_allowed=False,hardware_ready=False,
                freshness=freshness,elapsed_s=time.monotonic()-started,
                detector_source='manifest-verified v119 rgbd_scattered_rings; package import adaptation only',
                limitations=['Thresholds remain v119 ideal-depth settings; real reflective-metal noise is unverified',
                             'Geometric visible-face candidates need actual robot path/contact validation',
                             'No initial semantic actor labels or simulator object pose input',
                             'Signed axis requires asymmetric visible face; axial yaw is unobservable'])
    if scope=='held' and held_tcp_pose is not None and len(selected)==1:
        tcp=rigid(held_tcp_pose)
        result['measured_tcp_pose']=tcp.tolist()
        result['T_tcp_part']=(np.linalg.inv(tcp)@np.asarray(selected[0]['T_cell_part'])).tolist()
        result['held_robot']=held_robot
        result['held_roi_reference']='actual encoder TCP/search position and calibrated camera viewing ray; current RGB-D determines object pose'
    return result


def _snapshot(path,config=None,*,replay=False):
    import numpy as np
    path=Path(path)
    with np.load(path,allow_pickle=False) as z:packet={key:z[key].copy() for key in z}
    sidecar=path.with_suffix('.json')
    if sidecar.is_file():return packet,json.loads(sidecar.read_text())
    expected=(config or {}).get('replay_frame_sha256',{}).get(path.name)
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    if not replay or not (config or {}).get('_replay_only') or expected!=digest:
        raise ValueError('Snapshot metadata sidecar required; only hash-verified packaged simulation replay can omit it')
    metadata=dict(schema_version=1,capture_id='v119-replay:'+digest[:20],
                  captured_at_utc=datetime.fromtimestamp(path.stat().st_mtime,timezone.utc).isoformat(),
                  timestamp_scope='replay source file modification time; sensor exposure timestamp unavailable',
                  source='verified_simulation_rgbd_replay',replay=True,units='m',
                  camera_frame=config['camera_frame'],point_cloud_frame=config['camera_frame'],
                  camera_serial=config['camera_serial'],calibration_id=config['calibration_id'],
                  calibration_sha256=config['_calibration_sha256'],rectified_registered_depth=True,
                  source_file_sha256=digest,hardware_ready=False)
    return packet,metadata


def observe(config_path,output,session_dir,*,frames=0,scope='basket',part=None,allow_unknown_basket=False):
    from .camera import RealSenseCapture,publish_latest,save_capture
    # Warm scipy and detector imports before starting the sensor stream.
    from .stabilized.rgbd_scattered_rings import detect
    output,session_dir=Path(output),Path(session_dir)
    status=lambda state,**kw:dict(state=state,hardware_ready=False,execution_allowed=False,robot_command_sent=False,**kw)
    publish_latest(output,status('random_observation_starting'))
    done=0
    try:
        config=load_random_config(config_path)
        if config.get('_replay_only'):raise RuntimeError('Replay-only calibration cannot start a live observer')
        if frames<0:raise ValueError('frames must be >=0')
        if scope=='held':raise ValueError('CLI held observation requires runtime actual TCP; use cycle integration')
        if session_dir.exists() and any(session_dir.iterdir()):raise FileExistsError('Use a new empty session directory')
        session_dir.mkdir(parents=True,exist_ok=True)
        previous=None
        with RealSenseCapture(config) as stream:
            i=0
            while frames==0 or i<frames:
                try:
                    packet,metadata=stream.read();validate_observation(metadata,config,previous_frame=previous)
                    source=session_dir/f'capture-{i:06d}.npz';save_capture(source,packet,metadata)
                    report=detect_packet(packet,config,metadata,scope=scope,part=part,allow_unknown_basket=allow_unknown_basket)
                    report['snapshot_sha256']=hashlib.sha256(source.read_bytes()).hexdigest()
                    report['snapshot_path']=str(source.resolve())
                    (session_dir/f'observation-{i:06d}.json').write_text(json.dumps(report,indent=2)+'\n')
                    publish_latest(output,report);previous=metadata;done+=1
                except (RuntimeError,ValueError,KeyError,TypeError,OSError) as e:
                    failure=status('random_observation_rejected',reason=str(e),attempt=i)
                    publish_latest(output,failure)
                    (session_dir/f'rejection-{i:06d}.json').write_text(json.dumps(failure,indent=2)+'\n')
                i+=1
        return status('random_observer_finished',published=done)
    except KeyboardInterrupt:
        publish_latest(output,status('random_observation_stopped'))
        return status('random_observer_stopped',published=done)
    except Exception as e:
        publish_latest(output,status('random_observation_rejected',reason=str(e)));raise


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',choices=['detect','observe']);p.add_argument('--snapshot',type=Path)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--replay',action='store_true');p.add_argument('--part',choices=['ring1','ring2','pin'])
    p.add_argument('--scope',choices=['basket','pin_holder'],default='basket')
    p.add_argument('--session-dir',type=Path);p.add_argument('--frames',type=int,default=0)
    p.add_argument('--allow-unknown-basket',action='store_true',help='Scene-only: absence becomes measured occupied OBB, never a pickup target')
    a=p.parse_args(argv)
    try:
        if a.command=='observe':
            if a.replay or not a.session_dir:raise ValueError('Live observe requires --session-dir and forbids --replay')
            result=observe(a.config,a.output,a.session_dir,frames=a.frames,scope=a.scope,part=a.part,allow_unknown_basket=a.allow_unknown_basket)
        else:
            if not a.snapshot:raise ValueError('detect requires --snapshot')
            if a.output.exists():raise FileExistsError('Use a new observation output path')
            config=load_random_config(a.config);packet,metadata=_snapshot(a.snapshot,config,replay=a.replay)
            result=detect_packet(packet,config,metadata,replay=a.replay,scope=a.scope,part=a.part,allow_unknown_basket=a.allow_unknown_basket)
            result['snapshot_sha256']=hashlib.sha256(a.snapshot.read_bytes()).hexdigest()
            result['snapshot_path']=str(a.snapshot.resolve())
            a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(dict(state=result['state'],output=str(a.output),hardware_ready=False,robot_command_sent=False)))
        return 0
    except (RuntimeError,ValueError,KeyError,TypeError,OSError,ImportError) as e:
        print(json.dumps(dict(state='random_observation_rejected',reason=str(e),robot_command_sent=False)))
        return 2


if __name__=='__main__':raise SystemExit(main())
