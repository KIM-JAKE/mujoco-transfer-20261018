"""Stopped actual SDK/image transaction for observed Hand surfaces only.

Before/after SDK samples are application evidence, never a certificate of
unobserved mechanical motion. UNKNOWN and camera shadows stay blocking.
"""
from datetime import datetime,timezone
import copy,hashlib,json
import numpy as np
from .calibration import rigid,utc_stamp
from .hand_aperture import sample


def settings(value):
    if not isinstance(value,dict):raise ValueError('Explicit known-Hand surface bounds required')
    tol=value.get('surface_tolerance_m');uncertainty=value.get('pose_uncertainty_m');gap=value.get('max_sdk_capture_gap_s')
    if (type(tol)not in(int,float)or not np.isfinite(tol)or not .0002<=tol<=.003 or
        type(uncertainty)not in(int,float)or not np.isfinite(uncertainty)or not 0<uncertainty<tol or
        type(gap)not in(int,float)or not np.isfinite(gap)or not 0<gap<=.2):
        raise ValueError('Actual calibration/mount uncertainty and <=3mm surface/200ms SDK bounds required')
    result=dict(surface_tolerance_m=float(tol),pose_uncertainty_m=float(uncertainty),max_sdk_capture_gap_s=float(gap))
    if value.get('robot_material_prior')is not None and value.get('collision_envelope_prior')is not None:raise ValueError('Choose one explicit robot-self model prior')
    key='collision_envelope_prior'if value.get('collision_envelope_prior')is not None else'robot_material_prior'
    prior=value.get(key)
    if prior is not None:
        bound=prior.get('geometry_uncertainty_m')
        if type(bound)not in(int,float)or not np.isfinite(bound)or not 0<bound<=.003:raise ValueError('Explicit installed physical visual-geometry uncertainty required')
        result[key]=dict(geometry_uncertainty_m=float(bound))
    return result


def read_sample(runtime,side):
    started=datetime.now(timezone.utc).isoformat();state=runtime.sdk_state(side)
    hand=runtime.hand_status(side);sample(hand);ended=datetime.now(timezone.utc).isoformat()
    if state.get('robot_mode_name')!='idle'or any(abs(v)>.001 for v in state['dq']+state['dq_d']):raise RuntimeError('Idle actual SDK arms required for camera Hand association')
    for key in ['q','q_d']:
        value=np.asarray(state[key],float)
        if value.shape!=(7,)or not np.isfinite(value).all():raise ValueError('Actual seven-joint state required')
    if np.max(abs(np.asarray(state['q'])-state['q_d']))>.005:raise RuntimeError('Actual SDK pose tracking differs during camera association')
    return dict(read_started_utc=started,read_returned_utc=ended,state=copy.deepcopy(state),hand=copy.deepcopy(hand))


def bind_states(before,after,metadata,robots,mounts,options):
    opts=settings(options);capture=utc_stamp(metadata['captured_at_utc']);exposure=max(metadata.get('exposure_s',{}).values(),default=0.)
    if not np.isfinite(exposure)or not 0<=exposure<=.5:raise ValueError('Actual camera exposure duration required')
    states=[]
    if set(before)!={'left','right'}or set(after)!=set(before):raise ValueError('Both actual SDK arms must bracket this camera exposure')
    for side in ['left','right']:
        b,a=before[side],after[side];bs,as_=b['state'],a['state']
        for state in [bs,as_]:
            if state.get('read_only')is not True or state.get('server_protocol_version')!=10 or state.get('robot_mode_name')!='idle'or any(abs(v)>.001 for v in state['dq']+state['dq_d']):raise RuntimeError('Actual protocol10 idle SDK exposure bracket required')
            if np.max(abs(np.asarray(state['q'])-state['q_d']))>.005:raise RuntimeError('Actual arm tracking drift during Hand association')
        times=[utc_stamp(v[k])for v in [b,a]for k in ['read_started_utc','read_returned_utc']]
        if not(times[0]<=times[1]<=capture and capture.timestamp()+exposure<=times[2].timestamp()<=times[3].timestamp()):raise RuntimeError('Camera exposure must lie between actual stopped SDK reads')
        if max(abs((capture-v).total_seconds())for v in times)>opts['max_sdk_capture_gap_s']:raise RuntimeError('Actual SDK/camera bracket exceeds declared synchronization budget')
        if (not np.isfinite(bs.get('robot_time_s',np.nan))or not np.isfinite(as_.get('robot_time_s',np.nan))or as_['robot_time_s']<=bs['robot_time_s']):raise RuntimeError('Actual SDK arm clock must advance across this exposure')
        if np.max(abs(np.asarray(bs['q'])-as_['q']))>1e-5 or np.max(abs(np.asarray(bs['q_d'])-as_['q_d']))>1e-6:raise RuntimeError('Actual arm moved across Hand association exposure')
        F=rigid(np.asarray(bs['F_T_EE_column_major']).reshape(4,4,order='F'))
        if not np.allclose(F.ravel(order='F'),as_['F_T_EE_column_major'],atol=1e-12,rtol=0):raise RuntimeError('Actual SDK TCP changed across camera exposure')
        tcp=rigid(robots[side]['T_cell_base'])@rigid(np.asarray(bs['O_T_EE_column_major']).reshape(4,4,order='F'))
        tcp2=rigid(robots[side]['T_cell_base'])@rigid(np.asarray(as_['O_T_EE_column_major']).reshape(4,4,order='F'))
        angle=np.arccos(np.clip((np.trace(tcp2[:3,:3]@tcp[:3,:3].T)-1)/2,-1,1));motion=np.linalg.norm(tcp2[:3,3]-tcp[:3,3])+.15*angle
        if motion>opts['pose_uncertainty_m']:raise RuntimeError('Observed SDK pose drift exceeds declared Hand surface uncertainty')
        width=sample(b['hand'],now=utc_stamp(b['read_returned_utc']))
        width2=sample(a['hand'],now=utc_stamp(a['read_returned_utc']))
        if (a['hand']['robot_time_s']<b['hand']['robot_time_s']or abs(width-width2)>.0001):raise RuntimeError('Actual Hand aperture changed across camera exposure')
        states.append(dict(side=side,capture_id=metadata['capture_id'],T_cell_tcp=tcp.tolist(),F_T_EE=F.tolist(),T_flange_hand=mounts[side]['T_flange_hand'],width_m=width,
            captured_at_utc=b['read_returned_utc'],robot_time_s=bs['robot_time_s'],source='actual_sdk_fk',pose_uncertainty_m=opts['pose_uncertainty_m'],stopped_under_capture_lock=True))
    return dict(schema_version=1,before=copy.deepcopy(before),after=copy.deepcopy(after),options=opts,actual_states=states,sampled_pose_only=True,physical_unsampled_motion_certified=False)


def associate(snapshot,packet,metadata,record):
    from .known_hand_surface import associate_hand_pixels,planner_boxes_with_hand_association
    opts=settings(record['options']);association=associate_hand_pixels(packet,metadata,record['actual_states'],surface_tolerance_m=opts['surface_tolerance_m'],max_sdk_gap_s=opts['max_sdk_capture_gap_s'])
    material=None
    if opts.get('robot_material_prior')is not None or opts.get('collision_envelope_prior')is not None:
        from .native_hand_solid import material_owner
        is_envelope=opts.get('collision_envelope_prior')is not None
        prior=opts['collision_envelope_prior'if is_envelope else'robot_material_prior']
        material=material_owner(snapshot,record['actual_states'],prior['geometry_uncertainty_m'],collision_envelope=is_envelope)
    rows,proof=planner_boxes_with_hand_association(snapshot,packet,metadata,association,material=material)
    return rows,proof


def planner_boxes(snapshot,observation,config,site_config,root,dependencies):
    """Independently rebuild same-body labels from immutable source/SDK bytes."""
    from pathlib import Path
    from .random_planner import digest,canonical,load
    record=observation.get('known_hand_surface');options=config['scene'].get('known_hand_surface')
    if record is None:return snapshot.planner_boxes(),None
    if options is None or settings(options)!=record.get('options'):raise ValueError('Observed Hand surface bounds differ from configured actual field bounds')
    mounts={}
    for side in ['left','right']:
        env=config['robots'][side];F=rigid(np.asarray(env['F_T_EE_column_major']).reshape(4,4,order='F'))
        expected_mount=F@rigid(env['native_hand_aperture_geometry']['T_tcp_hand'])
        recorded_mount=record['actual_states'][['left','right'].index(side)]['T_flange_hand']
        if not np.allclose(rigid(recorded_mount),expected_mount,atol=1e-10,rtol=0):raise ValueError('Observed Hand mount differs from bound installed model')
        mounts[side]=dict(T_flange_hand=recorded_mount)
        for endpoint in ['before','after']:
            if not np.allclose(record[endpoint][side]['state']['F_T_EE_column_major'],env['F_T_EE_column_major'],atol=1e-12,rtol=0):raise ValueError('Observed actual SDK TCP changed from bound installed Hand model')
    rebuilt=bind_states(record['before'],record['after'],observation['source_metadata'],site_config['robots'],mounts,options)
    if canonical(rebuilt['actual_states'])!=canonical(record['actual_states']):raise ValueError('Hand surface poses do not follow actual exposure-bracketing SDK state')
    packet_path=Path(observation['source_packet'])
    if digest(packet_path)!=observation['source_packet_sha256']:raise ValueError('Actual Hand association source bytes changed')
    paths=config.get('camera_configs')or {'default':config.get('camera_config')}
    camera=[(Path(root)/name).resolve()for name in paths.values()if isinstance(name,str)and name]
    camera=sorted(set(p for p in camera if digest(p)==observation['calibration_sha256']))
    if not camera:raise ValueError('Exact calibrated Hand association camera required')
    with np.load(packet_path,allow_pickle=False)as z:packet={k:z[k]for k in z.files}
    packet['T_cell_camera']=load(camera[0])['T_cell_camera']
    rows,proof=associate(snapshot,packet,observation['residual_occupancy_metadata'],record)
    if canonical(proof)!=canonical(record['association_proof']):raise ValueError('Observed Hand surface proof differs from independently rebuilt source evidence')
    dependencies['known_hand_camera_'+observation['capture_id']]=dict(path=str(camera[0]),sha256=digest(camera[0]))
    return rows,proof['association_sha256']
