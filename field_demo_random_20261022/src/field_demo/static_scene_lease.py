"""Short-lived stationary-stock assumption after strictly checked empty approach.

Never changes camera exposure time or claims physical contact absence from
wrench alone. Any authorized stock interaction invalidates the runtime lease.
"""
from datetime import datetime,timezone,timedelta
from pathlib import Path
import hashlib,json,copy,csv,os,uuid
import numpy as np
from .calibration import utc_stamp,rigid


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def canonical(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def options(value):
    if not isinstance(value,dict):raise ValueError('Explicit stationary-stock map age assumption required')
    age=value.get('maximum_scene_age_s')
    if type(age)not in(int,float)or not np.isfinite(age)or not 0<age<=15:raise ValueError('Stationary stock map age must be explicit and<=15s')
    return dict(maximum_scene_age_s=float(age))

def telemetry_evidence(path,result,tracking_limit):
    count=0;last_robot=None;last_t=None;peak_force=0.;peak_moment=0.;peak_tracking=0.
    with Path(path).open()as stream:
      for row in csv.DictReader(stream):
        values={k:float(row[k])for k in ['index','t_s','robot_s','io_ready','abort_requested','current_errors',*['O_F_ext_hat_K'+str(i)for i in range(6)],*['q'+str(i)for i in range(1,8)],*['q_d'+str(i)for i in range(1,8)]]}
        if not np.isfinite(list(values.values())).all()or values['index']!=count or (last_robot is not None and values['robot_s']<=last_robot)or (last_t is not None and values['t_s']<=last_t):raise RuntimeError('Actual strict-approach SDK telemetry is incomplete/nonmonotonic')
        if values['io_ready']!=1 or values['abort_requested']!=0 or values['current_errors']!=0:raise RuntimeError('Strict approach had an actual SDK/interlock event')
        peak_force=max(peak_force,float(np.linalg.norm([values['O_F_ext_hat_K'+str(i)]for i in range(3)])));peak_moment=max(peak_moment,float(np.linalg.norm([values['O_F_ext_hat_K'+str(i)]for i in range(3,6)])))
        peak_tracking=max(peak_tracking,max(abs(values['q'+str(i)]-values['q_d'+str(i)])for i in range(1,8)));count+=1;last_robot=values['robot_s'];last_t=values['t_s']
    if count<2 or count!=result['samples']or peak_force>5 or peak_moment>1 or peak_tracking>tracking_limit:raise RuntimeError('Actual approach telemetry failed native force/tracking/count bounds')
    return dict(samples=count,peak_force_n=peak_force,peak_moment_nm=peak_moment,peak_tracking_error_rad=peak_tracking,physical_no_contact_certified=False)


def create(observation,plan,result,config,*,now=None,validity_file=None):
    now=now or datetime.now(timezone.utc);cfg=options(config['scene']['static_scene_lease']);proof=plan['proof'];row=plan['stage_row']
    if plan['phase']!='pickup'or proof.get('contact')!='strict_empty_approach'or proof.get('held_parts')or proof.get('test_only')is not False:raise ValueError('Only strictly checked EMPTY pickup approach can establish an untouched stock map')
    if result.get('stage_completed')is not True or result.get('telemetry_dropped')!=0:raise RuntimeError('Actual complete SDK approach and undropped telemetry required')
    if row['dynamic_observation_file']!=observation['_path']or row['dynamic_observation_sha256']!=observation['_sha256']:raise ValueError('Lease must use exact last pre-motion scene revalidation')
    if not result.get('actual_telemetry_file')or not result.get('actual_telemetry_sha256'):raise RuntimeError('Actual native SDK telemetry file is missing; cannot lease untouched stock scene')
    source=Path(observation['_path']);telemetry=Path(result['actual_telemetry_file']);driver=Path(result['actual_driver_file'])
    if sha(source)!=observation['_sha256']or sha(telemetry)!=result['actual_telemetry_sha256']or sha(driver)!=result['actual_driver_sha256']:raise ValueError('Actual source/SDK/driver evidence changed')
    if str(driver.resolve())!=proof['dependencies']['native_motion']['path']or sha(driver)!=proof['dependencies']['native_motion']['sha256']:raise ValueError('Actual executor differs from strict trajectory proof')
    for hand in (proof.get('aperture_binding')or{}).get('samples',{}).values():
        if hand.get('is_grasped')is not False:raise RuntimeError('Strict empty approach cannot lease stock while a Hand holds an unknown object')
    sdk=telemetry_evidence(telemetry,result,config.get('joint_tracking_error_limit_rad',.02))
    expires=utc_stamp(observation['captured_at_utc'])+timedelta(seconds=cfg['maximum_scene_age_s'])
    if not now<expires:raise RuntimeError('Untouched stock scene expired during strict approach; reacquire an unoccluded view')
    if validity_file is None:raise ValueError('Revocable runtime lease-validity file required')
    validity_file=Path(validity_file).resolve()
    if validity_file.exists():raise ValueError('Do not reuse an existing scene lease validity record')
    lease=dict(schema_version=2,kind='stationary_stock_scene_assumption',validity_file=str(validity_file),validity_nonce=uuid.uuid4().hex,capture_id=observation['capture_id'],captured_at_utc=observation['captured_at_utc'],maximum_scene_age_s=cfg['maximum_scene_age_s'],expires_at_utc=expires.isoformat(),
        source_observation_file=str(source),source_observation_sha256=sha(source),source_packet_sha256=observation['source_packet_sha256'],calibration_sha256=observation['calibration_sha256'],
        selected_part=observation['selected_part'],selected_instance=copy.deepcopy(observation['selected_instance']),
        exact_approach_csv=plan['csv'],exact_approach_csv_sha256=plan['csv_sha256'],dynamic_proof_file=row['dynamic_proof_file'],dynamic_proof_sha256=row['dynamic_proof_sha256'],
        actual_telemetry_file=str(telemetry),actual_telemetry_sha256=sha(telemetry),actual_driver_file=str(driver),actual_driver_sha256=sha(driver),actual_stage_completed_at_utc=result['actual_completed_at_utc'],
        actual_sdk_telemetry=sdk,explicit_stock_interaction_occurred=False,physical_no_contact_certified=False,assumption='Static loose inventory under a strictly collision-checked empty approach and actual SDK force/tracking guards; no independently sensed no-contact certificate')
    lease['record_sha256']=canonical(lease)
    _write_validity(validity_file,dict(active=True,nonce=lease['validity_nonce'],lease_record_sha256=lease['record_sha256']))
    return lease

def validate(lease,*,now=None,expected_capture_id=None,expected_packet_sha256=None,maximum_age_s=None):
    now=now or datetime.now(timezone.utc)
    if not isinstance(lease,dict):raise ValueError('Actual strict-approach map lease required')
    raw={k:v for k,v in lease.items()if k!='record_sha256'}
    if canonical(raw)!=lease.get('record_sha256')or lease.get('schema_version')!=2 or lease.get('kind')!='stationary_stock_scene_assumption'or lease.get('explicit_stock_interaction_occurred')is not False:raise ValueError('Invalidated/modified static-stock map lease')
    state=json.loads(Path(lease['validity_file']).read_text())
    if state.get('active')is not True or state.get('nonce')!=lease['validity_nonce']or state.get('lease_record_sha256')!=lease['record_sha256']:raise RuntimeError('Static scene lease revoked or belongs to another runtime admission')
    age=options(dict(maximum_scene_age_s=lease['maximum_scene_age_s']))['maximum_scene_age_s'];captured=utc_stamp(lease['captured_at_utc']);expires=utc_stamp(lease['expires_at_utc'])
    if expires!=captured+timedelta(seconds=age)or not captured<=now<expires:raise RuntimeError('Original stock scene exposure is stale/future for its explicit map lease')
    if maximum_age_s is not None and age>maximum_age_s:raise ValueError('Scene map lease exceeds configured age assumption')
    if expected_capture_id is not None and lease['capture_id']!=expected_capture_id:raise ValueError('Static map lease belongs to another camera capture')
    if expected_packet_sha256 is not None and lease['source_packet_sha256']!=expected_packet_sha256:raise ValueError('Static map lease belongs to different actual pixels')
    for path,checksum in [('source_observation_file','source_observation_sha256'),('exact_approach_csv','exact_approach_csv_sha256'),('dynamic_proof_file','dynamic_proof_sha256'),('actual_telemetry_file','actual_telemetry_sha256'),('actual_driver_file','actual_driver_sha256')]:
        if sha(lease[path])!=lease[checksum]:raise ValueError('Static map lease evidence changed: '+path)
    proof=json.loads(Path(lease['dynamic_proof_file']).read_text())
    if proof.get('phase')!='pickup'or proof.get('contact')!='strict_empty_approach'or proof.get('held_parts'):raise ValueError('Lease approach was not a strict empty trajectory')
    return age

def target_agrees(lease,observation,config):
    if observation.get('selected_part')!=lease['selected_part']:raise RuntimeError('Contact target class changed from untouched map')
    old=rigid(lease['selected_instance']['T_cell_part']);new=rigid(observation['selected_instance']['T_cell_part'])
    angle=np.arccos(np.clip((np.trace(old[:3,:3].T@new[:3,:3])-1)/2,-1,1))
    if np.linalg.norm(old[:3,3]-new[:3,3])>config['target_position_tolerance_m']or angle>config['target_angle_tolerance_rad']:raise RuntimeError('Fresh contact target changed; untouched scene lease invalid')
    source=json.loads(Path(lease['source_observation_file']).read_text())
    for current in observation.get('instances',[]):
        if current.get('instance_id')==observation['selected_instance']['instance_id']:continue
        visible=rigid(current['T_cell_part'])
        matches=[v for v in source.get('instances',[])if v.get('part')==current.get('part')and np.linalg.norm(rigid(v['T_cell_part'])[:3,3]-visible[:3,3])<=config['target_position_tolerance_m']]
        if not matches:raise RuntimeError('Fresh visible stock contradicts untouched scene map')
    return lease['selected_instance']['instance_id']


def _write_validity(path,value):
    path=Path(path);temporary=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    with temporary.open('x')as f:
        json.dump(value,f,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
    temporary.chmod(0o600);os.replace(temporary,path)


def revoke(lease,reason):
    path=Path(lease['validity_file']);state=json.loads(path.read_text())
    if state.get('nonce')!=lease['validity_nonce']or state.get('lease_record_sha256')!=lease['record_sha256']:raise RuntimeError('Lease validity identity changed; inhibit command')
    if state.get('active')is False:return
    _write_validity(path,dict(active=False,nonce=lease['validity_nonce'],lease_record_sha256=lease['record_sha256'],reason=str(reason),revoked_at_utc=datetime.now(timezone.utc).isoformat()))


class ClosureAdmission:
    """One use of a completed stopped-Hand proof, not a reusable map token."""
    def __init__(self,proof):
        if proof.get('scope')!='actual_stopped_SDK_pose_full_commanded_Hand_aperture_sweep'or proof.get('arm_motion_issued')is not False:raise ValueError('Actual completed stopped Hand sweep proof required')
        self.proof=copy.deepcopy(proof);self.consumed=False
        path=Path(proof['observation_file'])
        if sha(path)!=proof['observation_sha256']:raise ValueError('Hand admission observation changed')
        observation=json.loads(path.read_text());self.lease=observation.get('static_scene_lease')
        if self.lease is None:raise ValueError('Scene lease missing from stopped closure admission')
        validate(self.lease)
    def consume(self):
        if self.consumed:raise RuntimeError('Stopped Hand closure admission already consumed')
        from .random_planner import load,observation_valid,scene_packets,dependencies_valid
        p=self.proof;now=datetime.now(timezone.utc)
        if sha(p['config_file'])!=p['config_sha256']or sha(p['observation_file'])!=p['observation_sha256']:raise ValueError('Admitted Hand configuration/observation changed')
        config=load(p['config_file']);observation=load(p['observation_file']);observation.update(_path=p['observation_file'],_sha256=p['observation_sha256']);deps={}
        observation_valid(observation,config,Path(p['config_file']).parent,deps,now);scene_packets(observation,config,Path(p['config_file']).parent,deps,now);dependencies_valid(p['dependencies']);dependencies_valid(deps)
        # Current SDK pose/wrench/aperture are independently rechecked by the
        # caller's ready() immediately before this one-use dispatch boundary.
        self.consumed=True;revoke(self.lease,'One-use stopped Hand closure admitted; stock contact may begin')
        return True
