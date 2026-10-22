"""Opt-in432 signed-face RGB-D observation; no robot connection or motion."""
import argparse,json,time,uuid
from datetime import datetime,timezone
from pathlib import Path
import numpy as np
from .calibration import rigid,utc_stamp
from .random_perception import load_random_config,_packet
from .fixed_native.rgbd_geometry import axis_frame
from .fixed_native.rgbd_scattered_rings import detect
from .fixed_native.capture_realsense import RealSenseStream
from isaac_port.fieldkit.perception import CadPoseEstimator

def sdk_hand_pose(config,record,side,*,replay=False):
    """Actual libfranka read-only EE FK plus measured frame conversions only."""
    state=record.get('state',record)
    if state.get('read_only')is not True:raise ValueError('Read-only public SDK state required')
    if not replay:
        age=(datetime.now(timezone.utc)-utc_stamp(record['captured_at_utc'])).total_seconds()
        if not 0<=age<=.25:raise ValueError('SDK state older than250ms or future')
    raw=np.asarray(state['O_T_EE_column_major'],float)
    if raw.shape!=(16,):raise ValueError('Public SDK O_T_EE_column_major must contain16 values')
    ee=rigid(raw.reshape(4,4,order='F'),'SDK O_T_EE')
    row=config['robots'][side];base=rigid(row['T_cell_base'],'measured robot base')
    hand=rigid(config['fixed_native']['T_EE_hand'][side],'measured SDK EE-to-nativeHand TCP')
    return base@ee@hand

def observe(packet,config,estimator,*,part,mode='initial',metadata=None,sdk_record=None,grasp=None,replay=False):
    started=time.monotonic();metadata=dict(metadata or {})
    if part not in ['ring1','ring2','pin']or mode not in ['initial','held']:raise ValueError('Known class and initial/held mode required')
    if not replay and config.get('_replay_only'):raise ValueError('Replay calibration cannot observe a live camera')
    camera=_packet(packet,config,metadata,replay);H=None;seed=None
    if mode=='held':
        if sdk_record is None or grasp is None:raise ValueError('Held ROI requires public SDK state and previous RGB-D measured grasp')
        if grasp.get('source')!='RGBD_measured_hand_to_part'or grasp.get('part')!=part or not grasp.get('capture_id'):raise ValueError('Matching earlier RGB-D measured hand-to-part record required')
        side=config['part_robot'][part];H=sdk_hand_pose(config,sdk_record,side,replay=replay);G=rigid(grasp['measured_grasp'],'previous RGB-D measured grasp');predicted=H@G;seed=predicted[:3,3]
        cal=dict(floor_origin_world_m=seed-[0,0,.06],basket_to_world_R=np.eye(3),outer_bounds_world_m=[seed-.065,seed+.065],allow_sideways_faces=True)
    else:
        b=config['basket'];cal=dict(floor_origin_world_m=b['floor_origin_cell_m'],basket_to_world_R=b['basket_to_cell_R'],outer_bounds_world_m=b['outer_bounds_cell_m'],global_plane_faces=True,fixed_supply_mode=True)
    if part=='pin':
        from .fixed_native.rgbd_fast_tracker import FastRGBDTracker
        if mode=='initial':
            if not config.get('pin_holder',{}).get('enabled'):raise ValueError('Measured physical pin-holder seed required')
            predicted=rigid(config['pin_holder']['T_cell_pin_seed']);seed=predicted[:3,3]
        pose,mask=FastRGBDTracker(estimator).estimate(camera,part,seed,predicted[:3,2],initial_pin=mode=='initial',robust_pin_cap_circle=True);poses=[pose]
    else:
        found,masks=detect(camera,cal,estimator,desired_part=part,seed=seed);poses=found['poses']
        poses=[p for p in poses if p['part']==part and p.get('front_back_from_RGBD') and p.get('front_back_orientation_margin_m',0)>.0005]
        if not poses:raise RuntimeError('No current signed annular face passed original metric/CAD/free-space gates')
        if mode=='held':
            from .fixed_native.rgbd_ring_translation import refine
            poses=[refine(camera,pose,estimator) for pose in poses]
    for pose in poses:
        T=np.eye(4);T[:3,:3]=axis_frame(pose['axis']);T[:3,3]=pose['center_m'];pose['T_cell_part']=T.tolist();pose['axial_yaw_observable']=False
    return dict(state='fixed_native_RGBD_observation',part=part,mode=mode,poses=poses,capture_id=metadata.get('capture_id',str(uuid.uuid4())),calibration_id=config['calibration_id'],camera_frame=config['camera_frame'],output_frame=config['cell_frame'],units='m',replay=bool(replay),processing_s=time.monotonic()-started,depth_sensor_input=True,simulator_body_pose_input=False,agent_called=False,robot_connected=False,robot_command_sent=False,hardware_ready=False,SDK_hand_pose_cell=H.tolist()if H is not None else None,scope='Opt-in camera observation only; reachability, contact, assembly and hardware cycle not certified')

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--camera-config',type=Path,required=True);p.add_argument('--part',choices=['ring1','ring2','pin'],required=True);p.add_argument('--mode',choices=['initial','held'],default='initial');p.add_argument('--output',type=Path,required=True);p.add_argument('--frames',type=int,default=1);p.add_argument('--packet-npz',type=Path);p.add_argument('--enable-live-camera',action='store_true');p.add_argument('--sdk-state-json',type=Path);p.add_argument('--grasp-json',type=Path);p.add_argument('--width',type=int,default=640);p.add_argument('--height',type=int,default=480);p.add_argument('--fps',type=int,default=30);a=p.parse_args(argv)
    if a.frames<1:raise ValueError('frames must be positive')
    if a.packet_npz and a.enable_live_camera:raise ValueError('Choose recorded packet or live camera')
    if not a.packet_npz and not a.enable_live_camera:raise ValueError('Live acquisition needs explicit --enable-live-camera')
    config=load_random_config(a.camera_config);estimator=CadPoseEstimator();stream=None;last_frame=None
    from .camera import publish_latest
    try:
        if not a.packet_npz:stream=RealSenseStream(a.camera_config,config['camera_serial'],a.width,a.height,a.fps).start()
        for _ in range(1 if a.packet_npz else a.frames):
            try:
                if a.packet_npz:
                    with np.load(a.packet_npz,allow_pickle=False)as z:packet={k:z[k].copy()for k in z.files}
                    metadata=dict(capture_id=str(uuid.uuid4()),rectified_registered_depth=True,source='recorded_camera_packet')
                else:
                    packet,metadata=stream.read(max_acquisition_s=.25);identity=metadata['frame_number']
                    if last_frame is not None and identity<=last_frame:raise RuntimeError('Repeated camera frame rejected')
                    last_frame=identity
                state=json.loads(a.sdk_state_json.read_text())if a.sdk_state_json else None;grasp=json.loads(a.grasp_json.read_text())if a.grasp_json else None
                result=observe(packet,config,estimator,part=a.part,mode=a.mode,metadata=metadata,sdk_record=state,grasp=grasp,replay=bool(a.packet_npz));result['camera_metadata']=metadata
            except Exception as error:result=dict(state='observation_rejected',reason=type(error).__name__+': '+str(error),robot_connected=False,robot_command_sent=False,hardware_ready=False)
            publish_latest(a.output,result);print(json.dumps(result),flush=True)
    finally:
        if stream is not None:stream.close()
    return 0
if __name__=='__main__':raise SystemExit(main())
