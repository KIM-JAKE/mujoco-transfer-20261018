"""Optional persistent RealSense acquisition; output is data, never motion."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import socket
import tempfile
import time
import uuid
from .calibration import load_config


def finite_sdk_cloud(vertices, rgb=None):
    """Convert librealsense structured vertices or HxWx3 arrays to finite Nx3."""
    import numpy as np
    a = np.asarray(vertices)
    if a.dtype.fields:
        names = a.dtype.names
        if len(names) != 3:
            raise ValueError('SDK vertices must have exactly three coordinates')
        a = np.stack([a[n] for n in names], axis=-1)
    a = np.asarray(a, dtype=np.float32)
    if a.size % 3:
        raise ValueError('Point cloud coordinate count is not divisible by 3')
    points = a.reshape(-1, 3)
    valid = np.isfinite(points).all(axis=1) & (points[:, 2] > .05) & (points[:, 2] < 5.)
    return points[valid].copy()


def clock_id():
    try:
        boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
        boot = 'unavailable'
    return socket.gethostname() + ':' + boot


def list_devices(sdk=None):
    """Enumerate descriptors only. Does not start streams or touch a robot."""
    if sdk is None:
        try:
            import pyrealsense2 as sdk
        except ImportError as e:
            raise RuntimeError('RealSense SDK missing. Install requirements-camera.txt on the camera PC.') from e
    devices = []
    for device in sdk.context().query_devices():
        row = {}
        for key, attribute in (('model','name'),('serial','serial_number'),('firmware','firmware_version'),('usb','usb_type_descriptor')):
            info = getattr(sdk.camera_info,attribute,None)
            row[key] = device.get_info(info) if info is not None and device.supports(info) else None
        devices.append(row)
    return dict(state='camera_devices_listed',devices=devices,count=len(devices),streams_started=False,
                robot_command_sent=False,hardware_ready=False)


class RealSenseCapture:
    def __init__(self, config, serial=None, width=640, height=480, fps=30, max_age_s=.2):
        if not 0 < max_age_s <= .5:
            raise ValueError('max_age_s must be in (0, 0.5]')
        self.config = config
        if serial is not None and serial != config['camera_serial']:
            raise ValueError('Requested camera serial differs from measured calibration')
        self.serial = serial or config['camera_serial']
        self.width, self.height, self.fps = width, height, fps
        self.max_age_s = max_age_s
        self.pipe = None
        self.last_frame = None

    def start(self):
        if self.pipe is not None:
            return self
        try:
            import pyrealsense2 as rs
        except ImportError as e:
            raise RuntimeError('RealSense SDK missing. Install requirements-camera.txt on the camera PC; replay needs only numpy/scipy.') from e
        self.rs = rs
        pipe, cfg = rs.pipeline(), rs.config()
        if self.serial:
            cfg.enable_device(self.serial)
        cfg.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)
        try:
            profile = pipe.start(cfg)
            self.pipe, self.profile = pipe, profile
            for sensor in profile.get_device().query_sensors():
                if sensor.supports(rs.option.global_time_enabled):
                    sensor.set_option(rs.option.global_time_enabled, 1)
                if sensor.supports(rs.option.frames_queue_size):
                    sensor.set_option(rs.option.frames_queue_size, 1)
            self.align, self.pc = rs.align(rs.stream.color), rs.pointcloud()
            self.scale = profile.get_device().first_depth_sensor().get_depth_scale()
            for _ in range(30):
                pipe.wait_for_frames(2000)
            self.rectifier=None
            settings=self.config.get('residual_occupancy')
            if settings is not None:
                from .camera_rectification import RealSenseRectifier
                intr=profile.get_stream(rs.stream.color).as_video_stream_profile().intrinsics
                actual_serial=profile.get_device().get_info(rs.camera_info.serial_number)
                self.rectifier=RealSenseRectifier(rs,intr,camera_serial=actual_serial,depth_uncertainty_m=settings['depth_uncertainty_m'])
        except Exception as e:
            self.close()
            raise RuntimeError('RealSense start failed: verify USB3, serial, RGB-D stream support and SDK/firmware: ' + str(e)) from e
        return self

    def read(self):
        import numpy as np
        if self.pipe is None:
            raise RuntimeError('Start persistent camera stream first')
        started = time.monotonic()
        frames = self.pipe.wait_for_frames(1000)
        for _ in range(16):
            newer = self.pipe.poll_for_frames()
            if not newer:
                break
            frames = newer
        raw = {'rgb': frames.get_color_frame(), 'depth': frames.get_depth_frame()}
        if not all(raw.values()):
            raise RuntimeError('RGB-D frames unavailable')
        now = time.time()
        timestamps, exposures, ages = {}, {}, []
        for name, frame in raw.items():
            domain = frame.get_frame_timestamp_domain()
            if domain not in (self.rs.timestamp_domain.global_time, self.rs.timestamp_domain.system_time):
                raise RuntimeError('Timestamp domain is hardware_clock; bounded exposure age cannot be established. Enable SDK global time and retry.')
            stamp = float(frame.get_timestamp())
            exposure = None
            if frame.supports_frame_metadata(self.rs.frame_metadata_value.actual_exposure):
                exposure = float(frame.get_frame_metadata(self.rs.frame_metadata_value.actual_exposure)) / 1e6
            if exposure is None or exposure < 0 or exposure > .5:
                raise RuntimeError('Camera actual exposure metadata unavailable/invalid; cannot bound observation age')
            age = now - stamp / 1000
            if age < 0 or age + exposure > self.max_age_s:
                raise RuntimeError('Stale/future camera exposure rejected: ' + name)
            timestamps[name] = dict(timestamp_ms=stamp, domain=str(domain), frame_number=int(frame.get_frame_number()))
            exposures[name] = exposure
            ages.append(age + exposure)
        identity = (timestamps['rgb']['frame_number'], timestamps['rgb']['timestamp_ms'])
        if self.last_frame is not None and (identity[0] <= self.last_frame[0] or identity[1] <= self.last_frame[1]):
            raise RuntimeError('Repeated or out-of-order RGB frame rejected')
        if abs(timestamps['rgb']['timestamp_ms']-timestamps['depth']['timestamp_ms']) > 50:
            raise RuntimeError('RGB/depth timestamps differ by over 50 ms')
        aligned = self.align.process(frames)
        depth, color = aligned.get_depth_frame(), aligned.get_color_frame()
        vertices = np.asanyarray(self.pc.calculate(depth).get_vertices())
        # Retain pixel correspondence for calibration sampling. This is the SDK
        # deprojection of aligned depth, not a distortion-free pinhole guess.
        if vertices.dtype.fields:
            pixel_points = np.stack([vertices[n] for n in vertices.dtype.names], axis=-1).astype(np.float32)
        else:
            pixel_points = np.asarray(vertices,dtype=np.float32)
        pixel_points = pixel_points.reshape(self.height,self.width,3).copy()
        points = finite_sdk_cloud(pixel_points)
        if len(points) < 150:
            raise RuntimeError('Insufficient valid measured depth points')
        elapsed = time.monotonic()-started
        if max(ages)+elapsed > self.max_age_s:
            raise RuntimeError('Acquisition completed outside exposure-age budget')
        self.last_frame = identity
        intr = color.profile.as_video_stream_profile().intrinsics
        device = self.profile.get_device()
        captured = datetime.fromtimestamp(min(x['timestamp_ms'] for x in timestamps.values())/1000, timezone.utc)
        meta = dict(schema_version=1, capture_id=str(uuid.uuid4()), source='realsense_live', replay=False,
                    captured_at_utc=captured.isoformat(), camera_frame=self.config['camera_frame'],
                    calibration_id=self.config['calibration_id'], calibration_sha256=self.config['_calibration_sha256'],
                    camera_serial=device.get_info(self.rs.camera_info.serial_number), camera_model=device.get_info(self.rs.camera_info.name),
                    units='m', point_cloud_frame=self.config['camera_frame'], aligned_to='RGB optical frame',
                    frame_number=identity[0], hardware_timestamp_ms=identity[1], timestamp_details=timestamps,
                    exposure_s=exposures, conservative_exposure_age_s=max(ages)+elapsed,
                    received_at_utc=datetime.now(timezone.utc).isoformat(), received_monotonic_s=time.monotonic(),
                    monotonic_clock_id=clock_id(), depth_scale_m=self.scale,
                    intrinsics=[[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1]],
                    distortion_model=str(intr.model), distortion_coefficients=list(intr.coeffs),
                    rectified_registered_depth=False, deprojection='SDK aligned depth vertices; no pinhole approximation',
                    robot_connected=False, hardware_ready=False)
        depth_m=np.asanyarray(depth.get_data()).astype(np.float32)*self.scale
        depth_m[depth_m<=0]=np.nan
        K=np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1.]],dtype=float)
        packet=dict(points_camera_m=points, points_camera_image_m=pixel_points,depth_m=depth_m,K=K,rgb=np.asanyarray(color.get_data()).copy())
        if getattr(self,'rectifier',None)is not None:
            packet,meta=self.rectifier.apply(packet,meta)
            meta['conservative_exposure_age_s']=max(ages)+time.monotonic()-started
            if meta['conservative_exposure_age_s']>self.max_age_s:raise RuntimeError('Rectified exposure exceeded actual age budget')
        return packet,meta

    def close(self):
        if self.pipe is not None:
            self.pipe.stop()
            self.pipe = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.close()


def save_capture(output, packet, metadata):
    import numpy as np
    output = Path(output)
    if output.suffix != '.npz':
        raise ValueError('Capture output must end in .npz')
    if output.exists() or output.with_suffix('.json').exists():
        raise FileExistsError('Capture output already exists; use a new path')
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output, **packet)
    output.with_suffix('.json').write_text(json.dumps(metadata, indent=2)+'\n')


def publish_latest(output, report):
    """One complete JSON replaces the latest status; failures contain no targets."""
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.'+output.name+'.', dir=output.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(report, stream, indent=2)
            stream.write('\n')
        os.replace(temporary, output)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def run_observer(config_path, output, session_dir, *, parts=None, frames=0,
                 stream_factory=None, estimate_fn=None, estimator_factory=None):
    """Persistent acquisition/local fit publisher. No robot connection.

    Warm estimator imports/models before camera startup. A failed attempt
    atomically replaces the latest target with a target-free rejection status.
    Immutable captures and estimates remain in the session directory.
    """
    stream_factory = stream_factory or RealSenseCapture
    output, session_dir = Path(output), Path(session_dir)
    status = lambda state, **kw: dict(state=state, hardware_ready=False,
                                     execution_allowed=False, robot_command_sent=False, **kw)
    publish_latest(output, status('observation_starting'))
    completed, rejected = 0, 0
    try:
        from .observations import estimate_snapshot
        from capx_autoring.isaac_port.fieldkit.perception import CadPoseEstimator
        estimate_fn = estimate_fn or estimate_snapshot
        estimator_factory = estimator_factory or CadPoseEstimator
        if type(frames) is not int or frames < 0:
            raise ValueError('frames must be a nonnegative integer; 0 means continuous')
        if session_dir.exists() and any(session_dir.iterdir()):
            raise FileExistsError('Use a new empty observer session directory')
        session_dir.mkdir(parents=True, exist_ok=True)
        config = load_config(config_path)
        estimator = estimator_factory()
        with stream_factory(config) as stream:
            i = 0
            while frames == 0 or i < frames:
                try:
                    packet, metadata = stream.read()
                    snapshot = session_dir / f'capture-{i:06d}.npz'
                    estimate_output = session_dir / f'observation-{i:06d}.json'
                    save_capture(snapshot, packet, metadata)
                    report = estimate_fn(snapshot, config_path, estimate_output, parts=parts,
                                         replay=False, ledger=session_dir/'used-captures.json', estimator=estimator)
                    if report.get('state') != 'seeded_cad_pose_proposals' or report.get('replay') is not False:
                        raise RuntimeError('Estimator did not return a fresh live observation proposal')
                    publish_latest(output, report)
                    completed += 1
                    print(json.dumps(status('observation_published', capture_id=metadata['capture_id'])), flush=True)
                except (ValueError, KeyError, TypeError, OSError, RuntimeError) as e:
                    rejected += 1
                    failure = status('observation_rejected', reason=str(e), attempt=i)
                    publish_latest(output, failure)
                    (session_dir/f'rejection-{i:06d}.json').write_text(json.dumps(failure,indent=2)+'\n')
                    print(json.dumps(failure), flush=True)
                i += 1
        # A bounded observer leaves the final sample for UTC-age checked reading.
        return status('observer_finished', published=completed, rejected=rejected)
    except KeyboardInterrupt:
        publish_latest(output, status('observation_stopped', reason='Operator interrupted observer'))
        return status('observer_stopped', published=completed, rejected=rejected)
    except Exception as e:
        publish_latest(output, status('observation_rejected', reason=str(e)))
        raise


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=['list','capture','observe','sample'])
    p.add_argument('--config', type=Path)
    p.add_argument('--output', type=Path)
    p.add_argument('--serial')
    p.add_argument('--count', type=int, default=1, help='Persistent stream; multiple outputs receive -000001 suffixes')
    p.add_argument('--part', choices=['ring1','ring2','pin'], action='append')
    p.add_argument('--frames', type=int, default=0, help='observe attempts; 0 runs until Ctrl-C')
    p.add_argument('--session-dir', type=Path)
    a = p.parse_args(argv)
    try:
        if a.command == 'list':
            print(json.dumps(list_devices(),indent=2))
            return 0
        if not a.output:
            raise ValueError('capture, sample and observe require --output')
        if a.command == 'observe':
            if not a.config or not a.session_dir:
                raise ValueError('observe requires --config and a new --session-dir')
            result = run_observer(a.config,a.output,a.session_dir,parts=a.part,frames=a.frames)
            print(json.dumps(result))
            return 0 if result.get('published',0) else 2
        if not 1 <= a.count <= 10000:
            raise ValueError('count must be 1..10000')
        if a.command == 'sample':
            if not a.serial:
                raise ValueError('sample requires explicit --serial; no calibrated transform is claimed')
            c = dict(camera_serial=a.serial,camera_frame='unmeasured_camera_optical',
                     calibration_id='UNMEASURED',_calibration_sha256='UNMEASURED')
        else:
            if not a.config:
                raise ValueError('capture requires --config')
            c = load_config(a.config)
        with RealSenseCapture(c, a.serial) as stream:
            for i in range(a.count):
                packet, metadata = stream.read()
                if a.command == 'sample':
                    metadata.update(source='uncalibrated_sensor_sample', calibration_verified=False,
                                    observation_eligible=False, execution_allowed=False,
                                    purpose='Raw camera points for measured correspondence collection only')
                target = a.output if a.count == 1 else a.output.with_name(a.output.stem+f'-{i:06d}.npz')
                save_capture(target, packet, metadata)
                print(json.dumps(dict(state='captured', snapshot=str(target), capture_id=metadata['capture_id'], robot_command_sent=False)), flush=True)
        return 0
    except (ValueError, KeyError, TypeError, OSError, RuntimeError, ImportError) as e:
        print(json.dumps(dict(state='capture_rejected', reason=str(e), robot_command_sent=False)))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
