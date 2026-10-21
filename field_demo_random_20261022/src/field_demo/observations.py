"""Seeded CAD pose estimation and base-frame proposals. Never controls a robot."""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
from pathlib import Path
import time
from .calibration import load_config, rigid, utc_stamp


def validate_observation(metadata, config, *, replay=False, now=None, max_age_s=None,
                         previous_frame=None, static_scene_lease=None):
    """UTC is portable; monotonic counters are deliberately never cross-host clocks.

    Call again immediately before accepting a proposal for planning/execution.
    Replay relaxes age only; it never grants execution permission.
    """
    if metadata.get('units') != 'm':
        raise ValueError('Observation must explicitly use metres')
    for key in ('camera_frame', 'calibration_id'):
        if metadata.get(key) != config[key]:
            raise ValueError('Observation ' + key + ' mismatch')
    if metadata.get('point_cloud_frame') != config['camera_frame']:
        raise ValueError('Point cloud frame mismatch')
    if metadata.get('calibration_sha256') != config['_calibration_sha256']:
        raise ValueError('Calibration hash mismatch')
    if not isinstance(metadata.get('capture_id'), str) or not metadata['capture_id']:
        raise ValueError('Capture ID required')
    if not replay:
        if metadata.get('source') != 'realsense_live' or metadata.get('replay') is not False:
            raise ValueError('Live use requires explicit live sensor provenance; replay cannot authorize motion')
        if type(metadata.get('frame_number')) is not int or metadata['frame_number'] < 0:
            raise ValueError('Live frame number required')
        if not metadata.get('camera_serial'):
            raise ValueError('Live camera serial required')
        if metadata['camera_serial'] != config['camera_serial']:
            raise ValueError('Camera serial differs from measured calibration')
    current = now or datetime.now(timezone.utc)
    age = (current-utc_stamp(metadata['captured_at_utc'])).total_seconds()
    if not replay:
        limit = max_age_s if max_age_s is not None else config.get('max_observation_age_s', .5)
        ceiling=2
        if static_scene_lease is not None:
            from .static_scene_lease import validate
            limit=validate(static_scene_lease,now=current,expected_capture_id=metadata['capture_id']);ceiling=15
        if type(limit) not in (float, int) or not 0 < limit <= ceiling:
            raise ValueError('Observation age limit must be in (0, 2] seconds')
        exposure = metadata.get('exposure_s', {})
        if set(exposure) != {'rgb', 'depth'}:
            raise ValueError('Both RGB/depth exposure durations required')
        import math
        if any(type(v) not in (int, float) or not math.isfinite(v) or not 0 <= v <= .5 for v in exposure.values()):
            raise ValueError('Invalid exposure durations')
        if age < 0 or age + max(exposure.values()) > limit:
            raise RuntimeError('Observation stale or from the future; recapture before planning/motion')
    if previous_frame:
        if metadata['capture_id'] == previous_frame.get('capture_id'):
            raise RuntimeError('Repeated capture rejected')
        if metadata.get('camera_serial') == previous_frame.get('camera_serial'):
            oldstamp, stamp = previous_frame.get('hardware_timestamp_ms'), metadata.get('hardware_timestamp_ms')
            if stamp is not None and oldstamp is not None and stamp <= oldstamp:
                raise RuntimeError('Repeated or out-of-order hardware frame rejected')
    return dict(age_s=age, replay=bool(replay), static_scene_assumption=static_scene_lease is not None, original_exposure_timestamp_preserved=True, execution_allowed=False)


def target_proposals(estimates, config, metadata, *, replay=False):
    """T_base_part = inverse(T_cell_base) @ T_cell_part; no assumed grasp TCP."""
    import numpy as np
    result = {}
    for part, estimate in estimates.items():
        side = config['part_robot'][part]
        t = rigid(estimate['T_cell_part'], part + '.T_cell_part')
        base = rigid(config['robots'][side]['T_cell_base'], side+'.T_cell_base')
        result[part] = dict(robot=side, base_frame=config['robots'][side]['base_frame'],
                            T_base_part=(np.linalg.inv(base) @ t).tolist(), T_cell_part=t.tolist(),
                            source_capture_id=metadata['capture_id'], units='m',
                            grasp_transform_required='T_part_tcp measured for actual hand; object center is not TCP',
                            execution_allowed=False, replay=bool(replay))
    return result


def estimate_snapshot(snapshot, config_path, output, *, replay=False, ledger=None, parts=None, estimator=None):
    import numpy as np
    from isaac_port.fieldkit.perception import CadPoseEstimator
    started = time.monotonic()
    snapshot, output = Path(snapshot), Path(output)
    if output.exists():
        raise FileExistsError('Observation output exists; use a new path')
    c = load_config(config_path)
    metadata = json.loads(snapshot.with_suffix('.json').read_text())
    validate_observation(metadata, c, replay=replay)
    with np.load(snapshot, allow_pickle=False) as z:
        points = np.asarray(z['points_camera_m'])
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError('Finite Nx3 points_camera_m required')
        labels = np.asarray(z['labels']) if 'labels' in z else None
    priors = {n: rigid(c['nominal_stock_poses'][n]) for n in ('ring1', 'ring2', 'pin')}
    estimator = estimator if estimator is not None else CadPoseEstimator()
    selected = tuple(parts or ('ring1', 'ring2', 'pin'))
    if not selected or len(set(selected)) != len(selected) or any(n not in priors for n in selected):
        raise ValueError('parts must be a nonempty unique subset of ring1/ring2/pin')
    calibration = rigid(c['T_cell_camera'])
    cell_points = points@calibration[:3,:3].T+calibration[:3,3]
    if labels is not None and (labels.ndim != 1 or len(labels) != len(points)):
        raise ValueError('Labels must be a vector matching the point cloud')
    fits = {}
    for name in selected:
        prior = priors[name]
        expected = estimator.models[name]@prior[:3,:3].T+prior[:3,3]
        margin = c.get('roi_margin_m', .035)
        keep = np.all((cell_points >= expected.min(0)-[margin,margin,.006]) &
                      (cell_points <= expected.max(0)+[margin,margin,.006]), axis=1)
        if labels is not None:
            keep &= labels == ('ring1','ring2','pin').index(name)+1
        fits[name] = estimator.fit(name,cell_points[keep],prior)
    estimates = {part: fit.as_dict() for part, fit in fits.items()}
    # A long CAD fit does not magically refresh the exposure it used.
    freshness = validate_observation(metadata, c, replay=replay)
    result = dict(schema_version=1, state='seeded_cad_pose_proposals', hardware_ready=False,
                  execution_allowed=False, replay=bool(replay), units='m', output_frame=c['cell_frame'],
                  captured_at_utc=metadata['captured_at_utc'], capture_id=metadata['capture_id'],
                  calibration_id=c['calibration_id'], calibration_sha256=c['_calibration_sha256'],
                  source_metadata=metadata, snapshot_sha256=hashlib.sha256(snapshot.read_bytes()).hexdigest(),
                  parts=estimates, proposals=target_proposals(estimates, c, metadata, replay=replay),
                  freshness=freshness, elapsed_s=time.monotonic()-started,
                  limitations=['Local CAD fit near measured part seed; not initial random-bin object detection',
                               'Ring axial rotation remains a prior',
                               'Metal depth bias, calibration error and physical contact are unverified',
                               'Measured T_part_tcp, actual robot-state planning and field path review required',
                               'Revalidate source observation immediately before accepting a plan'])
    output.parent.mkdir(parents=True, exist_ok=True)
    if replay:
        # Replay never changes the live observation ledger, regardless of metadata.
        output.write_text(json.dumps(result, indent=2)+'\n')
    else:
        ledger = Path(ledger) if ledger else output.parent / '.used-camera-captures.json'
        ledger.parent.mkdir(parents=True, exist_ok=True)
        with ledger.open('a+') as stream:
            fcntl.flock(stream, fcntl.LOCK_EX)
            stream.seek(0)
            stored = stream.read()
            previous = json.loads(stored) if stored else {}
            validate_observation(metadata, c, previous_frame=previous or None)
            output.write_text(json.dumps(result, indent=2)+'\n')
            stream.seek(0)
            stream.truncate()
            json.dump(metadata, stream)
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=['estimate'])
    p.add_argument('--snapshot', type=Path, required=True)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--replay', action='store_true')
    p.add_argument('--ledger', type=Path)
    p.add_argument('--part', choices=['ring1','ring2','pin'], action='append', help='Fit selected visible part only; repeat to select several')
    a = p.parse_args(argv)
    try:
        r = estimate_snapshot(a.snapshot, a.config, a.output, replay=a.replay, ledger=a.ledger, parts=a.part)
        print(json.dumps(dict(state=r['state'], replay=r['replay'], output=str(a.output), execution_allowed=False)))
        return 0
    except (ValueError, KeyError, TypeError, OSError, RuntimeError, ImportError) as e:
        print(json.dumps(dict(state='observation_rejected', reason=str(e), execution_allowed=False)))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
