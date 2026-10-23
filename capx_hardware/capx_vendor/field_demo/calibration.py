"""Measured, named, metre-based camera/cell calibration. Never moves hardware."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path


def utc_stamp(value):
    stamp = datetime.fromisoformat(value.replace('Z', '+00:00'))
    if stamp.tzinfo is None:
        raise ValueError('Timestamp must include timezone')
    return stamp.astimezone(timezone.utc)


def rigid(value, name='transform'):
    import numpy as np
    t = np.asarray(value, dtype=float)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError(name + ': finite 4x4 matrix required')
    if not np.allclose(t[3], [0, 0, 0, 1], atol=1e-9):
        raise ValueError(name + ': invalid homogeneous last row')
    r = t[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-6) or abs(np.linalg.det(r)-1) > 1e-6:
        raise ValueError(name + ': proper orthonormal rotation required')
    if np.linalg.norm(t[:3, 3]) > 5:
        raise ValueError(name + ': translation exceeds 5 m; check mm/m units')
    return t


def load_config(path):
    path = Path(path).resolve()
    c = json.loads(path.read_text())
    if c.get('schema_version') != 1 or c.get('units') != 'm':
        raise ValueError('schema_version 1 and explicit units m required')
    for key in ('calibration_id', 'camera_frame', 'cell_frame', 'camera_serial'):
        if not isinstance(c.get(key), str) or not c[key].strip():
            raise ValueError('Named ' + key + ' required')
    if c['camera_frame'] == c['cell_frame']:
        raise ValueError('Camera optical frame and cell frame must be distinct')
    record = c.get('measured', {})
    if record.get('verified') is not True:
        raise ValueError('Measured calibration required; template values cannot enable capture')
    for key in ('by', 'method', 'evidence_file', 'at_utc'):
        if not isinstance(record.get(key), str) or not record[key].strip():
            raise ValueError('Measured provenance required: ' + key)
    if utc_stamp(record['at_utc']) > datetime.now(timezone.utc):
        raise ValueError('Measurement timestamp is in the future')
    evidence = (path.parent / record['evidence_file']).resolve()
    if not evidence.is_file() or not evidence.read_bytes().strip():
        raise ValueError('Nonempty measured calibration evidence_file required')
    if type(record.get('correspondences')) is not int or record['correspondences'] < 3:
        raise ValueError('At least 3 measured calibration correspondences required')
    import math
    residual = record.get('residual_m')
    if type(residual) not in (float, int) or not math.isfinite(residual) or residual < 0 or residual > .01:
        raise ValueError('Measured residual_m in [0, 0.01] required; assess task tolerance separately')
    rigid(c.get('T_cell_camera'), 'T_cell_camera')
    frames = {c['camera_frame'], c['cell_frame']}
    for side in ('left', 'right'):
        row = c.get('robots', {}).get(side, {})
        if not isinstance(row.get('base_frame'), str) or not row['base_frame'].strip() or row['base_frame'] in frames:
            raise ValueError('Distinct named base_frame required: ' + side)
        frames.add(row['base_frame'])
        rigid(row.get('T_cell_base'), side + '.T_cell_base')
    for part in ('ring1', 'ring2', 'pin'):
        rigid(c.get('nominal_stock_poses', {}).get(part), part + '.nominal_stock_pose')
        if c.get('part_robot', {}).get(part) not in ('left', 'right'):
            raise ValueError('part_robot must map each part to left/right')
    if type(c.get('pin_holder_tracks_pin')) is not bool:
        raise ValueError('Explicit pin_holder_tracks_pin boolean required')
    c['_calibration_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    c['_evidence_sha256'] = hashlib.sha256(evidence.read_bytes()).hexdigest()
    return c


def fit_correspondences(data, *, source_frame='camera', target_frame='cell', max_residual_m=.003):
    """Least-squares proper rigid fit p_target = T_target_source @ p_source.

    Camera aliases permit an unambiguous camera-to-cell default input. At least
    three noncollinear corresponding measured points are needed. This is a fit,
    not field verification of the resulting frame assignment or task accuracy.
    """
    import numpy as np
    if data.get('units') != 'm':
        raise ValueError('Correspondences must explicitly use units m')
    if not source_frame or not target_frame or source_frame == target_frame:
        raise ValueError('Distinct source and target frame names required')
    if not 0 < max_residual_m <= .01:
        raise ValueError('Residual limit must be in (0, 0.01] m')
    source = np.asarray(data.get('points_source_m', data.get('points_camera_m')), dtype=float)
    target = np.asarray(data.get('points_target_m', data.get('points_cell_m')), dtype=float)
    if source.ndim != 2 or source.shape[1] != 3 or target.shape != source.shape or len(source) < 3:
        raise ValueError('At least 3 corresponding Nx3 measured points required')
    if not np.isfinite(source).all() or not np.isfinite(target).all():
        raise ValueError('Measured points must be finite')
    if np.max(np.linalg.norm(source, axis=1)) > 5 or np.max(np.linalg.norm(target, axis=1)) > 5:
        raise ValueError('Coordinates exceed 5 m; check mm/m units')
    a, b = source-source.mean(0), target-target.mean(0)
    for cloud in (a, b):
        s = np.linalg.svd(cloud, compute_uv=False)
        if s[1] < .001 or s[1] / max(s[0], 1e-12) < .001:
            raise ValueError('Calibration points collinear/degenerate or under 1 mm span')
    u, _, vt = np.linalg.svd(a.T @ b)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    t = np.eye(4)
    t[:3,:3], t[:3,3] = rotation, target.mean(0)-rotation@source.mean(0)
    rigid(t)
    errors = np.linalg.norm(source@rotation.T+t[:3,3]-target, axis=1)
    if np.max(errors) > max_residual_m:
        raise ValueError('Rigid fit residual exceeds limit; check correspondences, units and reflection')
    result = dict(state='rigid_fit_requires_field_verification', units='m', source_frame=source_frame,
                  target_frame=target_frame, direction='p_target = T_target_source @ p_source',
                  T_target_source=t.tolist(), residual_m=float(np.sqrt(np.mean(errors**2))),
                  maximum_residual_m=float(errors.max()), point_residuals_m=errors.tolist(),
                  correspondences=len(source), measured_verified=False, hardware_ready=False)
    if source_frame == 'camera' and target_frame == 'cell':
        result['T_cell_camera'] = t.tolist()
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=['validate', 'fit'])
    p.add_argument('--config', type=Path)
    p.add_argument('--correspondences', type=Path)
    p.add_argument('--output', type=Path)
    p.add_argument('--source-frame', default='camera')
    p.add_argument('--target-frame', default='cell')
    p.add_argument('--max-residual-m', type=float, default=.003)
    a = p.parse_args(argv)
    try:
        if a.command == 'fit':
            if not a.correspondences or not a.output:
                raise ValueError('fit requires --correspondences and --output')
            if a.output.exists():
                raise FileExistsError('Calibration fit output exists; use a new path')
            result = fit_correspondences(json.loads(a.correspondences.read_text()), source_frame=a.source_frame,
                                         target_frame=a.target_frame, max_residual_m=a.max_residual_m)
            result['correspondences_sha256'] = hashlib.sha256(a.correspondences.read_bytes()).hexdigest()
            a.output.parent.mkdir(parents=True, exist_ok=True)
            a.output.write_text(json.dumps(result, indent=2)+'\n')
            print(json.dumps(dict(state=result['state'], output=str(a.output), hardware_ready=False)))
            return 0
        if not a.config:
            raise ValueError('validate requires --config')
        c = load_config(a.config)
        print(json.dumps(dict(state='calibration_validated', calibration_id=c['calibration_id'], hardware_ready=False)))
        return 0
    except (ValueError, KeyError, TypeError, OSError, ImportError) as e:
        print(json.dumps(dict(state='calibration_rejected', reason=str(e), hardware_ready=False)))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
