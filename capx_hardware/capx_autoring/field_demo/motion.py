"""Camera target -> measured FR3 model -> reviewed candidate stage.

Planning reads hardware only with --read-hardware. It never commands a robot.
Registration requires a recorded review of the entire actual-cell path. An
endpoint IK solution is not a collision-free approach or a successful grasp.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import subprocess
import tempfile

from .calibration import load_config, rigid, utc_stamp
from .observations import validate_observation


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_new(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write('\n')


def target(observation_path, camera_path, grasp_path, part, *, live=False):
    import numpy as np
    c = load_config(camera_path)
    observation = json.loads(Path(observation_path).read_text())
    if observation.get('units') != 'm' or observation.get('calibration_sha256') != c['_calibration_sha256']:
        raise ValueError('Observation units/calibration mismatch')
    for key in ('capture_id', 'captured_at_utc'):
        if observation.get(key) != observation.get('source_metadata', {}).get(key):
            raise ValueError('Observation source '+key+' mismatch')
    if live and observation.get('replay') is not False:
        raise ValueError('Replay observations cannot plan hardware motion')
    validate_observation(observation['source_metadata'], c, replay=not live)
    p = observation['proposals'][part]
    if p.get('units') != 'm' or p.get('robot') != c['part_robot'][part]:
        raise ValueError('Part/robot observation mapping mismatch')
    if p.get('base_frame') != c['robots'][p['robot']]['base_frame']:
        raise ValueError('Proposal base frame does not match calibration')
    if p.get('source_capture_id') != observation.get('capture_id'):
        raise ValueError('Capture identity mismatch')
    if live and p.get('replay') is not False:
        raise ValueError('Replay proposal rejected')
    part_base = rigid(p['T_base_part'])
    expected = np.linalg.inv(rigid(c['robots'][p['robot']]['T_cell_base'])) @ rigid(p['T_cell_part'])
    if not np.allclose(part_base, expected, atol=1e-9):
        raise ValueError('Proposal frame transform inconsistent')
    g = json.loads(Path(grasp_path).read_text())
    if g.get('schema_version') != 1 or g.get('units') != 'm' or g.get('part') != part or g.get('robot') != p['robot']:
        raise ValueError('Measured grasp must match part, robot and units')
    record = g.get('measured', {})
    if record.get('verified') is not True or not record.get('by') or not record.get('evidence_file'):
        raise ValueError('Measured hand-to-part relation and evidence required; never assume object center is TCP')
    if utc_stamp(record['at_utc']) > datetime.now(timezone.utc):
        raise ValueError('Grasp measurement is from the future')
    evidence = Path(grasp_path).resolve().parent / record['evidence_file']
    if not evidence.is_file() or not evidence.read_bytes().strip():
        raise ValueError('Nonempty grasp measurement evidence required')
    tcp = part_base @ rigid(g['T_part_tcp'], 'T_part_tcp')
    return dict(part=part, robot=p['robot'], base_frame=p['base_frame'],
                T_base_tcp=tcp.tolist(), T_base_part=part_base.tolist(), units='m',
                calibration_sha256=c['_calibration_sha256'], grasp_sha256=digest(grasp_path),
                source_capture_id=observation['capture_id'], captured_at_utc=observation['captured_at_utc'])


def candidate(observation, camera, grasp, part, output, *, site=None, read_hardware=False):
    import numpy as np
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError('Use a new candidate output path')
    t = target(observation, camera, grasp, part, live=read_hardware)
    report = dict(schema_version=1, state='target_preview', target=t,
                  hardware_ready=False, motion_executed=False, collision_checked=False,
                  source_observation=str(Path(observation).resolve()), camera_config=str(Path(camera).resolve()),
                  grasp_config=str(Path(grasp).resolve()), robot_model_read=False,
                  requires_full_actual_cell_path_review=True)
    if read_hardware:
        if site is None:
            raise ValueError('--config required for read-only measured-model planning')
        site = Path(site).resolve()
        s = json.loads(site.read_text())
        robot = s['robots'][t['robot']]
        ip = str(ipaddress.ip_address(robot['ip']))
        c = load_config(camera)
        if not np.allclose(rigid(robot['T_cell_base']), rigid(c['robots'][t['robot']]['T_cell_base']), atol=1e-9):
            raise ValueError('Site and camera robot-base calibration differ')
        for k in ('tool_mass_kg', 'load_mass_kg'):
            if type(robot[k]) not in (float, int) or not np.isfinite(robot[k]) or robot[k] < 0:
                raise ValueError('Measured site tool/load masses required')
        root = Path(__file__).resolve().parents[2]
        binary = Path(s.get('planner_binary', root / 'build/m80_camera_plan'))
        if not binary.is_absolute():
            binary = site.parent / binary
        with tempfile.TemporaryDirectory(prefix='field-demo-target-') as temp:
            f = Path(temp) / 'target.txt'
            # Check again immediately before a bounded read-only connection.
            newest = target(observation, camera, grasp, part, live=True)
            for key in ('robot', 'calibration_sha256', 'grasp_sha256'):
                if newest[key] != t[key]:
                    raise ValueError('Planning calibration/grasp/robot changed during preparation')
            t = newest
            report['target'] = t
            f.write_text(' '.join(str(v) for row in t['T_base_tcp'] for v in row)+'\n')
            p = subprocess.run([str(binary), '--read-model', ip, str(f)], check=True,
                               capture_output=True, text=True, timeout=30)
        model = json.loads(p.stdout)
        if model.get('read_only') is not True or model.get('motion_executed') is not False or model.get('server_protocol_version') != 10:
            raise ValueError('Expected read-only measured FR3 protocol-10 model output')
        if model.get('collision_checked') is not False:
            raise ValueError('Endpoint planner must not claim collision checking')
        for key, actual in [('tool_mass_kg', 'm_ee_kg'), ('load_mass_kg', 'm_load_kg')]:
            if abs(model['state'][actual]-robot[key]) > .001:
                raise ValueError('Measured robot payload differs from site configuration')
        for key in ('q_start', 'q_goal'):
            if np.asarray(model[key]).shape != (7,) or not np.isfinite(model[key]).all():
                raise ValueError('Invalid measured model joint result')
        duration = model['duration_s']
        if type(duration) not in (float, int) or not 2 <= duration <= 60:
            raise ValueError('Invalid candidate duration')
        output.parent.mkdir(parents=True, exist_ok=True)
        path = output.with_suffix('.csv')
        if path.exists():
            raise FileExistsError(path)
        with path.open('x', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time_s']+[k+str(j) for k in ('q', 'dq', 'ddq') for j in range(1, 8)])
            w.writerow([0, *model['q_start'], *([0]*14)])
            w.writerow([duration, *model['q_goal'], *([0]*14)])
        motion_binary = Path(s['motion_binary'])
        if not motion_binary.is_absolute():
            motion_binary = site.parent / motion_binary
        try:
            subprocess.run([str(motion_binary), '--validate-csv', str(path)], check=True,
                           capture_output=True, text=True, timeout=15)
        except BaseException:
            path.unlink()
            raise
        report.update(state='unreviewed_camera_candidate', robot_model_read=True,
                      robot_ip=ip, model=model, csv=str(path), csv_sha256=digest(path),
                      site_config_sha256=digest(site), continuous_10percent_checked=True)
    write_new(output, report)
    return report


def validate_binding(binding, site_dir, *, row=None, side=None, site_config=None):
    """Called by the live runner twice, including after acquiring its cell lock.

    The observation file may be refreshed externally, but its target must still
    match the field-reviewed candidate. Missing/stale/replay inputs stop motion.
    """
    import numpy as np
    root = Path(site_dir)
    def path(name):
        p = Path(binding[name])
        return p if p.is_absolute() else root / p
    # The frame can be refreshed, but the approved plan and its target cannot.
    candidate_path, review_path = path('candidate'), path('review')
    if digest(candidate_path) != binding['candidate_sha256'] or digest(review_path) != binding['review_sha256']:
        raise ValueError('Reviewed camera candidate/review changed')
    c, review = json.loads(candidate_path.read_text()), json.loads(review_path.read_text())
    if (c.get('state') != 'unreviewed_camera_candidate' or c.get('robot_model_read') is not True or
        review.get('candidate_sha256') != binding['candidate_sha256'] or
        review.get('full_path_reviewed') is not True or review.get('checked_actual_cell') is not True):
        raise ValueError('Missing exact reviewed camera plan')
    approved_target = c['target']
    for name in ('part', 'robot', 'T_base_tcp', 'calibration_sha256', 'grasp_sha256'):
        if binding[name] != approved_target[name]:
            raise ValueError('Camera binding differs from reviewed candidate: '+name)
    if row is None or side is None or site_config is None:
        raise ValueError('Live stage context required for camera binding')
    csv_path = Path(row['csv'])
    if not csv_path.is_absolute():
        csv_path = root / csv_path
    if (side != approved_target['robot'] or row.get('robot_ip') != c['robot_ip'] or
        site_config['robots'][side]['ip'] != c['robot_ip'] or
        row.get('sha256') != c['csv_sha256'] or digest(csv_path) != c['csv_sha256'] or
        row.get('F_T_EE_column_major') != c['model']['state']['F_T_EE_column_major'] or
        row.get('review_sha256') != binding['review_sha256'] or row.get('field_path_reviewed') is not True):
        raise ValueError('Live stage no longer matches the reviewed camera candidate')
    camera_config = load_config(path('camera_config'))
    if not np.allclose(rigid(site_config['robots'][side]['T_cell_base']),
                       rigid(camera_config['robots'][side]['T_cell_base']), atol=1e-9):
        raise ValueError('Live site and camera base calibrations differ')
    part = binding['part']
    t = target(path('observation'), path('camera_config'), path('grasp_config'), part, live=True)
    if t['robot'] != binding['robot'] or t['calibration_sha256'] != binding['calibration_sha256'] or t['grasp_sha256'] != binding['grasp_sha256']:
        raise ValueError('Camera binding robot/calibration/grasp changed; replan and review')
    current = rigid(t['T_base_tcp'])
    approved = rigid(binding['T_base_tcp'])
    tol = binding.get('position_tolerance_m', .001)
    angle_tol = binding.get('angle_tolerance_rad', .01)
    if type(tol) not in (int, float) or not 0 < tol <= .001 or type(angle_tol) not in (int, float) or not 0 < angle_tol <= .01:
        raise ValueError('Camera target agreement tolerance invalid')
    angle = np.arccos(np.clip((np.trace(current[:3, :3] @ approved[:3, :3].T)-1)/2, -1, 1))
    if np.linalg.norm(current[:3, 3]-approved[:3, 3]) > tol or angle > angle_tol:
        raise RuntimeError('Fresh observed target moved outside reviewed candidate; replan')
    return t


def register(candidate_path, site_path, stage, review_path, observation, *, cycle_recipe=None):
    from capx_autoring.isaac_port.fieldkit.policy import PART_SIDES
    site_path, candidate_path = Path(site_path).resolve(), Path(candidate_path).resolve()
    c = json.loads(candidate_path.read_text())
    s = json.loads(site_path.read_text())
    review_path = Path(review_path).resolve()
    review = json.loads(review_path.read_text())
    if c.get('state') != 'unreviewed_camera_candidate' or c.get('robot_model_read') is not True:
        raise ValueError('A live measured-model candidate is required; previews cannot register')
    if review.get('candidate_sha256') != digest(candidate_path) or review.get('full_path_reviewed') is not True or review.get('checked_actual_cell') is not True or not review.get('reviewer'):
        raise ValueError('Record actual-cell full-path review, reviewer and exact candidate SHA256')
    if utc_stamp(review['at_utc']) > datetime.now(timezone.utc):
        raise ValueError('Review date is in the future')
    if digest(site_path) != c['site_config_sha256']:
        raise ValueError('Site configuration changed since planning; regenerate candidate')
    if stage != c['target']['robot']+'.grasp.'+c['target']['part']:
        raise ValueError('This bridge only registers the selected reviewed pickup stage')
    if cycle_recipe is None:
        if PART_SIDES.get(c['target']['part']) != c['target']['robot']:
            raise ValueError('Camera robot mapping differs from the field three-part policy; use an explicit cycle recipe for handover')
    else:
        recipe = json.loads(Path(cycle_recipe).read_text())
        matches = [p for p in recipe['parts'] if p['part'] == c['target']['part']]
        if len(matches) != 1 or matches[0].get('pickup_side') != c['target']['robot'] or matches[0].get('pickup_stage') != stage:
            raise ValueError('Camera pickup differs from the explicit cycle recipe')
    csv_path = Path(c['csv'])
    if digest(csv_path) != c['csv_sha256']:
        raise ValueError('Candidate trajectory changed after review')
    if s['robots'][c['target']['robot']]['ip'] != c['robot_ip']:
        raise ValueError('Robot IP changed')
    def relative(p):
        return os.path.relpath(Path(p).resolve(), site_path.parent)
    t = c['target']
    binding = dict(part=t['part'], robot=t['robot'], T_base_tcp=t['T_base_tcp'],
                   observation=relative(observation), camera_config=relative(c['camera_config']),
                   grasp_config=relative(c['grasp_config']), calibration_sha256=t['calibration_sha256'],
                   grasp_sha256=t['grasp_sha256'], position_tolerance_m=.001, angle_tolerance_rad=.01,
                   candidate=relative(candidate_path), candidate_sha256=digest(candidate_path),
                   review=relative(review_path), review_sha256=digest(review_path))
    # Registration does not waive observation freshness. Execution will reread it.
    s.setdefault('stages', {})[stage] = dict(csv=relative(csv_path), sha256=c['csv_sha256'],
        tool_type='franka_hand', robot_ip=c['robot_ip'], field_path_reviewed=True,
        F_T_EE_column_major=c['model']['state']['F_T_EE_column_major'],
        planning_source='measured_sdk_model_camera_candidate',
        review_file=relative(review_path), review_sha256=digest(review_path),
        observation_binding=binding)
    s['assembly_commissioned'] = False
    backup = site_path.with_name(site_path.name+'.before-camera-'+digest(site_path)[:12]+'.json')
    if not backup.exists():
        backup.write_bytes(site_path.read_bytes())
    temp = site_path.with_suffix('.candidate.json')
    with temp.open('x') as f:
        json.dump(s, f, indent=2, allow_nan=False);f.write('\n')
    os.replace(temp, site_path)
    return dict(state='reviewed_pickup_registered', stage=stage, assembly_commissioned=False,
                hardware_ready=False, observation_refresh_required=True)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)
    plan = sub.add_parser('plan')
    for name in ('observation', 'camera-config', 'grasp-transform', 'output'):
        plan.add_argument('--'+name, type=Path, required=True)
    plan.add_argument('--part', choices=['ring1', 'ring2', 'pin'], required=True)
    plan.add_argument('--config', type=Path)
    plan.add_argument('--read-hardware', action='store_true')
    reg = sub.add_parser('register')
    for name in ('candidate', 'config', 'review', 'observation'):
        reg.add_argument('--'+name, type=Path, required=True)
    reg.add_argument('--stage', required=True)
    reg.add_argument('--cycle-recipe', type=Path, help='Explicit pickup arm mapping; the cycle still requires full actual-cell review')
    a = p.parse_args(argv)
    try:
        if a.command == 'plan':
            r = candidate(a.observation, a.camera_config, a.grasp_transform, a.part, a.output,
                          site=a.config, read_hardware=a.read_hardware)
            print(json.dumps(dict(state=r['state'], output=str(a.output), motion_executed=False)))
        else:
            print(json.dumps(register(a.candidate, a.config, a.stage, a.review, a.observation, cycle_recipe=a.cycle_recipe)))
        return 0
    except (OSError, ValueError, KeyError, RuntimeError, TypeError, ImportError, subprocess.SubprocessError) as e:
        print(json.dumps(dict(state='camera_motion_rejected', reason=str(e), motion_executed=False)))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
