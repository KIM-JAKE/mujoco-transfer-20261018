"""First measured 1 mm / <=10% arm jog and explicit SDK STOP test.

No connection without --execute. This is an operational software-stop test,
not an emergency-stop or safety-rated stopping certification.
"""
import argparse
import csv
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import threading
import time
from datetime import datetime, timezone
from capx_autoring.isaac_port.fieldkit.site import Site, rotation, finite


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate(site, side):
    c = site.config
    if c.get('schema_version') != 1:
        raise ValueError('Unsupported site schema')
    # This bootstrap test establishes stop_tested, so never require or invent it.
    for key in ('workspace_checked', 'tool_and_payload_checked', 'single_arm_test_area_clear'):
        if c.get('commissioning', {}).get(key) is not True:
            raise ValueError('Measured operator field check required: '+key)
    calibration = site.path(c['calibration_record'])
    if not calibration.is_file() or not calibration.read_bytes().strip():
        raise ValueError('Nonempty actual calibration record required')
    row = c['robots'][side]
    ipaddress.ip_address(row['ip'])
    addresses = [r['ip'] for r in c['robots'].values() if r.get('ip')]
    if len(set(addresses)) != len(addresses):
        raise ValueError('Configured arm IPs must differ')
    rotation(c, side)
    if finite(row['tool_mass_kg']) < 0 or finite(row['load_mass_kg']) != 0:
        raise ValueError('First stop test requires configured tool and empty load (0 kg)')
    for key in ('motion_binary', 'state_binary'):
        path = site.path(c[key])
        if not path.is_file() or not os.access(path, os.X_OK):
            raise ValueError('Build the configured executable first: '+key)
    if c['grippers'][side].get('type') != 'franka_hand':
        raise ValueError('This first-test command requires native Franka Hand')
    return row


def validate_state(state, row):
    if state.get('server_protocol_version') != 10 or state.get('robot_mode_name') != 'idle':
        raise ValueError('FR3 protocol10 and actual idle SDK state required')
    for key in ('q', 'q_d', 'dq', 'dq_d'):
        values = state[key]
        if len(values) != 7 or any(not math.isfinite(float(v)) for v in values):
            raise ValueError('Invalid SDK '+key)
        if key in ('dq', 'dq_d') and max(map(abs, values)) > .001:
            raise ValueError('SDK state must be stationary')
    for key, configured in (('m_ee_kg', 'tool_mass_kg'), ('m_load_kg', 'load_mass_kg')):
        if abs(finite(state[key])-row[configured]) > .001:
            raise ValueError('Actual SDK mass differs from measured configuration')
    for key in ('O_T_EE_column_major', 'F_T_EE_column_major'):
        values = state[key]
        if len(values) != 16 or any(not math.isfinite(float(v)) for v in values):
            raise ValueError('Invalid actual SDK transform')
    return state


def read_state(site, row):
    result = subprocess.run([str(site.path(site.config['state_binary'])), row['ip']],
                            check=True, capture_output=True, text=True, timeout=10)
    return validate_state(json.loads(result.stdout), row)


def telemetry_progress(path):
    """Only complete native telemetry rows count; never substitute planned poses."""
    if not path.exists():
        return None
    text = path.read_text()
    if not text.endswith('\n'):
        text = text[:text.rfind('\n')+1]
    rows = list(csv.DictReader(text.splitlines()))
    if len(rows) < 2:
        return None
    first, last = rows[0], rows[-1]
    try:
        distance = math.sqrt(sum((float(last['O_T_EE'+str(k)])-float(first['O_T_EE'+str(k)]))**2
                                 for k in (12, 13, 14)))
        elapsed = float(last['t_s'])-float(first['t_s'])
        if not all(map(math.isfinite, (distance, elapsed))):
            return None
        return dict(observed_displacement_m=distance, observed_motion_interval_s=elapsed,
                    native_samples=len(rows), native_last_t_s=float(last['t_s']))
    except (KeyError, TypeError, ValueError):
        return None


def telemetry_tracking(path):
    if not path.exists():return None
    text=path.read_text()
    if not text.endswith('\n'):text=text[:text.rfind('\n')+1]
    rows=list(csv.DictReader(text.splitlines()))
    if len(rows)<2:return None
    try:
        values=[abs(float(row['q'+str(i)])-float(row['q_d'+str(i)]))for row in rows for i in range(1,8)]
        return max(values)if all(math.isfinite(v)for v in values)else None
    except(KeyError,TypeError,ValueError):return None


def stop_evidence(*, observed, markers, stop_sent_s, finished_s, returncode, after):
    return bool(observed and observed['observed_displacement_m'] >= .00002
                and observed['observed_motion_interval_s'] >= .3
                and {'EXPLICIT_STOP_ACCEPTED', 'SDK_STOP_RETURNED'} <= set(markers)
                and markers.index('EXPLICIT_STOP_ACCEPTED') < markers.index('SDK_STOP_RETURNED')
                and stop_sent_s is not None and 0 <= finished_s-stop_sent_s <= 1.
                and returncode != 0 and after.get('robot_mode_name') == 'idle'
                and max(map(abs, after['dq'])) <= .001
                and max(map(abs, after['dq_d'])) <= .001)


def execute(site, side, output, *, axis='z', direction=1, tracking_profile=None):
    row = validate(site, side)
    from .low_speed_profile import load_tracking_profile
    tracking_binding=load_tracking_profile(tracking_profile)if tracking_profile is not None else None
    tracking_limit=tracking_binding['joint_tracking_error_limit_rad']if tracking_binding else .02
    output.mkdir(parents=True, exist_ok=False)
    record = output/'stop-test.json'
    report = dict(state='IN_PROGRESS', stop_tested=False, hardware_motion_tested=False,
                  side=side, robot_ip=row['ip'], started_utc=datetime.now(timezone.utc).isoformat(),
                  scope='1mm <=10% commissioning jog; explicit SDK software STOP only',
                  hardware_execution_attempted=False,
                  joint_tracking_error_limit_rad=tracking_limit, tracking_profile=tracking_binding,
                  configuration_sha256=digest(site.filename),
                  motion_binary_sha256=digest(site.path(site.config['motion_binary'])),
                  calibration_sha256=digest(site.path(site.config['calibration_record'])))
    record.write_text(json.dumps(report, indent=2)+'\n')
    process = None
    try:
        with site.control_lock():
            site.require_ready()
            report['before'] = read_state(site, row)
            if max(abs(a-b)for a,b in zip(report['before']['q'],report['before']['q_d']))>min(.005,tracking_limit):
                raise ValueError('Actual initial SDK tracking error exceeds selected profile')
            hand = site.grippers[side].status()
            if hand.get('is_grasped') is not False:
                raise ValueError('Remove supported workpiece manually before first empty-hand test')
            xyz = [0., 0., 0.];xyz['xyz'.index(axis)] = direction*.001
            telemetry = output/'native-telemetry.csv'
            command = [str(site.path(site.config['motion_binary'])), '--execute-cartesian', row['ip'],
                       str(row['tool_mass_kg']), '0', str(site.path(site.config['calibration_record'])),
                       str(record), str(telemetry), *map(str, xyz), '4']
            if tracking_binding is not None:command += ['--tracking-error-rad',str(tracking_limit)]
            lines = queue.Queue()
            with (output/'native-stderr.txt').open('w') as errors:
                process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                           stderr=errors, text=True, bufsize=1)
                site.process = process
                report['hardware_execution_attempted']=True
                reader = threading.Thread(target=lambda: [lines.put(line.strip()) for line in process.stdout], daemon=True)
                reader.start()
                began = time.monotonic();last_ready = began;armed=False;stop_sent=None;observed=None;markers=[]
                try:
                    while process.poll() is None:
                        now=time.monotonic()
                        while not lines.empty():
                            line=lines.get_nowait();markers.append(line)
                            if line=='HEARTBEAT_REQUIRED':armed=True
                        if now-began > 20:
                            raise TimeoutError('Native stop-test deadline exceeded')
                        if stop_sent is None:
                            site.require_ready()
                            if armed:
                                current=telemetry_progress(telemetry)
                                if current and current['observed_displacement_m'] >= .00002 and current['observed_motion_interval_s'] >= .3:
                                    observed=current;process.stdin.write('STOP\n');process.stdin.flush();stop_sent=time.monotonic()
                                    report.update(hardware_motion_tested=True, observed_motion=observed,
                                                  stop_sent_utc=datetime.now(timezone.utc).isoformat())
                                    site.event('commissioning_explicit_STOP', **observed)
                                elif now-last_ready >= .03:
                                    process.stdin.write('READY\n');process.stdin.flush();last_ready=now
                        elif now-stop_sent > 1:
                            raise TimeoutError('SDK STOP did not return within1s')
                        time.sleep(.01)
                    finished=time.monotonic();reader.join(timeout=1)
                    while not lines.empty():markers.append(lines.get_nowait())
                    report.update(after=read_state(site, row), observed_motion=observed, native_markers=markers,
                                  stop_to_process_exit_upper_bound_s=None if stop_sent is None else finished-stop_sent,
                                  native_returncode=process.returncode,
                                  telemetry_sha256=digest(telemetry) if telemetry.exists() else None,
                                  hardware_motion_tested=observed is not None)
                    passed=stop_evidence(observed=observed, markers=markers, stop_sent_s=stop_sent,
                                         finished_s=finished, returncode=process.returncode, after=report['after'])
                    report['observed_max_tracking_error_rad']=telemetry_tracking(telemetry)
                    if tracking_binding is not None:
                        passed=passed and report['observed_max_tracking_error_rad']is not None and report['observed_max_tracking_error_rad']<=tracking_limit
                        passed=passed and max(abs(a-b)for a,b in zip(report['after']['q'],report['after']['q_d']))<=min(.005,tracking_limit)
                    report.update(state='PASS' if passed else 'FAIL', stop_tested=passed)
                finally:
                    if process.poll() is None:
                        site.stop()
                        try:process.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            process.terminate()
                            try:process.wait(timeout=3)
                            except subprocess.TimeoutExpired:process.kill();process.wait()
                    site.process=None
                    for stream in (process.stdin, process.stdout):stream.close()
    except BaseException as error:
        report.update(state='FAIL', stop_tested=False, reason=str(error))
        raise
    finally:
        record.write_text(json.dumps(report, indent=2)+'\n')
    return report, record


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--side', choices=('left','right'), required=True)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--axis', choices=('x','y','z'), default='z', help='Clear direction in robot BASE frame,1mm only')
    parser.add_argument('--direction', type=int, choices=(-1,1), default=1)
    parser.add_argument('--tracking-profile',type=Path,help='Same motion JSON used by planner/CSV/Hand monitor; earlier tracking abort, no prior PASS required')
    parser.add_argument('--execute', action='store_true')
    args=parser.parse_args(argv)
    output=(args.output_dir or args.config.resolve().parent/'logs'/('first-stop-'+args.side+'-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))).resolve()
    site=Site(args.config, output.parent, sides=(args.side,))
    validate(site,args.side)
    if args.tracking_profile is not None:
        from .low_speed_profile import load_tracking_profile
        load_tracking_profile(args.tracking_profile)
    if not args.execute:
        print(json.dumps(dict(state='configuration_validated', robot_connected=False, stop_tested=False,
                              next='After clearing a1mm base-frame jog, repeat with --execute'), indent=2));return 0
    report,record=execute(site,args.side,output,axis=args.axis,direction=args.direction,tracking_profile=args.tracking_profile)
    print(json.dumps(dict(**report, record=str(record)),indent=2))
    return 0 if report['stop_tested'] else 2


if __name__=='__main__':raise SystemExit(main())
