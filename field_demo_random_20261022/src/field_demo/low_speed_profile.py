"""Opt-in5mrad tracking-stop commissioning profile; never claims commissioning.

One planner JSON supplies the threshold to CSV execution, stopped-Hand SDK
monitoring and the first1mm Cartesian STOP test. Main20mrad files are untouched.
"""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_tracking_profile(path):
    path = Path(path).resolve()
    value = json.loads(path.read_text())
    guard = value.get('joint_tracking_error_limit_rad')
    if value.get('schema_version') != 2 or value.get('units') != 'm' or type(guard) not in (int, float) or not .0001 <= guard <= .02:
        raise ValueError('Metric schema2 motion profile with bounded tracking threshold required')
    return dict(path=str(path), sha256=digest(path), joint_tracking_error_limit_rad=float(guard))


def prepare(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    if source.parent != output.parent or source == output or output.exists():
        raise ValueError('Use a NEW profile beside the original motion config; relative inputs must stay valid')
    value = json.loads(source.read_text())
    if value.get('schema_version') != 2 or value.get('units') != 'm':
        raise ValueError('Existing metric schema2 motion config required')
    value['joint_tracking_error_limit_rad'] = .005
    value['tracking_guard_measurement'] = None
    value['low_speed_commissioning'] = dict(schema_version=1,
        source_config=source.name, source_sha256=digest(source),
        scope='Explicit <=10% commissioning only;5mrad means earlier abort, not verified tracking accuracy',
        hardware_tested=False, native_fraction=.1)
    output.write_text(json.dumps(value, indent=2)+'\n')
    return dict(profile=str(output), sha256=digest(output), hardware_tested=False,
                joint_tracking_error_limit_rad=.005, actual_two_arm_first_stop_required=True)


def activate(profile, records, output):
    profile, output = Path(profile).resolve(), Path(output).resolve()
    binding = load_tracking_profile(profile)
    value = json.loads(profile.read_text())
    if binding['joint_tracking_error_limit_rad'] != .005 or value.get('low_speed_commissioning', {}).get('native_fraction') != .1:
        raise ValueError('Explicit5mrad low-speed commissioning profile required')
    if set(records) != {'left', 'right'} or output.parent != profile.parent or output == profile or output.exists():
        raise ValueError('Both real arm records and a NEW ready profile beside its source required')
    site_path=(profile.parent/value['site_config']).resolve()
    site=json.loads(site_path.read_text())
    motion_path=(site_path.parent/site['motion_binary']).resolve()
    calibration_path=(site_path.parent/site['calibration_record']).resolve()
    proof = {}
    for side, path in records.items():
        path = Path(path).resolve(); report = json.loads(path.read_text())
        if (report.get('side') != side or report.get('state') != 'PASS' or
            report.get('stop_tested') is not True or report.get('hardware_motion_tested') is not True or
            report.get('hardware_execution_attempted') is not True or
            report.get('tracking_profile') != binding or
            report.get('joint_tracking_error_limit_rad') != .005 or
            type(report.get('observed_max_tracking_error_rad')) not in (int, float) or
            not 0 <= report['observed_max_tracking_error_rad'] <= .005):
            raise ValueError('Actual matching5mrad1mm/STOP PASS record required: '+side)
        telemetry=path.parent/'native-telemetry.csv'
        from .commissioning_test import telemetry_tracking
        if (report.get('configuration_sha256')!=digest(site_path)or
            report.get('motion_binary_sha256')!=digest(motion_path)or
            report.get('calibration_sha256')!=digest(calibration_path)or
            report.get('telemetry_sha256')!=digest(telemetry)or
            telemetry_tracking(telemetry)!=report['observed_max_tracking_error_rad']or
            report.get('robot_ip')!=site['robots'][side]['ip']):
            raise ValueError('Actual STOP record/source/config/telemetry binding changed: '+side)
        proof[side] = dict(path=str(path), sha256=digest(path), robot_ip=report['robot_ip'],
                          observed_max_tracking_error_rad=report['observed_max_tracking_error_rad'])
    if proof['left']['robot_ip'] == proof['right']['robot_ip']:
        raise ValueError('Two distinct actual robot IP records required')
    evidence = output.with_suffix('.tracking-evidence.json')
    if evidence.exists():raise FileExistsError('Tracking evidence already exists')
    now = datetime.now(timezone.utc).isoformat()
    evidence.write_text(json.dumps(dict(schema_version=1, profile=binding, records=proof,
        scope='Actual software STOP/5mrad input-bound commissioning evidence; not a safety-rated or full task certificate',
        at_utc=now), indent=2)+'\n')
    value['tracking_guard_measurement'] = dict(verified=True, by='Actual two-arm commissioning-test PASS records',
        at_utc=now, evidence_file=evidence.name)
    value['low_speed_commissioning']['actual_tracking_stop_records'] = proof
    output.write_text(json.dumps(value, indent=2)+'\n')
    return dict(profile=str(output), joint_tracking_error_limit_rad=.005, full_task_hardware_tested=False)


def make_cycle(source, profile, output):
    source,profile,output=map(lambda p:Path(p).resolve(),(source,profile,output))
    binding=load_tracking_profile(profile);settings=json.loads(profile.read_text())
    if binding['joint_tracking_error_limit_rad']!=.005 or not settings.get('tracking_guard_measurement'):
        raise ValueError('Use the separate ready5mrad profile after actual two-arm STOP records')
    if source.parent!=profile.parent or output.parent!=profile.parent or output.exists():
        raise ValueError('Use a NEW cycle recipe beside the ready profile and original recipe')
    recipe=json.loads(source.read_text());recipe['planner_config']=profile.name
    recipe['pickup_search_max_plans']=24
    recipe['pickup_ik_seed_offsets_rad']=[[0.]*7,[0.,0.,.8,0.,0.,0.,0.],[0.,0.,-.8,0.,0.,0.,0.]]
    output.write_text(json.dumps(recipe,indent=2)+'\n')
    return dict(recipe=str(output),profile=str(profile),maximum_read_only_pickup_plans=24,hardware_task_tested=False)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    prep=sub.add_parser('prepare');prep.add_argument('--source',type=Path,required=True);prep.add_argument('--output',type=Path,required=True)
    ready=sub.add_parser('activate');ready.add_argument('--profile',type=Path,required=True);ready.add_argument('--left-record',type=Path,required=True);ready.add_argument('--right-record',type=Path,required=True);ready.add_argument('--output',type=Path,required=True)
    cycle=sub.add_parser('make-cycle');cycle.add_argument('--source',type=Path,required=True);cycle.add_argument('--profile',type=Path,required=True);cycle.add_argument('--output',type=Path,required=True)
    a=p.parse_args(argv)
    if a.command=='prepare':result=prepare(a.source,a.output)
    elif a.command=='activate':result=activate(a.profile,dict(left=a.left_record,right=a.right_record),a.output)
    else:result=make_cycle(a.source,a.profile,a.output)
    print(json.dumps(result,indent=2));return 0


if __name__=='__main__':raise SystemExit(main())
