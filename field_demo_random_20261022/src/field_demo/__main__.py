"""One entry point for the isolated FR3 field package. Help never opens hardware."""
import argparse
import hashlib
import importlib
import json
from pathlib import Path
import sys
import subprocess

ROOT = Path(__file__).resolve().parents[2]
MODULES = {
    'common-duo-mount':'field_demo.common_duo_mount',
    'convex-self-model':'field_demo.convex_self','low-speed-profile':'field_demo.low_speed_profile','model-preset':'field_demo.model_preset','commissioning-test': 'field_demo.commissioning_test', 'doctor': 'field_demo.doctor', 'run': 'field_demo.execution',
           'camera': 'field_demo.camera', 'calibration': 'field_demo.calibration',
           'observe': 'field_demo.observations', 'motion': 'field_demo.motion',
           'handover': 'field_demo.handover',
           'preview': 'field_demo.preview',
           'random-vision': 'field_demo.random_perception',
           'random-cycle': 'field_demo.random_cycle',
           'stabilization': 'field_demo.stabilization',
           'cycle': 'field_demo.cycle', 'review': 'field_demo.review',
           'teach': 'isaac_port.fieldkit.teach', 'hand': 'isaac_port.fieldkit.hand'}


def init_site(output, left_ip=None, right_ip=None):
    import ipaddress
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    templates = ('camera.json', 'grasp.json', 'path-review.json', 'cycle.json', 'handover.json')
    paths = [output / x for x in ('site.json', *templates)]
    if any(p.exists() for p in paths):
        raise FileExistsError('Existing site files are preserved; choose a new directory or edit them explicitly')
    c = json.loads((ROOT / 'config/templates/site.json').read_text())
    for side, ip in [('left', left_ip), ('right', right_ip)]:
        c['robots'][side]['ip'] = str(ipaddress.ip_address(ip)) if ip else None
        c['grippers'][side]['binary'] = str(ROOT / 'build/m80_franka_hand')
    if left_ip and right_ip and left_ip == right_ip:
        raise ValueError('Left and right robot IPs must differ')
    for name, binary in [('motion_binary', 'm80_site_motion'), ('state_binary', 'm80_read_state'), ('planner_binary', 'm80_camera_plan')]:
        c[name] = str(ROOT / 'build' / binary)
    values = [c] + [json.loads((ROOT / 'config/templates' / name).read_text()) for name in templates]
    for p, value in zip(paths, values):
        p.write_text(json.dumps(value, indent=2)+'\n')
    for d in ('poses', 'trajectories'):
        (output / d).mkdir(exist_ok=True)
    for name in ('calibration.md', 'commissioning.md'):
        (output / name).touch(exist_ok=True)  # intentionally empty: cannot qualify as measured evidence
    return dict(state='unconfigured_site_created', directory=str(output),
                hardware_ready=False, next='Fill actual measured values; ./field-demo doctor --config '+str(paths[0]))


def verify():
    source = ROOT / 'SHA256SUMS'
    if not source.is_file():
        raise FileNotFoundError('Release checksums not generated yet')
    errors, count = [], 0
    for line in source.read_text().splitlines():
        expected, name = line.split('  ', 1)
        p = (ROOT / name).resolve()
        if not p.is_relative_to(ROOT) or not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest() != expected:
            errors.append(name)
        count += 1
    return dict(state='checksums_pass' if not errors else 'checksums_failed', checked_files=count, changed_or_missing=errors)


def init_random(output, left_ip=None, right_ip=None, *, model_installation=None, left_state=None, right_state=None, workspace=None):
    if any((model_installation,left_state,right_state)) and not all((model_installation,left_state,right_state)):
        raise ValueError('Model starter requires installation record AND both actual SDK snapshots')
    output = Path(output).resolve()
    names = ('random-camera.json', 'random-motion.json', 'random-cycle.json', 'payload-profile.json')
    if any((output / name).exists() for name in names):
        raise FileExistsError('Existing random site files are preserved; choose a new directory')
    # Read every template before creating any site files.
    configs = {name: json.loads((ROOT / 'config/templates' / name).read_text()) for name in names}
    result = init_site(output, left_ip, right_ip)
    for name, value in configs.items():
        if name == 'payload-profile.json':
            value['driver_binary']=str(ROOT / 'build/m80_payload_load')
        if name == 'random-motion.json':
            value['site_config'] = 'site.json'
            value['planner_binary'] = str(ROOT / 'build/m80_dynamic_plan')
            value['camera_configs']['basket'] = 'random-camera.json'
        if name == 'random-cycle.json':
            value.update(site_config='site.json', camera_config='random-camera.json', planner_config='random-motion.json')
            profile = ROOT / 'stabilization/v128/field-profile.json'
            if not profile.is_file():
                raise FileNotFoundError('The selected v128 release profile is missing')
            value['stabilization_profile'] = dict(path=str(profile), sha256=hashlib.sha256(profile.read_bytes()).hexdigest())
            if isinstance(value.get('camera_configs'), dict):
                value['camera_configs']['basket'] = 'random-camera.json'
        (output / name).write_text(json.dumps(value, indent=2)+'\n')
    result.update(state='unconfigured_random_site_created', next='Follow docs/RANDOM_ONSITE.md; actual measured values are required')
    if model_installation:
        from .model_preset import author
        result['model_starter']=author(output,model_installation,left_state,right_state,workspace_path=workspace)
    return result


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    if args and args[0] in MODULES:
        # Override legacy /tmp build defaults only in this relocatable wrapper.
        if not any(x == '--binary' or x.startswith('--binary=') for x in args[1:]):
            if args[0] == 'hand':
                args += ['--binary', str(ROOT / 'build/m80_franka_hand')]
            elif args[0] == 'teach' and len(args) > 1 and args[1] == 'capture':
                args += ['--binary', str(ROOT / 'build/m80_read_state')]
        try:
            return importlib.import_module(MODULES[args[0]]).main(args[1:])
        except KeyboardInterrupt:
            print('Interrupted; follow the field stop procedure.', file=sys.stderr)
            return 130
        except (OSError, ValueError, KeyError, RuntimeError, ImportError, TypeError, subprocess.SubprocessError) as e:
            print(json.dumps(dict(state='command_failed', reason=str(e), hardware_ready=False)), file=sys.stderr)
            return 2
    p = argparse.ArgumentParser(description=__doc__, epilog='Subcommand help: ./field-demo COMMAND --help. Start: START_HERE.md')
    sub = p.add_subparsers(dest='command')
    for command in MODULES:
        sub.add_parser(command, help='Use '+command+' --help for its options', add_help=False)
    init = sub.add_parser('init', help='Create unconfigured site files without overwriting existing measurements')
    init.add_argument('--output', type=Path, default=ROOT / 'site')
    init.add_argument('--left-ip')
    init.add_argument('--right-ip')
    random_init = sub.add_parser('init-random', help='Create the real-cell random-bin templates; no hardware connection')
    random_init.add_argument('--output', type=Path, default=ROOT / 'site/random')
    random_init.add_argument('--left-ip')
    random_init.add_argument('--right-ip')
    random_init.add_argument('--model-installation',type=Path)
    random_init.add_argument('--left-state',type=Path)
    random_init.add_argument('--right-state',type=Path)
    random_init.add_argument('--workspace',type=Path)
    sub.add_parser('verify', help='Check fixed release files against SHA256SUMS')
    a = p.parse_args(args)
    try:
        if a.command == 'init':
            print(json.dumps(init_site(a.output, a.left_ip, a.right_ip), indent=2));return 0
        if a.command == 'init-random':
            print(json.dumps(init_random(a.output,a.left_ip,a.right_ip,model_installation=a.model_installation,left_state=a.left_state,right_state=a.right_state,workspace=a.workspace), indent=2));return 0
        if a.command == 'verify':
            r = verify();print(json.dumps(r, indent=2));return 0 if not r['changed_or_missing'] else 2
    except (OSError, ValueError, KeyError) as e:
        print(json.dumps(dict(state='initialization_failed', reason=str(e))));return 2
    p.print_help()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
