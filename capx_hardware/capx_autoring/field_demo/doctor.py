"""Offline installation inventory; --probe explicitly enables read-only robot queries."""
import argparse
import ipaddress
import importlib.util
import importlib.metadata
import json
import os
import platform
from pathlib import Path
import re
try:
    import resource
except ImportError:
    resource = None
import shutil
import subprocess
import sys
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[2]


def command(argv, timeout=5):
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=timeout, check=False)
        return {'ok': result.returncode == 0, 'returncode': result.returncode,
                'stdout': result.stdout[:16000].strip(), 'stderr': result.stderr[:4000].strip()}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {'ok': False, 'error': str(exc)}


def search_cmake(package):
    roots = [Path(p).expanduser() for p in os.environ.get('CMAKE_PREFIX_PATH', '').split(os.pathsep) if p]
    roots += [Path('/usr'), Path('/usr/local'), Path('/opt'), Path('/opt/openrobots')]
    matches = []
    for root in roots:
        for pattern in (f'lib/cmake/{package}/*Config.cmake', f'lib/*/cmake/{package}/*Config.cmake',
                        f'share/{package}/cmake/*Config.cmake', f'share/{package}/*Config.cmake',
                        f'*/lib/cmake/{package}/*Config.cmake'):
            matches.extend(str(p) for p in root.glob(pattern) if p.is_file())
    return sorted(set(matches))


def network_inventory():
    """Read only local interface addresses; never discover peers or alter routes."""
    tool = shutil.which('ip')
    if not tool:
        return {'source': 'sysfs_names_only', 'interfaces': [
            {'name': path.name, 'addresses': []} for path in sorted(Path('/sys/class/net').glob('*'))],
            'addresses_available': False, 'robot_ip_inferred': False,
            'note': 'iproute2 is not available; obtain each robot controller IP from its Desk/onsite operator.'}
    response = command([tool, '-json', 'address', 'show'], timeout=3)
    interfaces = []
    if response['ok']:
        try:
            for row in json.loads(response['stdout']):
                interfaces.append({'name': row.get('ifname'), 'state': row.get('operstate'),
                    'addresses': [{'family': address.get('family'), 'local': address.get('local'),
                                   'prefix_length': address.get('prefixlen'), 'scope': address.get('scope')}
                                  for address in row.get('addr_info', [])]})
        except (ValueError, TypeError, AttributeError) as exc:
            response = {'ok': False, 'error': str(exc)}
    return {'source': 'local_iproute2_address_inventory', 'interfaces': interfaces,
            'addresses_available': response['ok'], 'robot_ip_inferred': False,
            'error': response.get('error', response.get('stderr')) if not response['ok'] else None,
            'note': 'Local PC addresses only. No peer scans, robot IP inference, link changes or route changes.'}


def optional_camera_dependencies():
    result = {}
    for name in ('numpy', 'scipy', 'pyrealsense2'):
        try:
            available = importlib.util.find_spec(name) is not None
        except (ImportError, ValueError):
            available = False
        try:
            version = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            version = None
        result[name] = {'available': available, 'version': version}
    return {'optional': True, 'packages': result,
            'note': 'Package inventory only; camera SDK/device, calibrated depth, and capture are unverified.'}


def missing_values(value, path=''):
    missing = []
    if isinstance(value, dict):
        if not value:
            return [path]
        for key, item in value.items():
            missing.extend(missing_values(item, f'{path}.{key}' if path else key))
    elif isinstance(value, list):
        if not value:
            return [path]
        for index, item in enumerate(value):
            missing.extend(missing_values(item, f'{path}[{index}]'))
    elif value is None or value == '':
        missing.append(path)
    return missing


def site_inventory(config_path):
    if not config_path.is_file():
        return {'present': False, 'path': str(config_path), 'missing_values': ['site configuration'],
                'field_checks_complete': False}, None
    try:
        cfg = json.loads(config_path.read_text())
        if not isinstance(cfg, dict):
            raise ValueError('Expected a JSON object')
    except (OSError, ValueError) as exc:
        return {'present': True, 'error': str(exc), 'field_checks_complete': False}, None
    flags = cfg.get('commissioning', {})
    records = {}
    for key in ('calibration_record', 'commissioning_record'):
        name = cfg.get(key)
        path = (config_path.parent / name).resolve() if isinstance(name, str) and name else None
        records[key] = {'path': str(path) if path else None,
                        'nonempty': bool(path and path.is_file() and path.stat().st_size > 0)}
    return {'present': True, 'path': str(config_path), 'missing_values': missing_values(cfg),
            'unconfirmed_field_checks': [k for k, v in flags.items() if v is not True],
            'assembly_commissioned': cfg.get('assembly_commissioned') is True,
            'records': records, 'field_checks_complete': False,
            'note': 'Inventory only; populated values and flags do not certify measured hardware readiness.'}, cfg


def probe(cfg, config_path):
    """Use only state read and native hand status; never recovery, control, or stop."""
    results = []
    for side in ('left', 'right'):
        address = cfg.get('robots', {}).get(side, {}).get('ip')
        try:
            ipaddress.ip_address(address)
        except (ValueError, TypeError):
            results.append({'side': side, 'ok': False, 'error': 'A measured robot IP is required; no connection attempted.'})
            continue
        state_binary = cfg.get('state_binary')
        hand = cfg.get('grippers', {}).get(side, {})
        queries = [('robot_state', state_binary, [address])]
        if hand.get('type') == 'franka_hand':
            queries.append(('hand_status', hand.get('binary'), ['status', address]))
        for kind, name, args in queries:
            executable = (config_path.parent / name).resolve() if isinstance(name, str) and name else None
            if not executable or not executable.is_file() or not os.access(executable, os.X_OK):
                result = {'ok': False, 'error': 'Build the configured read-only binary first.'}
            else:
                result = command([str(executable), *args], timeout=10)
                result['connection_attempted'] = True
                if result['ok']:
                    try:
                        result['observed'] = json.loads(result['stdout'])
                    except ValueError:
                        result['ok'] = False
                        result['error'] = 'Probe did not return JSON.'
            results.append({'side': side, 'query': kind, **result})
    return results


def collect(config_path, with_probe=False):
    system, arch = platform.system(), platform.machine()
    supported = system == 'Linux' and arch in ('x86_64', 'amd64', 'aarch64', 'arm64')
    tools = {}
    for name in ('python3', 'cmake', 'c++', 'make'):
        path = shutil.which(name)
        local_tool = Path(sys.executable).parent / name
        if path is None and local_tool.is_file() and os.access(local_tool, os.X_OK):
            path = str(local_tool)
        tools[name] = {'path': path, 'available': path is not None}
        if path:
            tools[name]['version'] = command([path, '--version'])
    sdk = search_cmake('Franka')
    eigen = search_cmake('eigen3') + search_cmake('Eigen3')
    sdk_versions = []
    for name in sdk:
        version_file = Path(name).with_name('FrankaConfigVersion.cmake')
        if version_file.is_file():
            for line in version_file.read_text(errors='replace').splitlines():
                if 'PACKAGE_VERSION ' in line:
                    sdk_versions.append(line.strip())
                    break
    versions = []
    for line in sdk_versions:
        found = re.search(r'PACKAGE_VERSION\s+"(\d+)\.(\d+)(?:\.(\d+))?', line)
        if found:
            versions.append(tuple(int(item or 0) for item in found.groups()))
    config, cfg = site_inventory(config_path)
    checks = {'supported_host': supported, 'python_3_10_or_newer': sys.version_info >= (3, 10),
              'cmake_available': tools['cmake']['available'], 'cpp_compiler_available': tools['c++']['available'],
              'build_tool_available': tools['make']['available'] or shutil.which('ninja') is not None,
              'franka_cmake_package_found': bool(sdk), 'franka_minimum_0_18_found': any(v >= (0, 18, 0) for v in versions),
              'eigen_cmake_package_found': bool(eigen)}
    try:
        rt_limit = resource.getrlimit(resource.RLIMIT_RTPRIO)
        memlock = resource.getrlimit(resource.RLIMIT_MEMLOCK)
    except (AttributeError, ValueError):
        rt_limit, memlock = None, None
    realtime_file = Path('/sys/kernel/realtime')
    realtime = realtime_file.read_text().strip() == '1' if realtime_file.is_file() else None
    result = {'schema_version': 1, 'generated_at_utc': datetime.now(timezone.utc).isoformat(),
              'mode': 'read_only_probe' if with_probe else 'offline', 'network_attempted': with_probe and cfg is not None,
              'hardware_ready': False, 'live_motion_enabled': False,
              'installation_checks_passed': all(checks.values()), 'installation_checks': checks,
              'host': {'system': system, 'architecture': arch, 'kernel': platform.release(),
                       'python': sys.version, 'python_executable': sys.executable, 'supported': supported,
                       'os_release': platform.freedesktop_os_release() if hasattr(platform, 'freedesktop_os_release') and Path('/etc/os-release').is_file() else {},
                       'supported_hosts': 'Linux x86_64 or aarch64; SDK must support the target OS/architecture.'},
              'tools': tools, 'sdk': {'FrankaConfig': sdk, 'declared_versions': sdk_versions,
                                    'Eigen3Config': sorted(set(eigen)), 'robot_version_compatibility_verified': False},
              'realtime': {'kernel_realtime_indicator': realtime, 'rtprio_limits': rt_limit,
                           'memlock_limits': memlock, 'effective_uid': os.geteuid() if hasattr(os, 'geteuid') else None,
                           'note': 'Inventory only. Control timing, FCI, permissions and SDK/System Image compatibility need onsite verification.'},
              'network_interfaces': network_inventory(),
              'camera_dependencies': optional_camera_dependencies(),
              'devices': {'video_nodes': sorted(str(p) for p in Path('/dev').glob('video*')),
                          'note': 'Names only; no camera or robot device opened.'}, 'site': config}
    if with_probe:
        result['probes'] = probe(cfg, config_path) if cfg is not None else [{'ok': False, 'error': 'Valid site config required.'}]
        result['network_attempted'] = any(row.get('connection_attempted', False) for row in result['probes'])
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'site/site.json')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--probe', action='store_true', help='Explicitly read robot and Hand state; 10-second timeout per query, no motion.')
    args = parser.parse_args(argv)
    result = collect(args.config.resolve(), args.probe)
    payload = json.dumps(result, indent=2, ensure_ascii=False) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload)
    print(payload, end='')
    passed = result['installation_checks_passed']
    if args.probe:
        passed = passed and bool(result['probes']) and all(row['ok'] for row in result['probes'])
    return 0 if passed else 2


if __name__ == '__main__':
    raise SystemExit(main())
