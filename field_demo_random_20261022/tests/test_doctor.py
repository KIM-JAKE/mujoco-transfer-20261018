import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from field_demo import doctor


class DoctorTests(unittest.TestCase):
    def test_collect_offline_never_probes_or_connects(self):
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(doctor, 'probe', side_effect=AssertionError('Must not probe')), \
                patch.object(doctor.sys, 'executable', '/nonexistent-field-test/python3'), \
                patch.object(doctor.shutil, 'which', return_value=None), \
                patch.object(doctor, 'search_cmake', return_value=[]), \
                patch.object(doctor, 'command', side_effect=AssertionError('No commands when tools absent')):
            report = doctor.collect(Path(folder) / 'site.json')
        self.assertEqual(report['mode'], 'offline')
        self.assertFalse(report['network_attempted'])
        self.assertFalse(report['hardware_ready'])
        self.assertFalse(report['live_motion_enabled'])
        self.assertFalse(report['installation_checks_passed'])
        self.assertFalse(report['sdk']['robot_version_compatibility_verified'])

    def test_local_interface_inventory_uses_no_scans(self):
        response = {'ok': True, 'stdout': json.dumps([{'ifname': 'enp1s0', 'operstate': 'UP',
                    'addr_info': [{'family': 'inet', 'local': '192.0.2.100', 'prefixlen': 24, 'scope': 'global'}]}])}
        with patch.object(doctor.shutil, 'which', return_value='/fake/ip'), \
                patch.object(doctor, 'command', return_value=response) as run:
            result = doctor.network_inventory()
        run.assert_called_once_with(['/fake/ip', '-json', 'address', 'show'], timeout=3)
        self.assertFalse(result['robot_ip_inferred'])
        self.assertEqual(result['interfaces'][0]['addresses'][0]['local'], '192.0.2.100')

    def test_camera_dependencies_are_inventory_only(self):
        with patch.object(doctor.importlib.util, 'find_spec', return_value=None), \
                patch.object(doctor.importlib.metadata, 'version', side_effect=doctor.importlib.metadata.PackageNotFoundError):
            result = doctor.optional_camera_dependencies()
        self.assertTrue(result['optional'])
        self.assertEqual(set(result['packages']), {'numpy', 'scipy', 'pyrealsense2'})
        self.assertTrue(all(not row['available'] for row in result['packages'].values()))

    def test_missing_values_preserves_valid_zero_and_false_for_separate_checks(self):
        self.assertEqual(doctor.missing_values({'ip': None, 'mass': 0, 'flag': False, 'stages': {}}), ['ip', 'stages'])

    def test_probe_without_ips_never_executes(self):
        with patch.object(doctor, 'command', side_effect=AssertionError('No robot IP')):
            rows = doctor.probe({'robots': {'left': {'ip': None}, 'right': {'ip': 'unset'}}}, Path('/tmp/site.json'))
        self.assertEqual(len(rows), 2)
        self.assertTrue(all(not row['ok'] for row in rows))

    def test_probe_only_state_and_hand_status_with_timeout(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            binary = root / 'fake_binary'
            binary.write_text('fake')
            binary.chmod(0o755)
            cfg = {'state_binary': 'fake_binary', 'robots': {}, 'grippers': {}}
            for side, address in [('left', '192.0.2.10'), ('right', '192.0.2.11')]:
                cfg['robots'][side] = {'ip': address}
                cfg['grippers'][side] = {'type': 'franka_hand', 'binary': 'fake_binary'}
            with patch.object(doctor, 'command', return_value={'ok': True, 'stdout': '{}', 'stderr': '', 'returncode': 0}) as run:
                rows = doctor.probe(cfg, root / 'site.json')
            self.assertEqual(len(rows), 4)
            self.assertTrue(all(row['ok'] for row in rows))
            for call in run.call_args_list:
                args = call.args[0]
                self.assertTrue(args[1:] in (['192.0.2.10'], ['192.0.2.11'], ['status', '192.0.2.10'], ['status', '192.0.2.11']))
                self.assertEqual(call.kwargs['timeout'], 10)

    def test_command_timeout_is_reported(self):
        with patch.object(doctor.subprocess, 'run', side_effect=subprocess.TimeoutExpired('read', 10)):
            report = doctor.command(['fake-read'], timeout=10)
        self.assertFalse(report['ok'])
        self.assertIn('timed out', report['error'])

    def test_invalid_config_stays_incomplete(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'site.json'
            path.write_text('[]')
            report, cfg = doctor.site_inventory(path)
        self.assertIsNone(cfg)
        self.assertFalse(report['field_checks_complete'])
        self.assertIn('error', report)

    def test_output_report_and_distinct_install_exit(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'doctor.json'
            report = {'hardware_ready': False, 'installation_checks_passed': True}
            with patch.object(doctor, 'collect', return_value=report), contextlib.redirect_stdout(io.StringIO()):
                code = doctor.main(['--output', str(path)])
            self.assertEqual(code, 0)
            self.assertFalse(json.loads(path.read_text())['hardware_ready'])
            report['installation_checks_passed'] = False
            with patch.object(doctor, 'collect', return_value=report), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(doctor.main([]), 2)

    def test_offline_build_guard_stops_before_extraction(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / 'package'
            (root / 'scripts').mkdir(parents=True)
            original = doctor.ROOT / 'scripts/build-offline-ubuntu22.sh'
            script = root / 'scripts/build-offline-ubuntu22.sh'
            script.write_bytes(original.read_bytes())
            script.chmod(0o755)
            env = dict(os.environ, FIELD_DEMO_PYTHON='/bin/false')
            result = subprocess.run([str(script)], env=env, text=True, capture_output=True, timeout=5)
            self.assertEqual(result.returncode, 2)
            self.assertIn('Ubuntu 22.04 x86_64', result.stderr)
            self.assertFalse((root / '.local-sdk').exists())

    def test_install_fallback_uses_bundled_bootstrap_without_download(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder) / 'package with spaces'
            for directory in ('scripts', 'vendor', 'config/templates'):
                (root / directory).mkdir(parents=True)
            script = root / 'scripts/install.sh'
            script.write_bytes((doctor.ROOT / 'scripts/install.sh').read_bytes())
            script.chmod(0o755)
            (root / 'vendor/virtualenv.pyz').write_text('fake bootstrap')
            (root / 'config/templates/site.json').write_text('{}')
            fake_python = root / 'fake-python'
            fake_python.write_text('#!/bin/sh\nif [ "$1" = "-c" ]; then exit 0; fi\nif [ "$1" = "-m" ]; then exit 1; fi\nprintf "%s\\n" "$@" > "' + str(root / 'bootstrap-args.txt') + '"\nexit 0\n')
            fake_python.chmod(0o755)
            env = dict(os.environ, FIELD_DEMO_PYTHON=str(fake_python))
            result = subprocess.run([str(script), '--offline'], env=env, text=True, capture_output=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)
            args = (root / 'bootstrap-args.txt').read_text().splitlines()
            self.assertIn('--no-download', args)
            self.assertIn('--no-periodic-update', args)
            self.assertIn(str(root / '.virtualenv-cache'), args)
            self.assertEqual(args[-1], str(root / '.venv'))
            self.assertTrue((root / 'site/site.json').is_file())
            self.assertTrue((root / 'reports/venv.log').is_file())

    def test_unsupported_host_install_fails_without_claiming_ready(self):
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(doctor.platform, 'system', return_value='Darwin'), \
                patch.object(doctor.shutil, 'which', return_value=None), \
                patch.object(doctor, 'search_cmake', return_value=[]):
            report = doctor.collect(Path(folder) / 'site.json')
        self.assertFalse(report['installation_checks']['supported_host'])
        self.assertFalse(report['hardware_ready'])


if __name__ == '__main__':
    unittest.main()
