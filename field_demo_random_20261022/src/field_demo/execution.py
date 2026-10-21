"""Conservative field runner for taught FR3 stages and measured PLC feedback.

No camera-to-motion, contact search or handover is implemented. Dry run exercises
only the policy contract. Every actuator command requires --execute.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import threading

from isaac_port.fieldkit.policy import DryRunCell, PART_SIDES, assemble_parts
from isaac_port.fieldkit.site import ConfiguredCell, Site, execute as legacy_execute

CAPABILITIES = dict(taught_stage_motion=True, native_franka_hand=True,
                    measured_plc_feedback=True, three_part_profile=True,
                    optional_button=True, camera_reviewed_pickup_bridge=True, camera_guided_motion=False,
                    automatic_contact_search=False, handover=False,
                    parallel_arm_motion=False, automatic_retry=False,
                    hardware_execution_verified=False)


def stop_socket_path(config):
    addresses = sorted(row['ip'] for row in config['robots'].values() if row.get('ip'))
    if not addresses:
        raise ValueError('Configure at least one robot IP')
    root = Path('/tmp') / ('field-demo-stop-' + str(os.getuid()))
    root.mkdir(mode=0o700, exist_ok=True)
    if root.stat().st_uid != os.getuid() or root.stat().st_mode & 0o077:
        raise RuntimeError('Stop socket directory must be private and owned by this user')
    return root / (hashlib.sha256('|'.join(addresses).encode()).hexdigest()[:24] + '.sock')


class StopListener:
    """Independent process channel: stop bypasses the held cell motion lock.

    The acknowledgement means cancellation was requested, never that a physical
    stop has been measured. Native watchdogs and external hardware stop remain.
    """
    def __init__(self, site):
        self.site = site
        self.path = stop_socket_path(site.config)
        self.closed = threading.Event()

    def __enter__(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            # Do not unlink another process's active or stale endpoint blindly.
            self.sock.bind(str(self.path))
            self.sock.listen(2)
            self.sock.settimeout(.1)
        except BaseException:
            self.sock.close()
            raise
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()
        return self

    def _serve(self):
        while not self.closed.is_set():
            try:
                connection, _ = self.sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with connection:
                connection.settimeout(.2)
                try:
                    if connection.recv(16).strip() != b'STOP':
                        continue
                    self.site.cancelled.set()
                    connection.sendall(b'{"stop_requested":true,"physical_stop_confirmed":false}\n')
                except OSError:
                    continue
            self.site.stop()

    def __exit__(self, *_):
        self.closed.set()
        self.sock.close()
        self.thread.join(timeout=1)
        self.path.unlink(missing_ok=True)


def request_stop(config):
    path = stop_socket_path(config)
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(1)
        try:
            connection.connect(str(path))
            connection.sendall(b'STOP\n')
            result = json.loads(connection.recv(4096))
        except OSError as error:
            raise RuntimeError('No responsive field-demo execution session; use the field hardware stop procedure') from error
    if result.get('stop_requested') is not True:
        raise RuntimeError('No stop acknowledgement')
    return result


def run_parts(site, *, parts=tuple(PART_SIDES), profile='three-part', full_assembly=False):
    if full_assembly and site.config.get('assembly_commissioned') is not True:
        raise RuntimeError('Complete individual field-stage commissioning before assembly')
    cell = ConfiguredCell(site)
    try:
        cell.validate(profile=profile, parts=parts)
        assemble_parts(cell, parts, button=profile == 'button')
    except BaseException:
        site.stop()
        raise
    return dict(completed_parts=list(parts), button_verified=cell.state.button_verified,
                hardware_ready=False, capabilities=CAPABILITIES)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=['dry-run', 'validate', 'status', 'part', 'assemble', 'trajectory', 'stop', 'jog', 'cartesian-jog', 'gripper-move'])
    p.add_argument('--config', type=Path)
    p.add_argument('--log-dir', type=Path, default=Path('field-logs'))
    p.add_argument('--profile', choices=['three-part', 'button'], default='three-part')
    p.add_argument('--part', choices=tuple(PART_SIDES))
    p.add_argument('--stage', help='Run only a taught arm CSV; does not operate fingers or clamps')
    p.add_argument('--side', choices=['left', 'right'], default='left')
    p.add_argument('--joint', type=int, default=7)
    p.add_argument('--delta', type=float, default=.01)
    p.add_argument('--duration', type=float, default=3.)
    p.add_argument('--axis', choices=['x', 'y', 'z'], default='z')
    p.add_argument('--distance-mm', type=float, default=1.)
    p.add_argument('--width-mm', type=float)
    p.add_argument('--speed-mm-s', type=float, default=5.)
    p.add_argument('--execute', action='store_true')
    return p


def main(argv=None):
    p = parser()
    a = p.parse_args(argv)
    actuator = a.command in ('part', 'assemble', 'trajectory', 'stop', 'jog', 'cartesian-jog', 'gripper-move')
    # This precedes configuration reads, Site construction and all connections.
    if actuator and not a.execute:
        p.error('Actuation requires --execute')
    if a.command != 'dry-run' and a.config is None:
        p.error('--config is required')
    if a.command == 'part' and not a.part:
        p.error('part requires --part')
    if a.command == 'part' and a.profile == 'button':
        p.error('part uses the three-part profile; button requires all three parts')
    if a.command == 'trajectory' and (not a.stage or a.stage.split('.')[0] not in ('left', 'right')):
        p.error('trajectory requires --stage beginning left. or right.')
    if a.command == 'dry-run':
        cell = DryRunCell()
        parts = (a.part,) if a.part else tuple(PART_SIDES)
        assemble_parts(cell, parts, button=a.profile == 'button')
        result = dict(state='policy_contract_only', completed_parts=list(parts),
                      events=cell.events, button_verified=cell.state.button_verified,
                      robot_connections=0, hardware_ready=False, capabilities=CAPABILITIES)
    elif a.command == 'stop':
        result = request_stop(json.loads(a.config.read_text()))
    else:
        parts = (a.part,) if a.part else tuple(PART_SIDES)
        sides = tuple(dict.fromkeys(PART_SIDES[part] for part in parts))
        if a.command == 'trajectory':
            sides = (a.stage.split('.')[0],)
        if a.command in ('jog', 'cartesian-jog', 'gripper-move'):
            sides = (a.side,)
        site = Site(a.config, a.log_dir, sides=sides)
        if a.command == 'validate':
            ConfiguredCell(site).validate(profile=a.profile, parts=parts, check_feedback=False)
            result = dict(state='offline_configuration_checked', robot_connections=0,
                          hardware_ready=False, capabilities=CAPABILITIES)
        elif a.command == 'status':
            result = site.status()
            result['capabilities'] = CAPABILITIES
        else:
            with site.control_lock(), StopListener(site):
                try:
                    if a.command in ('trajectory', 'jog', 'cartesian-jog', 'gripper-move'):
                        a.side = sides[0]
                        a.position, a.speed, a.force = 0, 16, 16
                        # Native hash/tool/TCP/start/motion/watchdog guards remain.
                        return legacy_execute(a, site)
                    result = run_parts(site, parts=parts, profile=a.profile,
                                       full_assembly=a.command == 'assemble')
                except BaseException:
                    site.stop()
                    raise
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
