"""Offline commands; live actuation is deliberately not implemented here."""
import argparse
import hashlib
import json
from pathlib import Path
from .policy import DryRunCell, assemble

ROOT = Path(__file__).resolve().parents[1]


def configured(value):
    if isinstance(value, dict):
        return bool(value) and all(configured(v) for v in value.values())
    if isinstance(value, list):
        return bool(value) and all(configured(v) for v in value)
    return value is not None and value != ''


def preflight(config_path=None):
    evidence = ROOT / 'reports/latest-readiness.json'
    if not evidence.exists():
        evidence = ROOT / 'reports/hardware-audit/14p729-readiness.json'
    report = json.loads(evidence.read_text())
    blockers = list(report['blockers'])
    if not report['inverse_dynamics_screen']['all_without_gravity_rate_within_limits']:
        blockers.append(dict(id='model_torque_rate', status='warning',
                             detail='Imported-model screening exceeds 1000 Nm/s; not measured robot torque.'))
    config = json.loads(Path(config_path).read_text()) if config_path else {}
    for key in ['robot_system_versions', 'robot_ips', 'base_to_cell_transforms',
                'tool_calibration', 'payloads', 'robotiq_interface', 'clamp_plc_io',
                'part_pose_sensor', 'button_feedback', 'commissioning_evidence']:
        if not configured(config.get(key)):
            blockers.append(dict(id=key, status='missing', detail='Site-specific configuration/evidence required.'))
    # Site configuration cannot turn a simulation result into a hardware certificate.
    return dict(state='integration_required', hardware_ready=False,
                live_motion_enabled=False, blockers=blockers,
                evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['preflight', 'dry-run'])
    parser.add_argument('--config', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.command == 'preflight':
        result = preflight(args.config)
        code = 2  # Explicitly incomplete; useful for automated commissioning gates.
    else:
        cell = DryRunCell()
        assemble(cell)
        result = dict(state='dry_run_passed', hardware_ready=False,
                      physical_simulation=False, cycle_time_s=None, events=cell.events)
        code = 0
    text = json.dumps(result, indent=2)
    print(text)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + '\n')
    return code


if __name__ == '__main__':
    raise SystemExit(main())
