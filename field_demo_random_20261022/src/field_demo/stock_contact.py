"""Narrow permission for measured fingertip contact with loose ring stock.

This never changes residual occupancy to FREE or removes obstacles. Consumers
mask only the permitted active-finger trajectory samples in collision tests.
Actual native aggregate-wrench stop/heartbeat enforcement is a required external
execution invariant, bound to the measured driver and stop-test hashes below.
"""
from datetime import datetime
import numpy as np
from .collision import box, inside_box, finite, segment_box_distance


def _measurement(record):
    if not isinstance(record, dict) or record.get('verified') is not True:
        raise ValueError('Measured stock-contact provenance required')
    for name in ('by', 'at_utc', 'evidence_file'):
        if not isinstance(record.get(name), str) or not record[name].strip():
            raise ValueError('Named stock-contact measurement/evidence required')
    if datetime.fromisoformat(record['at_utc'].replace('Z', '+00:00')).tzinfo is None:
        raise ValueError('Timezone-aware stock-contact measurement required')


def _hash(value):
    if not isinstance(value, str) or len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
        raise ValueError('Exact actual native-driver/stop-test SHA-256 required')


def validate_stock_contact_policy(phase, envelope):
    """Validate configuration; caller verifies evidence files/executor binding."""
    policy = phase.get('bounded_loose_stock_contact')
    if not isinstance(policy, dict) or policy.get('schema_version') != 1:
        raise ValueError('Explicit bounded loose-stock contact policy required')
    if phase.get('_phase_name') != 'pickup' or phase.get('_contact') != 'grasp' or phase.get('_active_part') not in ('ring1', 'ring2'):
        raise ValueError('Loose-stock contact applies only during ring pickup/grasp')
    inventory = policy.get('inventory_contract', {})
    if inventory.get('parts') != ['ring1', 'ring2'] or type(inventory.get('maximum_count')) is not int or not 1 <= inventory['maximum_count'] <= 10:
        raise ValueError('Measured mixed loose-ring inventory only; unknown material/part families forbidden')
    _measurement(inventory.get('measurement'))
    region = policy.get('bin_interior', {})
    _, half = box(region)
    if np.max(half) > .5:
        raise ValueError('Bounded measured bin interior required')
    _measurement(region.get('measurement'))
    monitor = policy.get('monitoring', {})
    force = phase.get('force_monitor', {})
    if (force.get('enabled') is not True or force.get('force_limit_n') != 5 or force.get('moment_limit_nm') != 1 or
            monitor.get('wrench_source') != 'actual_fr3_sdk_aggregate_wrench' or monitor.get('force_limit_n') != 5 or
            monitor.get('moment_limit_nm') != 1 or type(monitor.get('heartbeat_timeout_s')) not in (int, float) or
            not 0 < monitor['heartbeat_timeout_s'] <= .25):
        raise ValueError('Actual SDK aggregate5N/1Nm stop and native heartbeat must remain enforced')
    _measurement(force.get('measurement')); _measurement(monitor.get('measurement'))
    _hash(monitor.get('native_driver_sha256')); _hash(monitor.get('stop_test_sha256'))
    tools = envelope.get('tool_capsules_tcp_m', [])
    fingers = policy.get('finger_capsules', [])
    if not isinstance(fingers, list) or not fingers:
        raise ValueError('Explicit measured native-finger capsule identities required')
    names = set(); indices = set()
    for entry in fingers:
        index = entry.get('index'); name = entry.get('name')
        if (type(index) is not int or index < 8 or index-8 >= len(tools) or index in indices or
                not isinstance(name, str) or not name or name in names or tools[index-8].get('name') != name):
            raise ValueError('Named measured finger must match its actual tool capsule, never an arm/palm alias')
        _measurement(entry.get('measurement')); names.add(name); indices.add(index)
    return dict(bin_interior=region, finger_indices=indices, finger_names=names,
                native_driver_sha256=monitor['native_driver_sha256'], stop_test_sha256=monitor['stop_test_sha256'])


def stock_contact_mask(obstacle, *, phase, capsule_index, capsule, envelope, active_robot=False, is_payload=False):
    """Return allowed contact samples; all other samples keep normal collision."""
    a, b = finite(capsule['a']), finite(capsule['b'])
    if a.shape != b.shape or a.shape[-1:] != (3,):
        raise ValueError('Actual swept capsule endpoints required')
    denied = np.zeros(a.shape[:-1], dtype=bool)
    if not active_robot or is_payload or capsule_index < 8:
        return denied
    if obstacle.get('source') != 'conservative_registered_RGBD_rays' or obstacle.get('occupancy_kind') not in ('unknown', 'occupied'):
        return denied
    try:
        policy = validate_stock_contact_policy(phase, envelope)
    except (ValueError, KeyError, TypeError):
        return denied
    if capsule_index not in policy['finger_indices'] or capsule.get('name') not in policy['finger_names']:
        return denied
    tool = envelope['tool_capsules_tcp_m'][capsule_index-8]
    if capsule.get('name') != tool.get('name'):
        return denied
    radius = finite(capsule['r'])
    if np.any(radius <= 0):
        raise ValueError('Conservative measured swept capsule radius required')
    # A merged ray box may straddle the bin interior boundary. Only the
    # ACTUAL full capsule can contact it, so its complete containment proves
    # every possible contact lies inside the reviewed stock region. The
    # exterior box portion stays occupied and every nonfinger query remains
    # checked; separately measured walls are never eligible for this rule.
    box(obstacle)
    contacted = segment_box_distance(a, b, obstacle) <= radius
    return np.asarray(contacted & inside_box(a, b, radius, policy['bin_interior']), dtype=bool)


def allow_stock_contact(obstacle, **kwargs):
    """Convenience for a scalar capsule or an entirely permitted trajectory."""
    return bool(np.all(stock_contact_mask(obstacle, **kwargs)))
