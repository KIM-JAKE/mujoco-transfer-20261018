"""Compile an immutable carry once; join it from a live RGB-D pickup.

The saved field contains public robot commands only. The certificate skips
only static queries proved for its grasp/width envelope. Current counterpart,
own payload, installed parts, contact and command limits remain authoritative.
"""
from pathlib import Path
from contextlib import contextmanager
import copy, hashlib, json, time
import numpy as np
from fixed_canonical_plan import META, right_carry


def audit_saved_field(q):
    from fr3_motion_control import position_velocity_bounds
    dt = META['dt_s']
    v = np.diff(q, axis=0, prepend=q[:1]) / dt
    a = np.diff(v, axis=0, prepend=np.zeros((1, 7))) / dt
    j = np.diff(a, axis=0, prepend=np.zeros((1, 7))) / dt
    lo, hi = position_velocity_bounds(q)
    cap = .95 * np.where(v >= 0, hi, -lo)
    peaks = [float(np.max(abs(v) / np.maximum(cap, 1e-8))),
        float(np.max(abs(a))), float(np.max(abs(j)))]
    if peaks[0] > 1 or peaks[1] > 9.5 or peaks[2] > 4750:
        raise RuntimeError('Saved command field exceeds original FR3 limits ' + str(peaks))
    return dict(velocity_fraction_of_95_percent_PBV=peaks[0],
        max_acceleration_rad_s2=peaks[1], max_jerk_rad_s3=peaks[2])


def prepare(checkers, art, indices, fingers, paths, goals, config, cache_dir):
    if not (config.get('fixed_supply_mode') and config.get('fixed_prevalidated_routes')):
        return None
    from fixed_static_route_certificate import (StaticRows, static_world_seal,
        compile_static_rows, PrismaticFixtureSweeps, payload_geometry_sha256)
    from exact_collision_proof_cache import own_model_fingerprint
    from isaac_kinematics import frame
    start = time.perf_counter()
    command_audit = audit_saved_field(right_carry())
    rc, lc = checkers['right'], checkers['left']
    from fixed_saved_routes import robot_hint
    if robot_hint(dict(config=dict(config, fixed_saved_route_priors=True), arm=rc.arm), 'ring1') is None:
        return dict(ready=False, reason='Stored robot geometry or calibration changed')
    if hashlib.sha256(right_carry().astype(np.float32).tobytes()).hexdigest() != META['command_field_sha256']:
        raise RuntimeError('Saved command field fingerprint mismatch')
    if not np.array_equal(np.asarray(goals['ring1']['P']), np.asarray(META['goal'])):
        return dict(ready=False, reason='Stored fixture calibration changed')
    seal = static_world_seal(rc)
    model = own_model_fingerprint(rc.same_arm_clearance)
    key = hashlib.sha256((seal['sha256'] + model + META['command_field_sha256']).encode()).hexdigest()
    target = Path(cache_dir) / (key + '.json')
    target.parent.mkdir(parents=True, exist_ok=True)
    saved = (rc.payload_path, rc.payload_rows, rc.payload_boxes, rc.payload_relative)
    G = frame([0, 0, .1034], np.eye(3)) @ np.asarray(META['fixture']['grasp'])
    try:
        rc.attach_payload(paths['workpiece_ring1'], G)
        cache_hit = False
        if target.exists():
            try:
                cert = StaticRows.load(target)
                m = cert.metadata
                cache_hit = (m['world_sha256'] == seal['sha256'] and m['model_sha256'] == model
                    and m['payload_geometry_sha256'] == payload_geometry_sha256(rc)
                    and m['width_interval_m'] == META['width_interval_m']
                    and m['nominal_hand_object_T'] == G.tolist()
                    and m['translation_m'] == META['payload_translation_m']
                    and m['rotation_rad'] == META['payload_rotation_rad'])
            except (ValueError, KeyError, OSError):
                cache_hit = False
        if not cache_hit:
            actual = art.get_dof_positions()[0]
            with rc.other_arm_at(lc, actual[indices['left']], float(actual[fingers['left']].sum())):
                cert, mask, why = compile_static_rows(rc, right_carry(), META['width_interval_m'],
                    world_seal=seal, model_sha256=model, nominal_hand_object_T=G,
                    translation_m=META['payload_translation_m'], rotation_rad=META['payload_rotation_rad'],
                    fixture_sweep_clear=PrismaticFixtureSweeps(rc, seal))
            tmp = target.with_suffix('.tmp')
            cert.save(tmp)
            tmp.replace(target)
        summary = dict(ready=True, cache_hit=cache_hit, preparation_s=time.perf_counter()-start,
            canonical_rows=len(right_carry()), static_certified_unique_rows=len(cert.row_keys),
            world_sha256=seal['sha256'], model_sha256=model, proof_file=str(target),
            compile_excluded_from_cycle=True, only_static_queries_reused=True,
            actual_start_counterpart_payload_contact_limits_remain_live=True)
        summary['saved_command_limit_audit'] = command_audit
        return dict(certificate=cert, world_sha256=seal['sha256'], model_sha256=model, summary=summary)
    finally:
        rc.payload_path, rc.payload_rows, rc.payload_boxes, rc.payload_relative = saved


def eligible(c, part, G, width):
    state = c.get('fixed_prevalidated_routes')
    if part != 'ring1' or not state or not state.get('certificate'):
        return False
    from fixed_static_route_certificate import grasp_in_envelope
    lo, hi = META['width_interval_m']
    return (lo + 1e-9 <= float(width) <= hi - 1e-9
        and np.array_equal(np.asarray(c['goals'][part]['P']), np.asarray(META['goal']))
        and grasp_in_envelope(np.asarray(META['fixture']['grasp']), np.asarray(G),
            META['payload_translation_m'], META['payload_rotation_rad']))


@contextmanager
def static_scope(c, checker):
    state = c.get('fixed_prevalidated_routes')
    if not state or not state.get('certificate'):
        yield {}
        return
    from fixed_static_route_certificate import bound_static_rows
    with bound_static_rows(checker, state['certificate'],
        current_world_sha256=state['world_sha256'],
        current_model_sha256=state['model_sha256']) as stats:
        yield stats


def fixture_candidate(c, part, G, width, context):
    """Try one saved fixture branch, retaining fresh complete path admission."""
    if not eligible(c, part, G, width):
        return None
    checker = c['checkers']['right']
    candidate = copy.deepcopy(META['fixture'])
    started = time.perf_counter()
    from gpu_pair_path_scope import certified_path_scope
    # These candidate checks retain robot stroke/release/withdrawal validity.
    # Current observed grasp still admits the free-space preinsert endpoint.
    with context():
        for qs, widths in [(candidate['stroke_q'], [width]),
                ([candidate['stroke_q'][-1], candidate['stroke_q'][-1]],
                    np.linspace(width, candidate['release_width_m'], 5)),
                (candidate['withdraw_q'], [candidate['release_width_m']])]:
            with certified_path_scope(c, checker, np.asarray(qs), widths):
                if any(not checker.path(qs, float(w))[0] for w in widths):
                    return None
    with context(G):
        if not checker.clear(np.asarray(candidate['pre_q']), width)[0]:
            return None
    candidate.update(saved_canonical_field=True, grasp=np.asarray(G).tolist(),
        runtime_fixture_check_s=time.perf_counter()-started)
    c['planning_diagnostics'].append(dict(event='fixed_saved_fixture_branch_admitted',
        part=part, wall_s=candidate['runtime_fixture_check_s'], fresh_other_arm_checks=True,
        public_RGBD_grasp_envelope=True, online_fixture_search_omitted=True,
        uses_free_body_poses=False))
    return candidate


def execute_carry(c, G, context, move, step):
    """Check a changed entry bridge, then issue the exact compiled middle."""
    checker = c['checkers']['right']
    rows = right_carry()
    ids, fi = c['ids'], c['fi']
    actual = c['art'].get_dof_positions()[0]
    widths = np.unique(np.linspace(min(float(actual[fi].sum()), float(c['cmd'][0, fi].sum())),
        max(float(actual[fi].sum()), float(c['cmd'][0, fi].sum())), 5))
    if not all(eligible(c, 'ring1', G, w) for w in widths):
        return False
    # This bridge is fully live-checked. Its geometry was changed by vision.
    if not np.array_equal(c['cmd'][0, ids].astype(np.float32), rows[0].astype(np.float32)):
        move([c['cmd'][0, ids].copy(), rows[0]], .05, 'join_saved_carry', G=G)
    actual = c['art'].get_dof_positions()[0]
    actual_q = actual[ids].copy()
    widths = np.unique(np.linspace(min(float(actual[fi].sum()), float(c['cmd'][0, fi].sum())),
        max(float(actual[fi].sum()), float(c['cmd'][0, fi].sum())), 5))
    if not all(eligible(c, 'ring1', G, w) for w in widths):
        return False
    from aperture_path_check import path_openings
    from gpu_pair_path_scope import certified_path_scope
    qs = np.vstack([actual_q, c['cmd'][0, ids], rows])
    stats = {}
    with context(G):
        with static_scope(c, checker) as proof_stats:
            with certified_path_scope(c, checker, qs, widths):
                clear, why = path_openings(checker, qs, widths, diagnostics=stats)
        stats.update(proof_stats)
    if not clear:
        # No saved row has been issued. The stopped, checked entry remains safe.
        raise RuntimeError('Saved carry current-context admission failed ' + str(why))
    c['planning_diagnostics'].append(dict(event='fixed_saved_canonical_carry_admitted',
        rows=len(rows)-1, original_dt_s=META['dt_s'], current_encoder_bridge_live=True,
        fresh_public_RGBD_grasp=True, **stats))
    for q in rows[1:]:
        c['cmd'][0, ids] = q
        step('carry_to_preinsert')
    return True
