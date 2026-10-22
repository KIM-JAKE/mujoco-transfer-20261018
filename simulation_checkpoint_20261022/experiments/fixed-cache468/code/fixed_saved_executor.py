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


def _prepare_domain(checkers, art, indices, fingers, paths, goals, config, cache_dir, domain_tcp_grasp):
    domain_meta=copy.deepcopy(META)
    domain_meta['fixture']['grasp']=np.asarray(domain_tcp_grasp,float).tolist()
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
    if hashlib.sha256(right_carry().astype(np.float32).tobytes()).hexdigest() != domain_meta['command_field_sha256']:
        raise RuntimeError('Saved command field fingerprint mismatch')
    if not np.array_equal(np.asarray(goals['ring1']['P']), np.asarray(domain_meta['goal'])):
        return dict(ready=False, reason='Stored fixture calibration changed')
    seal = static_world_seal(rc)
    model = own_model_fingerprint(rc.same_arm_clearance)
    plan_identity = hashlib.sha256(json.dumps(domain_meta,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    key = hashlib.sha256((seal['sha256'] + model + plan_identity).encode()).hexdigest()
    target = Path(cache_dir) / (key + '.json')
    target.parent.mkdir(parents=True, exist_ok=True)
    saved = (rc.payload_path, rc.payload_rows, rc.payload_boxes, rc.payload_relative)
    G = frame([0, 0, .1034], np.eye(3)) @ np.asarray(domain_meta['fixture']['grasp'])
    try:
        rc.attach_payload(paths['workpiece_ring1'], G)
        cache_hit = False
        if target.exists():
            try:
                cert = StaticRows.load(target)
                m = cert.metadata
                cache_hit = (m['world_sha256'] == seal['sha256'] and m['model_sha256'] == model
                    and m['payload_geometry_sha256'] == payload_geometry_sha256(rc)
                    and m['width_interval_m'] == domain_meta['width_interval_m']
                    and m['nominal_hand_object_T'] == G.tolist()
                    and m['translation_m'] == domain_meta['payload_translation_m']
                    and m['rotation_rad'] == domain_meta['payload_rotation_rad'])
            except (ValueError, KeyError, OSError):
                cache_hit = False
        if not cache_hit:
            actual = art.get_dof_positions()[0]
            with rc.other_arm_at(lc, actual[indices['left']], float(actual[fingers['left']].sum())):
                cert, mask, why = compile_static_rows(rc, right_carry(), domain_meta['width_interval_m'],
                    world_seal=seal, model_sha256=model, nominal_hand_object_T=G,
                    translation_m=domain_meta['payload_translation_m'], rotation_rad=domain_meta['payload_rotation_rad'],
                    fixture_sweep_clear=PrismaticFixtureSweeps(rc, seal))
            tmp = target.with_suffix('.'+str(__import__('os').getpid())+'.tmp')
            cert.save(tmp)
            tmp.replace(target)
        summary = dict(ready=True, cache_hit=cache_hit, preparation_s=time.perf_counter()-start,
            canonical_rows=len(right_carry()), static_certified_unique_rows=len(cert.row_keys),
            world_sha256=seal['sha256'], model_sha256=model, proof_file=str(target),
            compile_excluded_from_cycle=True, only_static_queries_reused=True,
            actual_start_counterpart_payload_contact_limits_remain_live=True)
        summary['saved_command_limit_audit'] = command_audit
        from fixed_own_route_certificate import prepare_like_static
        try:
            own_proof=prepare_like_static(rc,right_carry(),cert,cache_dir)
        except Exception as error:
            own_proof=dict(certificate=None,summary=dict(ready=False,reason=type(error).__name__+': '+str(error),original_own_checks_retained=True))
        summary['own_envelope_preparation']=own_proof['summary']
        return dict(certificate=cert,own_certificate=own_proof['certificate'], world_sha256=seal['sha256'], model_sha256=model, summary=summary)
    finally:
        rc.payload_path, rc.payload_rows, rc.payload_boxes, rc.payload_relative = saved


def prepare(checkers,art,indices,fingers,paths,goals,config,cache_dir):
    if not(config.get('fixed_supply_mode')and config.get('fixed_prevalidated_routes')):return None
    nominal=np.asarray(META['fixture']['grasp'],float)
    half_turn=np.diag([-1.,-1.,1.,1.])
    # These are TWO different independently proved CAD poses, not a claim
    # that ring geometry is symmetric or that the public pose was normalized.
    domains=[]
    candidates=[('nominal_public_G',nominal),('public_G_half_turn_domain',nominal@half_turn)]
    from fixed_public_grasp_domains import RIGHT_EXTRA_DOMAINS
    from fixed_static_route_certificate import rigid
    for record in RIGHT_EXTRA_DOMAINS:
        G=np.asarray(record['nominal_TCP_object_T'],float)
        if not rigid(G):raise ValueError('Additional public grasp domain must be rigid and finite')
        candidates.append((record['label'],G.copy()))
    for label,G in candidates:
        state=_prepare_domain(checkers,art,indices,fingers,paths,goals,config,cache_dir,G)
        if state and state.get('certificate'):
            state['domain_label']=label;domains.append(state)
    if not domains:return dict(ready=False,reason='No stored grasp domain prepared')
    result=dict(domains[0]);result['domains']=domains
    result['summary']=dict(domains[0]['summary'],preparation_s=sum(x['summary'].get('preparation_s',0)for x in domains),cache_hit=all(x['summary'].get('cache_hit',False)for x in domains),independently_compiled_public_G_domains=[dict(x['summary'],domain_label=x['domain_label'])for x in domains],public_G_unchanged=True,class_symmetry_assumed=False)
    return result


def select_domain(state,current_hand_G,width=None):
    from fixed_static_route_certificate import grasp_in_envelope
    if not state:return None
    for domain in state.get('domains',[state]):
        cert=domain.get('certificate')
        if cert is None:continue
        m=cert.metadata;lo,hi=m['width_interval_m']
        if width is not None and not lo+1e-9<=float(width)<=hi-1e-9:continue
        if grasp_in_envelope(np.asarray(m['nominal_hand_object_T']),current_hand_G,m['translation_m'],m['rotation_rad']):return domain
    return None


def eligible(c,part,G,width):
    if part!='ring1' or not np.array_equal(np.asarray(c['goals'][part]['P']),np.asarray(META['goal'])):return False
    from isaac_kinematics import frame
    return select_domain(c.get('fixed_prevalidated_routes'),frame([0,0,.1034],np.eye(3))@np.asarray(G),width)is not None


@contextmanager
def static_scope(c, checker):
    state = select_domain(c.get('fixed_prevalidated_routes'),checker.payload_relative)
    if not state or not state.get('certificate'):
        yield {}
        return
    from fixed_static_route_certificate import bound_static_rows
    with bound_static_rows(checker, state['certificate'],
        current_world_sha256=state['world_sha256'],
        current_model_sha256=state['model_sha256']) as stats:
        stats['public_G_domain']=state.get('domain_label','nominal_public_G')
        from fixed_own_route_certificate import bound_own_rows
        own=state.get('own_certificate')
        if own is None:
            yield stats
        else:
            with bound_own_rows(checker,own,current_world_sha256=state['world_sha256'],current_model_sha256=state['model_sha256']) as own_stats:
                yield stats
            stats.update(own_stats)


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
        start=c['cmd'][0,ids].copy()
        # The unchanged current-context planner checks obstacles instead of
        # assuming a straight joint bridge to an archived entry is reachable.
        # This proposal still receives move()'s FULL actual/command aperture
        # and exact retimed-curve admission before its first physical row.
        with context(G):
            bridge=checker.route(start,rows[0],float(max(widths)),seconds=3.)
        if bridge is None:
            c['planning_diagnostics'].append(dict(event='saved_carry_entry_bridge_unavailable',no_motion_issued=True,original_current_start_planner_required=True))
            return False
        move(bridge,.05,'join_saved_carry',G=G,smooth=bool(getattr(bridge,'verified_cubic',False)))
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
