"""Optional exact PIN tape, sealed fixed/own proofs and live stopped bridges.

No dynamic other-arm, installed-part, G/width, native contact or hardware guard
is skipped. A nominal tape is a proposal until its complete current field passes
its original measured-aperture guard. Preparation performs no actuation.
"""
from pathlib import Path
from contextlib import contextmanager
import hashlib,json,time,os
import numpy as np
from fixed_pin_carry_plan import META,command_rows,proof_metadata


def audit_field(rows):
    from fr3_motion_control import position_velocity_bounds,LOWER,UPPER
    q=np.asarray(rows,float);dt=META['dt_s']
    if q.ndim!=2 or q.shape[1]!=7 or len(q)<2 or not np.isfinite(q).all():
        raise RuntimeError('Invalid exact PIN command field')
    if np.any(q<LOWER) or np.any(q>UPPER):raise RuntimeError('Exact PIN field exceeds original FR3 position limits')
    v=np.diff(q,axis=0,prepend=q[:1])/dt
    a=np.diff(v,axis=0,prepend=np.zeros((1,7)))/dt
    jerk=np.diff(a,axis=0,prepend=np.zeros((1,7)))/dt
    lo,hi=position_velocity_bounds(q);cap=.95*np.where(v>=0,hi,-lo)
    peaks=[float(np.max(abs(v)/np.maximum(cap,1e-8))),float(np.max(abs(a))),float(np.max(abs(jerk)))]
    if not np.isfinite(peaks).all() or peaks[0]>1 or peaks[1]>9.5 or peaks[2]>4750:
        raise RuntimeError('Exact PIN field exceeds unchanged original FR3 limits '+str(peaks))
    return dict(fraction_of_95_percent_PBV=peaks[0],maximum_acceleration_rad_s2=peaks[1],maximum_jerk_rad_s3=peaks[2])


def _calibration_matches(arm):
    return all(np.array_equal(np.asarray(getattr(arm,k),float),np.asarray(v,float))
               for k,v in META['robot_calibration'].items())


def _hand_G(axis_G):
    from pin_native_contact import axis_to_body_grasp
    from isaac_kinematics import frame
    return frame([0,0,.1034],np.eye(3))@axis_to_body_grasp(axis_G)


def prepare(checkers,art,indices,fingers,paths,goals,config,cache_dir,*,outside_cycle=False):
    if not config.get('pin_saved_canonical_carry',False):return None
    meta=proof_metadata(config)
    if not(config.get('fixed_supply_mode')and config.get('pin_measured_carry_aperture_guard')):
        raise ValueError('Saved PIN carry requires fixed supply and original full-width measured carry guard')
    from fixed_static_route_certificate import (StaticRows,static_world_seal,
        compile_static_rows,PrismaticFixtureSweeps,payload_geometry_sha256)
    from exact_collision_proof_cache import own_model_fingerprint
    from fixed_saved_proof_rows import rows_and_identity
    rc,lc=checkers['right'],checkers['left'];began=time.perf_counter()
    if not _calibration_matches(rc.arm):return dict(ready=False,reason='Exact stored robot calibration changed')
    if not np.array_equal(np.asarray(goals['pin']['P']),np.asarray(meta['body_origin_goal_world_m'])):
        return dict(ready=False,reason='Original PIN jig body calibration changed')
    if not np.array_equal(np.asarray(config.get('pin_axis_goal_world_m',[])),np.asarray(meta['goal_axis_world_m'])):
        return dict(ready=False,reason='PIN physical-axis calibration changed')
    rows=command_rows();audit=audit_field(rows);proof_rows,proof_identity=rows_and_identity(rows)
    seal=static_world_seal(rc);model=own_model_fingerprint(rc.same_arm_clearance)
    G=_hand_G(np.asarray(meta['nominal_axis_TCP_G']))
    identity=dict(meta=meta,proof_rows=proof_identity,executor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    key=hashlib.sha256(json.dumps(identity,sort_keys=True,separators=(',',':')).encode()+seal['sha256'].encode()+model.encode()).hexdigest()
    target=Path(cache_dir)/('pin-'+key+'.json');target.parent.mkdir(parents=True,exist_ok=True)
    saved=(rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative)
    try:
        rc.attach_payload(paths['workpiece_pin'],G);cache_hit=False
        if target.exists():
            try:
                cert=StaticRows.load(target);m=cert.metadata
                cache_hit=(m.get('pin_tape_identity')==identity and m['world_sha256']==seal['sha256']
                    and m['model_sha256']==model and m['payload_geometry_sha256']==payload_geometry_sha256(rc)
                    and m['width_interval_m']==meta['width_interval_m'] and m['nominal_hand_object_T']==G.tolist()
                    and m['translation_m']==meta['translation_m'] and m['rotation_rad']==meta['rotation_rad'])
            except(ValueError,KeyError,TypeError,OSError):cache_hit=False
        if not cache_hit:
            actual=art.get_dof_positions()[0]
            with rc.other_arm_at(lc,actual[indices['left']],float(actual[fingers['left']].sum())):
                cert,_,_=compile_static_rows(rc,proof_rows,meta['width_interval_m'],world_seal=seal,
                    model_sha256=model,nominal_hand_object_T=G,translation_m=meta['translation_m'],
                    rotation_rad=meta['rotation_rad'],fixture_sweep_clear=PrismaticFixtureSweeps(rc,seal))
            cert.metadata['pin_tape_identity']=identity
            temporary=target.with_suffix('.'+str(os.getpid())+'.tmp');cert.save(temporary);temporary.replace(target)
        from fixed_own_route_certificate import prepare_like_static
        # A compiler failure never creates a positive bit or bypasses a check.
        own=prepare_like_static(rc,proof_rows,cert,cache_dir)
        own['summary']=dict(own['summary'],compile_excluded_from_cycle=bool(outside_cycle))
        summary=dict(ready=bool(cert.row_keys),cache_hit=cache_hit,preparation_s=time.perf_counter()-began,
            canonical_rows=len(rows),proof_rows=len(proof_rows),static_rows=len(cert.row_keys),
            proof_domain=dict(translation_m=meta['translation_m'],rotation_rad=meta['rotation_rad'],
                entry_bridge_rad=meta['maximum_entry_joint_delta_rad'],exit_bridge_rad=meta['maximum_exit_joint_delta_rad']),
            own=own['summary'],proof_file=str(target),command_limit_audit=audit,
            compile_excluded_from_cycle=bool(outside_cycle),current_pairs_placed_width_G_and_bridges_remain_live=True)
        return dict(ready=summary['ready'],certificate=cert,own_certificate=own['certificate'],
            world_sha256=seal['sha256'],model_sha256=model,summary=summary)
    finally:rc.payload_path,rc.payload_rows,rc.payload_boxes,rc.payload_relative=saved


def prepare_from_context(ctx,cache_dir,*,outside_cycle=False):
    """Call after completed rings; first cold preparation time is counted."""
    if not ctx.get('config',{}).get('pin_saved_canonical_carry',False):return None
    checker=ctx['checker'];callback=getattr(checker,'other_configuration',None)
    if not callable(callback):return dict(ready=False,reason='No original opposite-arm encoder context')
    other,_,_=callback();names=list(ctx['art'].shared_metatype.dof_names)
    side=other.arm.hand_path.split('/')[-1].split('_')[0]
    left_ids=[names.index(side+'_arm_fr3v2_joint'+str(i))for i in range(1,8)]
    left_fi=[i for i,n in enumerate(names)if n.startswith(side+'_native')]
    if side!='left' or len(left_fi)!=2:raise RuntimeError('Saved PIN carry requires original native LEFT counterpart')
    return prepare({'right':checker,'left':other},ctx['art'],{'right':ctx['ids'],'left':left_ids},
        {'right':ctx['fi'],'left':left_fi},ctx['paths'],{'pin':{'P':ctx['goal']}},
        ctx['config'],cache_dir,outside_cycle=outside_cycle)


@contextmanager
def _proof_scope(state,checker,stats):
    from fixed_static_route_certificate import bound_static_rows
    from fixed_own_route_certificate import bound_own_rows
    with bound_static_rows(checker,state['certificate'],current_world_sha256=state['world_sha256'],current_model_sha256=state['model_sha256'])as static:
        own=state.get('own_certificate')
        if own is None:
            yield
        else:
            with bound_own_rows(checker,own,current_world_sha256=state['world_sha256'],current_model_sha256=state['model_sha256'])as own_stats:
                yield
            stats.update(own_stats)
        stats.update(static)


def _bridge(start,end,fingers,dt):
    from fr3_path_timing import retime
    a,b=np.asarray(start,float),np.asarray(end,float)
    if np.array_equal(a,b):return a[None,:].copy()
    q,_,_=retime([a,b],.05,dt,fingers,fingers,velocity_fraction=.95,
        acceleration=9.5,jerk=4750.,finger_command_speed=.0475,optimize=True)
    return q


def execute_if_ready(ctx,axis_G,body_G,goal_R,admit,check_aperture,step):
    """Return False BEFORE motion on a proposal miss; physical faults propagate."""
    if not ctx['config'].get('pin_saved_canonical_carry',False):return False
    meta=proof_metadata(ctx['config'])
    report=ctx['report'];began=time.perf_counter()
    def miss(reason):
        report.setdefault('pin_saved_carry_misses',[]).append(dict(reason=reason,no_motion_issued=True));return False
    if not ctx['config'].get('fixed_supply_mode'):return miss('Saved PIN carry requires original fixed-supply mode')
    if not ctx['config'].get('pin_measured_carry_aperture_guard'):raise RuntimeError('Saved PIN tape requires measured carry guard')
    if ctx.get('stability_time_scale',1.)!=1. or float(ctx['dt'])!=meta['dt_s']:
        return miss('Exact tape timing changed')
    if not _calibration_matches(ctx['arm']):return miss('Robot calibration changed')
    if not np.array_equal(np.asarray(ctx['config'].get('pin_axis_goal_world_m',[])),np.asarray(meta['goal_axis_world_m'])):
        return miss('PIN axis target calibration changed')
    if not np.array_equal(np.asarray(goal_R),np.asarray(meta['goal_part_R'])):return miss('Original target rotation changed')
    from fixed_static_route_certificate import grasp_in_envelope,static_world_seal
    from exact_collision_proof_cache import own_model_fingerprint
    from isaac_kinematics import frame
    hand_G=frame([0,0,.1034],np.eye(3))@np.asarray(body_G,float)
    if not np.array_equal(hand_G,_hand_G(np.asarray(axis_G,float))):return miss('Public axis/body PIN grasp conversion inconsistent')
    # This cheap CURRENT public-G gate runs BEFORE any cold compiler/world
    # scan. A class pose outside the independent domain uses the old planner.
    nominal=_hand_G(np.asarray(meta['nominal_axis_TCP_G'],float))
    if not grasp_in_envelope(nominal,hand_G,meta['translation_m'],meta['rotation_rad']):
        return miss('Fresh public PIN grasp outside independently proved domain')
    rows=command_rows().astype(float);start=ctx['cmd'][0,ctx['ids']].copy()
    if np.max(abs(start-rows[0]))>meta['maximum_entry_joint_delta_rad']:return miss('Entry is outside tiny bridge domain')
    target=frame(np.asarray(meta['goal_axis_world_m'])-[0,0,.045],np.asarray(goal_R))@np.linalg.inv(axis_G)
    endpoint=ctx['arm'].solve(target[:3,3],target[:3,:3],rows[-1])
    if endpoint is None or np.max(abs(endpoint-rows[-1]))>meta['maximum_exit_joint_delta_rad']:
        return miss('Canonical-seeded exact current-target exit unavailable')
    state=ctx.get('fixed_prevalidated_pin_routes')
    if state is None:
        cache_dir=ctx.get('_pin_saved_carry_cache_dir',ctx['config'].get('fixed_route_cache_dir'))
        if cache_dir is None:return miss('No deferred cache preparation directory configured')
        # Already AFTER the original before_transport/LEFT parking operation.
        # This cold preparation remains inside the task's original wall clock.
        state=prepare_from_context(ctx,cache_dir,outside_cycle=False)
        ctx['fixed_prevalidated_pin_routes']=state
        report['pin_saved_carry_preparation']=(state or {}).get('summary',state)
    if not(state and state.get('ready')):return miss('No prepared sealed PIN tape')
    identity=state['certificate'].metadata.get('pin_tape_identity',{})
    if identity.get('meta')!=meta or identity.get('executor_sha256')!=hashlib.sha256(Path(__file__).read_bytes()).hexdigest():
        return miss('Prepared certificate is not this exact PIN command tape/source')
    # Rebuild actual fixed/calibration/model seals once at admission; no proof
    # or posed verdict survives any unsealed scene/model change.
    if static_world_seal(ctx['checker'])['sha256']!=state['world_sha256'] or own_model_fingerprint(ctx['checker'].same_arm_clearance)!=state['model_sha256']:
        return miss('Current world/model seal changed')
    checker=ctx['checker'];saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative)
    from pin_terminal_geometry import check_terminal
    try:
        checker.attach_payload(ctx['paths']['workpiece_pin'],hand_G)
        actual=ctx['art'].get_dof_positions()[0];width=max(float(actual[ctx['fi']].sum()),float(ctx['cmd'][0,ctx['fi']].sum()))
        terminal,evidence=check_terminal(ctx,ctx['arm'],checker,endpoint,.045,width,ctx['paths']['workpiece_pin'],margin=.03)
    finally:checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved
    recorded=dict(evidence,label='pin_preinsert_carry',accepted=bool(terminal),candidate_source='stored_endpoint_seed_for_exact_current_public_target')
    report.setdefault('pin_terminal_branch_preflight',[]).append(recorded)
    print('PIN_TERMINAL_BRANCH_PREFLIGHT',recorded,flush=True)
    if not terminal:return miss('Original full current-target terminal rejected canonical-seeded exit '+str(evidence))
    fingers=ctx['cmd'][0,ctx['fi']].copy();entry=_bridge(start,rows[0],fingers,ctx['dt']);exit=_bridge(rows[-1],endpoint,fingers,ctx['dt'])
    full=np.vstack([entry,rows[1:],exit[1:]])
    audit=audit_field(full);stats={}
    local=dict(ctx,_pin_saved_carry_static_scope=lambda ck:_proof_scope(state,ck,stats))
    try:
        domain=admit(local,full,body_G,'pin_tail_direct_carry_to_preinsert')
    except RuntimeError as error:
        # This exact original admission result is issued before ANY row;
        # unexpected native errors and every physical-step fault propagate.
        if str(error).startswith('Fresh full-width loaded PIN carry blocked before motion '):
            return miss('Original complete current-context canonical field rejected '+str(error))
        raise
    # The above original FULL field checker admitted every bridge/current
    # other/placed/G/width context before the first physical command.
    admission_s=time.perf_counter()-began
    for q in full[1:]:
        check_aperture(ctx,domain)
        ctx['cmd'][0,ctx['ids']]=q
        step()
        check_aperture(ctx,domain)
    event=dict(event='pin_tail_direct_carry_to_preinsert',wall_s=time.perf_counter()-ctx['started'],physical_duration_s=(len(full)-1)*ctx['dt'])
    report['events'].append(event)
    report['pin_saved_canonical_carry']=dict(executed=True,exact_middle_rows=len(rows)-1,
        entry_rows=len(entry)-1,exit_rows=len(exit)-1,admission_wall_s=admission_s,
        proof_reuse=stats,original_terminal=evidence,command_limit_audit=audit,
        no_middle_retiming=True,all_current_pair_placed_G_width_checks_retained=True,
        uses_free_body_poses=False)
    print('PIN_SAVED_CANONICAL_CARRY',report['pin_saved_canonical_carry'],flush=True)
    return True
