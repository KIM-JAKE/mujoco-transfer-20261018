"""Filter an empty-Hand park against the ENTIRE saved carrier pair path.

The future parked encoder pose cannot be known before motion. This model
proof uses the exact planned float32 endpoint; the existing fresh actual
post-park/current-start/whole-path guard remains mandatory before RIGHT moves.
No free actor poses enter this helper. Unknown grasp domains retain the
original complete current-start planner rather than guessing a cached pose.
"""
import time
import numpy as np
from isaac_kinematics import frame


def _geometry_sha(left,right):
    # SAME canonical literal-CAD hash as the existing binary proof client.
    # Recompute on both sides of the synchronous request: object-version
    # checks alone would miss in-place edits to a proxy's numpy vertices.
    from gpu_pair_binary_protocol import KEYS,geometry_sha256
    def geometry(checker):
        return [{key:np.ascontiguousarray(shape[i],dtype=np.float64)
                 for i,key in enumerate(KEYS)}
                for _,_,shape in list(checker.pair_boxes)+list(checker.payload_boxes)]
    return geometry_sha256(geometry(left),geometry(right))


def _context_version(c,left,right):
    from gpu_pair_schedule_scope import _geometry_version
    def array_version(value):
        a=np.ascontiguousarray(value)
        return(a.dtype.str,a.shape,a.tobytes())
    # No physical steps are performed by this filter. Public state snapshots
    # additionally reject an unexpected concurrent encoder/command change.
    def model_version(checker):
        arm=checker.arm
        return(tuple((name,array_version(getattr(arm,name)))for name in ['base','tip','axes','frames','bounds']if hasattr(arm,name)),tuple(getattr(arm,'link_paths',[])),getattr(arm,'hand_path',None))
    actual=None if 'art'not in c else array_version(c['art'].get_dof_positions())
    command=None if 'cmd'not in c else array_version(c['cmd'])
    return(_geometry_version(left),_geometry_version(right),model_version(left),model_version(right),
           c.get('_collision_physics_epoch'),actual,command)


def _pair_candidate_filter(c,left,right,contexts,qs,widths):
    from aperture_path_check import _samples
    from gpu_pair_schedule import prove
    config=c['config'];began=time.perf_counter();stats=dict(candidate_filter_only=True,
        full_actual_motion_checks_deferred_not_waived=True,
        no_physics_or_commands_issued=True,uses_free_body_poses=False)
    if not config.get('gpu_pair_schedule_certificate',False):
        return False,dict(stats,fallback_reason='GPU candidate filter disabled')
    socket=config.get('gpu_pair_schedule_socket','')
    if not str(socket).endswith('pair-binary.sock'):
        return False,dict(stats,fallback_reason='Original whole future guard retained for nonbinary transport')
    try:
        before=_context_version(c,left,right);sha_before=_geometry_sha(left,right)
        samples=[q for _,_,q in _samples(right,qs)]
        pairs=[(oq,float(ow),q,float(w))for q in samples for w in widths for oq,ow in contexts]
        service={};mask=np.asarray(prove(left,right,pairs,dict(socket=socket,
            timeout_s=float(config.get('gpu_pair_schedule_timeout_s',5.)),diagnostics=service)))
        sha_after=_geometry_sha(left,right);after=_context_version(c,left,right)
        # The existing client already enforces request identity, exact local
        # CAD SHA, original 1mm pair margin, float64 and 1nm outward reserve.
        # Retain those facts explicitly on this independently consumed mask.
        if before!=after or sha_before!=sha_after:raise RuntimeError('Candidate geometry/public context changed during proof')
        if service.get('geometry_sha256')!=sha_before:raise RuntimeError('Registered candidate geometry SHA mismatch')
        if mask.shape!=(len(pairs),)or mask.dtype!=np.bool_:raise RuntimeError('Candidate proof mask is not exact-row boolean CLEAR')
        if service.get('certificate')!='original_proxy_separation_only'or service.get('proposal_only')is not False or service.get('float64')is not True or service.get('outward_reserve_m')!=1e-9 or service.get('original_pair_margin_m')!=.001:
            raise RuntimeError('Candidate proof policy differs from original conservative pair proof')
        # Diagnostic masks are large and add no authority beyond their count.
        service.pop('all_pairs_clear',None)
        stats.update(sampled_rows=len(samples),pair_states=len(pairs),
            GPU_CLEAR_states=int(mask.sum()),GPU_unknown_states=int((~mask).sum()),
            geometry_sha256_before=sha_before,geometry_sha256_after=sha_after,
            exact_geometry_bindings_and_public_state_unchanged=True,service=service,
            wall_s=time.perf_counter()-began)
        if len(mask)and bool(mask.all()):return True,stats
        return False,dict(stats,fallback_reason='GPU uncertainty is NOT collision; original whole future guard required')
    except Exception as error:
        return False,dict(stats,fallback_reason=type(error).__name__+': '+str(error),
            wall_s=time.perf_counter()-began)


def check(c,park_q,G,right_widths,left_widths):
    from fixed_canonical_plan import right_carry
    from fixed_saved_executor import select_domain,static_scope
    from serial_gpu_pair_scope import serial_path_scope
    from dual_aperture_path_check import path_other_contexts
    rc,lc=c['checkers']['right'],c['checkers']['left']
    G=np.asarray(G,float);widths=np.unique(np.asarray(right_widths,float));left_widths=np.unique(np.asarray(left_widths,float))
    planned=np.asarray(park_q,float).copy();handG=frame([0,0,.1034],np.eye(3))@G
    state=c.get('fixed_prevalidated_routes')
    selected=[select_domain(state,handG,float(w))for w in widths]
    if not len(widths)or not len(left_widths)or any(x is None for x in selected):
        return dict(applies=False,clear=None,reason='No independently compiled matching current public G/width domain; original full current-start planner required',public_G_unchanged=True,uses_free_body_poses=False)
    if planned.shape!=(7,)or not np.isfinite(planned).all():raise ValueError('Finite exact future LEFT command required')
    if not rc.payload_path or rc.payload_relative is None or not np.array_equal(np.asarray(rc.payload_relative),handG):
        raise RuntimeError('Future saved-carry proof requires attached current RGB-D held CAD')
    if lc.payload_path:raise RuntimeError('Future saved-carry park requires logically empty LEFT CAD')
    qs=np.asarray(right_carry(),float)
    contexts=[(planned.copy(),float(w))for w in left_widths]
    began=time.perf_counter();stats={}
    pair_clear,pair_filter=_pair_candidate_filter(c,lc,rc,contexts,qs,widths)
    if pair_clear:
        # Selecting a candidate LEFT park only needs the varying arm-pair
        # component. Full terminal and LEFT issued-route checks still occur
        # before LEFT moves; fresh ACTUAL postpark RIGHT own/fixed/placed/pair
        # admission remains mandatory before any saved RIGHT row is issued.
        return dict(applies=True,clear=True,collision=None,rows=len(qs),
            wall_s=time.perf_counter()-began,planned_future_LEFT_q=planned.tolist(),
            left_widths_m=left_widths.tolist(),right_widths_m=widths.tolist(),
            candidate_pair_filter_only=True,full_saved_middle_pair_coverage_including_endpoint=True,
            hypothetical_static_own_placed_checks_deferred_to_original_actual_postpark_guard=True,
            original_full_terminal_and_issued_LEFT_guards_still_required=True,
            planned_endpoint_only_not_future_actual_encoder=True,
            fresh_actual_postpark_whole_path_required=True,public_G_unchanged=True,
            uses_free_body_poses=False,query_diagnostics=dict(future_candidate_pair_filter=pair_filter))
    # Every original q interpolation and BOTH Hand aperture combinations are
    # kept. Only existing conservatively qualified static/own/pair CLEAR bits
    # may avoid an original query; all uncertainty retains its original test.
    with rc.other_arm_at(lc,*contexts[0]):
        with static_scope(c,rc)as stored:
            with serial_path_scope(c,rc,lc,contexts,qs,widths):
                clear,why=path_other_contexts(rc,lc,contexts,qs,widths,diagnostics=stats)
        stats.update(stored)
    stats['future_candidate_pair_filter']=pair_filter
    return dict(applies=True,clear=bool(clear),collision=why,rows=len(qs),wall_s=time.perf_counter()-began,
        planned_future_LEFT_q=planned.tolist(),left_widths_m=left_widths.tolist(),right_widths_m=widths.tolist(),
        full_saved_middle_including_endpoint=True,existing_static_own_and_original_live_guards=True,
        planned_endpoint_only_not_future_actual_encoder=True,fresh_actual_postpark_whole_path_required=True,
        public_G_unchanged=True,uses_free_body_poses=False,query_diagnostics=stats)
