"""Before an empty-Hand park, admit the ENTIRE saved carrier middle.

The future parked encoder pose cannot be known before motion. This model
proof uses the exact planned float32 endpoint; the existing fresh actual
post-park/current-start/whole-path guard remains mandatory before RIGHT moves.
No free actor poses enter this helper. Unknown grasp domains retain the
original complete current-start planner rather than guessing a cached pose.
"""
import time
import numpy as np
from isaac_kinematics import frame


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
    # Every original q interpolation and BOTH Hand aperture combinations are
    # kept. Only existing conservatively qualified static/own/pair CLEAR bits
    # may avoid an original query; all uncertainty retains its original test.
    with rc.other_arm_at(lc,*contexts[0]):
        with static_scope(c,rc)as stored:
            with serial_path_scope(c,rc,lc,contexts,qs,widths):
                clear,why=path_other_contexts(rc,lc,contexts,qs,widths,diagnostics=stats)
        stats.update(stored)
    return dict(applies=True,clear=bool(clear),collision=why,rows=len(qs),wall_s=time.perf_counter()-began,
        planned_future_LEFT_q=planned.tolist(),left_widths_m=left_widths.tolist(),right_widths_m=widths.tolist(),
        full_saved_middle_including_endpoint=True,existing_static_own_and_original_live_guards=True,
        planned_endpoint_only_not_future_actual_encoder=True,fresh_actual_postpark_whole_path_required=True,
        public_G_unchanged=True,uses_free_body_poses=False,query_diagnostics=stats)
