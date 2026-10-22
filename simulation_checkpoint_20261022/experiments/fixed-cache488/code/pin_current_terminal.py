"""Fresh-image PIN terminal branch admission and fully checked local rebranch.

No actuation occurs here. Current camera grasp and robot encoders are the only
pose inputs. Returned exact commands require another original precision image
qualification BEFORE insertion. No failed partial stroke is ever resumed.
"""
from contextlib import contextmanager
import time
import numpy as np
from scipy.interpolate import CubicSpline
from isaac_kinematics import frame
from fr3_path_timing import retime,retime_safe_polyline


@contextmanager
def _held(ctx,body_G):
    checker=ctx['checker'];saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative)
    checker.attach_payload(ctx['paths']['workpiece_pin'],frame([0,0,.1034],np.eye(3))@np.asarray(body_G,float))
    try:yield checker
    finally:checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved


def _other_contexts(ctx):
    checker=ctx['checker'];callback=getattr(checker,'other_configuration',None)
    if callback is None:raise RuntimeError('Fresh PIN terminal replan requires current opposite-arm SDK context')
    other,actual_q,actual_gap=callback()
    names=list(ctx['art'].shared_metatype.dof_names);side=other.arm.hand_path.split('/')[-1].split('_')[0]
    ids=[names.index(side+'_arm_fr3v2_joint'+str(i))for i in range(1,8)]
    fi=[i for i,name in enumerate(names)if name.startswith(side+'_native')]
    if len(fi)!=2:raise RuntimeError('Fresh PIN terminal needs original native opposite Hand aperture')
    command_q=ctx['cmd'][0,ids].copy();command_gap=float(ctx['cmd'][0,fi].sum())
    widths=np.unique(np.linspace(min(actual_gap,command_gap),max(actual_gap,command_gap),5))
    return other,[(np.asarray(q,float),float(w))for q in [actual_q,command_q]for w in widths]


def _timed(ctx,knots,requested,checked_edges):
    start=ctx['cmd'][0,ctx['fi']].copy();dt=float(ctx['dt']);scale=max(1.,float(ctx.get('stability_time_scale',1.)))
    options=dict(velocity_fraction=.95,acceleration=9.5,jerk=4750.)
    if checked_edges and len(knots)>2:
        q,h,timing=retime_safe_polyline(knots,dt,start,start,scale=scale,**options)
    else:
        q,h,timing=retime(knots,requested,dt,start,start,finger_command_speed=.0475,optimize=bool(ctx.get('optimized_motion_timing',False)),**options)
        if scale>1:q,h,timing=retime(knots,timing['duration_s']*scale,dt,start,start,finger_command_speed=.0475,optimize=False,**options)
    return q,timing


def _line_knots(arm,q,depth):
    H=arm.fk(q);knots=[np.asarray(q,float).copy()]
    for u in np.linspace(0,1,7)[1:]:
        target=H.copy();target[2,3]+=float(depth)*u
        nextq=arm.solve(target[:3,3],H[:3,:3],knots[-1])
        if nextq is None:return None
        knots.append(nextq)
    return np.asarray(knots)


def assess(ctx,body_G,depth,q=None):
    """Original complete terminal policy at the LAST camera-qualified G."""
    from pin_terminal_geometry import check_terminal
    from predictive_pair_clearance import pair_clear
    from aperture_path_check import _samples
    from dual_aperture_path_check import path_other_contexts
    arm,checker,art,ids,fi,cmd=[ctx[k]for k in ['arm','checker','art','ids','fi','cmd']]
    actual=art.get_dof_positions()[0].copy();is_current=q is None;q=np.asarray(cmd[0,ids]if is_current else q,float)
    widths=np.unique(np.linspace(min(float(actual[fi].sum()),float(cmd[0,fi].sum())),max(float(actual[fi].sum()),float(cmd[0,fi].sum())),5))
    began=time.perf_counter();other,contexts=_other_contexts(ctx)
    with _held(ctx,body_G):
        with checker.other_arm_at(other,*contexts[0]):
            ok,evidence=check_terminal(ctx,arm,checker,q,float(depth),float(max(widths)),ctx['paths']['workpiece_pin'],margin=.03)
            if not ok:return False,dict(stage='original_full_terminal',evidence=evidence,wall_s=time.perf_counter()-began)
            knots=_line_knots(arm,q,depth)
            if knots is None:return False,dict(stage='original_single_line_IK')
            # Same7knots/41curve as line(), not an earlier13-knot surrogate.
            curve=CubicSpline(np.linspace(0,1,len(knots)),knots)(np.linspace(0,1,41))
            # Nominal candidate q may differ from current q. Retiming must
            # retain this proposed start instead of overwriting it with cmd.
            field,timing=_timed(ctx,knots,.8,False)
            full=np.vstack([actual[ids],q,field])if is_current else np.vstack([q,field])
            for oq,ow in contexts:
                with checker.other_arm_at(other,oq,ow):
                    for width in widths:
                        for _,_,sample in _samples(checker,full):
                            ok,why=pair_clear(checker,sample,float(width))
                            if not ok:return False,dict(stage='fresh_loaded_PIN_vs_other',collision=why)
            saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative)
            checker.detach_payload()
            try:
                for points,stage in [(curve,'same_original_7knot_line'),(full,'full_exact_retimed_stroke')]:
                    ok,why=path_other_contexts(checker,other,contexts,points,widths,ctx['paths']['workpiece_pin'])
                    if not ok:return False,dict(stage=stage,collision=why,wall_s=time.perf_counter()-began)
            finally:checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved
    return True,dict(stage='fresh_full_terminal_admitted',original_terminal=evidence,exact_line_knots=len(knots),exact_issued_rows=len(field)-1,current_image_grasp=True,depth_m=float(depth),wall_s=time.perf_counter()-began,uses_free_body_poses=False)


def plan_rebranch(ctx,body_G,depth):
    """Propose same-Hand-pose redundant IK, then admit full loaded issue field."""
    from fr3_gpu_ik import propose_ik
    from dual_aperture_path_check import path_other_contexts
    arm,checker,art,cmd,ids,fi=[ctx[k]for k in ['arm','checker','art','cmd','ids','fi']]
    measured=art.get_dof_positions()[0].copy();actual=measured[ids].copy();start=cmd[0,ids].copy();H=arm.fk(start)
    widths=np.unique(np.linspace(min(float(measured[fi].sum()),float(cmd[0,fi].sum())),max(float(measured[fi].sum()),float(cmd[0,fi].sum())),5))
    candidates=propose_ik(ctx,arm,H[:3,3],H[:3,:3],actual,'PIN_fresh_terminal_same_prepose_branch')
    seeds=[start,actual]
    for joint in [2,4]:
        for delta in [-.2,.2]:seed=start.copy();seed[joint]+=delta;seeds.append(seed)
    for seed in seeds:
        q=arm.solve(H[:3,3],H[:3,:3],seed)
        if q is not None:candidates.append(q)
    unique=[]
    for q in sorted(candidates,key=lambda q:float(np.linalg.norm(q-start))):
        if np.linalg.norm(q-start)>.025 and all(np.linalg.norm(q-old)>.025 for old in unique):unique.append(np.asarray(q,float))
    began=time.perf_counter();failures=[];other,contexts=_other_contexts(ctx)
    for index,q in enumerate(unique[:10]):
        ok,terminal=assess(ctx,body_G,depth,q)
        if not ok:failures.append(dict(candidate=index,terminal=terminal));continue
        with _held(ctx,body_G):
            with checker.other_arm_at(other,*contexts[0]):
                route=checker.route(start,q,float(max(widths)),ctx['paths']['workpiece_pin'],seconds=2.)
                if route is None:failures.append(dict(candidate=index,reason='loaded_current_route'));continue
                knots=np.asarray(route,float).copy();knots[0]=start
                field,timing=_timed(ctx,knots,.35,not bool(getattr(route,'verified_cubic',False)))
                path=np.vstack([actual,start,field]);stats={}
                old=getattr(checker,'resolution_rad',.035);checker.resolution_rad=min(old,.005)
                try:ok,why=path_other_contexts(checker,other,contexts,path,widths,ctx['paths']['workpiece_pin'],diagnostics=stats)
                finally:checker.resolution_rad=old
                if not ok:failures.append(dict(candidate=index,reason='full_loaded_exact_field',collision=why));continue
        return dict(commands=field,timing=timing,terminal=terminal,candidate=index,failures=failures,source_command=start.copy(),source_actual=actual.copy(),source_command_all=cmd.copy(),source_actual_all=measured.copy(),wall_s=time.perf_counter()-began,original_all_limits_and_collision_checks=True,same_preinsert_Hand_pose=True,fresh_precision_RGBD_required_after_rebranch=True,uses_free_body_poses=False,issued_guard=stats)
    raise RuntimeError('Current RGB-D PIN has no fully checked terminal rebranch '+str(failures[-4:]))
