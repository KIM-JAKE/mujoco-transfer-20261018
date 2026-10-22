"""Replan only a rejected BEFORE-MOTION empty-Hand withdrawal.

The original Cartesian withdrawal is first choice. A specific preflight-only
collision error permits alternate IK/routes to the SAME worldZ-80mm target.
Actuator/camera/unexpected errors are never retried. All alternatives pass the
original exact-issued move guard before their first commanded row.
"""
import time
from contextlib import contextmanager
import numpy as np
from isaac_kinematics import log


@contextmanager
def _released_part_scope(c,part,seated_world,uncertainty_m):
    from placed_part_clearance import PlacedPartClearance
    checker=c['checker'];sentinel=object();previous=getattr(checker,'placed_part_clearance',sentinel)
    placed=PlacedPartClearance()
    if previous is not sentinel:
        # A pre-existing completed ring keeps its original, possibly larger
        # reserve. This new-release lifecycle must never silently replace it.
        if part in previous.parts:raise RuntimeError('Released ring already registered in completed-part world; explicit lifecycle reconciliation required')
        placed.parts=dict(previous.parts)
        placed.generation=previous.generation
    placed.register(checker,part,c['part_collision_templates'][part],seated_world,uncertainty_m=uncertainty_m)
    checker.placed_part_clearance=placed
    try:yield placed
    finally:
        if previous is sentinel:delattr(checker,'placed_part_clearance')
        else:checker.placed_part_clearance=previous


def execute(c,part,target,width,approved,seated_world,seated_RGBD,line,move,query_context,record,preflight_error):
    try:
        line(target,.6,'world_down_withdraw',width=width)
        return dict(strategy='original_checked_Cartesian_world_down',same_original_target=True)
    except preflight_error as blocked:
        # This type can only be raised after the original issued-curve guard
        # rejects AND before move() issues ANY row. A partial-motion failure
        # has a different type and propagates without replanning.
        original_rejection=str(blocked)
    arm,checker,art,cmd,ids,fi=[c[k]for k in ['arm','checker','art','cmd','ids','fi']]
    current=cmd[0,ids].copy();measured=art.get_dof_positions()[0].copy()
    actual=measured[ids].copy();openings=np.unique(np.linspace(min(float(measured[fi].sum()),float(cmd[0,fi].sum())),max(float(measured[fi].sum()),float(cmd[0,fi].sum())),5))
    target=np.asarray(target,float);began=time.perf_counter();rejections=[]
    from fr3_gpu_ik import propose_ik
    proposals=propose_ik(c,arm,target[:3,3],target[:3,:3],actual,'empty_RIGHT_postrelease_same_target_withdraw')
    withdraw_seed=np.asarray(approved['withdraw_q'][-1],float)
    seeds=[current,actual,withdraw_seed,np.asarray(approved['pre_q'],float)]
    # The same Cartesian endpoint has redundant elbow branches. A bounded
    # four-seed family proposes alternatives to a blocked original branch;
    # these offsets never alter a command or an endpoint/clearance tolerance.
    for joint in [0,2]:
        for delta in [-.2,.2]:
            seed=withdraw_seed.copy();seed[joint]+=delta;seeds.append(seed)
    for seed in seeds:
        q=arm.solve(target[:3,3],target[:3,:3],seed)
        if q is not None:proposals.append(q)
    unique=[]
    for q in sorted(proposals,key=lambda q:float(np.linalg.norm(q-current))):
        if all(np.linalg.norm(q-old)>.025 for old in unique):unique.append(np.asarray(q,float))
    # This is an ADDITIONAL obstacle absent from the original straight-line
    # release policy. Existing completed parts keep their original3.5mm
    # reserve. The freshly exposed seated ring uses a recorded residual-based
    # reserve (>=1mm); this is a simulation-model reserve, not a hardware CI.
    residuals=[float(seated_RGBD[k])for k in ['rms_m','lateral_CAD_registration_rms_m']if k in seated_RGBD]
    if residuals and not np.isfinite(residuals).all():raise RuntimeError('Nonfinite released-ring RGB-D residual')
    uncertainty=max(.001,3*max(residuals))if residuals else .0035
    with _released_part_scope(c,part,np.asarray(seated_world,float),uncertainty)as placed:
        with query_context():
            for index,q in enumerate(unique[:8]):
                H=arm.fk(q)
                # The original CPU IK endpoint tolerances remain unchanged;
                # GPU outputs are proposal-only until these exact checks.
                if np.linalg.norm(H[:3,3]-target[:3,3])>=1e-5 or np.linalg.norm(log(target[:3,:3]@H[:3,:3].T))>=1e-4:
                    rejections.append(dict(candidate=index,stage='original_IK_residual'));continue
                if not all(checker.clear(q,float(gap))[0]for gap in openings):
                    rejections.append(dict(candidate=index,stage='current_full_endpoint'));continue
                route=checker.route(current,q,float(max(openings)),seconds=3.)
                if route is None:
                    rejections.append(dict(candidate=index,stage='current_full_route'));continue
                try:move(route,.6,'world_down_withdraw',width=width,smooth=bool(getattr(route,'verified_cubic',False)and c['config'].get('right_direct_smooth_verified_lines')))
                except preflight_error as blocked:
                    rejections.append(dict(candidate=index,stage='full_exact_issued_curve',reason=str(blocked)));continue
                evidence=dict(strategy='current_obstacle_aware_empty_RIGHT_same_target',same_original_target=True,target_world_TCP=target.tolist(),candidate=index,candidates=len(unique),original_pre_motion_rejection=original_rejection,earlier_rejections=rejections,elapsed_s=time.perf_counter()-began,fresh_actual_q=actual.tolist(),command_q=current.tolist(),current_openings_m=openings.tolist(),fresh_RGBD_released_part_scoped=True,released_part_uncertainty_m=uncertainty,released_part_reserve_basis='max(1mm,3x fresh RGBD circle/lateral fit residual); simulation reserve, not calibrated hardware confidence',placed_statistics=dict(placed.stats),original_exact_issued_guards=True,uses_free_body_poses=False)
                c['planning_diagnostics'].append(dict(event='empty_RIGHT_postrelease_checked_withdraw_fallback',**evidence));return evidence
    raise RuntimeError('No full-current-context empty RIGHT withdrawal to the original world-down target '+str(dict(original=original_rejection,rejections=rejections)))
