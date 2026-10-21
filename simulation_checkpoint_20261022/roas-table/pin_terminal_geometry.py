"""Nominal terminal branch admission from current image grasp and robot model.

Planning only. No simulator free-body pose input or actuation. The pin is allowed
its intended jig contact, while the complete robot and carried-pin/other-arm
pair remain protected by the original geometry checkers.
"""
from contextlib import nullcontext
import numpy as np
from scipy.interpolate import CubicSpline
from fr3_path_timing import retime
from predictive_pair_clearance import pair_clear
from gpu_route_curve_validation import issued_geometry_samples

def check_terminal(ctx,arm,checker,q,depth,opening,selected,margin=.03):
    q=np.asarray(q,float);depth=float(depth);bounds=np.asarray(arm.bounds)
    if q.shape!=(7,)or not np.isfinite(q).all()or not .035<=depth<=.055 or not .0<=opening<=.08:raise ValueError('Invalid terminal stroke input')
    H=arm.fk(q);rows=[q.copy()]
    for u in np.arange(1,13)/12:
        target=H.copy();target[2,3]+=depth*u;nextq=arm.solve(target[:3,3],H[:3,:3],rows[-1])
        if nextq is None:return False,dict(reason='full_axial_IK',fraction=float(u))
        rows.append(nextq)
    rows=np.asarray(rows)
    nominal_margin=float(np.minimum(rows-bounds[:,0],bounds[:,1]-rows).min())
    if nominal_margin<margin:return False,dict(reason='terminal_joint_margin',minimum_rad=nominal_margin,required_rad=float(margin))
    # Same nominal stroke/withdraw curve and original FR3 retiming, with a
    # stricter placement reserve to leave room for later image corrections.
    DT=float(ctx['dt']);width=np.full(2,opening/2)
    def issued(points,fingers):
        curve,_,_=retime(points,.8,DT,fingers,fingers,velocity_fraction=.95,acceleration=9.5,jerk=4750.,finger_command_speed=.0475,optimize=bool(ctx.get('optimized_motion_timing',False)))
        return curve
    stroke=issued(rows,width);withdraw=issued(rows[::-1],np.full(2,.009))
    minimum=float(np.minimum(stroke-bounds[:,0],bounds[:,1]-stroke).min())
    if minimum<margin:return False,dict(reason='terminal_joint_margin',minimum_rad=minimum,required_rad=float(margin))
    stroke_geometry,stroke_sampling=issued_geometry_samples(stroke,step_rad=.003)
    withdraw_geometry,withdraw_sampling=issued_geometry_samples(withdraw,step_rad=.003)
    saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative)
    other=getattr(checker,'other_configuration',None)
    context=checker.other_arm_at(*other())if other is not None else nullcontext()
    old_resolution=getattr(checker,'resolution_rad',.035)
    with context:
        checker.resolution_rad=min(old_resolution,.010)
        # Separate carried pin vs other arm from intended pin/jig contact.
        if checker.preview_other_boxes is None:
            checker.resolution_rad=old_resolution
            return False,dict(reason='missing_current_other_arm_geometry')
        try:
            for sample in stroke_geometry:
                ok,why=pair_clear(checker,sample,opening)
                if not ok:return False,dict(reason='loaded_terminal_pair',collision=why)
            checker.detach_payload()
            ok,why=checker.path(stroke_geometry,opening,selected)
            if not ok:return False,dict(reason='terminal_robot_fixed_pair',collision=why)
            # Full release aperture sweep at seated pose, then empty reverse.
            for w in np.linspace(opening,.018,13):
                ok,why=checker.clear(stroke[-1],float(w),selected)
                if not ok:return False,dict(reason='terminal_release_sweep',collision=why)
            ok,why=checker.path(withdraw_geometry,.018,selected)
            if not ok:return False,dict(reason='terminal_empty_withdraw',collision=why)
        finally:
            checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved
            checker.resolution_rad=old_resolution
    return True,dict(reason='full_nominal_stroke_release_withdraw_approved',minimum_joint_margin_rad=minimum,required_joint_margin_rad=float(margin),stroke_samples=len(stroke),withdraw_samples=len(withdraw),stroke_geometry_sampling=stroke_sampling,withdraw_geometry_sampling=withdraw_sampling,all_issued_FR3_array_checks_retained=True,uses_free_body_poses=False,robot_encoder_other_arm=True,actual_execution_not_performed=True)
