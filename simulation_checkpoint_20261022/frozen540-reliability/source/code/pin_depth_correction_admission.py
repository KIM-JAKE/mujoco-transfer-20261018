"""Readmit the exact correction field with the last image-measured PIN grasp.

Uses the original terminal division: intended PIN/jig contact is allowed;
robot/static geometry and the complete loaded PIN/opposite-arm pair are not.
This module plans only, never steps physics or reads free object poses.
"""
import time
import numpy as np
from isaac_kinematics import frame


def admit(ctx,body_G,commands):
    from pin_current_terminal import _other_contexts
    from aperture_path_check import _samples
    from predictive_pair_clearance import pair_clear
    from dual_aperture_path_check import path_other_contexts
    checker=ctx['checker'];actual=ctx['art'].get_dof_positions()[0].copy()
    rows=np.asarray(commands,float);ids=ctx['ids'];fi=ctx['fi']
    if rows.ndim!=2 or rows.shape[1]!=7 or len(rows)<2 or not np.isfinite(rows).all():raise ValueError('Finite exact retimed PIN correction field required')
    if not np.array_equal(rows[0],ctx['cmd'][0,ids]):raise RuntimeError('PIN correction source command changed before admission')
    widths=np.unique(np.linspace(min(float(actual[fi].sum()),float(ctx['cmd'][0,fi].sum())),max(float(actual[fi].sum()),float(ctx['cmd'][0,fi].sum())),5))
    other,contexts=_other_contexts(ctx);full=np.vstack([actual[ids],rows])
    saved=(checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative)
    began=time.perf_counter()
    try:
        checker.attach_payload(ctx['paths']['workpiece_pin'],frame([0,0,.1034],np.eye(3))@np.asarray(body_G,float))
        for oq,ow in contexts:
            with checker.other_arm_at(other,oq,ow):
                for width in widths:
                    for _,_,sample in _samples(checker,full):
                        ok,why=pair_clear(checker,sample,float(width))
                        if not ok:return False,dict(stage='fresh_PIN_loaded_pair',collision=why,wall_s=time.perf_counter()-began)
        checker.detach_payload()
        ok,why=path_other_contexts(checker,other,contexts,full,widths,ctx['paths']['workpiece_pin'])
        if not ok:return False,dict(stage='original_robot_fixed_and_pair',collision=why,wall_s=time.perf_counter()-began)
    finally:checker.payload_path,checker.payload_rows,checker.payload_boxes,checker.payload_relative=saved
    return True,dict(stage='fresh_G_exact_retimed_depth_correction_admitted',issued_rows=len(rows)-1,current_RGBD_grasp=True,own_apertures=len(widths),other_contexts=len(contexts),original_intended_PIN_jig_contact_policy=True,original_robot_static_and_pair_checks=True,original_FR3_retiming_unchanged=True,wall_s=time.perf_counter()-began,uses_free_body_poses=False)
