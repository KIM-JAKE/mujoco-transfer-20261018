from pathlib import Path
import difflib,json,ast
base=Path('/tmp/fixed475-full/overrides/direct_pin_skill.py');s=base.read_text();out=Path('/tmp/fixed476-pin-component-feedback')
s=s.replace('axis_deadband_rad=0.):','axis_deadband_rad=0.,axis_gain=None):',1)
a="""    target_xy=observed[:2,3] if translation_held else goal[:2]
    desired=frame(np.r_[target_xy,min(float(observed_z),float(goal[2]-.045))]-rotation@G[:3,3],rotation)"""
b="""    gain=float(gain)
    if not .3<=gain<=1.:raise ValueError('Pin geometric feedback gain must be0.3--1')
    component_gains=axis_gain is not None
    if component_gains:
        axis_gain=float(axis_gain)
        if not .3<=axis_gain<=1.:raise ValueError('Pin axis feedback gain must be0.3--1')
        # Apply independent gains in OBSERVED PART space. Scaling the Hand
        # translation separately from its rotation would move the pin center
        # around the grasp lever arm and break a previously qualified XY.
        rotation=exp(axis_gain*log(rotation@A[:3,:3].T))@A[:3,:3]
        target_xy=observed[:2,3] if translation_held else observed[:2,3]+gain*(goal[:2]-observed[:2,3])
        target_z=float(observed_z)+gain*(min(float(observed_z),float(goal[2]-.045))-float(observed_z))
    else:
        target_xy=observed[:2,3] if translation_held else goal[:2]
        target_z=min(float(observed_z),float(goal[2]-.045))
    desired=frame(np.r_[target_xy,target_z]-rotation@G[:3,3],rotation)"""
assert s.count(a)==1;s=s.replace(a,b)
a="""    gain=float(gain)
    if not .3<=gain<=1.:raise ValueError('Pin geometric feedback gain must be0.3--1')
    target[:3,3]=C[:3,3]+gain*(target[:3,3]-C[:3,3])
    target[:3,:3]=exp(gain*log(target[:3,:3]@C[:3,:3].T))@C[:3,:3]"""
b="""    if not component_gains:
        target[:3,3]=C[:3,3]+gain*(target[:3,3]-C[:3,3])
        target[:3,:3]=exp(gain*log(target[:3,:3]@C[:3,:3].T))@C[:3,:3]"""
assert s.count(a)==1;s=s.replace(a,b)
s=s.replace('geometric_feedback_gain=gain,qualified_XY_preserved=', 'geometric_feedback_gain=gain,independent_axis_feedback_gain=axis_gain,part_space_component_feedback=component_gains,qualified_XY_preserved=',1)
a="""    correction_gain=float(pin_config.get('pin_visual_servo_gain',1.))
    previous_alignment_score=None"""
b="""    independent_component_feedback=bool(pin_config.get('pin_visual_independent_component_feedback',False))
    correction_gain=float(pin_config.get('pin_visual_servo_gain',.5 if independent_component_feedback else 1.))
    previous_alignment_score=None
    previous_radial_error=None"""
assert s.count(a)==1;s=s.replace(a,b)
a="""        if adaptive_pin_gain and previous_alignment_score is not None and alignment_score>=.95*previous_alignment_score:
            correction_gain=max(.3,correction_gain*.65)"""
b="""        if independent_component_feedback:
            # XY overshoot must not damp an independently unqualified axis.
            # The original unit axis correction worked on474 (6.19→0.88mrad);
            # its later0.3 damping was triggered by the coupled XY score.
            if adaptive_pin_gain and previous_radial_error is not None and radial>.000075 and radial>=.95*previous_radial_error:
                correction_gain=max(.3,correction_gain*.65)
        elif adaptive_pin_gain and previous_alignment_score is not None and alignment_score>=.95*previous_alignment_score:
            correction_gain=max(.3,correction_gain*.65)
        previous_radial_error=radial"""
assert s.count(a)==1;s=s.replace(a,b)
a="axis_deadband_rad=.003 if pin_config.get('pin_precision_component_deadband') else 0.);evidence.update(bias)"
b="axis_deadband_rad=.003 if pin_config.get('pin_precision_component_deadband') else 0.,axis_gain=1. if independent_component_feedback else None);evidence.update(bias)"
assert s.count(a)==1;s=s.replace(a,b)
ast.parse(s);(out/'direct_pin_skill.py').write_text(s);(out/'direct_pin_skill.py.diff').write_text(''.join(difflib.unified_diff(base.read_text().splitlines(True),s.splitlines(True),fromfile='a/direct_pin_skill.py',tofile='b/direct_pin_skill.py')))
(out/'config-delta.json').write_text(json.dumps({'pin_visual_independent_component_feedback':True,'pin_visual_servo_gain':.5},indent=2)+'\n')
