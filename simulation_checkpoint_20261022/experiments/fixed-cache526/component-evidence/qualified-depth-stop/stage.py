from pathlib import Path
import hashlib,json,difflib,ast
root=Path('/tmp/fixed527-qualified-depth-stop');base=Path('/tmp/fixed522-entry-before-contact/overrides/direct_pin_skill.py');s=base.read_text()
old="    if qualified_full_depth and not(visual_preinsert and pin_config.get('robust_pin_cap_circle')):raise ValueError('Qualified PIN full-depth observation requires original visual preinsert and robust circle settings')\n"
new=old+"    avoid_unneeded_depth_correction=bool(pin_config.get('pin_avoid_unneeded_depth_correction',False))\n    if avoid_unneeded_depth_correction and not qualified_full_depth:raise ValueError('Stopping at qualified PIN full depth requires original robust-circle full-depth policy')\n"
assert s.count(old)==1;s=s.replace(old,new)
old="    for correction_index in range(2):\n        remaining=float(goal[2]-pose['center_m'][2]);radial=float(np.linalg.norm(np.array(pose['center_m'])[:2]-goal[:2]))\n"
new="    for correction_index in range(2):\n        if avoid_unneeded_depth_correction and error<.0015 and ae<.05:\n            report['pin_unneeded_depth_correction_skipped']=dict(reason='Fresh qualified full-depth RGB-D already satisfies original final position/axis gates; 0.5 mm extra-push threshold is a refinement heuristic',position_error_m=error,axis_error_rad=ae,observed_center_m=list(pose['center_m']),observed_axis=list(pose['axis']),remaining_axial_depth_m=float(goal[2]-pose['center_m'][2]),robust_cap_circle_from_pixels=pose.get('robust_cap_circle_from_pixels') is True,completed_depth_corrections=correction_index,original_final_position_limit_m=.0015,original_final_axis_limit_rad=.05,extra_images_or_motion=0,uses_free_body_poses=False)\n            break\n        remaining=float(goal[2]-pose['center_m'][2]);radial=float(np.linalg.norm(np.array(pose['center_m'])[:2]-goal[:2]))\n"
assert s.count(old)==1;s=s.replace(old,new);ast.parse(s)
out=root/'overrides/direct_pin_skill.py';out.write_text(s)
(root/'direct_pin_skill.py.diff').write_text(''.join(difflib.unified_diff(base.read_text().splitlines(True),s.splitlines(True),fromfile=str(base),tofile=str(out))))
(root/'source-manifest.json').write_text(json.dumps({'base':str(base),'base_sha256':hashlib.sha256(base.read_bytes()).hexdigest(),'staged_sha256':hashlib.sha256(out.read_bytes()).hexdigest(),'flag':'pin_avoid_unneeded_depth_correction','default':False,'requires':'pin_full_depth_require_robust_circle'},indent=2)+'\n')
print((root/'source-manifest.json').read_text())
