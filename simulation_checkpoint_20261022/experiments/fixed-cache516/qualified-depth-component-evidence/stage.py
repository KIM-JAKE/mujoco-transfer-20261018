from pathlib import Path
import hashlib,json,difflib,ast
O=Path(__file__).parent;B=Path('/tmp/fixed512-camera-priority-full/overrides/direct_pin_skill.py');old=B.read_text();s=old
oldline="    visual_preinsert=bool(pin_config.get('pin_visual_preinsert_alignment'))\n"
newline=oldline+"    qualified_full_depth=bool(pin_config.get('pin_full_depth_require_robust_circle'))\n    if qualified_full_depth and not(visual_preinsert and pin_config.get('robust_pin_cap_circle')):raise ValueError('Qualified PIN full-depth observation requires original visual preinsert and robust circle settings')\n"
assert s.count(oldline)==1;s=s.replace(oldline,newline,1)
oldline="            if error:\n                errors.append(str(error).splitlines()[-1]);continue\n            report['events'].append(dict(event=label,wall_s=e['delivered_wall_s'],result=value['result'],view_index=view_index));print('PIN_CONTACT_VISION',label,e['delivered_wall_s'],flush=True);return value\n"
newline="""            if qualified_full_depth and label=='pin_full_depth_RGBD' and not(pose and pose.get('robust_cap_circle_from_pixels')is True):
                error=error or 'PIN full-depth requires the original robust class-radius circular-cap proof'
            if error:
                errors.append(str(error).splitlines()[-1]);continue
            if qualified_full_depth and label=='pin_full_depth_RGBD':
                report.setdefault('pin_full_depth_qualified_observations',[]).append(dict(view_index=view_index,captured_wall_s=captured,ready_wall_s=ready,delivered_wall_s=e['delivered_wall_s'],robust_cap_circle_from_pixels=True,original_cap_quality={key:pose.get(key)for key in ['cap_expected_radius_m','visible_end_radius_m','cap_edge_coverage_deg','cap_circular_edge_inliers','cap_total_hull_edge_points','cap_circular_edge_rms_m','cap_points','depth_points','plane_rms_m']},same_existing_camera_view_bank=True,original_depth_axis_precision_and_motor_gates_unchanged=True,uses_free_body_poses=False))
            report['events'].append(dict(event=label,wall_s=e['delivered_wall_s'],result=value['result'],view_index=view_index));print('PIN_CONTACT_VISION',label,e['delivered_wall_s'],flush=True);return value
"""
assert s.count(oldline)==1;s=s.replace(oldline,newline,1)
assert s.count("observe('pin_full_depth_RGBD',G)")==2;s=s.replace("observe('pin_full_depth_RGBD',G)","observe('pin_full_depth_RGBD',G,require_circle=qualified_full_depth)")
ast.parse(s);P=O/'overrides/direct_pin_skill.py';P.write_text(s);(O/'direct_pin_skill.py.diff').write_text(''.join(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(B),tofile=str(P))))
(O/'source-manifest.json').write_text(json.dumps(dict(base=str(B),base_sha256=hashlib.sha256(old.encode()).hexdigest(),file=str(P),sha256=hashlib.sha256(s.encode()).hexdigest(),new_flag='pin_full_depth_require_robust_circle',fresh_imports=[],unchanged_view_bank=13,all_original_gates_and_limits_unchanged=True),indent=2)+'\n')
print((O/'source-manifest.json').read_text())
