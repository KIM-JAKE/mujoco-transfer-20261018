from pathlib import Path
import ast,difflib,json,hashlib
O=Path(__file__).parent;B=Path('/tmp/fixed517-preclose-gain-full/overrides/direct_pin_skill.py');old=B.read_text();s=old
s=s.replace('axis_gain=None):','axis_gain=None,preinsert_depth_m=.045):',1)
s=s.replace("    C,A,G=[np.asarray(x,float)for x in [command_H,actual_H,measured_G]]","    preinsert_depth_m=float(preinsert_depth_m)\n    if preinsert_depth_m not in(.045,.018):raise ValueError('PIN feedback depth plane must be original45mm or explicit18mm entry')\n    C,A,G=[np.asarray(x,float)for x in [command_H,actual_H,measured_G]]",1);s=s.replace('float(goal[2]-.045)','float(goal[2]-preinsert_depth_m)')
needle="    pin_sensor_hold=bool(pin_config.get('pin_rgbd_sdk_hold_confirmation'))\n";s=s.replace(needle,needle+'''    entry_feedback_enabled=bool(pin_config.get('pin_two_stage_entry_feedback'))
    if entry_feedback_enabled and not(visual_preinsert and pin_sensor_hold and qualified_full_depth and pin_config.get('pin_terminal_stroke_preflight')):raise ValueError('Two-stage PIN entry needs original visual/hold/full-terminal and qualified-depth policies')
''',1)
s=s.replace('    terminal_replans=0;axial_inserted=False','    terminal_replans=0;axial_inserted=False;entry_stage=False;far_preinsert_H=None;far_remaining_depth=None',1)
s=s.replace("v=observe('pin_preinsert_RGBD',G,require_circle=visual_preinsert)","v=observe('pin_entry_RGBD'if entry_stage else'pin_preinsert_RGBD',G,require_circle=visual_preinsert)",1)
s=s.replace("        if not .035<remaining_depth<.055:raise RuntimeError('Pin visual prealignment must remain35–55mm below the calibrated jig target')",'''        if entry_stage:
            if not .014<remaining_depth<.022:raise RuntimeError('Pin entry RGB-D must remain14–22mm below the calibrated jig target')
        elif not .035<remaining_depth<.055:raise RuntimeError('Pin visual prealignment must remain35–55mm below the calibrated jig target')
        evidence['entry_stage']=entry_stage;evidence['original_shared_alignment_budget']=alignment_budget''',1)
s=s.replace('                admitted,terminal=assess_current_terminal(ctx,payload_grasp(),remaining_depth)',"                entry_options=dict(entry_stage=True,withdraw_H=far_preinsert_H,requested_s=.8*remaining_depth/far_remaining_depth)if entry_stage else{}\n                admitted,terminal=assess_current_terminal(ctx,payload_grasp(),remaining_depth,**entry_options)",1)
s=s.replace("                pre=arm.fk(cmd[0,ids]);end=pre.copy();end[2,3]+=remaining_depth\n                line(pre,end,.8,'pin_single_axial_insert')",'''                current_pre=arm.fk(cmd[0,ids])
                if entry_feedback_enabled and not entry_stage:
                    if alignment_index==alignment_budget-1:raise RuntimeError('PIN shared image budget exhausted before near-entry feedback; partial stroke blocked')
                    far_preinsert_H=current_pre.copy();far_remaining_depth=remaining_depth
                    entry_target=current_pre.copy();entry_distance=remaining_depth-.018;entry_target[2,3]+=entry_distance
                    line(current_pre,entry_target,.8*entry_distance/remaining_depth,'pin_axial_approach_to_entry')
                    entry_stage=True
                    report['pin_entry_feedback_transition']=dict(alignment_index=alignment_index,remaining_original_images=alignment_budget-alignment_index-1,far_preinsert_Hand=far_preinsert_H.tolist(),far_remaining_depth_m=far_remaining_depth,entry_remaining_depth_m=.018,shaft_tip_below_original_fixture_bottom_m=.00175,original_full_far_terminal_admitted=True,fresh_entry_RGBD_required=True,no_budget_or_feedback_history_reset=True,uses_free_body_poses=False)
                    continue
                pre=far_preinsert_H.copy()if entry_stage else current_pre
                end=current_pre.copy();end[2,3]+=remaining_depth
                line(current_pre,end,.8*remaining_depth/far_remaining_depth if entry_stage else .8,'pin_single_axial_insert')''',1)
s=s.replace('                recovery=plan_rebranch(ctx,payload_grasp(),remaining_depth)',"                recovery=plan_rebranch(ctx,payload_grasp(),remaining_depth,**(dict(entry_stage=True,withdraw_H=far_preinsert_H,requested_s=.8*remaining_depth/far_remaining_depth)if entry_stage else{}))",1)
needle="axis_gain=(.65 if bounded_component_recovery else 1.) if independent_component_feedback else None);evidence.update(bias)";rep="axis_gain=(.65 if bounded_component_recovery else 1.) if independent_component_feedback else None,**(dict(preinsert_depth_m=.018)if entry_stage else{}));evidence.update(bias)";assert needle in s;s=s.replace(needle,rep,1)
s=s.replace("    if np.linalg.norm(Pobs[:2]-goal[:2])>=.0015 or not .035<remaining_depth<.055:raise RuntimeError('Preinsert RGB-D alignment/depth outside direct insertion range')",'''    depth_range_ok=.014<remaining_depth<.022 if entry_stage else .035<remaining_depth<.055
    if np.linalg.norm(Pobs[:2]-goal[:2])>=.0015 or not depth_range_ok:raise RuntimeError('Preinsert RGB-D alignment/depth outside explicit insertion state range')
    if entry_feedback_enabled and not(entry_stage and axial_inserted):raise RuntimeError('Two-stage PIN entry did not qualify final insertion within original image budget')''',1)
ast.parse(s);(O/'overrides/direct_pin_skill.py').write_text(s);(O/'direct_pin_skill.py.diff').write_text(''.join(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(B),tofile=str(O/'overrides/direct_pin_skill.py'))))
m=json.loads((O/'helper-manifest.json').read_text());m[B.name]=dict(base=str(B),base_sha256=hashlib.sha256(old.encode()).hexdigest(),sha256=hashlib.sha256(s.encode()).hexdigest());(O/'source-manifest.json').write_text(json.dumps(dict(files=m,new_flag='pin_two_stage_entry_feedback',fresh_imports=[],base517_gain_qualified_depth_preserved=True,no_physical_trials_performed=True),indent=2)+'\n');print((O/'source-manifest.json').read_text())
