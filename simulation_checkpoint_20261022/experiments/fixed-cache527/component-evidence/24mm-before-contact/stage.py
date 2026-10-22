from pathlib import Path
import ast,json,hashlib,difflib,numpy as np
O=Path(__file__).parent;B=Path('/tmp/fixed521-measured-entry-full/overrides');m={}
for name in('direct_pin_skill.py','pin_current_terminal.py','pin_terminal_geometry.py'):
 p=B/name;old=p.read_text();s=old
 if name=='direct_pin_skill.py':
  changes={'preinsert_depth_m not in(.045,.018)':'preinsert_depth_m not in(.045,.024)','explicit18mm entry':'explicit24mm entry','.014<remaining_depth<.022':'.022<remaining_depth<.028','remain14–22mm':'remain22–28mm','entry_distance=remaining_depth-.018':'entry_distance=remaining_depth-.024','entry_remaining_depth_m=.018,shaft_tip_below_original_fixture_bottom_m=.00175':'entry_remaining_depth_m=.024,physical_pin_axial_half_length_m=.01775,CAD_fixture_bottom_below_axis_goal_m=.00275,physical_pin_tip_below_original_fixture_bottom_m=.024-.01775-.00275','dict(preinsert_depth_m=.018)':'dict(preinsert_depth_m=.024)'}
 elif name=='pin_current_terminal.py':changes={'issued_depth=float(depth)-.018':'issued_depth=float(depth)-.024'}
 else:changes={'.014<=depth<=.022':'.022<=depth<=.028'}
 for a,b in changes.items():assert a in s,(name,a);s=s.replace(a,b)
 ast.parse(s);q=O/'overrides'/name;q.write_text(s);(O/(name+'.diff')).write_text(''.join(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(p),tofile=str(q))));m[name]=dict(base=str(p),base_sha256=hashlib.sha256(old.encode()).hexdigest(),sha256=hashlib.sha256(s.encode()).hexdigest())
models=Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/fieldkit/assets/part-models.npz');v=np.load(models)['pin_points'];assert abs(float(v[:,2].max())-.01775)<1e-7 and abs(float(v[:,2].min())+.01775)<1e-7
proof=dict(scope='Original public class visualCADpoints and unchanged original four-cylinder contact model +RGBDphysicalcapoffset; no actorposes or runtime geometry changes.',class_model=str(models),class_model_sha256=hashlib.sha256(models.read_bytes()).hexdigest(),CAD_axis_min_z_m=float(v[:,2].min()),CAD_axis_max_z_m=float(v[:,2].max()),contact_model_tip_extent_m=.01775,RGBD_end_to_midpoint_m=.01775,shaft_precision_sample_is_not_tip_end_m=.0135,fixture_bottom_z_m=1.526,axis_goal_z_m=1.52875,old18mm_nominal_tip_above_fixture_bottom_m=1.52875-.018+.01775-1.526,new24mm_nominal_tip_below_fixture_bottom_m=1.526-(1.52875-.024+.01775),gates={'original_FAR_depth_range_m':[.035,.055],'ENTRY_depth_range_m':[.022,.028]},gripper_release_aperture_unchanged_m=.018)
(O/'CAD-endpoint-verification.json').write_text(json.dumps(proof,indent=2)+'\n');(O/'source-manifest.json').write_text(json.dumps(dict(files=m,existing_flag='pin_two_stage_entry_feedback',fresh_imports=[],only_optional_entry_depth_state_changed=True),indent=2)+'\n');print(json.dumps(proof,indent=2));print('FILES',json.dumps(m,indent=2))
