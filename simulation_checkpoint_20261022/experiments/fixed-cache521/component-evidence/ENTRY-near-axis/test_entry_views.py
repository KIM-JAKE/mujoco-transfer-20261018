from pathlib import Path
import ast,json,numpy as np
out=Path('/tmp/fixed521-pin-entry-axis-view');base=ast.parse(Path('/tmp/fixed520-entry-axis-gpu-full/overrides/direct_pin_skill.py').read_text());new=ast.parse((out/'overrides/direct_pin_skill.py').read_text());name='_pin_full_depth_near_axis_views';node=next(n for n in new.body if isinstance(n,ast.FunctionDef)and n.name==name);scope={'np':np};exec(compile(ast.Module(body=[node],type_ignores=[]),'entry_view_only','exec'),scope);fn=scope[name];H=np.eye(4);G=np.eye(4);old=[(i,[i,0,0])for i in range(13)];offsets=[x[1]for x in old];cfg=dict(pin_entry_near_axis_views=True,pin_two_stage_entry_feedback=True,fixed_supply_mode=True,pin_full_depth_require_robust_circle=True,robust_pin_cap_circle=True)
assert fn(H,G,offsets,'pin_entry_RGBD',{},True)==[];assert fn(H,G,offsets,'pin_preinsert_RGBD',cfg,True)==[];views=fn(H,G,offsets,'pin_entry_RGBD',cfg,True)+old;assert len(views)==15 and views[2:]==old and views[0][0]==13;assert np.allclose(views[0][1],[0,0,-.25]);assert next(index for index,value in views if index==13)==13
for flag in ['pin_two_stage_entry_feedback','pin_full_depth_require_robust_circle','robust_pin_cap_circle']:
 bad=dict(cfg);bad[flag]=False
 try:fn(H,G,offsets,'pin_entry_RGBD',bad,True)
 except ValueError:pass
 else:raise AssertionError('Missing required original policy '+flag)
# Entire execute (all sensing, motor, servo and control), and all other top-level functions unchanged.
a={n.name:ast.dump(n,include_attributes=False)for n in base.body if isinstance(n,ast.FunctionDef)};b={n.name:ast.dump(n,include_attributes=False)for n in new.body if isinstance(n,ast.FunctionDef)};assert set(a)==set(b);assert all(a[k]==b[k]for k in a if k!=name)
result=dict(pass_=True,defaultoff=True,only_ENTRY_and_existing_full_depth=True,first_success_one_fresh_capture=True,all13_old_fallbacks_preserved=True,original_two_stage_robust_qualified_depth_required=True,entire_execute_and_other_functions_unchanged=True,no_sensor_image_or_pose_cache=True,no_GPU_PhysX_jobs=True);(out/'lifecycle-tests.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
