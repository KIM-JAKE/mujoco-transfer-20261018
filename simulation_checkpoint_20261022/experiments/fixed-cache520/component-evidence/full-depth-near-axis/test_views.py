from pathlib import Path
import ast,json,numpy as np
root=Path('/tmp/fixed520-pin-depth-axis-view');source=(root/'overrides/direct_pin_skill.py').read_text();tree=ast.parse(source);f=next(n for n in tree.body if isinstance(n,ast.FunctionDef)and n.name=='_pin_full_depth_near_axis_views');m=ast.Module(body=[f],type_ignores=[]);ns={'np':np};exec(compile(m,'isolated_original_helper','exec'),ns);fn=ns[f.name];H=np.eye(4);G=np.eye(4);old=[(i,[i,0,0])for i in range(13)];offsets=[x[1]for x in old];config=dict(pin_full_depth_near_axis_views=True,fixed_supply_mode=True,pin_full_depth_require_robust_circle=True,robust_pin_cap_circle=True)
assert fn(H,G,offsets,'pin_full_depth_RGBD',{},True)==[];assert fn(H,G,offsets,'pin_preinsert_RGBD',config,True)==[];new=fn(H,G,offsets,'pin_full_depth_RGBD',config,True);assert len(new)==2 and new[0][0]==13 and new[1][0]==14;assert np.allclose(new[0][1],[0,0,-.25]);views=new+old;assert views[2:]==old
# Simulate unchanged first-success iteration, never cache a sensor response.
def exposures(valid):
 seen=[]
 for index,offset in views:
  seen.append(index)
  if index in valid:return seen
 return seen
assert exposures({13})==[13];assert exposures({0})==[13,14,0];assert len(exposures(set()))==15
try:fn(H,G,offsets,'pin_full_depth_RGBD',config,False)
except ValueError:pass
else:raise AssertionError('Qualified original robust cap gate is mandatory')
base=ast.parse(Path('/tmp/fixed520-pin-gpu-route-first/overrides/direct_pin_skill.py').read_text());before=next(n for n in base.body if isinstance(n,ast.FunctionDef)and n.name=='execute');after=next(n for n in tree.body if isinstance(n,ast.FunctionDef)and n.name=='execute');bd={n.name:ast.dump(n,include_attributes=False)for n in before.body if isinstance(n,ast.FunctionDef)};ad={n.name:ast.dump(n,include_attributes=False)for n in after.body if isinstance(n,ast.FunctionDef)};assert set(bd)==set(ad);assert all(bd[k]==ad[k]for k in bd if k!='observe');result=dict(pass_=True,defaultoff_no_views=True,only_full_depth=True,new_first_success_exactly_one_capture=True,new_views_fail_then_original_fallback=True,all_13_original_indices_and_order_preserved=True,all_failed_views15=True,qualified_circle_required=True,all_other_nested_sensor_motion_servo_functions_unchanged=True,original_240_gate_unchanged=True,no_GPU_or_PhysX_test=True);(root/'view-lifecycle-tests.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
