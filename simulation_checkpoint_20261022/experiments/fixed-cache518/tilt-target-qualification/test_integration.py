from pathlib import Path
import ast,json,numpy as np
O=Path(__file__).parent;old=ast.parse(Path('/tmp/fixed517-preclose-gain-full/overrides/direct_pin_skill.py').read_text());new=ast.parse((O/'overrides/direct_pin_skill.py').read_text());function=lambda t:next(n for n in t.body if isinstance(n,ast.FunctionDef)and n.name=='execute');e=function(new)
# New config validation occurs before candidate/robot access.
validate=compile(ast.Module(body=e.body[:2],type_ignores=[]),'<earlytiltguard>','exec');tests=[]
for cfg in({'pin_collar_tilt30_target':True},{'pin_collar_tilt30_target':True,'pin_collar_flat_contact_target':True,'pin_collar_flat_offset_m':-.002},{'pin_collar_tilt30_target':True,'pin_collar_flat_contact_target':True,'pin_collar_flat_offset_m':.001}):
 try:exec(validate,{'ctx':{'config':cfg}});raise AssertionError('invalid mode accepted')
 except ValueError:pass
tests.append('requires_existing_collar_negative1mm_before_candidate_robot_access')
env={'ctx':{'config':{'pin_collar_tilt30_target':True,'pin_collar_flat_contact_target':True,'pin_collar_flat_offset_m':-.001}}};exec(validate,env);assert env['tilt30_enabled'];tests.append('explicit_existing_minus1_mode_accepted')
# Existing physicaltarget mutualflag gate stays literally unchanged.
oldguard=next(n for n in ast.walk(function(old))if isinstance(n,ast.If)and 'int(flat_tail_enabled)'in ast.unparse(n.test));newguard=next(n for n in ast.walk(e)if isinstance(n,ast.If)and 'int(flat_tail_enabled)'in ast.unparse(n.test));assert ast.dump(oldguard,include_attributes=False)==ast.dump(newguard,include_attributes=False)
for a,b in((True,False),(False,True)):
 try:exec(compile(ast.Module(body=[newguard],type_ignores=[]),'<original-mutual>','exec'),dict(flat_tail_enabled=a,parallel_tail_enabled=b,collar_flat_enabled=True));raise AssertionError('double mode allowed')
 except ValueError:pass
tests.append('original_mutual_physical_target_guard_preserved')
def funcs(t):return{n.name:ast.dump(n,include_attributes=False)for n in ast.walk(t)if isinstance(n,ast.FunctionDef)and n.name!='execute'}
assert funcs(old)==funcs(new);tests.append('every_original_motion_route_sensor_feedback_gain_precision_retention_function_AST_identical')
assert 'pin_preclose_servo_gain' in ast.unparse(new);assert 'pin_full_depth_require_robust_circle'in ast.unparse(new);assert'pin_prefer_canonical_pickup_branch'in ast.unparse(new);tests.append('base517_gain_qualifieddepth_canonicalfeatures_retained')
(O/'integration-tests.json').write_text(json.dumps(dict(passed=True,tests=tests,actual_fullscene_or_physics_tested=False),indent=2)+'\n');print(tests)
