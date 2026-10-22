from pathlib import Path
import numpy as np,json,copy,sys,ast
sys.path.insert(0,'/tmp/fixed500-pin-parallel-tail/overrides')
from pin_parallel_tail_target import retarget
O=Path('/tmp/fixed500-pin-parallel-tail');root=Path('/home/gpuadmin/workspace/jake/mujoco');candidate=json.load(open(root/'isaac_port/runs/native-roas-table-scatter-stable-v251-corrected-pin-catalog/pin-native-grasp-catalog.json'))['candidates'][0];old=copy.deepcopy(candidate);new,e=retarget(candidate);H,G=[np.asarray(candidate[k])for k in['pickup_H','grasp_transform']];h,g=[np.asarray(new[k])for k in['pickup_H','grasp_transform']];assert candidate==old and'native_pad_support'not in new
assert np.max(abs(H@G-h@g))<1e-12 and np.array_equal(g[:3,:3],np.eye(3)) and np.max(abs(g@np.array([0,0,-.0155,1])-[0,0,.003,1]))<1e-12
assert abs(e['physical_pickup_translation_m']-.003)<1e-12 and abs(e['physical_pickup_rotation_rad']-np.deg2rad(40))<1e-12 and'native_pad_support'not in new
failures=[]
for name,key,value in [('nonfinite','grasp_transform',np.eye(4)*np.nan),('shape','pickup_H',np.eye(3)),('nonrigid','pickup_H',np.diag([1,2,1,1])),('translation_bound','grasp_transform',np.array([[1,0,0,.1],[0,1,0,0],[0,0,1,.0185],[0,0,0,1]])),('rotation_bound','grasp_transform',np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,.0185],[0,0,0,1]]))]:
 c=copy.deepcopy(candidate);c[key]=value.tolist()
 try:retarget(c)
 except ValueError:failures.append(name)
 else:raise AssertionError(name)
a=ast.parse(Path('/tmp/fixed499-pin-tangent-yaw/overrides/direct_pin_skill.py').read_text());b=ast.parse((O/'overrides/direct_pin_skill.py').read_text())
for name in['motion','line','observe','measured_grasp','await_precision_encoder_window','release_inside_existing_holder']:
 f=lambda t:next(n for n in ast.walk(t)if isinstance(n,ast.FunctionDef)and n.name==name)
 assert ast.dump(f(a),include_attributes=False)==ast.dump(f(b),include_attributes=False)
fn=next(n for n in ast.walk(b)if isinstance(n,ast.FunctionDef)and n.name=='execute');prefix=fn.body[1:6];ns=dict(ctx={'candidate':candidate,'config':{'pin_tail_flat_contact_target':True,'pin_parallel_tail_contact_target':True}})
try:exec(compile(ast.fix_missing_locations(ast.Module(body=prefix,type_ignores=[])),'init','exec'),ns)
except ValueError as ex:assert'mutually exclusive'in str(ex)
else:raise AssertionError('mutualexclusive')
(O/'target-tests.json').write_text(json.dumps(dict(all_pass=True,original_candidate_unchanged=True,world_part_target_unchanged=True,exact_tail_point_mapping=True,real_Hand_rotation_deg=40.,real_Hand_translation_m=.003,old_contact_certificate_removed=True,mutual_exclusion_before_any_robot_access=True,original_motion_sensor_guard_functions_AST_identical=True,malformed_and_bound_rejections=failures,physical_validation=False,proposal=e),indent=2));print('PASS target math,5negativecases,mutualexclusion,6originalmotion/sensorfunctionsAST')
