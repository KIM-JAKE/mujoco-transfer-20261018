from pathlib import Path
import sys,json,copy,ast,hashlib
import numpy as np
sys.path.insert(0,'/tmp/fixed497-pin-tail-flat/overrides')
from pin_tail_contact_target import retarget
R=Path('/home/gpuadmin/workspace/jake/mujoco');p=R/'isaac_port/runs/native-roas-table-scatter-stable-v251-corrected-pin-catalog/pin-native-grasp-catalog.json';candidate=json.loads(p.read_text())['candidates'][0];before=copy.deepcopy(candidate);new,e=retarget(candidate);G,H=np.asarray(candidate['grasp_transform']),np.asarray(candidate['pickup_H']);g,h=np.asarray(new['grasp_transform']),np.asarray(new['pickup_H']);assert candidate==before and 'native_pad_support'not in new
assert np.max(abs(H@G-h@g))<1e-12 and np.max(abs(H[:3,:3]-h[:3,:3]))<1e-12
assert abs(np.linalg.norm(H[:3,3]-h[:3,3])-.003)<1e-12 and np.array_equal(G[:3,:3],g[:3,:3])
assert np.max(abs(H[:3,3]-h[:3,3]-.003*H[:3,2]))<1e-12
errors=[]
for name,key,value in [('nonfinite','grasp_transform',np.eye(4)*np.nan),('shape','pickup_H',np.eye(3)),('nonrigid','pickup_H',np.diag([2,1,1,1])),('wrongfamily','grasp_transform',np.eye(4))]:
 c=copy.deepcopy(candidate);c[key]=value.tolist()
 try:retarget(c)
 except ValueError:errors.append(name)
 else:raise AssertionError(name)
base=ast.parse(Path('/tmp/fixed493-intern-and-small-full/overrides/direct_pin_skill.py').read_text());actual=ast.parse(Path('/tmp/fixed497-pin-tail-flat/overrides/direct_pin_skill.py').read_text());names=['motion','line','observe','measured_grasp','await_precision_encoder_window','release_inside_existing_holder'];unchanged=[]
for name in names:
 a=next(n for n in ast.walk(base)if isinstance(n,ast.FunctionDef)and n.name==name);b=next(n for n in ast.walk(actual)if isinstance(n,ast.FunctionDef)and n.name==name);assert ast.dump(a,include_attributes=False)==ast.dump(b,include_attributes=False);unchanged.append(name)
Path('/tmp/fixed497-pin-tail-flat/target-tests.json').write_text(json.dumps(dict(all_pass=True,original_candidate_unchanged=True,object_target_world_pose_unchanged=True,orientation_unchanged=True,physical_empty_hand_displacement_m=.003,old_contact_certificate_not_reused=True,malformed_rejected=errors,original_guard_functions_AST_identical=unchanged,proposal=e),indent=2));print('PASS target +6 original function AST invariants')
