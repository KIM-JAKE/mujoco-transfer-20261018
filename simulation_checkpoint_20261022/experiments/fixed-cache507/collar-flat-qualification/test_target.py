from pathlib import Path
import numpy as np,json,copy,sys,ast
sys.path.insert(0,'/tmp/fixed504-pin-collar-flat/overrides')
from pin_collar_contact_target import retarget
O=Path('/tmp/fixed504-pin-collar-flat');root=Path('/home/gpuadmin/workspace/jake/mujoco');candidate=json.load(open(root/'isaac_port/runs/native-roas-table-scatter-stable-v251-corrected-pin-catalog/pin-native-grasp-catalog.json'))['candidates'][0];old=copy.deepcopy(candidate);H,G=[np.asarray(candidate[k])for k in['pickup_H','grasp_transform']];valid=[]
for offset in[-.001,-.002]:
 new,e=retarget(candidate,offset);h,g=[np.asarray(new[k])for k in['pickup_H','grasp_transform']];assert candidate==old and'native_pad_support'not in new
 assert np.max(abs(H@G-h@g))<1e-12 and np.max(abs(H[:3,:3]-h[:3,:3]))<1e-12 and np.array_equal(g[:3,:3],G[:3,:3])
 assert np.max(abs(h[:3,3]-H[:3,3]+offset*H[:3,2]))<1e-12 and abs(e['physical_pickup_translation_m']-abs(offset))<1e-12;valid.append(e)
for offset in[0.,.001,-.003,float('nan')]:
 try:retarget(candidate,offset)
 except ValueError:pass
 else:raise AssertionError(offset)
errors=[]
for name,key,value in [('nonfinite','grasp_transform',np.eye(4)*np.nan),('shape','pickup_H',np.eye(3)),('nonrigid','pickup_H',np.diag([1,2,1,1])),('wrongfamily','grasp_transform',np.eye(4))]:
 c=copy.deepcopy(candidate);c[key]=value.tolist()
 try:retarget(c)
 except ValueError:errors.append(name)
 else:raise AssertionError(name)
a=ast.parse(Path('/tmp/fixed502-pin-air-tangent/overrides/direct_pin_skill.py').read_text());b=ast.parse((O/'overrides/direct_pin_skill.py').read_text())
for name in['motion','line','observe','measured_grasp','await_precision_encoder_window','release_inside_existing_holder','endpoint_candidates']:
 f=lambda t:next(n for n in ast.walk(t)if isinstance(n,ast.FunctionDef)and n.name==name)
 assert ast.dump(f(a),include_attributes=False)==ast.dump(f(b),include_attributes=False)
fn=next(n for n in ast.walk(b)if isinstance(n,ast.FunctionDef)and n.name=='execute');prefix=fn.body[1:7]
for enabled in[{'pin_tail_flat_contact_target':True,'pin_collar_flat_contact_target':True},{'pin_parallel_tail_contact_target':True,'pin_collar_flat_contact_target':True}]:
 ns=dict(ctx={'candidate':candidate,'config':enabled})
 try:exec(compile(ast.fix_missing_locations(ast.Module(body=prefix,type_ignores=[])),'init','exec'),ns)
 except ValueError as e:assert'mutually exclusive'in str(e)
 else:raise AssertionError('mutualexclusive')
(O/'target-tests.json').write_text(json.dumps(dict(all_pass=True,original_candidate_and_part_world_target_unchanged=True,original_orientation_unchanged=True,negative1_and2mm_target_mapping_exact=True,old_contact_cert_not_reused=True,invalid_offsets_reject_before_anymotion=True,negative_input_cases=errors,mutual_exclusion_before_robot_access=True,seven_original_motion_sensor_terminal_functions_AST_identical=True,physical_validation=False,valid_target_evidence=valid),indent=2));print('PASS 1/2mm targets,offset/malformed negatives,mutualexclusion,7originalfunctionsAST')
