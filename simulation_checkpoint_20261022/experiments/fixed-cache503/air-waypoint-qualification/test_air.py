import ast,json,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
O=Path('/tmp/fixed502-pin-air-tangent');base=ast.parse(Path('/tmp/fixed500-pin-parallel-tail/overrides/direct_pin_skill.py').read_text());new=ast.parse((O/'overrides/direct_pin_skill.py').read_text());node=next(n for n in ast.walk(new)if isinstance(n,ast.If)and ast.unparse(n.test)=='air_tangent_flip');code=compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),'air','exec')
def frame(t,R):
 T=np.eye(4);T[:3,3]=t;T[:3,:3]=R;return T
G=frame([-.0099,0,.0148],Rotation.from_rotvec([0,-.7,0]).as_matrix());R=np.eye(3);pre=frame([-.3,.22,1.48],R)@np.linalg.inv(G)
for enabled in[True,False]:
 ns=dict(np=np,air_tangent_flip=enabled,R=R.copy(),G=G.copy(),pre=pre.copy(),safe=np.eye(4),report={},transports=['original'],frame=frame,exp=lambda r:Rotation.from_rotvec(r).as_matrix());exec(code,ns)
 if enabled:
  assert len(ns['transports'])==2 and ns['transports'][0][1]=='pin_tail_orient_free'and ns['transports'][1][1]=='pin_tail_carry_to_preinsert';assert np.array_equal(ns['transports'][1][0],pre);P=ns['transports'][0][0]@G;assert np.max(abs(P[:3,3]-[-.1,-.25,1.35]))<1e-12 and np.max(abs(P[:3,2]-[0,0,1]))<1e-12
 else:assert ns['transports']==['original']and not ns['report']
 assert np.array_equal(ns['R'],R)and np.array_equal(ns['G'],G)
for name in['motion','line','observe','measured_grasp','await_precision_encoder_window','endpoint_candidates']:
 f=lambda tree:next(n for n in ast.walk(tree)if isinstance(n,ast.FunctionDef)and n.name==name)
 assert ast.dump(f(base),include_attributes=False)==ast.dump(f(new),include_attributes=False)
old=next(n for n in ast.walk(base)if isinstance(n,ast.For)and ast.unparse(n.target)=='(target, label)');newloop=next(n for n in ast.walk(new)if isinstance(n,ast.For)and ast.unparse(n.target)=='(target, label)');assert ast.dump(old,include_attributes=False)==ast.dump(newloop,include_attributes=False)
(O/'air-tests.json').write_text(json.dumps(dict(all_pass=True,fixture_target_R_and_pre_H_unchanged=True,free_midpoint_signedaxis_unchanged=True,G_not_modified=True,disabled_default_unchanged=True,original_two_segment_loop_AST_identical=True,original_fullmotion_sensor_terminal_functions_AST_identical=True,physical_validation=False),indent=2));print('PASS freewaypoint/fixture/G/default and originalguardedloop invariant tests')
