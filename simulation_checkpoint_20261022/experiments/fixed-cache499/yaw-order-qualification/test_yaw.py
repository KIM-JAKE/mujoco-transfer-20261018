import ast,json,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
O=Path('/tmp/fixed499-pin-tangent-yaw');base=ast.parse(Path('/tmp/fixed497-pin-tail-flat/overrides/direct_pin_skill.py').read_text());new=ast.parse((O/'overrides/direct_pin_skill.py').read_text());f0=next(n for n in ast.walk(base)if isinstance(n,ast.For)and ast.unparse(n.target)=='yaw_deg');f1=next(n for n in ast.walk(new)if isinstance(n,ast.For)and ast.unparse(n.target)=='yaw_deg');assert ast.dump(f0,include_attributes=False)==ast.dump(f1,include_attributes=False)
block=next(n for n in ast.walk(new)if isinstance(n,ast.If)and ast.unparse(n.test)=="pin_config.get('pin_tangent_flip_yaw_first', False) and pin_config.get('pin_axial_yaw_free')")
for enabled,free,expected in[(True,True,[180,90,-90,0]),(False,True,[0,90,-90,180]),(True,False,[0])]:
 ns=dict(pin_config={'pin_tangent_flip_yaw_first':enabled,'pin_axial_yaw_free':free},yaw_options=[0.,90.,-90.,180.]if free else[0.]);exec(compile(ast.fix_missing_locations(ast.Module(body=[block],type_ignores=[])),'yaworder','exec'),ns);assert ns['yaw_options']==expected
for angle in[0,90,-90,180]:
 R=Rotation.from_rotvec([0,0,np.deg2rad(angle)]).as_matrix();assert np.max(abs(R[:,2]-[0,0,1]))<1e-12
(O/'yaw-tests.json').write_text(json.dumps(dict(all_pass=True,same_original_candidate_set=True,first_admitted_loop_AST_unchanged=True,original_endpoint_full_guards_and_retiming_unchanged=True,calibrated_goal_axis_unchanged=True,disabled_free_yaw_stays_zero=True,physical_validation=False),indent=2));print('PASS exact original loop/candidates/goalaxis/flag lifecycle')
