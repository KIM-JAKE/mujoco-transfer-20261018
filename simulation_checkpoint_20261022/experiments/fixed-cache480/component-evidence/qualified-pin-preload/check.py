from pathlib import Path
import ast,json,numpy as np
from scipy.spatial.transform import Rotation
out=Path('/tmp/fixed478-pin-qualified-preload');driver=ast.parse(Path('/tmp/fixed476-full/overrides/run_realtime_recipe.py').read_text());funcs={n.name:n for n in ast.walk(driver)if isinstance(n,ast.FunctionDef)}
class Art:
 def __init__(self):self.calls=[]
 def set_dof_stiffnesses(self,x,i):self.calls.append('stiffness')
 def set_dof_dampings(self,x,i):self.calls.append('damping')
 def get_dof_positions(self):return np.array([[.0055,.0055]])
art=Art();env=dict(np=np,native_grip_started={'right':0.},physical=1.,loaded_ramp_s=.3,native_preload_initial={'right':.0075},native_preload_target={'right':.0075},native_grip_hold={'right':True},loaded_initial_speed=.0075,loaded_final_speed=.0075,report={},pad_force=30.,native_grip_direction={'right':-1.},finger_ids={'right':np.array([0,1])},stiffness=np.zeros((1,2)),damping=np.full((1,2),2000.),shared_motor=True,art=art,idx=np.array([0]))
exec(compile(ast.Module(body=[funcs['loaded_preload'],funcs['set_native_grip_hold']],type_ignores=[]),'actual-driver-functions','exec'),env)
env['set_native_grip_hold']('right',True,preload_m_s=.015);samples=[]
for t in [1.,1.15,1.3,1.35]:env['physical']=t;samples.append(env['loaded_preload']('right'))
assert np.allclose(samples,[.0075,.01125,.015,.015],atol=1e-15)
assert env['report']['native_contact_force_stages'][0]['force_cap_n']==60.
for invalid in [0.,.015001,float('nan')]:
 try:env['set_native_grip_hold']('right',True,preload_m_s=invalid)
 except ValueError:pass
 else:raise AssertionError('Invalid driver target admitted')
mod=ast.parse((out/'direct_pin_skill.py').read_text());hold=next(n for n in mod.body if isinstance(n,ast.FunctionDef)and n.name=='_pin_rgbd_sdk_hold_evidence');scope={'np':np};exec(compile(ast.Module(body=[hold],type_ignores=[]),'actual-pin-hold','exec'),scope);guard=scope[hold.name]
pose=dict(robust_cap_circle_from_pixels=True,lateral_from_previous_RGBD_grasp_and_encoders=False,uses_depth_sensor=True,cap_edge_coverage_deg=240,visible_end_radius_m=.00525,cap_expected_radius_m=.00525)
assert guard(pose,np.eye(4),np.eye(4),.011)['confirmed']
for kind in ['translation','axis','aperture','coverage','propagated']:
 G=np.eye(4);pp=pose.copy();gap=.011
 if kind=='translation':G[0,3]=.0021
 if kind=='axis':G[:3,:3]=Rotation.from_rotvec([.051,0,0]).as_matrix()
 if kind=='aperture':gap=.014
 if kind=='coverage':pp['cap_edge_coverage_deg']=239
 if kind=='propagated':pp['lateral_from_previous_RGBD_grasp_and_encoders']=True
 assert not guard(pp,np.eye(4),G,gap)['confirmed']
execute=next(n for n in mod.body if isinstance(n,ast.FunctionDef)and n.name=='execute');block=next(n for n in execute.body if isinstance(n,ast.If)and 'pin_qualified_postlift_preload' in ast.unparse(n.test))
args=['pin_config','ctx','G','pin_sensor_hold','pin_axis_geometry','report','art','fi','observe','measured_grasp','wait'];probe=ast.FunctionDef(name='probe',args=ast.arguments(posonlyargs=[],args=[ast.arg(arg=x)for x in args],kwonlyargs=[],kw_defaults=[],defaults=[]),body=[block,ast.Return(value=ast.Name(id='G',ctx=ast.Load()))],decorator_list=[]);tree=ast.fix_missing_locations(ast.Module(body=[probe],type_ignores=[]));exec(compile(tree,'actual-postlift-block','exec'),scope)
for success in [True,False]:
 order=[];newG=np.eye(4);newG[0,3]=.00002 if success else .0021
 def obs(label,g,require_circle):order.append(('fresh_observe',label,require_circle));return {'result':{'pose':pose}}
 def wait(t,label):order.append(('wait',t,label))
 def grip(enabled,preload_m_s=None):order.append(('preload',enabled,preload_m_s))
 report={}
 try:result=scope['probe']({'pin_qualified_postlift_preload':True,'native_loaded_preload_after_contact_m_s':.015,'native_loaded_preload_ramp_s':.3},{'set_grip_hold':grip},np.eye(4),True,True,report,art,[0,1],obs,lambda v,g:newG,wait)
 except RuntimeError:
  assert not success
 else:
  assert success and np.array_equal(result,newG)
 assert [v[0]for v in order]==['preload','wait','fresh_observe'] and order[0][2]==.015 and abs(order[1][1]-.35)<1e-12
 assert report['pin_qualified_postlift_preload']['actual_contact_force_measured']is False
report=dict(status='PASS',scope='Actual extracted native driver ramp and postlift control-block mock checks only; no physical execution',actual_driver_ramp_samples_m_s=samples,unchanged_total_effort_cap_n=60.,invalid_targets_rejected=True,fresh_observation_after_ramp_order_pass=True,original_hold_failure_blocks_before_G_acceptance=True,original_hold_gate_failures_checked=['translation','axis','aperture','cap_coverage','propagated_center'],force_sensor_claim=False)
(out/'qualification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
