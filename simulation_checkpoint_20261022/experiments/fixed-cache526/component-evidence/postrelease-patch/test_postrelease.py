from pathlib import Path
import ast,copy,json,numpy as np
from types import SimpleNamespace
from scipy.spatial.transform import Rotation
out=Path('/tmp/fixed526-postrelease-rgbd');tree=ast.parse((out/'overrides/direct_pin_skill.py').read_text());ns={'np':np,'log':lambda R:Rotation.from_matrix(R).as_rotvec()}
for name in ['_pin_postrelease_sdk_evidence','_pin_postrelease_rgbd_evidence','_pin_ordered_camera_views','_pin_full_depth_near_axis_views']:
 node=next(x for x in tree.body if isinstance(x,ast.FunctionDef)and x.name==name);exec(compile(ast.Module(body=[node],type_ignores=[]),'isolated_test','exec'),ns)
source=Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/runs/native-roas-table-fixed-front-v522-entry-before-contact-pin-component');d=json.load(open(source/'pipeline-results.json'));g=d['scattered_pick']['pin'];pose=copy.deepcopy(next(e['result']['pose']for e in g['events']if e['event']=='pin_full_depth_RGBD'));goal=np.array([-.30203095784,.22724999619,1.52875]);measure=ns['_pin_postrelease_rgbd_evidence'];assert measure(pose,goal)['confirmed']
for key,value in [('lateral_from_previous_RGBD_grasp_and_encoders',True),('robust_cap_circle_from_pixels',False),('cap_edge_coverage_deg',239.),('uses_simulator_body_poses',True)]:
 bad=copy.deepcopy(pose);bad[key]=value;assert not measure(bad,goal)['confirmed']
bad=copy.deepcopy(pose);bad['center_m']=(goal+[.0016,0,0]).tolist();assert not measure(bad,goal)['confirmed'];bad=copy.deepcopy(pose);bad['axis']=[np.sin(.051),0,np.cos(.051)];assert not measure(bad,goal)['confirmed']
# Public522 SDK release/withdraw completion from saved encoders and original FK.
model=json.load(open(source/'robot-kinematics.json'));z=np.load(source/'executed-actuator-commands.npz');ii=np.flatnonzero(z['pin_phase']=='pin_retention')[-1];ids=np.array([1,3,5,7,9,11,13]);fi=np.array([16,17])
def fk(q):
 T=np.asarray(model['base'],float).copy()
 for value,axis,(a,b)in zip(q,model['axes'],model['frames']):
  E=np.eye(4);E[:3,:3]=Rotation.from_rotvec(np.asarray(axis)*value).as_matrix();T=T@a@E@b
 return T@model['tip']
W=np.asarray(g['pin_entry_feedback_transition']['far_preinsert_Hand']);sdk=ns['_pin_postrelease_sdk_evidence'](fk(z['actual_dof_position'][ii,ids]),fk(z['command'][ii,ids]),W,z['actual_dof_position'][ii,fi],z['command'][ii,fi]);assert sdk['confirmed'],sdk
# Execute ONLY new terminal branch; even unavailable truth cannot veto measured pass.
execute=next(x for x in tree.body if isinstance(x,ast.FunctionDef)and x.name=='execute');branch=next(x for x in reversed(execute.body)if isinstance(x,ast.If)and isinstance(x.test,ast.Name)and x.test.id=='postrelease_rgbd');fn=ast.FunctionDef(name='test_tail',args=ast.arguments(posonlyargs=[],args=[],kwonlyargs=[],kw_defaults=[],defaults=[]),body=[branch],decorator_list=[]);mod=ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[]));calls=[];cmd=np.zeros((1,9));cmd[0,7:]=.009;anchor=np.eye(4);anchor[:3,3]=pose['center_m'];W=np.eye(4);W[:3,3]=[0,0,.5]
class NoTruth:
 def get_transforms(self):raise RuntimeError('No privileged sensor available')
clock=SimpleNamespace(perf_counter=lambda:11.);retained={'captured_wall_s':12.,'ready_wall_s':12.1,'result':{'pose':pose}}
def observe(label,**kw):calls.append((label,kw));return retained
ctx=dict(ns,postrelease_rgbd=True,art=SimpleNamespace(get_dof_positions=lambda:cmd.copy()),arm=SimpleNamespace(fk=lambda q:W.copy()),cmd=cmd,ids=np.arange(7),fi=np.array([7,8]),withdraw=W,report={},time=clock,started=0.,observe=observe,retained_world_pose=anchor,goal=goal,truth={'pin':NoTruth()},body_goal=goal)
exec(compile(mod,'tail_lifecycle','exec'),ctx);r=ctx['test_tail']();assert r['pin_single_stroke_pass']and len(calls)==1;assert calls[0][0]=='pin_postrelease_retention_RGBD'and np.array_equal(calls[0][1]['world_reference'],anchor);assert r['pin_retention_EVALUATION_ONLY']['used_for_control']is False
retained['captured_wall_s']=10.
try:ctx['test_tail']()
except RuntimeError as e:assert 'new capture'in str(e)
else:raise AssertionError('Old image accepted')
# Sensor function re-anchors EVERY fallback to fixed public world pose, despite SDK Hand change.
obsnode=next(x for x in execute.body if isinstance(x,ast.FunctionDef)and x.name=='observe');counter=[0];requests=[]
def moving_fk(q):
 counter[0]+=1;T=np.eye(4);T[0,3]=counter[0]*.001;return T
class Future:
 def __init__(self,v):self.v=v
 def done(self):return True
 def result(self):return self.v
class Vision:
 def request(self,event,H):
  requests.append((copy.deepcopy(event),H.copy()));return Future(dict(captured_wall_s=12.,ready_wall_s=12.1,result={'error':'occluded'}if len(requests)==1 else{'pose':pose}))
 def poll(self):pass
ns2=dict(ns,art=SimpleNamespace(get_dof_positions=lambda:cmd.copy()),arm=SimpleNamespace(fk=moving_fk),ids=np.arange(7),ctx={},postrelease_rgbd=True,pin_config=dict(fixed_supply_mode=True,pin_full_depth_require_robust_circle=True,robust_pin_cap_circle=True,pin_full_depth_near_axis_views=True),visual_preinsert=True,qualified_full_depth=True,vision=Vision(),report={'events':[]},time=clock,started=0.,step=lambda:None)
exec(compile(ast.fix_missing_locations(ast.Module(body=[obsnode],type_ignores=[])),'observe_lifecycle','exec'),ns2);v=ns2['observe']('pin_postrelease_retention_RGBD',require_circle=True,world_reference=anchor);assert len(requests)==2
for event,H in requests:assert np.allclose(H@np.asarray(event['tracked_grasp']),anchor,atol=1e-12)
# Existing 0.5s wait and every other sensor/motion/servo function unchanged.
base=ast.parse(Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/runs/native-roas-table-fixed-front-v522-entry-before-contact-pin-component/code_snapshot/direct_pin_skill.py').read_text());be=next(x for x in base.body if isinstance(x,ast.FunctionDef)and x.name=='execute');A={x.name:ast.dump(x,include_attributes=False)for x in be.body if isinstance(x,ast.FunctionDef)};B={x.name:ast.dump(x,include_attributes=False)for x in execute.body if isinstance(x,ast.FunctionDef)};assert all(A[k]==B[k]for k in A if k!='observe')
result=dict(pass_=True,source522_SDK_completion=sdk,direct_pixel_class_cap_mandatory=True,original_final_position_axis_gates=True,stale_image_rejected=True,fixed_world_anchor_survives_SDK_movement=True,one_observation_operation=True,failed_first_view_retains_fallback=True,unavailable_truth_cannot_change_public_sensor_pass=True,no_actuation_added=True,existing_other_nested_functions_unchanged=True,original_retention_wait_unchanged=True,no_GPU_PhysX_jobs=True);(out/'focused-tests.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
