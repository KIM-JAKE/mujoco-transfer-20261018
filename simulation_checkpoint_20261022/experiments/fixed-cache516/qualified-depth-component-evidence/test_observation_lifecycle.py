import ast,types,json,numpy as np
from pathlib import Path
O=Path(__file__).parent;new=ast.parse((O/'overrides/direct_pin_skill.py').read_text());old=ast.parse(Path('/tmp/fixed512-camera-priority-full/overrides/direct_pin_skill.py').read_text());find=lambda tree,name:next(n for n in ast.walk(tree)if isinstance(n,ast.FunctionDef)and n.name==name)
node=find(new,'observe');code=compile(ast.Module(body=[node],type_ignores=[]),'<literal-new-observe>','exec');oldcode=compile(ast.Module(body=[find(old,'observe')],type_ignores=[]),'<literal-original-observe>','exec')
packets=json.loads(Path('/tmp/fixed515-pin-depth-diagnosis/public-depth-comparison.json').read_text());r=next(x for x in packets['rows']if x['run']==514);valid=r['capture_rows'][0]['pose'];fragment=r['capture_rows'][1]['pose'];assert valid['robust_cap_circle_from_pixels']and 'robust_cap_circle_from_pixels'not in fragment
tests=[]
def canonical(x):return json.dumps(x,sort_keys=True,default=lambda value:value.tolist())
def exercise(flag,poses,require=None,label='pin_full_depth_RGBD',fail=None,useold=False,latency=.2):
 calls=[];report={'events':[]};steps=[]
 class Future:
  def done(self):return True
  def result(self):
   if fail:raise RuntimeError(fail)
   pose=poses[min(len(calls)-1,len(poses)-1)]
   return dict(result={'pose':pose},captured_wall_s=10.,ready_wall_s=10.+latency,H=np.eye(4))
 class Vision:
  def request(self,e,H):calls.append(e);return Future()
  def poll(self):raise AssertionError('No wait or step added for ready frame')
 env=dict(np=np,ctx={},arm=types.SimpleNamespace(fk=lambda q:np.eye(4)),art=types.SimpleNamespace(get_dof_positions=lambda:np.zeros((1,7))),ids=list(range(7)),pin_config={'robust_pin_cap_circle':True},qualified_full_depth=flag,visual_preinsert=True,vision=Vision(),time=types.SimpleNamespace(perf_counter=lambda:11.),started=0.,step=lambda:steps.append(1),report=report,print=lambda *a,**k:None,_pin_ordered_camera_views=lambda offsets,label,cfg:list(enumerate(offsets)))
 exec(oldcode if useold else code,env);error=None;result=None
 try:result=env['observe'](label,np.eye(4),require_circle=flag if require is None else require)
 except RuntimeError as e:error=str(e)
 return result,calls,report,steps,error
result,calls,report,steps,error=exercise(True,[fragment,valid]);assert result['result']['pose']is valid and len(calls)==2 and not steps and error is None;assert all(e['robust_pin_cap_circle']for e in calls);assert report['pin_full_depth_qualified_observations'][0]['view_index']==1;tests.append('literal514_bad_final_fragment_rejected_next_existing_view_qualified')
result,calls,report,steps,error=exercise(True,[fragment]);assert result is None and len(calls)==13 and not report['events']and not steps and error.startswith('No valid pin RGB-D view');tests.append('all13bad_original_views_fail_before_depth_use')
newr=exercise(False,[fragment]);oldr=exercise(False,[fragment],useold=True);assert canonical(newr[:3])==canonical(oldr[:3]);tests.append('flagoff_literal514_packet_matches_original')
result,calls,report,steps,error=exercise(True,[valid]);assert result and len(calls)==1 and len(report['pin_full_depth_qualified_observations'])==1;tests.append('first_good_original_circle_no_extra_frame_or_step')
result,calls,report,steps,error=exercise(True,[valid],fail='camera_backend_fault');assert result is None and len(calls)==1 and error=='camera_backend_fault'and not steps;tests.append('backend_fault_not_caught_or_retried')
result,calls,report,steps,error=exercise(True,[valid],latency=3.01);assert error=='RGB-D capture-to-ready exceeded3s'and len(calls)==1;tests.append('original3second_gate_preserved')
# Same falsefragment handling for unrelated observations: this feature is
# deliberately scoped to depth, not a hidden replacement of all perception.
newr=exercise(True,[fragment],require=False,label='pin_lift_RGBD');oldr=exercise(True,[fragment],require=False,label='pin_lift_RGBD',useold=True);assert canonical(newr[:2])==canonical(oldr[:2]);tests.append('other_observation_labels_unchanged')
for name in('motion','line','fingers','endpoint_candidates','_pin_visual_preinsert_target','_pin_bounded_xy_gain','_pin_rgbd_sdk_hold_evidence','_pin_ordered_camera_views'):
 assert ast.dump(find(new,name),include_attributes=False)==ast.dump(find(old,name),include_attributes=False),name
tests.append('all_original_motion_proof_servo_precision_retention_functions_AST_identical')
calls=[n for n in ast.walk(new)if isinstance(n,ast.Call)and isinstance(n.func,ast.Name)and n.func.id=='observe'and n.args and isinstance(n.args[0],ast.Constant)and n.args[0].value=='pin_full_depth_RGBD'];assert len(calls)==2 and all(any(k.arg=='require_circle'and isinstance(k.value,ast.Name)and k.value.id=='qualified_full_depth'for k in n.keywords)for n in calls);tests.append('initial_and_both_correction_reobservations_require_same_qualified_gate')
(O/'observation-tests.json').write_text(json.dumps(dict(passed=True,cases=len(tests),tests=tests,actual514_fragment_fixture=True,full_scene_or_physical_validation=False),indent=2)+'\n');print(tests)
