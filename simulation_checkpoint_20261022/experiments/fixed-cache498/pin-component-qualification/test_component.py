import ast,sys,types,json,copy
from pathlib import Path
import numpy as np
sys.path.insert(0,'/tmp/fixed498-pin-component/overrides')
from pin_contact_evaluator import PinContactEvaluator
O=Path('/tmp/fixed498-pin-component');base=ast.parse(Path('/tmp/fixed493-intern-and-small-full/overrides/loose_bin_task.py').read_text());new=ast.parse((O/'overrides/loose_bin_task.py').read_text());node=next(n for n in ast.walk(new)if isinstance(n,ast.If)and ast.unparse(n.test)=="c['config'].get('pin_only_component', False)")
source='def component_boundary():\n'+''.join('    '+line+'\n'for line in ast.unparse(node).splitlines())
tests=[]
for name,config,empty,fault in [('enabled',{'pin_only_component':True},True,False),('disabled',{},True,False),('pinfirst',{'pin_only_component':True,'pin_first':True},True,False),('one_ring',{'pin_only_component':True,'one_ring_only':True},True,False),('notempty',{'pin_only_component':True},False,False),('skillfault',{'pin_only_component':True},True,True)]:
 calls=[]
 def run_pin():
  calls.append('ORIGINAL_RUN_PIN')
  if fault:raise RuntimeError('original issued failure')
  return dict(pin_single_stroke_pass=True)
 ns=dict(c=dict(config=config,left_hand_known_empty=empty,started=0),left_program=None,results=[],run_pin=run_pin,events=[],time=types.SimpleNamespace(perf_counter=lambda:1))
 exec(source,ns);error=None
 try:result=ns['component_boundary']()
 except(Exception)as e:error=str(e)
 if name=='enabled':assert result['component_only']and result['full_task_pass']is False and result['vision_end_to_end_pass']is False and result['ring_tasks_executed']==0 and calls==['ORIGINAL_RUN_PIN']
 elif name=='disabled':assert result is None and not calls
 elif name=='skillfault':assert error=='original issued failure'and calls==['ORIGINAL_RUN_PIN']
 else:assert error and not calls
 tests.append(dict(case=name,passed=True))
# All original run_pin and per-motion source functions unchanged.
for name in ['run_pin','move','step','clear_left_work_zone_for_pin']:
 a=next(n for n in ast.walk(base)if isinstance(n,ast.FunctionDef)and n.name==name);b=next(n for n in ast.walk(new)if isinstance(n,ast.FunctionDef)and n.name==name);assert ast.dump(a,include_attributes=False)==ast.dump(b,include_attributes=False)
# Driver only labels this explicit boundary; no whole-task path can override it.
driver=ast.parse((O/'driver-proposed.py').read_text());classifier=next(n for n in ast.walk(driver)if isinstance(n,ast.If)and any(isinstance(a,ast.Constant)and a.value=='pin_only_component_pass'for a in ast.walk(n))and ast.unparse(n.test)=="startup_config.get('pin_only_component', False)")
for accepted in [True,False]:
 ns=dict(startup_config={'pin_only_component':True},result={'state':'pin_only_component_executed','full_task_pass':False,'pin':{'pin_single_stroke_pass':accepted}},report={'full_task_pass':True},physical=1.)
 exec(compile(ast.fix_missing_locations(ast.Module(body=[classifier],type_ignores=[])),'classifier','exec'),ns);assert ns['report']['full_task_pass']is False and ns['report']['pin_component_pass']is accepted and ns['report']['end_to_end_under30']is False
# Optional diagnostics leave inputs unchanged; measure units, filters/cap/errors.
paths={1:'/World/Cell/Geometry/workpiece_pin',2:'/World/robot/right_arm_finger',3:'/World/robot/left_arm_finger'};ev=PinContactEvaluator(''+paths[1],paths.__getitem__,maximum_records=1);h=types.SimpleNamespace(actor0=1,actor1=2,contact_data_offset=0,num_contact_data=1);d=[types.SimpleNamespace(position=np.array([0.,1.,2.]),normal=np.array([0.,1.,0.]),impulse=np.array([0.,.001,0.]),separation=-.0001)];old=copy.deepcopy(d[0].__dict__);ev.capture([h],d,0.,1/480,4);assert abs(ev.report['records'][0]['normal_force_magnitude_n']-1.92)<1e-12
for key in ['position','normal','impulse']:assert np.array_equal(old[key],getattr(d[0],key))
ev.capture([h],d,0.,1/480,4);assert ev.report['dropped_records']==1
ev.capture([h],d,1/480,1/480,4);assert ev.report['dropped_records']==1
bad=PinContactEvaluator(paths[1],paths.__getitem__);bad.capture([h],[types.SimpleNamespace(position=np.array([np.nan,0,0]))],0.,1/480,4);assert bad.report['decode_errors']==1 and not bad.report['records']
# Diagnostic hook is AFTER all original safety callback statements.
b=ast.parse(Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/runs/native-roas-table-fixed-front-v497-tail-flat-contact-minus/code_snapshot/run_realtime_recipe.py').read_text());oldcb=next(n for n in ast.walk(b)if isinstance(n,ast.FunctionDef)and n.name=='contacts');newcb=next(n for n in ast.walk(driver)if isinstance(n,ast.FunctionDef)and n.name=='contacts');assert len(newcb.body)==len(oldcb.body)+1
for a,b in zip(oldcb.body,newcb.body):assert ast.dump(a,include_attributes=False)==ast.dump(b,include_attributes=False)
(O/'component-tests.json').write_text(json.dumps(dict(all_pass=True,boundary_cases=tests,original_skill_and_all_motion_functions_AST_unchanged=True,original_contact_safety_prefix_AST_identical=True,component_result_never_full_task=True,telemetry_unit_filter_cap_malformed_input_and_immutability_pass=True),indent=2));print('PASS6 component boundaries +original functions +optional evaluator/driver invariants')
