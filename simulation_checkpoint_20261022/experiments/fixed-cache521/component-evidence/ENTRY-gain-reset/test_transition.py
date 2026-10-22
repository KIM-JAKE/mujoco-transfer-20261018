from pathlib import Path
import ast,json,numpy as np
O=Path(__file__).parent;src=(O/'overrides/direct_pin_skill.py').read_text();tree=ast.parse(src);f=next(x for x in tree.body if isinstance(x,ast.FunctionDef)and x.name=='execute');node=next(n for n in ast.walk(f)if isinstance(n,ast.If)and ast.unparse(n.test)=='entry_feedback_enabled and (not entry_stage)');loop=ast.For(target=ast.Name(id='fixture_iteration',ctx=ast.Store()),iter=ast.List(elts=[ast.Constant(value=0)],ctx=ast.Load()),body=[node],orelse=[]);ast.fix_missing_locations(loop);code=compile(ast.Module(body=[loop],type_ignores=[]),'<literal-transition>','exec')
def run(flag,fail=False,last=False):
 lines=[];report={};H=np.eye(4)
 def line(*args):
  lines.append(args)
  if fail:raise RuntimeError('partial-motion-fault')
 e=dict(entry_feedback_enabled=True,entry_stage=False,alignment_index=15 if last else 4,alignment_budget=16,current_pre=H,remaining_depth=.045,line=line,report=report,correction_gain=.3,recovery_ceiling=.65,previous_radial_error=.000127,previous_alignment_score=5.,radial_improvement_streak=1,pin_config={'pin_reset_feedback_at_entry':flag})
 error=None
 try:exec(code,e)
 except RuntimeError as ex:error=str(ex)
 return e,lines,error
rows=[]
e,l,x=run(True);assert x is None and e['entry_stage']and len(l)==1 and e['correction_gain']==.65 and e['previous_radial_error']is None and e['previous_alignment_score']is None and e['radial_improvement_streak']==0;assert e['alignment_index']==4 and e['alignment_budget']==16;assert e['report']['pin_entry_feedback_history_reset']['remaining_original_images']==11;rows.append('successful_partial_move_then_history_only_reset_no_budget_reset')
e,l,x=run(False);assert x is None and e['correction_gain']==.3 and e['previous_radial_error']==.000127 and e['previous_alignment_score']==5. and e['radial_improvement_streak']==1 and 'pin_entry_feedback_history_reset'not in e['report'];rows.append('flagoff_original_feedback_state')
e,l,x=run(True,fail=True);assert x=='partial-motion-fault'and not e['entry_stage']and e['correction_gain']==.3 and not e['report'];rows.append('no_reset_after_partial_motion_fault')
e,l,x=run(True,last=True);assert x and not l and not e['entry_stage']and e['correction_gain']==.3;rows.append('shared_budget_exhausted_before_partial_no_reset')
# New config guard is independent of physical context.
guard=next(n for n in ast.walk(f)if isinstance(n,ast.If)and "pin_reset_feedback_at_entry"in ast.unparse(n.test)and 'not entry_feedback_enabled'in ast.unparse(n.test));
try:exec(compile(ast.Module(body=[guard],type_ignores=[]),'<guard>','exec'),dict(pin_config={'pin_reset_feedback_at_entry':True},entry_feedback_enabled=False));raise AssertionError('reset withoutstage accepted')
except ValueError:pass
old=ast.parse(Path('/tmp/fixed521-pin-entry-axis-view/overrides/direct_pin_skill.py').read_text());funcs=lambda t:{n.name:ast.dump(n,include_attributes=False)for n in ast.walk(t)if isinstance(n,ast.FunctionDef)and n.name!='execute'};assert funcs(old)==funcs(tree)
(O/'transition-tests.json').write_text(json.dumps(dict(passed=True,critical_cases=rows,requires_two_stage=True,all_original_nested_motion_sensor_proof_servo_functions_AST_identical=True,no_native_or_physics_trial=True),indent=2)+'\n');print(rows)
