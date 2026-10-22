import ast,json,numpy as np,types
from pathlib import Path
O=Path('/tmp/fixed506-pin-rebranch-feedback');tree=ast.parse((O/'overrides/direct_pin_skill.py').read_text());handler=next(n for n in ast.walk(tree)if isinstance(n,ast.ExceptHandler)and ast.unparse(n.type)=='PinCartesianPreflightBlocked');source='def recovery_handler():\n    global terminal_replans,phase,correction_gain,previous_radial_error,previous_alignment_score,radial_improvement_streak\n    for _once in range(1):\n'+''.join('        '+line+'\n'for stmt in handler.body for line in ast.unparse(stmt).splitlines())
results=[]
for case in['enabled','disabled','partial_step_fault','proof_fault','source_stale']:
 cmd=np.zeros((1,18),np.float32);actual=cmd[0].copy();steps=[];ns=dict(pin_config={'pin_reset_feedback_after_rebranch':case!='disabled'},terminal_replans=0,alignment_index=10,alignment_budget=16,ctx={},payload_grasp=lambda:np.eye(4),remaining_depth=.045,cmd=cmd,art=types.SimpleNamespace(get_dof_positions=lambda:actual[None]),np=np,ids=np.arange(7),correction_gain=.4615384615384615,recovery_ceiling=.65,previous_radial_error=.0000953,previous_alignment_score=1.5,radial_improvement_streak=1,phase='old',time=types.SimpleNamespace(perf_counter=lambda:1),started=0.,DT=1/480,report={'events':[]},rejected=RuntimeError('typed premotion block'),wait=lambda *a:None)
 def plan(*a):
  if case=='proof_fault':raise RuntimeError('original proof unavailable')
  return dict(commands=np.vstack([np.zeros(7),np.ones(7)*.001,np.ones(7)*.002]),source_command_all=np.ones((1,18),np.float32)if case=='source_stale'else cmd.copy(),source_actual_all=actual.copy(),candidate=0,timing={},terminal={})
 def step():
  steps.append('original_step')
  if case=='partial_step_fault':raise RuntimeError('original partial issue fault')
 ns['plan_rebranch']=plan;ns['step']=step;exec(source,ns);error=None
 try:ns['recovery_handler']()
 except RuntimeError as e:error=str(e)
 if case=='enabled':
  assert error is None and len(steps)==2 and ns['terminal_replans']==1 and ns['correction_gain']==.65 and ns['previous_radial_error']is None and ns['previous_alignment_score']is None and ns['radial_improvement_streak']==0 and len(ns['report']['pin_feedback_resets_after_rebranch'])==1
 elif case=='disabled':assert error is None and len(steps)==2 and ns['correction_gain']==.4615384615384615 and ns['previous_radial_error']==.0000953 and'pin_feedback_resets_after_rebranch'not in ns['report']
 else:assert error and ns['correction_gain']==.4615384615384615 and ns['previous_radial_error']==.0000953 and'pin_feedback_resets_after_rebranch'not in ns['report']and len(steps)==(1 if case=='partial_step_fault'else 0)
 assert ns['alignment_budget']==16 and ns['alignment_index']==10
 results.append(dict(case=case,passed=True,error=error,original_steps=len(steps)))
# Original full motion / terminal planning / feedback law are unchanged.
base=ast.parse(Path('/home/gpuadmin/workspace/jake/mujoco/isaac_port/runs/native-roas-table-fixed-front-v505-pin-collar-flat-1mm-component/code_snapshot/direct_pin_skill.py').read_text())
for name in['motion','line','observe','measured_grasp','endpoint_candidates','_pin_visual_preinsert_target','_pin_bounded_xy_gain']:
 f=lambda t:next(n for n in ast.walk(t)if isinstance(n,ast.FunctionDef)and n.name==name)
 assert ast.dump(f(base),include_attributes=False)==ast.dump(f(tree),include_attributes=False)
(O/'lifecycle-tests.json').write_text(json.dumps(dict(all_pass=True,tests=results,original_feedback_laws_and_motion_guards_AST_unchanged=True,total_image_budget_16_unchanged=True,reset_after_full_original_proof_and_completed_motion_only=True),indent=2));print('PASS5 reset/fault lifecycles +7original functions +sharedbudget invariants')
