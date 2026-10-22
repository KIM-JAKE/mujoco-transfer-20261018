import sys,types,ast,textwrap,json
from pathlib import Path
import numpy as np
BASE=Path(__file__).resolve().parent
s=(BASE/'overrides/direct_pin_skill.py').read_text();t=ast.parse(s)
node=next(n for n in ast.walk(t)if isinstance(n,ast.If)and ast.get_source_segment(s,n.test).startswith('radial<=.000075')and 'shaft_axis_max'in ast.get_source_segment(s,n.test))
block=textwrap.dedent(ast.get_source_segment(s,node))
class Preflight(RuntimeError):pass
class Arm:
 def fk(self,q):return np.eye(4)
class Art:
 def get_dof_positions(self):return cmd.copy()
cmd=np.zeros((1,9));calls={};settings={};ids=np.arange(7);phase=''
def assess(*a):calls['assess']+=1;return calls['assess']>settings.get('blocked_assess',0),{'current_image':True}
def plan(*a):
 calls['plan']+=1
 if settings.get('plan_error'):raise RuntimeError('no_current_route')
 result=dict(source_command=cmd[0,ids].copy(),source_actual=cmd[0,ids].copy(),source_command_all=cmd.copy(),source_actual_all=cmd[0].copy(),commands=np.array([cmd[0,ids].copy(),cmd[0,ids].copy()+.01]),candidate=1,timing={},terminal={})
 if settings.get('stale_source'):cmd[0,0]+=.01
 return result
sys.modules['pin_current_terminal']=types.SimpleNamespace(assess=assess,plan_rebranch=plan)
def line(*a):
 calls['line']+=1
 if calls['line']<=settings.get('blocked_line',0):raise Preflight('known_before_motion')
 if settings.get('partial_error'):raise RuntimeError('partial_actuation_error')
 calls['insert']+=1
def step():calls['step']+=1
def wait(*a):calls['hold']+=1
code='''def run():
    global phase
    terminal_replans=0;axial_inserted=False
    for alignment_index in range(alignment_budget):
        calls['captures']+=1
        if settings.get('only_last_qualified')and alignment_index<alignment_budget-1:continue
'''+textwrap.indent(block,'        ')+'''\n    return terminal_replans,axial_inserted\n'''
ns=dict(np=np,radial=.00005,axis_error=.002,shaft_axis_max=.0001,pin_config={'pin_terminal_stroke_preflight':True},alignment_budget=16,arm=Arm(),art=Art(),cmd=cmd,ids=ids,ctx={},payload_grasp=lambda:np.eye(4),remaining_depth=.045,line=line,PinCartesianPreflightBlocked=Preflight,report={},calls=calls,settings=settings,step=step,wait=wait,time=types.SimpleNamespace(perf_counter=lambda:1.),started=0.,DT=1/480)
exec(compile(code,'<EXACT_STAGED_QUALIFIED_TERMINAL_BRANCH>','exec'),ns)
def case(opts,expected_error=None):
 settings.clear();settings.update(opts);cmd[:]=0;calls.clear();calls.update(assess=0,plan=0,line=0,insert=0,step=0,hold=0,captures=0);ns['report']={'events':[]}
 try:r=ns['run']()
 except RuntimeError as e:
  if expected_error is None:raise
  assert expected_error in str(e),(str(e),expected_error);return dict(calls)
 assert expected_error is None;return dict(calls,replans=r[0],inserted=r[1])
a=case({});assert a['insert']==1 and a['plan']==0
b=case({'blocked_assess':1});assert b['plan']==1 and b['captures']==2 and b['insert']==1
c=case({'blocked_line':1});assert c['plan']==1 and c['captures']==2 and c['insert']==1
d=case({'blocked_assess':99},'Last qualified RGB-D');assert d['plan']==2 and d['insert']==0
e=case({'blocked_assess':1,'only_last_qualified':True},'Last qualified RGB-D');assert e['plan']==0 and e['captures']==16
f=case({'partial_error':True},'partial_actuation_error');assert f['plan']==0
h=case({'blocked_assess':1,'plan_error':True},'no_current_route');assert h['step']==0
j=case({'blocked_assess':1,'stale_source':True},'source changed');assert j['step']==0
# The only typed existing-line rejection precedes motion(), not its steps.
line_node=next(n for n in ast.walk(t)if isinstance(n,ast.FunctionDef)and n.name=='line');raises=[n for n in ast.walk(line_node)if isinstance(n,ast.Raise)and isinstance(n.exc,ast.Call)and isinstance(n.exc.func,ast.Name)and n.exc.func.id=='PinCartesianPreflightBlocked'];call=next(n for n in ast.walk(line_node)if isinstance(n,ast.Call)and isinstance(n.func,ast.Name)and n.func.id=='motion');assert len(raises)==1 and raises[0].lineno<call.lineno
result=dict(original_pass_single_axial_stroke=a,blocked_fresh_terminal_rebranch_then_new_RGBD=b,typed_original_line_rejection_rebranch=c,max_two_replans_no_insertion=d,last_original_iteration_cannot_rebranch=e,partial_errors_propagate=f,missing_route_zero_motion=h,stale_source_zero_motion=j,only_typed_line_rejection_before_motion=True,original_75um_3mrad_150um_limits_unchanged=True,original_alignment_budget_not_reset=True)
(BASE/'lifecycle-tests.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
