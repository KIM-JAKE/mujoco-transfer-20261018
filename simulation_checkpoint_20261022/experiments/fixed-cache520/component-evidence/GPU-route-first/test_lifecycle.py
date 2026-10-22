import ast,copy,types,sys,json
from pathlib import Path
import numpy as np
O=Path(__file__).parent;P=O/'overrides/direct_pin_skill.py';tree=ast.parse(P.read_text());base=ast.parse(Path('/tmp/fixed519-entry-feedback-full/overrides/direct_pin_skill.py').read_text())
# Strip exactly the new opt-in branch and helper: ALL prior code is AST-identical.
t=copy.deepcopy(tree);t.body=[n for n in t.body if not(isinstance(n,ast.FunctionDef)and n.name=='_pin_gpu_cubic_route_before_hint')]
class Strip(ast.NodeTransformer):
 def visit_If(self,n):
  if isinstance(n.test,ast.Call) and isinstance(n.test.func,ast.Attribute) and isinstance(n.test.func.value,ast.Name)and n.test.func.value.id=='pin_config'and n.test.args and isinstance(n.test.args[0],ast.Constant)and n.test.args[0].value=='pin_gpu_route_before_hint':return None
  return self.generic_visit(n)
t=Strip().visit(t);assert ast.dump(t)==ast.dump(base)
class Verified(list):verified_cubic=True
state={}
def proposal(*args):state.setdefault('calls',[]).append('proposal');return state['proposal']
def cubic(*args,**kwargs):state['calls'].append('continuous');assert kwargs=={'step_rad':.010};return state['continuous'],{'geometry_step_rad':.010}
class Checker:
 gpu_route_enabled=True
 def __init__(self):self.arm=types.SimpleNamespace(bounds=np.array([[-3,3]]*7));self.resolution_rad=.035;self.gpu_route_stats={};self.calls=[]
 def clear(self,*args):self.calls.append(('goal',args));return state.get('goal',True),None
 def path(self,*args):
  self.calls.append(('path',args,self.resolution_rad))
  if state.get('raise'):raise RuntimeError('original native fault')
  return state.get('clear',True),None
sys.modules['fr3_gpu_route']=types.SimpleNamespace(propose_route=proposal);sys.modules['gpu_route_curve_validation']=types.SimpleNamespace(cubic_geometry=cubic);sys.modules['grasp_transfer_feasibility']=types.SimpleNamespace(VerifiedCubicRoute=Verified)
helper=next(n for n in tree.body if isinstance(n,ast.FunctionDef)and n.name=='_pin_gpu_cubic_route_before_hint');env={'print':lambda *a,**k:None};exec(compile(ast.Module(body=[helper],type_ignores=[]),str(P),'exec'),env);f=env[helper.name];q=np.zeros(7);proposed=[q,np.ones(7)*.1];curve=np.array(proposed);tests=[]
for case,opts,expected in [('disabled',{},False),('goalblocked',{'goal':False},False),('no_proposal',{'proposal':None},False),('no_continuous',{'continuous':None},False),('nativeblocked',{'clear':False},False),('accepted',{},True),('nativefault',{'raise':True},None)]:
 state.clear();state.update(proposal=proposed,continuous=curve,**{k:v for k,v in opts.items()if k not in ['proposal','continuous']});state.update({k:v for k,v in opts.items()if k in ['proposal','continuous']});ck=Checker();ck.gpu_route_enabled=case!='disabled';before=q.copy()
 try:got=f(ck,q,np.ones(7),.0105,'/pin')
 except RuntimeError as e:assert case=='nativefault'and str(e)=='original native fault';got=None
 else:assert expected is not None and bool(got is not None)==expected
 assert ck.resolution_rad==.035 and np.array_equal(q,before)
 if expected:assert isinstance(got,Verified)and np.array_equal(got,proposed)and ck.calls[-1][2]==.010
 tests.append({'case':case,'PASS':True})
# Literal full original endpoint selection loop, including every original route fallback.
loop=next(n for n in ast.walk(tree)if isinstance(n,ast.For)and isinstance(n.iter,ast.Call)and isinstance(n.iter.func,ast.Name)and n.iter.func.id=='endpoint_candidates'and any(isinstance(x,ast.Constant)and x.value=='pin_preinsert_carry'for x in n.iter.args))
fn=ast.FunctionDef(name='choose',args=ast.arguments(posonlyargs=[],args=[],kwonlyargs=[],kw_defaults=[],defaults=[]),body=[copy.deepcopy(loop),ast.Return(value=ast.Tuple(elts=[ast.Name(id='shortcut',ctx=ast.Load()),ast.Name(id='report',ctx=ast.Load())],ctx=ast.Load()))],decorator_list=[]);mod=ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[]))
for case in ['flagoff_warm','GPUhit','GPUmiss_warm','GPUmiss_hardhint','GPUmiss_CPU','directedge_first','nativefault_propagates']:
 calls=[];report={};target=np.ones(7);warm=Verified([q,target]);cpuroute=[q,target]
 class C:
  def path(self,route,*args):calls.append('path');return case=='directedge_first'or(case=='GPUmiss_hardhint'and len(calls)>1),None
  def route(self,*args,**kwargs):calls.append('CPU');return cpuroute
 def gpu(*args):
  calls.append('GPU')
  if case=='nativefault_propagates':raise RuntimeError('native')
  return warm if case=='GPUhit'else None
 def hint(*args):calls.append('warm');return warm if case in ['flagoff_warm','GPUmiss_warm']else None
 sys.modules['grasp_transfer_feasibility']=types.SimpleNamespace(robot_route_hint=hint,hint_path_clear=lambda *a:True)
 e=dict(endpoint_candidates=lambda *a:iter([target]),candidate_pre=np.eye(4),robot_seeds={'pin_direct_preinsert':q,'pin_tail_carry_to_preinsert':q},cmd=np.zeros((1,7)),ids=np.arange(7),checker=C(),paths={'workpiece_pin':'/pin'},shortcut=None,report=report,ctx={},robot_hint_data={'routes':{'pin_direct_carry_to_preinsert':[q.tolist(),target.tolist()]}},np=np,pin_config={'pin_gpu_route_before_hint':case!='flagoff_warm'},_pin_gpu_cubic_route_before_hint=gpu)
 exec(compile(mod,str(P),'exec'),e)
 try:res=e['choose']()
 except RuntimeError as ex:assert case=='nativefault_propagates'and str(ex)=='native'
 else:assert case!='nativefault_propagates'and res[0]is not None
 if case=='flagoff_warm':assert 'GPU'not in calls
 if case=='GPUhit':assert calls==['path','GPU']and report['pin_GPU_cubic_route_before_hint_used']
 if case=='directedge_first':assert calls==['path']
 if case=='GPUmiss_CPU':assert calls[-1]=='CPU'
 tests.append({'case':case,'calls':calls,'PASS':True})
(O/'lifecycle-results.json').write_text(json.dumps({'original_AST_exact_after_removing_new_branch_and_helper':True,'tests':tests,'native_collision_equivalence_claim':False,'no_jobs_or_actuation':True},indent=2)+'\n');print('PASS',len(tests))
