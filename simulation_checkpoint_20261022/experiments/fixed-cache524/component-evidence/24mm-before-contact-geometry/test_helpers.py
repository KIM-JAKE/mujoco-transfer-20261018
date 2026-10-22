from pathlib import Path
import ast,json,numpy as np
from contextlib import nullcontext
O=Path(__file__).parent
class Arm:
 bounds=np.tile([-2.,2.],(7,1))
 def fk(self,q):T=np.eye(4);T[2,3]=q[0];return T
 def solve(self,P,R,seed):q=np.array(seed).copy();q[0]=P[2];return q
class Checker:
 payload_path=None;payload_rows=None;payload_boxes=None;payload_relative=None;preview_other_boxes=[];resolution_rad=.035
 def __init__(self):self.paths=[];self.widths=[]
 def detach_payload(self):pass
 def path(self,q,w,part):self.paths.append(np.array(q));return True,{}
 def clear(self,q,w,part):self.widths.append(w);return True,{}
def call(source,depth,entry=False):
 tree=ast.parse(source);f=next(x for x in tree.body if isinstance(x,ast.FunctionDef)and x.name=='check_terminal');env={'np':np,'nullcontext':nullcontext,'pair_clear':lambda *a:(True,{}),'issued_geometry_samples':lambda x,**kw:(x,{}),'retime':lambda points,*a,**kw:(np.asarray(points),None,{})};exec(compile(ast.Module(body=[f],type_ignores=[]),'<terminal>','exec'),env);c=Checker();q=np.zeros(7);q[0]=-depth;far=np.eye(4);far[2,3]=-.045;kw=dict(entry_stage=True,withdraw_H=far,requested_s=.32)if entry else{}
 try:r=env['check_terminal']({'dt':1/480},Arm(),c,q,depth,.0127,'/pin',**kw);return r,c,None
 except ValueError as e:return None,c,str(e)
new=(O/'overrides/pin_terminal_geometry.py').read_text();old=Path('/tmp/fixed521-measured-entry-full/overrides/pin_terminal_geometry.py').read_text();a,ca,e=call(old,.045);b,cb,e=call(new,.045);assert a[0]and b[0]and all(np.array_equal(x,y)for x,y in zip(ca.paths,cb.paths))and ca.widths==cb.widths
r,c,e=call(new,.024);assert r is None and e
r,c,e=call(new,.024,True);assert r[0]and e is None;assert abs(c.paths[-1][-1,0]+.045)<1e-12 and abs(c.paths[-1][0,0])<1e-12;assert len(c.widths)==13 and c.resolution_rad==.035
r,c,e=call(new,.045,True);assert r is None and e
(O/'helper-tests.json').write_text(json.dumps(dict(passed=True,original_far45_default_field_and_guards_identical=True,entry24_rejected_in_original_state=True,entry24_explicit_preserves_full45_far_withdraw_path=True,original13_release_widths_checked=True,entry_rejects_far45=True),indent=2)+'\n');print('criticalhelpertestsPASS')
