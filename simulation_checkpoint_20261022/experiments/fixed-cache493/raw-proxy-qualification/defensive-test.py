"""Helper-only mutation/failure/identity tests; no simulator/GPU imports."""
import sys,json
from pathlib import Path
import numpy as np
P=Path(__file__).parent;sys.path.insert(0,str(P/'overrides'));import payload_proxy_content_intern as helper
cube=np.array([[x,y,z]for x in [-.01,.01]for y in [-.01,.01]for z in [-.01,.01]])
def geometry(dtype=np.float64):return tuple(np.asarray(v,dtype=dtype).copy()for v in [np.zeros(3),np.eye(3),np.ones(3)*.01,cube,np.eye(3)])
class Original:
 def __init__(self):self.writes=0;self.kind='normal';self.body='body';self.source='source';self.dtype=np.float64
 def attach_payload(self,path,relative):
  self.writes+=1;self.payload_path=path;self.payload_relative=relative;self.payload_rows=[object()];self.original_query_rows=self.payload_rows;g=geometry(self.dtype)
  if self.kind=='nan':g[3][0,0]=np.nan
  if self.kind=='changed':g[3][0,0]+=.0001
  if self.kind=='malformed':g=(np.zeros(2),*g[1:])
  self.payload_boxes=[(self.body,self.source,g)];self.original_outer=self.payload_boxes;self.original_g=g;self.original_body=self.body;self.original_source=self.source
  return 'original-result'
info=helper.enable(Original);assert helper.enable(Original)is info
ck=Original();G=np.eye(4);assert ck.attach_payload('path',G)=='original-result';old=ck.payload_boxes[0][2];ck.attach_payload('path',G)
assert ck.payload_boxes[0][2]is old and ck.writes==2 and ck.payload_boxes is ck.original_outer and ck.payload_rows is ck.original_query_rows
assert ck.payload_boxes[0][0]is ck.original_body and ck.payload_boxes[0][1]is ck.original_source
fresh=G.copy();fresh[0,3]=.001;ck.attach_payload('path',fresh);assert ck.payload_relative is fresh and ck.payload_boxes[0][2]is old
for attr,value in [('dtype',np.float32),('body','new-body'),('source','new-source'),('kind','changed')]:
 setattr(ck,attr,value);ck.attach_payload('path',G);assert ck.payload_boxes[0][2]is ck.original_g
ck=Original();ck.attach_payload('path',G);old=ck.payload_boxes[0][2];old[3][0,0]+=.0001;before=info['stats']['cached_content_poisoned'];ck.attach_payload('path',G);assert ck.payload_boxes[0][2]is ck.original_g and info['stats']['cached_content_poisoned']==before+1
ck.attach_payload('path',G);old=ck.payload_boxes[0][2];old[0][0]=np.nan;before=info['stats']['cached_content_poisoned'];ck.attach_payload('path',G);assert ck.payload_boxes[0][2]is ck.original_g and info['stats']['cached_content_poisoned']==before+1
for kind in ['nan','malformed']:
 ck.kind=kind;ck.attach_payload('path',G);assert ck.payload_boxes[0][2]is ck.original_g
ck.kind='normal';bad=G.copy();bad[0,0]=np.nan;ck.attach_payload('path',bad);assert ck.payload_relative is bad and ck.payload_boxes[0][2]is ck.original_g
# Force a digest collision: exact bytes must still reject interning.
original_digest=helper._digest;helper._digest=lambda _:b'forced-digest';ck=Original();ck.attach_payload('path',G);old=ck.payload_boxes[0][2];ck.kind='changed';before=info['stats']['digest_collision_or_mismatch'];ck.attach_payload('path',G);assert ck.payload_boxes[0][2]is ck.original_g and ck.payload_boxes[0][2]is not old and info['stats']['digest_collision_or_mismatch']==before+1;helper._digest=original_digest
failure=RuntimeError('original attachment failed')
class Broken:
 def attach_payload(self,*args,**kwargs):raise failure
fault_info=helper.enable(Broken)
try:Broken().attach_payload('path',G)
except RuntimeError as exc:assert exc is failure
else:raise AssertionError('Original error swallowed')
assert fault_info['stats']['attachments']==0
result=dict(all_assertions_pass=True,original_writes_return_metadata_and_current_G_preserved=True,body_source_dtype_shape_bytes_change_no_stale_intern=True,valid_and_nonfinite_cached_poison_rejected=True,malformed_and_nonfinite_new_data_original_fallback=True,digest_collision_exact_confirmation=True,original_exception_identity_propagates=True,idempotent_enable=True,stats=info['stats'],GPU_work=False,physics_steps=0)
(P/'defensive-qualification.json').write_text(json.dumps(result,indent=2));print('PASS exact-field/currentG/write preservation + poison/malformed/collision/fault fixtures')
