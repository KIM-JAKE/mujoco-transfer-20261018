"""Raw alias cache: exact input fields, single original fallback, poison/errors."""
import sys,json,types,importlib.util
from pathlib import Path
import numpy as np
P=Path(__file__).parent;sys.path.insert(0,str(P/'overrides'));import payload_proxy_content_intern as helper
R=Path('/home/gpuadmin/workspace/jake/mujoco');f=R/'isaac_port/runs/native-roas-table-fixed-front-v490-width-memo-placed-broadphase-nominal/code_snapshot/predictive_pair_clearance.py';spec=importlib.util.spec_from_file_location('independent_original_predictive',f);pred=importlib.util.module_from_spec(spec);spec.loader.exec_module(pred);original=pred.proxy
namespace=dict(proxy=original);exec("def attach_payload(self,path,relative):\n self.payload_path=path;self.payload_relative=relative;self.payload_boxes=[('body','source',proxy(self.vertices))];self.payload_rows=[object()]\n",namespace);Native=type('Native',(),dict(attach_payload=namespace['attach_payload']));info=helper.enable(Native);assert info['raw_proxy_cache']['enabled']and pred.proxy is original;cached=namespace['proxy'];stats=info['stats']
cube=np.array([[x,y,z]for x in [-.01,.01]for y in [-.01,.01]for z in [-.01,.01]],dtype=np.float64)
a=cached(cube);before=stats['raw_proxy_original_calls'];b=cached(cube.copy());assert a is b and stats['raw_proxy_original_calls']==before
# Every raw field and byte belongs to the key, with no normalized dtypes/layout.
variants=[cube.astype(np.float32),np.asfortranarray(cube),np.vstack([cube,cube[:1]]),cube.copy()];variants[-1][0,0]+=.0001
for v in variants:
 before=stats['raw_proxy_original_calls'];out=cached(v);assert stats['raw_proxy_original_calls']==before+1 and helper._snapshot([('a','b',out)])==helper._snapshot([('a','b',original(v))])
# Cached output valid-byte and NaN corruption both evaluate original ONCE.
for poison in ['finite','nan']:
 x=cube.copy();x[0,0]+=(.00002 if poison=='finite'else .00003);out=cached(x)
 if poison=='finite':out[3][0,0]+=.00001
 else:out[0][0]=np.nan
 before=stats['raw_proxy_original_calls'];new=cached(x.copy());assert stats['raw_proxy_original_calls']==before+1 and new is not out and helper._snapshot([('a','b',new)])==helper._snapshot([('a','b',original(x))])
# Cached input strong reference is checked too, even when output was a copy.
x=cube.copy();x[0,0]+=.00004;saved=x.copy();out=cached(x);x[0,1]+=.0001;before=stats['raw_proxy_original_calls'];fresh=cached(saved);assert stats['raw_proxy_original_calls']==before+1 and fresh is not out
# Force SHA lookup collision; full exact input confirmation still falls back.
old_digest=helper._raw_digest;helper._raw_digest=lambda *args:b'forced';x=cube.copy();x[0,0]+=.00005;first=cached(x);x=x.copy();x[0,1]+=.00001;before=stats['raw_proxy_original_calls'];second=cached(x);assert second is not first and stats['raw_proxy_original_calls']==before+1 and stats['raw_proxy_digest_collision_or_mismatch']>0;helper._raw_digest=old_digest
# Unsupported/NaN data calls the actual original exactly once, preserving error.
for bad in [cube.tolist(),np.full((8,3),np.nan)]:
 before=stats['raw_proxy_original_calls']
 try:expected=original(bad)
 except Exception as expected_error:
  try:cached(bad)
  except Exception as actual_error:assert type(actual_error)is type(expected_error)and str(actual_error)==str(expected_error)
  else:raise AssertionError('Original error swallowed')
 else:assert helper._snapshot([('a','b',cached(bad))])==helper._snapshot([('a','b',expected)])
 assert stats['raw_proxy_original_calls']==before+1
# Original ConvexHull failure identity is propagated, never retried.
failure=RuntimeError('original proxy failed');hull=pred.ConvexHull
pred.ConvexHull=lambda *args,**kwargs:(_ for _ in()).throw(failure)
x=cube.copy();x[0,0]+=.00007;before=stats['raw_proxy_original_calls']
try:cached(x)
except RuntimeError as exc:assert exc is failure
else:raise AssertionError('Original fault swallowed')
assert stats['raw_proxy_original_calls']==before+1;pred.ConvexHull=hull
assert pred.proxy is original
# Oversized source arrays are evaluated afresh exactly once on every call;
# a small convex output cannot accidentally seed the raw-input cache.
big=np.tile(cube,(helper.MAX_RAW_INPUT_BYTES//cube.nbytes+1,1))
assert big.nbytes>helper.MAX_RAW_INPUT_BYTES
for _ in range(2):
 before=stats['raw_proxy_original_calls'];new_before=stats['raw_proxy_new_content'];cached(big)
 assert stats['raw_proxy_original_calls']==before+1 and stats['raw_proxy_new_content']==new_before
(P/'raw-defensive-qualification.json').write_text(json.dumps(dict(all_assertions_pass=True,exact_dtype_shape_strides_and_bytes_key=True,oversized_input_original_exactly_once_and_not_cached=True,maximum_cached_raw_input_bytes=helper.MAX_RAW_INPUT_BYTES,original_predictive_alias_untouched=True,changed_invalid_poison_collision_fallback_original_exactly_once=True,valid_and_nonfinite_output_poison_rejected=True,cached_input_mutation_rejected=True,original_error_identity_propagates=True,stats=stats,GPU_work=False,physics_steps=0),indent=2));print('PASS raw input/output/poison/collision/error fixtures')
