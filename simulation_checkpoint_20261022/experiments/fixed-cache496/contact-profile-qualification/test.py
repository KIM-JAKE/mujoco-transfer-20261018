"""Exact staged wrapper transparency and low-overhead CPU-only qualification."""
import ast,json,time
from pathlib import Path
D=Path(__file__).parent;s=(D/'overrides/run_realtime_recipe.py').read_text();tree=ast.parse(s)
fn=next(n for n in ast.walk(tree)if isinstance(n,ast.FunctionDef)and n.name=='profiled_contacts');code=compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),str(D/'overrides/run_realtime_recipe.py'),'exec')
stats=dict(callback_calls=0,header_records=0,data_records=0,empty_header_calls=0,max_headers=0,max_data_records=0,inclusive_wall_s=0.,inclusive_thread_cpu_s=0.,max_wall_s=0.,original_exceptions=0,instrumentation_errors=0,unique_actor_side_cache_entries=0);calls=[]
def original(h,d):calls.append((h,d));return d
ns=dict(time=time,contact_callback_profile=stats,contact_side_cache={1:0,2:1},contacts=original);exec(code,ns);wrapped=ns['profiled_contacts'];headers=[object(),object()];data=[object()]*5;assert wrapped(headers,data)is data and calls[-1][0]is headers and calls[-1][1]is data
assert stats['callback_calls']==1 and stats['header_records']==2 and stats['data_records']==5 and stats['unique_actor_side_cache_entries']==2
class UnsupportedLength:
 def __len__(self):raise RuntimeError('unsupported diagnostic len')
u=UnsupportedLength();assert wrapped(u,data)is data and calls[-1][0]is u and stats['instrumentation_errors']==1
failure=RuntimeError('original safety error')
def raising(h,d):raise failure
ns['contacts']=raising
try:wrapped(headers,data)
except RuntimeError as error:assert error is failure
else:raise AssertionError('Safety error suppressed')
assert stats['original_exceptions']==1
# No-op callback cost isolates the added diagnostic overhead; includes counters.
ns['contacts']=lambda h,d:None;N=100000;t=time.perf_counter()
for _ in range(N):wrapped([],[])
elapsed=time.perf_counter()-t
result=dict(all_assertions_pass=True,original_inputs_return_and_error_identity_preserved=True,diagnostic_failure_still_calls_original=True,CPU_only=True,GPU_work=False,physics_steps=0,noop_wrapper_calls=N,noop_wrapper_total_s=elapsed,us_per_callback=elapsed/N*1e6,projected_100000_callback_overhead_s=elapsed,scope='Microbenchmark of instrumentation only; actual callback share remains unknown')
(D/'qualification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
