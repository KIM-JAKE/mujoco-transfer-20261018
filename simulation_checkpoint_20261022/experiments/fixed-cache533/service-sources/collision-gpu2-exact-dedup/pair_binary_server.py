from pathlib import Path
import sys,os,socket,time,json,hashlib,torch,numpy as np
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT))
from pair_kernel import Geometry,prove
from gpu_pair_exact_rows import deduplicate
from gpu_pair_binary_protocol import SCHEMA,geometry_sha256,encode,receive,send
SOCKET=ROOT/'pair-exact-dedup.sock';DEVICE='cuda:0';torch.set_num_threads(1)
if SOCKET.exists():
 try:
  old=socket.socket(socket.AF_UNIX);old.settimeout(.2);old.connect(str(SOCKET));old.close();raise RuntimeError('Live binary service already exists')
 except (ConnectionRefusedError,FileNotFoundError):SOCKET.unlink()
server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);server.bind(str(SOCKET));SOCKET.chmod(0o660);server.listen(8);cache={}
source_files=['pair_binary_server.py','pair_kernel.py','gpu_pair_binary_protocol.py','gpu_pair_exact_rows.py'];source_sha256=hashlib.sha256(b''.join(n.encode()+hashlib.sha256((ROOT/n).read_bytes()).digest()for n in source_files)).hexdigest()
(ROOT/'ready.json').write_text(json.dumps(dict(pid=os.getpid(),socket=str(SOCKET),source_sha256=source_sha256,original_kernel_sha256=hashlib.sha256((ROOT/'pair_kernel.py').read_bytes()).hexdigest(),protocol=SCHEMA))+'\n')
print(json.dumps(dict(event='GPU_PAIR_BINARY_READY',socket=str(SOCKET),device=DEVICE,protocol=SCHEMA,source_sha256=source_sha256)),flush=True)
while True:
 conn,_=server.accept();conn.settimeout(15);start=time.perf_counter();request={}
 try:
  request,nbytes=receive(conn);decoded=time.perf_counter();sha=request['geometry_sha256'];op=request['op']
  if request.get('protocol')!=SCHEMA:raise ValueError('Binary geometry protocol mismatch')
  if op=='register':
   if geometry_sha256(request['left'],request['right'])!=sha:raise ValueError('Original geometry SHA mismatch')
   if sha not in cache:
    if len(cache)>=8:cache.pop(next(iter(cache)))
    cache[sha]=(Geometry(request['left'],DEVICE),Geometry(request['right'],DEVICE))
   response=dict(registered=True)
  elif op=='prove':
   if sha not in cache:raise ValueError('Geometry not registered')
   if request.get('margin_m')!=.0005:raise ValueError('Original margin mismatch')
   validated=[]
   for side,geometry in zip(('left','right'),cache[sha]):
    bank,index=request[side+'_bank'],request[side+'_index']
    if not isinstance(bank,np.ndarray)or bank.dtype!=np.float64 or bank.ndim!=3 or bank.shape[1:]!=(4,4) or not 0<len(bank)<=2000000:raise ValueError('Invalid transform bank')
    if not isinstance(index,np.ndarray)or index.dtype!=np.int32 or index.ndim!=2 or index.shape[1]!=geometry.count or not 0<len(index)<=16384 or index.min()<0 or index.max()>=len(bank):raise ValueError('Invalid exact transform indices')
    validated.append((bank,index))
   # Validate ALL original bank/index rows above before any exact reduction.
   plan=deduplicate(validated[0][1],validated[1][1])
   tensors=[torch.as_tensor(bank,device=DEVICE,dtype=torch.float64)[torch.as_tensor(index,device=DEVICE,dtype=torch.int64)]for (bank,_),index in zip(validated,(plan.left,plan.right))]
   kernel_start=time.perf_counter();unique_values,stats=prove(*cache[sha],*tensors,device=DEVICE);kernel_elapsed=time.perf_counter()-kernel_start
   values=plan.expand(unique_values);stats.update(plan.diagnostics());stats.update(total_rows=plan.original_rows,kernel_unique_rows=len(plan.first),kernel_pair_counters_scope='Unique rows passed to unchanged original kernel')
   response=dict(all_pairs_clear=values,certificate='original_proxy_separation_only',proposal_only=False,device=DEVICE,kernel_s=kernel_elapsed,source_sha256=source_sha256,**stats)
  else:raise ValueError('Unknown binary pair operation')
  response.update(id=request['id'],geometry_sha256=sha,protocol=SCHEMA,request_bytes=nbytes,receive_decode_s=decoded-start,server_s=time.perf_counter()-start)
 except Exception as error:response=dict(id=request.get('id'),geometry_sha256=request.get('geometry_sha256'),protocol=SCHEMA,error=type(error).__name__+': '+str(error),fallback_required=True)
 try:send(conn,encode(response))
 except (BrokenPipeError,ConnectionResetError):pass
 finally:conn.close()
 print(json.dumps(dict(event='GPU_PAIR_BINARY_RPC',op=request.get('op'),rows=len(response.get('all_pairs_clear',[])),clear=sum(response.get('all_pairs_clear',[])),server_s=response.get('server_s'),receive_decode_s=response.get('receive_decode_s'),kernel_s=response.get('kernel_s'),error=response.get('error'))),flush=True)
