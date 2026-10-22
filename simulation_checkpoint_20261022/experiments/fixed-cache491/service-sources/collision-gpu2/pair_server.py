from pathlib import Path
import os,json,socket,time,hashlib,traceback,torch
from pair_kernel import Geometry,prove
ROOT=Path(__file__).resolve().parent;SOCKET=ROOT/'pair.sock';DEVICE='cuda:0';torch.set_num_threads(1)
if SOCKET.exists():
 try:
  s=socket.socket(socket.AF_UNIX);s.settimeout(.2);s.connect(str(SOCKET));s.close();raise RuntimeError('Live pair service already exists')
 except (ConnectionRefusedError,FileNotFoundError):SOCKET.unlink()
cache={};server=socket.socket(socket.AF_UNIX);server.bind(str(SOCKET));SOCKET.chmod(0o660);server.listen(8);print(json.dumps(dict(event='GPU_PAIR_READY',socket=str(SOCKET),device=DEVICE,name=torch.cuda.get_device_name(),float64=True)),flush=True)
while True:
 conn,_=server.accept();conn.settimeout(15);start=time.perf_counter();request={}
 try:
  data=bytearray()
  while not data.endswith(b'\n'):
   part=conn.recv(1048576)
   if not part:raise ValueError('Incomplete pair request')
   data.extend(part)
   if len(data)>268435456:raise ValueError('Pair request exceeds256MiB')
  request=json.loads(data);op=request['op'];sha=request['geometry_sha256']
  if op=='register':
   raw={k:request[k]for k in ['left','right']};actual=hashlib.sha256(json.dumps(raw,sort_keys=True,separators=(',',':')).encode()).hexdigest()
   if actual!=sha:raise ValueError('Original geometrySHA mismatch')
   if sha not in cache:
    if len(cache)>=8:cache.pop(next(iter(cache)))
    cache[sha]=(Geometry(raw['left'],DEVICE),Geometry(raw['right'],DEVICE))
   response=dict(id=request.get('id'),geometry_sha256=sha,registered=True)
  elif op=='prove':
   if sha not in cache:raise ValueError('Geometry not registered')
   if request.get('margin_m')!=.0005:raise ValueError('Original pair margin must remain0.5mm per shape')
   values,stats=prove(*cache[sha],request['left_T'],request['right_T'],device=DEVICE);response=dict(id=request.get('id'),geometry_sha256=sha,all_pairs_clear=values,certificate='original_proxy_separation_only',proposal_only=False,device=DEVICE,**stats)
  else:raise ValueError('Unknown pair operation')
  response['server_s']=time.perf_counter()-start
 except Exception as e:response=dict(id=request.get('id'),error=type(e).__name__+': '+str(e),fallback_required=True)
 try:conn.sendall(json.dumps(response).encode()+b'\n')
 except (BrokenPipeError,ConnectionResetError):pass
 finally:conn.close()
 print(json.dumps(dict(event='GPU_PAIR_RPC',op=request.get('op'),rows=len(response.get('all_pairs_clear',[])),clear=sum(response.get('all_pairs_clear',[])),server_s=response.get('server_s'),error=response.get('error'))),flush=True)
