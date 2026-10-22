"""GPU conservative pair CLEAR receipts for one immutable exact state list.

False/any error requests the unchanged original CPU SAT/LP. No own fixed,
self, installed-part or PhysX query is bypassed by this client.
"""
import hashlib,json,socket,time,uuid
import numpy as np

# Geometry SHA registrations only; NEVER cache any posed CLEAR result.
_REGISTERED=set()


def prove(lc,rc,pairs,socketcfg):
    pairs=list(pairs);diag=socketcfg.get('diagnostics',{});began=time.perf_counter();empty=np.zeros(len(pairs),dtype=bool)
    if not pairs:return empty
    def geometry(checker):
        rows=list(checker.pair_boxes)+list(checker.payload_boxes)
        if not rows:raise ValueError('Missing original whole-arm native geometry')
        proxies=[]
        for _,_,box in rows:
            proxies.append({k:np.asarray(box[i],dtype=float).tolist()for i,k in enumerate(['center','axes','extents','vertices','normals'])})
        return rows,proxies
    def transforms(checker,rows,side):
        out=[]
        for ql,wl,qr,wr in pairs:
            q,width=(ql,wl)if side=='left'else(qr,wr)
            body=checker.arm.bodies(q,float(width))
            if checker.payload_path:body[checker.payload_path]=body[checker.arm.hand_path]@checker.payload_relative
            out.append([np.asarray(body[path],float).tolist()for path,_,_ in rows])
        return out
    def rpc(request):
        request=dict(request,id=uuid.uuid4().hex)
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)as conn:
            conn.settimeout(float(socketcfg.get('timeout_s',5.)));conn.connect(str(socketcfg['socket']));conn.sendall(json.dumps(request,separators=(',',':')).encode()+b'\n');data=bytearray()
            while not data.endswith(b'\n'):
                chunk=conn.recv(1048576)
                if not chunk:raise ValueError('Incomplete GPU pair response')
                data.extend(chunk)
                if len(data)>4194304:raise ValueError('Oversized GPU pair response')
        reply=json.loads(data)
        if reply.get('id')!=request['id']or reply.get('geometry_sha256')!=request['geometry_sha256']or reply.get('error'):raise ValueError('GPU pair response rejected '+str(reply.get('error')))
        return reply
    try:
        left_rows,left=geometry(lc);right_rows,right=geometry(rc);raw=dict(left=left,right=right);sha=hashlib.sha256(json.dumps(raw,sort_keys=True,separators=(',',':')).encode()).hexdigest();diag['geometry_sha256']=sha
        registration_key=(str(socketcfg['socket']),sha)
        if registration_key not in _REGISTERED:
            registered=rpc(dict(op='register',geometry_sha256=sha,**raw))
            if registered.get('registered')is not True:raise ValueError('Geometry registration not acknowledged')
            if len(_REGISTERED)>=128:_REGISTERED.clear()
            _REGISTERED.add(registration_key)
            diag['geometry_registration_sent']=True
        else:diag['geometry_registration_sent']=False
        response=rpc(dict(op='prove',geometry_sha256=sha,left_T=transforms(lc,left_rows,'left'),right_T=transforms(rc,right_rows,'right'),margin_m=.0005))
        if response.get('certificate')!='original_proxy_separation_only'or response.get('proposal_only')is not False or response.get('float64')is not True or response.get('outward_reserve_m')!=1e-9 or response.get('original_pair_margin_m')!=.001:raise ValueError('GPU certificate policy mismatch')
        mask=response.get('all_pairs_clear')
        if not isinstance(mask,list)or len(mask)!=len(pairs)or any(type(v)is not bool for v in mask):raise ValueError('Invalid exact-row CLEAR mask')
        diag.update(response);diag.update(client_wall_s=time.perf_counter()-began,rows=len(pairs),clear_rows=sum(mask),fallback_rows=len(mask)-sum(mask),uses_free_body_poses=False)
        return np.asarray(mask,dtype=bool)
    except Exception as error:
        # A restarted/evicted server may have lost registration. This scope
        # uses the original CPU fallback; the next scope registers it again.
        if 'registration_key'in locals():_REGISTERED.discard(registration_key)
        diag.update(error=type(error).__name__+': '+str(error),client_wall_s=time.perf_counter()-began,rows=len(pairs),clear_rows=0,fallback_rows=len(pairs));return empty
