"""Exact transform-bank transport to unchanged conservative GPU pair kernel."""
import socket,time,uuid,struct
import numpy as np
from gpu_pair_binary_protocol import KEYS,SCHEMA,geometry_sha256,encode,receive,send
_REGISTERED=set()

def prove(lc,rc,pairs,socketcfg):
    pairs=list(pairs);diag=socketcfg.get('diagnostics',{});began=time.perf_counter();empty=np.zeros(len(pairs),dtype=bool)
    if not pairs:return empty
    def geometry(checker):
        rows=list(checker.pair_boxes)+list(checker.payload_boxes)
        if not rows:raise ValueError('Missing original whole-arm geometry')
        return rows,[{key:np.ascontiguousarray(box[i],dtype=np.float64) for i,key in enumerate(KEYS)}for _,_,box in rows]
    def packed(checker,rows,side):
        # Exact q dtype+bytes+width keys only inside this immutable preflight.
        paths=list(dict.fromkeys(path for path,_,_ in rows));indices=np.empty((len(pairs),len(rows)),np.int32);bank=[];by_matrix={};by_state={}
        for index,(ql,wl,qr,wr) in enumerate(pairs):
            q,width=(ql,wl)if side=='left'else(qr,wr);q=np.asarray(q);state=(q.dtype.str,q.shape,q.tobytes(),struct.pack('<d',float(width)))
            found=by_state.get(state)
            if found is None:
                bodies=checker.arm.bodies(q,float(width))
                if checker.payload_path:bodies[checker.payload_path]=bodies[checker.arm.hand_path]@checker.payload_relative
                body_indices={}
                for path in paths:
                    matrix=np.ascontiguousarray(bodies[path],dtype=np.float64)
                    if matrix.shape!=(4,4):raise ValueError('Bad native body transform')
                    key=matrix.tobytes();bi=by_matrix.get(key)
                    if bi is None:bi=len(bank);by_matrix[key]=bi;bank.append(matrix.copy())
                    body_indices[path]=bi
                found=np.asarray([body_indices[path]for path,_,_ in rows],np.int32);by_state[state]=found
            indices[index]=found
        return np.asarray(bank,dtype=np.float64),indices,len(by_state)
    rpc_stats=[]
    def rpc(request):
        request=dict(request,id=uuid.uuid4().hex,protocol=SCHEMA);t=time.perf_counter();data=encode(request);encoded=time.perf_counter();
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)as conn:
            conn.settimeout(float(socketcfg.get('timeout_s',5.)));conn.connect(str(socketcfg['socket']));send(conn,data);reply,reply_bytes=receive(conn,4194304)
        rpc_stats.append(dict(op=request['op'],request_bytes=len(data),reply_bytes=reply_bytes,encode_s=encoded-t,rpc_s=time.perf_counter()-encoded))
        if reply.get('id')!=request['id']or reply.get('geometry_sha256')!=request['geometry_sha256']or reply.get('protocol')!=SCHEMA or reply.get('error'):raise ValueError('Binary GPU pair response rejected '+str(reply.get('error')))
        return reply
    try:
        t=time.perf_counter();left_rows,left=geometry(lc);right_rows,right=geometry(rc);sha=geometry_sha256(left,right);diag.update(geometry_sha256=sha,geometry_prepare_s=time.perf_counter()-t)
        registration_key=(str(socketcfg['socket']),sha)
        if registration_key not in _REGISTERED:
            reply=rpc(dict(op='register',geometry_sha256=sha,left=left,right=right))
            if reply.get('registered')is not True:raise ValueError('Geometry registration missing')
            if len(_REGISTERED)>=128:_REGISTERED.clear()
            _REGISTERED.add(registration_key);diag['geometry_registration_sent']=True
        else:diag['geometry_registration_sent']=False
        t=time.perf_counter();lb,li,ls=packed(lc,left_rows,'left');rb,ri,rs=packed(rc,right_rows,'right');diag.update(transform_prepare_s=time.perf_counter()-t,unpacked_transform_bytes=len(pairs)*(len(left_rows)+len(right_rows))*16*8,packed_transform_bytes=lb.nbytes+li.nbytes+rb.nbytes+ri.nbytes,unique_left_states=ls,unique_right_states=rs,left_transform_bank=len(lb),right_transform_bank=len(rb))
        # One immutable local transform bank; exact row-index slices obey the
        # unchanged service's 16k limit. A failed chunk invalidates ALL bits.
        mask=[];chunk_profiles=[];server_s=kernel_s=0.
        for offset in range(0,len(pairs),16000):
            stop=min(offset+16000,len(pairs))
            response=rpc(dict(op='prove',geometry_sha256=sha,left_bank=lb,left_index=li[offset:stop],right_bank=rb,right_index=ri[offset:stop],margin_m=.0005))
            if response.get('certificate')!='original_proxy_separation_only'or response.get('proposal_only')is not False or response.get('float64')is not True or response.get('outward_reserve_m')!=1e-9 or response.get('original_pair_margin_m')!=.001:raise ValueError('GPU certificate policy mismatch')
            bits=response.get('all_pairs_clear')
            if not isinstance(bits,list)or len(bits)!=stop-offset or any(type(x)is not bool for x in bits):raise ValueError('Invalid exact-row CLEAR mask')
            mask.extend(bits);server_s+=float(response.get('server_s',0));kernel_s+=float(response.get('kernel_s',0))
            chunk_profiles.append(dict(first_row=offset,rows=stop-offset,clear_rows=sum(bits),server_s=response.get('server_s'),kernel_s=response.get('kernel_s')))
        diag.update(response);diag.update(client_wall_s=time.perf_counter()-began,rows=len(pairs),clear_rows=sum(mask),fallback_rows=len(mask)-sum(mask),uses_free_body_poses=False,transport='numpy_pickle5_exact_transform_bank',rpc_profile=rpc_stats,chunk_profiles=chunk_profiles,server_s=server_s,kernel_s=kernel_s)
        return np.asarray(mask,dtype=bool)

    except Exception as error:
        if 'registration_key'in locals():_REGISTERED.discard(registration_key)
        diag.update(error=type(error).__name__+': '+str(error),client_wall_s=time.perf_counter()-began,rows=len(pairs),clear_rows=0,fallback_rows=len(pairs),rpc_profile=rpc_stats);return empty
