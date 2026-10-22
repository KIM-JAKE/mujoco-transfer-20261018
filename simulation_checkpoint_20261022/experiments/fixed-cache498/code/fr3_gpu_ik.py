"""Optional GPU IK endpoint proposals; original Isaac path checks stay mandatory."""
from pathlib import Path
import hashlib,json,socket,time,uuid
import numpy as np

def model_sha256(arm):
    data={key:np.round(np.asarray(getattr(arm,key),float),12).tolist()for key in ['base','tip','frames','axes','bounds']}
    return hashlib.sha256(json.dumps(data,sort_keys=True,separators=(',',':')).encode()).hexdigest()

def propose_ik(c,arm,P,R,current_q,label):
    """Return independently FK/bounds-checked joint proposals or CPU fallback[]."""
    cfg=c.get('config',{})
    if not cfg.get('gpu_ik_proposals'):return []
    started=time.perf_counter();record=dict(event='GPU_IK_endpoint_proposals',label=str(label),fallback_to_CPU=False,accepted=0);result=[]
    try:
        target=np.eye(4);target[:3,:3]=np.asarray(R,float);target[:3,3]=np.asarray(P,float);current=np.asarray(current_q,float);bounds=np.asarray(arm.bounds,float)
        if current.shape!=(7,)or bounds.shape!=(7,2)or not np.isfinite(target).all()or not np.isfinite(current).all()or np.max(abs(target[:3,:3].T@target[:3,:3]-np.eye(3)))>1e-5 or np.linalg.det(target[:3,:3])<.99999:raise ValueError('Invalid current encoder/TCP target for GPU endpoint request')
        from fr3_gpu_reference import reference_gauge
        reference_hash,world_gauge,tip_gauge,gauge_used=reference_gauge(arm,cfg);request_target=world_gauge@target@np.linalg.inv(tip_gauge)
        record.update(client_model_sha256=model_sha256(arm),reference_gauge_used=bool(gauge_used),intrinsic_chain_and_original_arm_FK_required=True)
        request=dict(id=uuid.uuid4().hex,model_sha256=reference_hash,target_TCP_world=request_target.tolist(),current_q=current.tolist());default=Path(__file__).resolve().parents[3]/'.local/curobo-gpu2/ik.sock';sockpath=Path(cfg.get('gpu_ik_socket',str(default)));timeout=float(np.clip(cfg.get('gpu_ik_timeout_s',.5),.05,2.))
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)as conn:
            conn.settimeout(timeout);conn.connect(str(sockpath));conn.sendall(json.dumps(request).encode()+b'\n');data=b'';deadline=time.perf_counter()+timeout
            while b'\n'not in data:
                left=deadline-time.perf_counter()
                if left<=0:raise TimeoutError('GPU endpoint proposal response deadline')
                conn.settimeout(left);chunk=conn.recv(8192)
                if not chunk:raise ValueError('Incomplete GPU endpoint response')
                data+=chunk
                if len(data)>1048576:raise ValueError('Oversized GPU endpoint response')
        response=json.loads(data.split(b'\n',1)[0])
        if response.get('error'):raise ValueError(response['error'])
        if response.get('id')!=request['id']or response.get('model_sha256')!=request['model_sha256']or response.get('proposal_only')is not True or not np.allclose(np.asarray(response.get('target_TCP_world'),float),request_target,atol=1e-10,rtol=0):raise ValueError('GPU response model/target/request identity mismatch')
        for item in response.get('proposals',[]):
            q=np.asarray(item['q'],float)
            if q.shape!=(7,)or not np.isfinite(q).all()or np.any(q<bounds[:,0])or np.any(q>bounds[:,1]):continue
            H=arm.fk(q);position_error=float(np.linalg.norm(H[:3,3]-target[:3,3]));delta=H[:3,:3]@target[:3,:3].T
            if np.max(abs(delta.T@delta-np.eye(3)))>1e-5:continue
            U,_,V=np.linalg.svd(delta);proper=U@np.diag([1.,1.,np.linalg.det(U@V)])@V;angle_error=float(np.arccos(np.clip((np.trace(proper)-1)/2,-1,1)))
            if position_error>1e-5 or angle_error>1e-4 or any(np.linalg.norm(q-old)<.05 for old in result):continue
            result.append(q)
        result.sort(key=lambda q:np.linalg.norm(q-current));record.update(accepted=len(result),model_sha256=request['model_sha256'],server_s=response.get('solve_s'),fallback_to_CPU=not bool(result),joint_bounds_and_CPU_FK_checked=True,original_PhysX_pair_payload_and_retiming_required=True)
    except Exception as e:record.update(fallback_to_CPU=True,reason=type(e).__name__+': '+str(e))
    record['wall_s']=time.perf_counter()-started;c.setdefault('report',{}).setdefault('GPU_IK_queries',[]).append(record);print('GPU_IK_ENDPOINT',record,flush=True);return result
