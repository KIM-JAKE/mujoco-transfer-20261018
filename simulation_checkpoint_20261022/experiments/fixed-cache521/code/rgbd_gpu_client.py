"""Optional public RGB-D GPU2 inference. Original CPU detection is fallback."""
import time,numpy as np
from multiprocessing.connection import Client
from rgbd_scattered_rings import detect as cpu_detect
_ALLOWED={'rgb','depth_m','K','T_cell_camera','fixture_depth_m','points_camera_m'}
def detect(packet,calibration,estimator,desired_part=None,seed=None,*,socket_path=None,expected_source_sha256=None):
    if not socket_path:return cpu_detect(packet,calibration,estimator,desired_part=desired_part,seed=seed)
    started=time.perf_counter();conn=None
    try:
        # Exclude every renderer-only snapshot/ID/pose by an explicit allowlist.
        public={k:v for k,v in packet.items()if k in _ALLOWED}
        conn=Client(str(socket_path),family='AF_UNIX');conn.send(dict(packet=public,calibration=calibration,desired_part=desired_part,seed=seed))
        if not conn.poll(2.):raise TimeoutError('GPU RGB-D timeout')
        value=conn.recv()
        if not value.get('ok'):raise RuntimeError(value.get('error','GPU RGB-D failure'))
        if expected_source_sha256 and value.get('source_sha256')!=expected_source_sha256:raise RuntimeError('GPU perception source fingerprint mismatch')
        result=value['result'];required={desired_part}if desired_part else{'ring1','ring2'}
        if not required<={p['part']for p in result.get('poses',[])}:raise RuntimeError('GPU proposal missed a required image class; original CPU fallback')
        result['gpu_metric_proposals']=dict(server_s=value['server_s'],rpc_wall_s=time.perf_counter()-started,device='assigned server GPU2',final_CPU_annular_gates_retained=True,renderer_metadata_sent=False,source_sha256=value.get('source_sha256'))
        return result,value['masks']
    except Exception as e:
        rpc_wait=time.perf_counter()-started
        result,masks=cpu_detect(packet,calibration,estimator,desired_part=desired_part,seed=seed)
        result['gpu_metric_fallback']=dict(reason=type(e).__name__,rpc_wait_s=rpc_wait)
        return result,masks
    finally:
        if conn is not None:conn.close()
