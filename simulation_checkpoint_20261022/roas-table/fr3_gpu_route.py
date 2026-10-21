"""Optional cuRobo route proposals; every result requires original PhysX proof."""
from pathlib import Path
import json,socket,time,uuid
import numpy as np
from scipy.spatial.transform import Rotation
from fr3_gpu_ik import model_sha256

def _obstacle_boxes(checker):
    rows=checker.preview_other_boxes;source='current explicit other-arm encoder planning context'
    if rows is None and getattr(checker,'other_configuration',None) is not None:
        from predictive_pair_clearance import at_pose
        other,q,width=checker.other_configuration();rows=at_pose(other,q,width);source='current actual other-arm encoder callback'
    boxes=[]
    for _,_,proxy in rows or []:
        center,R,extents=proxy[:3];R=np.asarray(R).copy()
        if np.linalg.det(R)<0:R[:,2]*=-1
        quaternion=Rotation.from_matrix(R).as_quat()[[3,0,1,2]];boxes.append(dict(dims_m=(2*(np.asarray(extents)+.0005)).tolist(),pose_world_wxyz=np.r_[center,quaternion].tolist(),source=source))
    points=getattr(checker,'gpu_rgbd_stock_points',None)
    if points is not None:
        points=np.asarray(points,float);points=points[np.isfinite(points).all(1)];room=192-len(boxes);size=.02
        if room<1:raise ValueError('No GPU obstacle cache for current observed stock')
        while len(points):
            cells=np.unique(np.floor(points/size).astype(np.int64),axis=0)
            if len(cells)<=room:break
            size*=1.25
            if size>.15:raise ValueError('RGB-D stock envelopes exceed bounded GPU cache')
        for cell in cells if len(points)else[]:boxes.append(dict(dims_m=[size+.001]*3,pose_world_wxyz=np.r_[(cell+.5)*size,[1,0,0,0]].tolist(),source='occupied RGB-D point-cloud voxel; no object actor poses'))
    return boxes

def propose_route(checker,start,goal,opening,selected=None):
    """Return finite joint path proposal or None; never waive final collision checks."""
    if not getattr(checker,'gpu_route_enabled',False)or checker.side!='right':return None
    started=time.perf_counter();stats=getattr(checker,'gpu_route_stats',None)
    if stats is None:stats=checker.gpu_route_stats=dict(queries=0,proposals=0,CPU_fallbacks=0)
    stats['queries']+=1
    try:
        a,b=np.asarray(start,float),np.asarray(goal,float);bounds=np.asarray(checker.arm.bounds);width=float(opening)
        if a.shape!=(7,)or b.shape!=(7,)or not np.isfinite(np.r_[a,b,width]).all()or not 0<=width<=.08:raise ValueError('Invalid current joint target/native aperture')
        payload=None
        if checker.payload_path:
            points=np.concatenate([row[2][3]for row in checker.payload_boxes]);center=(points.min(0)+points.max(0))/2;radius=float(np.linalg.norm(points-center,axis=1).max())+.0005;T=np.asarray(checker.payload_relative).copy();T[2,3]-=.1034;payload=dict(center_TCP_m=(T[:3,:3]@center+T[:3,3]).tolist(),radius_m=radius,source='camera-estimated carried CAD envelope; not simulator actor pose')
        request=dict(id=uuid.uuid4().hex,model_sha256=model_sha256(checker.arm),start_q=a.tolist(),goal_q=b.tolist(),opening_m=width,payload=payload,obstacle_boxes=_obstacle_boxes(checker));default=Path(__file__).resolve().parents[3]/'.local/curobo-gpu2/route.sock';sockpath=Path(getattr(checker,'gpu_route_socket',default));timeout=float(np.clip(getattr(checker,'gpu_route_timeout_s',.4),.05,2.))
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)as conn:
            conn.settimeout(timeout);conn.connect(str(sockpath));conn.sendall(json.dumps(request).encode()+b'\n');data=b'';deadline=time.perf_counter()+timeout
            while b'\n'not in data:
                remaining=deadline-time.perf_counter()
                if remaining<=0:raise TimeoutError('GPU route response deadline')
                conn.settimeout(remaining);part=conn.recv(65536)
                if not part:raise ValueError('Incomplete GPU route response')
                data+=part
                if len(data)>4194304:raise ValueError('Oversized GPU route response')
        response=json.loads(data.split(b'\n',1)[0])
        if response.get('error'):raise ValueError(response['error'])
        if response.get('id')!=request['id']or response.get('model_sha256')!=request['model_sha256']or response.get('proposal_only')is not True or not np.allclose(response['start_q'],a,atol=1e-10,rtol=0)or not np.allclose(response['goal_q'],b,atol=1e-10,rtol=0)or abs(response['opening_m']-width)>1e-12:raise ValueError('GPU route request/model/endpoints mismatch')
        stats.update(last_server_s=response.get('solve_s'),last_obstacle_boxes=len(request['obstacle_boxes']),last_payload=bool(payload))
        if response.get('route')is None:raise ValueError('No coarse collision-aware GPU route; '+str(response.get('start_sphere_collisions'))+' / '+str(response.get('goal_sphere_collisions')))
        path=np.asarray(response['route'],float)
        if path.ndim!=2 or path.shape[1]!=7 or not 2<=len(path)<=2048 or not np.isfinite(path).all()or np.any(path<bounds[:,0])or np.any(path>bounds[:,1])or np.linalg.norm(path[0]-a)>1e-7 or np.linalg.norm(path[-1]-b)>1e-7:raise ValueError('Invalid finite bounded GPU route/endpoints')
        stats['proposals']+=1;stats['last_wall_s']=time.perf_counter()-started;print('GPU_ROUTE_PROPOSAL',dict(knots=len(path),wall_s=stats['last_wall_s'],server_s=stats['last_server_s'],payload=bool(payload),obstacle_boxes=stats['last_obstacle_boxes'],full_PhysX_and_FR3_retime_still_required=True),flush=True);return path
    except Exception as e:
        stats['CPU_fallbacks']+=1;stats.update(last_error=type(e).__name__+': '+str(e),last_wall_s=time.perf_counter()-started);print('GPU_ROUTE_CPU_FALLBACK',stats['last_error'],stats['last_wall_s'],flush=True);return None
