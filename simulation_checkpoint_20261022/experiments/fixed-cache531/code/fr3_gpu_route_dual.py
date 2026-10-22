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
    for body,source,shape in rows or []:
        proxies=[shape]
        if 'finger' in str(body).lower() and len(shape)>3:
            # Tapered fingers need a tighter proposal than one PCA box.
            # Slabs enclose the clipped convex hull; original pair checks final.
            from scipy.spatial import ConvexHull
            from predictive_pair_clearance import proxy as make_proxy
            vertices=np.asarray(shape[3]);axis=np.asarray(shape[1])[:,int(np.argmax(shape[2]))];values=vertices@axis;hull=ConvexHull(vertices);edges=set()
            for face in hull.simplices:
                for i,j in [(face[0],face[1]),(face[1],face[2]),(face[2],face[0])]:edges.add(tuple(sorted((int(i),int(j)))))
            cuts=np.linspace(values.min(),values.max(),5);proxies=[]
            for low,high in zip(cuts[:-1],cuts[1:]):
                clip=[v for v,t in zip(vertices,values)if low-1e-10<=t<=high+1e-10]
                for i,j in edges:
                    for cut in [low,high]:
                        if (values[i]-cut)*(values[j]-cut)<0:clip.append(vertices[i]+(vertices[j]-vertices[i])*(cut-values[i])/(values[j]-values[i]))
                if len(clip)>=4:proxies.append(make_proxy(np.asarray(clip)))
        for proxy in proxies:
            center,R,extents=proxy[:3];R=np.asarray(R).copy()
            if np.linalg.det(R)<0:R[:,2]*=-1
            quaternion=Rotation.from_matrix(R).as_quat()[[3,0,1,2]];boxes.append(dict(dims_m=(2*(np.asarray(extents)+.0005)).tolist(),pose_world_wxyz=np.r_[center,quaternion].tolist(),source='convex-clipped finger slab'if len(proxies)>1 else source))
    completed=getattr(checker,'placed_part_clearance',None)
    if completed is not None:
        for label,part in completed.parts.items():
            vertices=np.concatenate([shape[3]for shape in part['placed']]);low=vertices.min(0)-part['uncertainty_m'];high=vertices.max(0)+part['uncertainty_m']
            boxes.append(dict(dims_m=(high-low).tolist(),pose_world_wxyz=np.r_[(low+high)/2,[1,0,0,0]].tolist(),source='RGB-D completed '+label+'; conservative whole-class envelope, exact final CAD checks retained'))
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
    if not getattr(checker,'gpu_route_enabled',False)or (checker.side!='right'and not getattr(checker,'gpu_dual_arm_reference',False)):return None
    started=time.perf_counter();stats=getattr(checker,'gpu_route_stats',None)
    if stats is None:stats=checker.gpu_route_stats=dict(queries=0,proposals=0,CPU_fallbacks=0)
    stats['queries']+=1
    try:
        a,b=np.asarray(start,float),np.asarray(goal,float);bounds=np.asarray(checker.arm.bounds);width=float(opening)
        if a.shape!=(7,)or b.shape!=(7,)or not np.isfinite(np.r_[a,b,width]).all()or not 0<=width<=.08:raise ValueError('Invalid current joint target/native aperture')
        payload=None
        if checker.payload_path:
            points=np.concatenate([row[2][3]for row in checker.payload_boxes]);center=(points.min(0)+points.max(0))/2;radius=float(np.linalg.norm(points-center,axis=1).max())+.0005;T=np.asarray(checker.payload_relative).copy();T[2,3]-=.1034;payload=dict(center_TCP_m=(T[:3,:3]@center+T[:3,3]).tolist(),radius_m=radius,source='camera-estimated carried CAD envelope; not simulator actor pose')
        if payload is not None:
            # Class-CAD partition envelopes preserve annular holes; they are
            # approximate proposals only, with the original full checker final.
            groups=[]
            if 'ring' in str(checker.payload_path):
                center_xy=(points[:,:2].min(0)+points[:,:2].max(0))/2;angles=np.mod(np.arctan2(points[:,1]-center_xy[1],points[:,0]-center_xy[0]),2*np.pi);sectors=np.minimum((angles/(2*np.pi)*24).astype(int),23);zmid=(points[:,2].min()+points[:,2].max())/2
                for sector in range(24):
                    for upper in [False,True]:
                        mask=(sectors==sector)&((points[:,2]>=zmid)==upper)
                        if np.any(mask):groups.append(points[mask])
            else:groups=[np.asarray(row[2][3])for row in checker.payload_boxes]
            if len(groups)>64:raise ValueError('Class payload partition exceeds GPU proposal cache')
            payload['spheres_TCP_m']=[]
            for group in groups:
                center=(group.min(0)+group.max(0))/2;radius=float(np.linalg.norm(group-center,axis=1).max())+.0005
                payload['spheres_TCP_m'].append((T[:3,:3]@center+T[:3,3]).tolist()+[radius])
            if getattr(checker,'gpu_payload_witness_proposal',False):
                # Deliberately optimistic class-surface witnesses. These are
                # optimizer hints, never collision certificates; exact original
                # CAD/pair/self/fixed curve validation remains mandatory.
                chosen=[int(np.argmax(points[:,2]))];distance=np.full(len(points),np.inf)
                for _ in range(min(64,len(points))):
                    distance=np.minimum(distance,np.sum((points-points[chosen[-1]])**2,axis=1));index=int(np.argmax(distance))
                    if distance[index]<1e-12:break
                    chosen.append(index)
                payload['spheres_TCP_m']=[(T[:3,:3]@point+T[:3,3]).tolist()+[.0002]for point in points[chosen[:64]]]
                payload['source']='optimistic class-CAD surface witnesses; original full geometry checks mandatory'
        from fr3_gpu_reference import reference_gauge
        config=dict(gpu_dual_arm_reference=bool(getattr(checker,'gpu_dual_arm_reference',False)))
        if getattr(checker,'gpu_reference_model_json',None):config['gpu_reference_model_json']=checker.gpu_reference_model_json
        reference_hash,world_gauge,tip_gauge,gauge_used=reference_gauge(checker.arm,config)
        boxes=_obstacle_boxes(checker)
        if gauge_used:
            for box in boxes:
                pose=np.asarray(box['pose_world_wxyz'],float);T=np.eye(4);T[:3,:3]=Rotation.from_quat(pose[[4,5,6,3]]).as_matrix();T[:3,3]=pose[:3];T=world_gauge@T;box['pose_world_wxyz']=np.r_[T[:3,3],Rotation.from_matrix(T[:3,:3]).as_quat()[[3,0,1,2]]].tolist()
        request=dict(id=uuid.uuid4().hex,model_sha256=reference_hash,client_model_sha256=model_sha256(checker.arm),side=checker.side,reference_gauge_used=bool(gauge_used),world_to_reference_world=world_gauge.tolist(),native_TCP_to_reference_TCP=tip_gauge.tolist(),start_q=a.tolist(),goal_q=b.tolist(),opening_m=width,payload=payload,obstacle_boxes=boxes);default=Path(__file__).resolve().parents[3]/'.local/curobo-gpu2'/('route-dual.sock'if checker.side=='left'else'route.sock');sockpath=Path(getattr(checker,'gpu_left_route_socket',default)if checker.side=='left'else getattr(checker,'gpu_right_ring_route_socket',getattr(checker,'gpu_route_socket',default)));timeout=float(np.clip(getattr(checker,'gpu_route_timeout_s',.4),.05,2.))
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
        if gauge_used and (response.get('client_model_sha256')!=request['client_model_sha256']or not np.allclose(response.get('world_to_reference_world'),world_gauge,atol=1e-10,rtol=0)or not np.allclose(response.get('native_TCP_to_reference_TCP'),tip_gauge,atol=1e-10,rtol=0)):raise ValueError('GPU reference gauge response mismatch; original arm CPU fallback')
        stats.update(last_server_s=response.get('solve_s'),last_obstacle_boxes=len(request['obstacle_boxes']),last_payload=bool(payload),reference_gauge_used=bool(gauge_used),original_arm_model_sha256=request['client_model_sha256'])
        if response.get('route')is None:raise ValueError('No coarse collision-aware GPU route; '+str(response.get('start_sphere_collisions'))+' / '+str(response.get('goal_sphere_collisions')))
        path=np.asarray(response['route'],float)
        if path.ndim!=2 or path.shape[1]!=7 or not 2<=len(path)<=2048 or not np.isfinite(path).all():raise ValueError('Invalid finite GPU route shape')
        # Network echo is exact float64. Raw solver rows are float32; permit
        # only their representational endpoint error, then restore exact joints
        # before bounds and ALL original PhysX/issued-curve checks.
        tolerance_a=2*np.finfo(np.float32).eps*np.maximum(1.,abs(a));tolerance_b=2*np.finfo(np.float32).eps*np.maximum(1.,abs(b))
        if np.any(abs(path[0]-a)>tolerance_a)or np.any(abs(path[-1]-b)>tolerance_b):raise ValueError('GPU route endpoint exceeds float32 representation tolerance')
        path=path.copy();path[0]=a;path[-1]=b
        if np.any(path<bounds[:,0])or np.any(path>bounds[:,1]):raise ValueError('GPU joint path outside original bounds')
        stats['proposals']+=1;stats['last_wall_s']=time.perf_counter()-started;print('GPU_ROUTE_PROPOSAL',dict(knots=len(path),wall_s=stats['last_wall_s'],server_s=stats['last_server_s'],payload=bool(payload),obstacle_boxes=stats['last_obstacle_boxes'],full_PhysX_and_FR3_retime_still_required=True),flush=True);return path
    except Exception as e:
        stats['CPU_fallbacks']+=1;stats.update(last_error=type(e).__name__+': '+str(e),last_wall_s=time.perf_counter()-started);print('GPU_ROUTE_CPU_FALLBACK',stats['last_error'],stats['last_wall_s'],flush=True);return None
