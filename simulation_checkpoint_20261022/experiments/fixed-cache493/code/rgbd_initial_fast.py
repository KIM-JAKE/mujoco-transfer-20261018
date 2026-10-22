"""Fresh RGB-D instance detection for accessible rings; no neural/image cache.

Calibrated bin bounds and floor are fixture information. Component masks,
metric widths and object poses come from the supplied RGB-D pixels.
"""
import numpy as np,time
from scipy.ndimage import label
from rgbd_geometry import cloud,axis_frame
from rgbd_annular_cap_gate import annular_cap_gate
from scipy.spatial.transform import Rotation

def detect(packet,calibration,estimator,desired_part=None,seed=None):
    started=time.perf_counter();P=cloud(packet);rgb=packet['rgb'];n=np.array(calibration['basket_to_world_R'])[:,2];floor=np.array(calibration['floor_origin_world_m']);R=axis_frame(n);h=(P-floor)@n;gray=rgb.mean(2);bounds=np.array(calibration['outer_bounds_world_m']);valid=np.isfinite(P).all(2)&((rgb.max(2)-rgb.min(2))<45)&(gray>35)&(gray<180)&(h>.0105)&(h<.019)&np.all((P>=bounds[0])&(P<=bounds[1]),axis=2);labels,count=label(valid);candidates=[]
    for k in range(1,count+1):
        mask=labels==k;pts=P[mask]
        if len(pts)<150:continue
        local=(pts-floor)@R;lo,hi=np.percentile(local,[.5,99.5],axis=0);D=float(max((hi-lo)[:2]));minor=float(min((hi-lo)[:2]))
        if not .050<D<.088 or minor<.025:continue
        part='ring1'if abs(D-.075)<abs(D-.062)else'ring2';half=.00775 if part=='ring1'else .007;center=floor+R@np.r_[(lo[:2]+hi[:2])/2,half]
        candidates.append((center[0],part,center,D,pts,mask))
    selected=[];cap_veto_rejections=0
    if desired_part is not None:candidates=[c for c in candidates if c[1]==desired_part and np.linalg.norm(c[2]-seed)<.085]
    for _,part,center,D,pts,mask in sorted(candidates,key=lambda row:np.linalg.norm(row[2]-seed)if seed is not None else row[0]):
        if any(p['part']==part for p,m in selected):continue
        obs=pts[np.linspace(0,len(pts)-1,min(800,len(pts))).astype(int)];r=R.copy();p=center.copy();tree=estimator.trees[part]
        for _ in range(14):
            dist,idx=tree.query((obs-p)@r);keep=dist<min(.006,max(.0015,float(np.quantile(dist,.85))))
            if keep.sum()<100:break
            vectors=estimator.models[part][idx[keep]]@r.T;normals=estimator.normals[part][idx[keep]]@r.T;res=np.sum(normals*(obs[keep]-p-vectors),axis=1);J=np.c_[normals,np.cross(vectors,normals)];weight=np.sqrt(np.minimum(1.,.0005/np.maximum(abs(res),1e-12)));step=np.linalg.lstsq(J*weight[:,None],res*weight,rcond=1e-5)[0];step[:3]*=min(1.,.003/max(np.linalg.norm(step[:3]),1e-12));step[3:]*=min(1.,.025/max(np.linalg.norm(step[3:]),1e-12));p+=step[:3];r=Rotation.from_rotvec(step[3:]).as_matrix()@r
            if np.linalg.norm(step[:3])+.03*np.linalg.norm(step[3:])<1e-7:break
        distance,_=tree.query((obs-p)@r);keep=distance<.003;rms=float(np.sqrt(np.mean(distance[keep]**2)))if keep.any()else 1.
        if keep.mean()<.8 or rms>.0012 or np.linalg.norm(p-center)>.015:continue
        pose=dict(part=part,center_m=p.tolist(),axis=r[:,2].tolist(),rms_m=rms,inlier_fraction=float(keep.mean()),depth_points=len(pts),measured_classification_diameter_m=D,size_classification='Fresh RGB-D component metric width; color unused for size class',uses_depth_sensor=True,uses_simulator_body_poses=False)
        accepted,proof=annular_cap_gate(packet,pose)
        if not accepted:cap_veto_rejections+=1;continue
        pose['annular_cap_free_space_gate']=proof
        selected.append((pose,mask))
        if len(selected)==(1 if desired_part else 2):break
    if {p['part']for p,m in selected}!=({desired_part}if desired_part else {'ring1','ring2'}):raise RuntimeError('Fresh RGB-D detection did not find accessible rings of both sizes')
    return dict(poses=[p for p,m in selected],segmentation=dict(backend='Fresh RGB-D floor-height connected components + metric CAD fit',initial_cap_free_space_veto=True,cap_veto_rejected_candidates=cap_veto_rejections,SAM_used=False,cached_image_masks_used=False,processing_ms=(time.perf_counter()-started)*1000),depth_sensor_input=True,simulator_body_pose_input=False),[m for p,m in selected]
