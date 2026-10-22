"""Low-latency RGB-D geometry tracking after SAM identifies the ring classes.

Only camera pixels/calibration, the known CAD family and encoder/previous-pose
seeds enter this tracker. No simulator API, body paths or body poses are used.
"""
import time,numpy as np
from rgbd_geometry import cloud
from rgbd_geometry import axis_frame
from rgbd_pin_axes import fit_pin

def axial_plane(P,mask,prior,max_candidates=80):
    dx=np.roll(P,-1,1)-np.roll(P,1,1);dy=np.roll(P,-1,0)-np.roll(P,1,0);N=np.cross(dx,dy);norm=np.linalg.norm(N,axis=2);N/=np.maximum(norm[:,:,None],1e-15)
    dot=np.einsum('ijk,k->ij',N,prior);N*=np.where(dot>=0,1.,-1.)[:,:,None]
    good=mask&(norm>1e-12)&(np.linalg.norm(dx,axis=2)<.004)&(np.linalg.norm(dy,axis=2)<.004)&(abs(dot)>.70)
    anchors=P[good];normals=N[good]
    if len(anchors)<30:raise RuntimeError('No observable axial surface in RGB-D tracking ROI')
    ix=np.linspace(0,len(anchors)-1,min(max_candidates,len(anchors))).astype(int);anchors=anchors[ix];normals=normals[ix];points=P[mask];sample=points[::max(1,len(points)//1800)]
    # The first prototype uses ideal simulated depth. Expose this tolerance
    # when commissioning real depth noise; do not pretend it is calibrated.
    offset=np.einsum('ij,ij->i',anchors,normals);distance=abs(sample@normals.T-offset);score=(distance<.00006).sum(0);best=int(np.argmax(score));n=normals[best];d=offset[best];face=points[abs(points@n-d)<.00006]
    if len(face)<50:raise RuntimeError('Axial plane support lost')
    center=face.mean(0);_,_,V=np.linalg.svd(face-center,full_matrices=False);n=V[-1];n*=1 if n@prior>=0 else -1
    return n,float(n@center),len(face)

class FastRGBDTracker:
    def __init__(self,estimator):self.estimator=estimator
    def estimate(self,packet,part,seed,axis,initial_pin=False,robust_pin_cap_circle=False,postrelease_cap_plane_roi=False):
        if not isinstance(postrelease_cap_plane_roi,bool):raise ValueError('Postrelease cap ROI flag must be boolean')
        if not postrelease_cap_plane_roi:return self._estimate_original(packet,part,seed,axis,initial_pin,robust_pin_cap_circle)
        if part!='pin' or initial_pin or not robust_pin_cap_circle:raise ValueError('Postrelease cap ROI requires held PIN and original robust-circle quality')
        if any(key in packet for key in ['body_poses','render_snapshot','actor_poses','free_body_poses']):raise ValueError('Private geometry is not a detector input')
        try:
            pose,mask=self._estimate_original(packet,part,seed,axis,initial_pin,robust_pin_cap_circle,pin_radial_roi_m=.007)
            pose=dict(pose,postrelease_known_class_plane_roi=dict(proposal_radius_m=.007,original_radius_m=.010,proposal_accepted=True,original_fallback=False,original_circle_quality_unchanged=True,source='Fresh RGB-D pixels and last qualified public world pose; CPU geometric fit'))
            return pose,mask
        except RuntimeError as error:
            pose,mask=self._estimate_original(packet,part,seed,axis,initial_pin,robust_pin_cap_circle)
            pose=dict(pose,postrelease_known_class_plane_roi=dict(proposal_radius_m=.007,original_radius_m=.010,proposal_accepted=False,original_fallback=True,proposal_error=str(error),original_circle_quality_unchanged=True,source='Unchanged original CPU tracker fallback on same fresh pixels'))
            return pose,mask

    def _estimate_original(self,packet,part,seed,axis,initial_pin=False,robust_pin_cap_circle=False,pin_radial_roi_m=.010):
        started=time.perf_counter();original_shape=packet['rgb'].shape[:2];seed=np.asarray(seed,float)
        # Crop before backprojection, retaining original pixel precision.
        T=packet['T_cell_camera'];camera=(seed-T[:3,3])@T[:3,:3];K=packet['K'].copy();radius=.055 if part!='pin'else .035
        if camera[2]<=radius:raise RuntimeError('Tracking seed too close to camera')
        uv=camera[:2]/camera[2]*[K[0,0],K[1,1]]+[K[0,2],K[1,2]];extent=np.array([K[0,0],K[1,1]])*radius/(camera[2]-radius)+5
        x0=max(0,int(np.floor(uv[0]-extent[0])));x1=min(original_shape[1],int(np.ceil(uv[0]+extent[0]))+1);y0=max(0,int(np.floor(uv[1]-extent[1])));y1=min(original_shape[0],int(np.ceil(uv[1]+extent[1]))+1)
        if x1<=x0 or y1<=y0:raise RuntimeError('Tracking seed outside camera frame')
        crop_bounds=(x0,x1,y0,y1);K[0,2]-=x0;K[1,2]-=y0
        packet=dict(packet,rgb=packet['rgb'][y0:y1,x0:x1],depth_m=packet['depth_m'][y0:y1,x0:x1],K=K)
        if 'points_camera_m'in packet:packet['points_camera_m']=packet['points_camera_m'][y0:y1,x0:x1]
        if 'fixture_depth_m'in packet:packet['fixture_depth_m']=packet['fixture_depth_m'][y0:y1,x0:x1]
        seed=np.array(seed,float);axis=np.array(axis,float);axis/=np.linalg.norm(axis);P=cloud(packet);rgb=packet['rgb'];gray=rgb.mean(2)
        mask=np.isfinite(P).all(2)&((rgb.max(2)-rgb.min(2))<45)&(gray>35)&(gray<180)&(np.linalg.norm(P-seed,axis=2)<(.055 if part!='pin'else .035))
        if 'fixture_depth_m'in packet:
            fixture=packet['fixture_depth_m'];mask&=(~np.isfinite(fixture))|(packet['depth_m']<fixture-.0003)
        if part=='pin':
            relative=P-seed;axial=np.einsum('ijk,k->ij',relative,axis);radial=np.linalg.norm(relative-axial[:,:,None]*axis,axis=2);mask&=(abs(axial)<.022)&(radial<pin_radial_roi_m)
        if part!='pin':
            relative=P-seed;axial=np.einsum('ijk,k->ij',relative,axis);radial=np.linalg.norm(relative-axial[:,:,None]*axis,axis=2)
            mask&=(abs(axial)<.028)&(radial<(.040 if part=='ring1'else .034))&(radial>(.030 if part=='ring1'else .023))
        v,u=np.nonzero(mask)
        if len(u)<150:raise RuntimeError('RGB-D tracking ROI has insufficient metal surface')
        # Compute depth normals only in the selected image rectangle.
        y0=max(0,int(v.min())-2);y1=min(P.shape[0],int(v.max())+3);x0=max(0,int(u.min())-2);x1=min(P.shape[1],int(u.max())+3)
        points=P[mask]
        if part!='pin'and max(original_shape)<300:
            # At low resolution an annular face may be less than three pixels
            # wide. Seed the fit from depth extent, then infer all observable
            # pose components from the CAD surface instead of noisy normals.
            n=axis.copy();side=1 if n@(packet['T_cell_camera'][:3,3]-seed)>0 else -1;level=float(np.quantile(points@n,.98 if side>0 else .02));support=0
        else:n,level,support=axial_plane(P[y0:y1,x0:x1],mask[y0:y1,x0:x1],axis)
        if part=='pin':
            visible_negative_end=bool((packet['T_cell_camera'][:3,3]-seed)@axis<0);result=fit_pin(points,seed,axis,visible_negative_end,measured_axis=n,robust_circle=robust_pin_cap_circle)
        else:
            R=axis_frame(n);basis=R[:,:2];local=points@basis;side=1 if n@(packet['T_cell_camera'][:3,3]-seed)>0 else -1;half=.00775 if part=='ring1'else .007
            lateral=seed@basis;center=basis@lateral+n*(level-side*half)
            obs=points[np.linspace(0,len(points)-1,min(1200,len(points))).astype(int)];tree=self.estimator.trees[part];model=self.estimator.models[part];normals=self.estimator.normals[part]
            iterations=0
            for iteration in range(24):
                dist,idx=tree.query((obs-center)@R);keep=dist<min(.008,max(.0015,float(np.quantile(dist,.85))))
                if keep.sum()<80:raise RuntimeError('RGB-D tracking CAD overlap lost')
                vectors=model[idx[keep]]@R.T;N=normals[idx[keep]]@R.T;res=np.sum(N*(obs[keep]-center-vectors),axis=1);J=np.c_[N,np.cross(vectors,N)];weight=np.sqrt(np.minimum(1.,.0005/np.maximum(abs(res),1e-12)));step=np.linalg.lstsq(J*weight[:,None],res*weight,rcond=1e-5)[0];step[:3]*=min(1.,.003/max(np.linalg.norm(step[:3]),1e-12));step[3:]*=min(1.,.025/max(np.linalg.norm(step[3:]),1e-12))
                from scipy.spatial.transform import Rotation
                center+=step[:3];R=Rotation.from_rotvec(step[3:]).as_matrix()@R;iterations=iteration+1
                if np.linalg.norm(step[:3])+.03*np.linalg.norm(step[3:])<.0000001:break
            n=R[:,2]
            dist,_=tree.query((obs-center)@R);keep=dist<.003;rms=float(np.sqrt(np.mean(dist[keep]**2)))if keep.any()else float('inf')
            if keep.mean()<.65 or rms>.0015 or np.linalg.norm(center-seed)>.04:raise RuntimeError('RGB-D geometry tracker quality gate failed '+str(dict(center=center.tolist(),axis=n.tolist(),rms=rms,fraction=float(keep.mean()),level=level)))
            result=dict(part=part,center_m=center.tolist(),axis=n.tolist(),rms_m=rms,inlier_fraction=float(keep.mean()),depth_points=len(points),axial_plane_points=support,translation_refinement_iterations=iterations,uses_depth_sensor=True,uses_simulator_body_poses=False,method='RGB-D ROI axial plane + local CAD surface tracking')
        result.update(processing_ms=(time.perf_counter()-started)*1000,neural_inference_this_frame=False)
        full_mask=np.zeros(original_shape,bool);x0,x1,y0,y1=crop_bounds;full_mask[y0:y1,x0:x1]=mask
        return result,full_mask
