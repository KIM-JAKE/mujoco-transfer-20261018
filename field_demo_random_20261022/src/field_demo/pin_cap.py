"""Opt-in measured PIN end-cap circle/plane from registered RGB-D pixels.

A prior IMAGE pose propagated by actual SDK FK only bounds search and signs the
axis. The fit must observe a class-radius arc; hidden XY never comes from prior.
No simulator imports, actor names/poses, segmentation IDs or camera relocation.
"""
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .calibration import rigid
from .pin_cap_geometry import fit_pin
from .pin_precision import BODY_TO_AXIS_M,AXIS_OFFSET
from .stabilized.rgbd_geometry import cloud


def axial_plane(P,mask,prior,max_candidates=80):
    dx=np.roll(P,-1,1)-np.roll(P,1,1);dy=np.roll(P,-1,0)-np.roll(P,1,0)
    N=np.cross(dx,dy);norm=np.linalg.norm(N,axis=2);N/=np.maximum(norm[:,:,None],1e-15)
    dot=np.einsum('ijk,k->ij',N,prior);N*=np.where(dot>=0,1.,-1.)[:,:,None]
    good=mask&(norm>1e-12)&(np.linalg.norm(dx,axis=2)<.004)&(np.linalg.norm(dy,axis=2)<.004)&(abs(dot)>.70)
    good[[0,-1],:]=False;good[:,[0,-1]]=False  # no wrapped image-neighbour evidence
    anchors=P[good];normals=N[good]
    if len(anchors)<30:raise RuntimeError('No observable axial surface in current pin RGB-D ROI')
    ix=np.linspace(0,len(anchors)-1,min(max_candidates,len(anchors))).astype(int);anchors=anchors[ix];normals=normals[ix]
    points=P[mask];sample=points[::max(1,len(points)//1800)]
    offsets=np.einsum('ij,ij->i',anchors,normals);distance=abs(sample@normals.T-offsets);best=int(np.argmax((distance<.00006).sum(0)))
    n=normals[best];d=offsets[best];face=points[abs(points@n-d)<.00006]
    if len(face)<50:raise RuntimeError('Pin axial plane lacks current depth support')
    center=face.mean(0);_,_,V=np.linalg.svd(face-center,full_matrices=False);n=V[-1];n*=1 if n@prior>=0 else -1
    return n,len(face)


def estimate(packet,prior_body):
    started=time.perf_counter();prior_body=rigid(prior_body).copy()
    # Remove only floating-point nonorthogonality of validated SDK/image
    # rotation, never translation or an image-fitted object position.
    prior_body[:3,:3]=Rotation.from_matrix(prior_body[:3,:3]).as_matrix()
    prior_axis_pose=prior_body@AXIS_OFFSET
    seed=prior_axis_pose[:3,3];axis=prior_axis_pose[:3,2]
    if set(packet)-{'rgb','depth_m','K','T_cell_camera','points_camera_m','points_camera_image_m'}:raise ValueError('Camera-only PIN packet required')
    rgb=np.asarray(packet['rgb']);depth=np.asarray(packet['depth_m'],float);K=np.asarray(packet['K'],float);T=rigid(packet['T_cell_camera'])
    if rgb.dtype!=np.uint8 or rgb.shape!=(*depth.shape,3)or depth.ndim!=2 or K.shape!=(3,3)or not np.isfinite(K).all()or min(K[0,0],K[1,1])<=0:raise ValueError('Registered RGB-D/calibrated K required')
    original_shape=depth.shape;camera=(seed-T[:3,3])@T[:3,:3];radius=.035
    if camera[2]<=radius:raise RuntimeError('Pin prior is too near/behind camera')
    uv=camera[:2]/camera[2]*[K[0,0],K[1,1]]+[K[0,2],K[1,2]];extent=np.array([K[0,0],K[1,1]])*radius/(camera[2]-radius)+5
    x0=max(0,int(np.floor(uv[0]-extent[0])));x1=min(depth.shape[1],int(np.ceil(uv[0]+extent[0]))+1);y0=max(0,int(np.floor(uv[1]-extent[1])));y1=min(depth.shape[0],int(np.ceil(uv[1]+extent[1]))+1)
    if x1<=x0 or y1<=y0:raise RuntimeError('Pin prior outside actual camera FOV')
    crop_K=K.copy();crop_K[0,2]-=x0;crop_K[1,2]-=y0
    cropped=dict(rgb=rgb[y0:y1,x0:x1],depth_m=depth[y0:y1,x0:x1],K=crop_K,T_cell_camera=T)
    organized=packet.get('points_camera_image_m',packet.get('points_camera_m'))
    if organized is not None:
        organized=np.asarray(organized,float)
        if organized.shape!=(*depth.shape,3):raise ValueError('Actual organized SDK XYZ must match pixels')
        valid=np.isfinite(organized).all(2)&np.isfinite(depth)&(depth>0)
        if np.any(abs(organized[:,:,2][valid]-depth[valid])>.001):raise ValueError('SDK XYZ/depth registration mismatch')
        cropped['points_camera_m']=organized[y0:y1,x0:x1]
    P=cloud(cropped);gray=cropped['rgb'].mean(2);rgb_float=cropped['rgb'].astype(float)
    mask=np.isfinite(P).all(2)&(np.ptp(rgb_float,axis=2)<45)&(gray>35)&(gray<180)&(np.linalg.norm(P-seed,axis=2)<radius)
    relative=P-seed;axial=np.einsum('ijk,k->ij',relative,axis);radial=np.linalg.norm(relative-axial[:,:,None]*axis,axis=2)
    mask&=(abs(axial)<.022)&(radial<.010)
    vv,uu=np.nonzero(mask)
    if len(vv)<150:raise RuntimeError('Current pin RGB-D has insufficient metal/depth pixels')
    yy0=max(0,int(vv.min())-2);yy1=min(P.shape[0],int(vv.max())+3);xx0=max(0,int(uu.min())-2);xx1=min(P.shape[1],int(uu.max())+3)
    n,support=axial_plane(P[yy0:yy1,xx0:xx1],mask[yy0:yy1,xx0:xx1],axis)
    negative_end=bool((T[:3,3]-seed)@axis<0)
    fitted=fit_pin(P[mask],seed,axis,negative_end,measured_axis=n,robust_circle=True)
    if fitted['lateral_from_previous_RGBD_grasp_and_encoders']or fitted['plane_rms_m']>.00006:raise RuntimeError('Pin cap plane/circle quality failed; no ICP/prior fallback')
    # Only axis swing is observed. Preserve prior image clocking; do not claim
    # cylindrical axial yaw is measured by a circular end.
    measured_axis=np.asarray(fitted['axis']);cross=np.cross(axis,measured_axis);length=np.linalg.norm(cross)
    R=Rotation.from_rotvec(cross/max(length,1e-12)*np.arctan2(length,axis@measured_axis)).as_matrix()@prior_body[:3,:3]
    physical_center=np.asarray(fitted['center_m']);body_center=physical_center-R@BODY_TO_AXIS_M
    body=np.eye(4);body[:3,:3]=R;body[:3,3]=body_center
    fitted.update(T_cell_part=body.tolist(),center_m=body_center.tolist(),measured_physical_axis_center_m=physical_center.tolist(),pose_origin='canonical CAD body; measured symmetry-axis midpoint converted by fixed class geometry',axis=measured_axis.tolist(),axial_plane_points=support,body_to_axis_offset_m=BODY_TO_AXIS_M.tolist(),orientation_scope='measured signed axis; axial yaw retained from previous image/holder calibration',processing_ms=(time.perf_counter()-started)*1000,robot_reachability_verified=False,quality_scope='Ideal-depth SIM thresholds; residuals are not physical75um/calibration accuracy guarantees',fallback_used=False)
    return fitted
