"""Subpixel lateral CAD registration with the measured cap plane fixed.

Only supplied camera pixels, a previous image estimate, and known class CAD
samples enter this function. Fixing the observed axis/depth removes the thin
annulus degeneracy that lets unrestricted ICP drift to the opposite face.
"""
import numpy as np
from rgbd_geometry import cloud,axis_frame

def refine(packet,pose,estimator):
    part=pose['part'];p0=np.array(pose['center_m']);p=p0.copy();R=axis_frame(pose['axis']);basis=R[:,:2];P=cloud(packet);rgb=packet['rgb'];gray=rgb.mean(2);relative=P-p;axial=relative@R[:,2];radial=np.linalg.norm(relative-axial[:,:,None]*R[:,2],axis=2);radius=.0375 if part=='ring1' else .031;valid=np.isfinite(P).all(2)&(np.ptp(rgb.astype(float),axis=2)<45)&(gray>35)&(gray<180)&(abs(axial)<.011)&(radial<radius+.003)&(radial>radius-.010)
    if 'fixture_depth_m' in packet:
        fixture=packet['fixture_depth_m'];valid&=(~np.isfinite(fixture))|(packet['depth_m']<fixture-.0003)
    points=P[valid]
    if len(points)<150:return pose
    obs=points[np.linspace(0,len(points)-1,min(2400,len(points))).astype(int)];tree=estimator.trees[part];model=estimator.models[part];normals=estimator.normals[part]
    for iteration in range(18):
        dd,ix=tree.query((obs-p)@R);keep=dd<min(.003,max(.001,float(np.quantile(dd,.80))))
        if keep.sum()<100:return pose
        vectors=model[ix[keep]]@R.T;N=normals[ix[keep]]@R.T;res=np.sum(N*(obs[keep]-p-vectors),axis=1);J=N@basis;weight=np.sqrt(np.minimum(1.,.00025/np.maximum(abs(res),1e-12)));delta=np.linalg.lstsq(J*weight[:,None],res*weight,rcond=1e-5)[0];delta*=min(1.,.001/max(np.linalg.norm(delta),1e-12));p+=basis@delta
        if np.linalg.norm(p-p0)>.004:return pose
        if np.linalg.norm(delta)<1e-7:break
    dd,ix=tree.query((obs-p)@R);keep=dd<.002;N=normals[ix[keep]]@R.T;vectors=model[ix[keep]]@R.T;res=np.sum(N*(obs[keep]-p-vectors),axis=1);rms=float(np.sqrt(np.mean(res**2)))
    if rms>.00065 or keep.mean()<.60:return pose
    return dict(pose,center_m=p.tolist(),lateral_CAD_correction_m=(p-p0).tolist(),lateral_CAD_registration_rms_m=rms,lateral_CAD_iterations=iteration+1,observed_cap_axis_and_depth_preserved=True)
