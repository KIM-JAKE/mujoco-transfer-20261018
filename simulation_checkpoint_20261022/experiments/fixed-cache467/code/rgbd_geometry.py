"""Image-only classical RGB-D circle/plane detector (NumPy, no simulator API).

Inputs: registered depth/RGB, K and camera calibration. No CAD mesh, object
identities, segmentation labels or simulator body poses are accepted. Rings
are rotationally symmetric: estimate center and axis; yaw is unobservable.
"""
import numpy as np

def axis_frame(n):
    n=np.array(n,float);n/=np.linalg.norm(n);x=np.array([1.,0,0]);x-=n*(x@n)
    if np.linalg.norm(x)<.1:x=np.array([0.,1.,0.]);x-=n*(x@n)
    x/=np.linalg.norm(x);return np.column_stack([x,np.cross(n,x),n])

def cloud(packet):
    if 'points_camera_m'in packet:
        cam=packet['points_camera_m'];T=packet['T_cell_camera'];return cam@T[:3,:3].T+T[:3,3]
    z=packet['depth_m'].astype(float);K=packet['K'];v,u=np.indices(z.shape)
    cam=np.stack([(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z],-1)
    T=packet['T_cell_camera'];return cam@T[:3,:3].T+T[:3,3]

def circles(xy,low,high,pixel_m,rng,iterations=1800):
    if len(xy)<20:return []
    best=[];remaining=np.ones(len(xy),bool);tol=max(pixel_m*.9,.00010)
    for repeat in range(8):
        ids=np.flatnonzero(remaining)
        if len(ids)<20:break
        win=None
        for _ in range(iterations):
            a,b,c=xy[rng.choice(ids,3,replace=False)];M=2*np.array([b-a,c-a])
            if abs(np.linalg.det(M))<1e-8:continue
            center=np.linalg.solve(M,np.array([b@b-a@a,c@c-a@a]));radius=np.linalg.norm(center-a)
            if not low<radius<high:continue
            residual=abs(np.linalg.norm(xy-center,axis=1)-radius);mask=remaining&(residual<tol);count=int(mask.sum())
            if win is None or count>win[0]:win=(count,center,radius,mask)
        if win is None or win[0]<18:break
        _,center,radius,mask=win
        for _ in range(5):
            pts=xy[mask];A=np.c_[2*pts,np.ones(len(pts))];b=(pts*pts).sum(1);coef=np.linalg.lstsq(A,b,rcond=None)[0];center=coef[:2];radius=np.sqrt(max(0,coef[2]+center@center));mask=remaining&(abs(np.linalg.norm(xy-center,axis=1)-radius)<tol)
        angle=np.arctan2(*(xy[mask]-center)[:,::-1].T);coverage=len(np.unique(np.floor((angle+np.pi)/(2*np.pi)*36)))
        best.append(dict(center=center,radius=radius,inliers=int(mask.sum()),coverage=coverage))
        remaining[mask]=False
    return best

def detect(packet,bounds=None,kind='ring',max_planes=14):
    P=cloud(packet);valid=np.isfinite(P).all(-1)
    if bounds is not None:valid&=np.all((P>=np.array(bounds[0]))&(P<=np.array(bounds[1])),axis=-1)
    # Normals are derived exclusively from neighboring depth pixels.
    dx=np.roll(P,-1,1)-np.roll(P,1,1);dy=np.roll(P,-1,0)-np.roll(P,1,0);N=np.cross(dx,dy);length=np.linalg.norm(N,axis=-1);N/=np.maximum(length[...,None],1e-15)
    good=valid&(length>1e-12)&(np.linalg.norm(dx,axis=-1)<.006)&(np.linalg.norm(dy,axis=-1)<.006)
    pixels=np.argwhere(good);rng=np.random.default_rng(441);remaining=valid.copy();detections=[];planes=[]
    pixel_m=float(np.nanmedian(packet['depth_m'])/packet['K'][0,0]);flat=P[valid];locations=np.argwhere(valid)
    if not len(pixels):return [],dict(planes=[],pixel_m=pixel_m)
    for repeat in range(max_planes):
        samples=pixels[remaining[tuple(pixels.T)]]
        if len(samples)<40:break
        pick=samples[rng.choice(len(samples),min(240,len(samples)),replace=False)];current=remaining[tuple(locations.T)];subset=flat[current];win=None
        # subsampled scoring only, then full-resolution refinement.
        scoring=subset[::max(1,len(subset)//12000)]
        for v,u in pick:
            normal=N[v,u];d=float(normal@P[v,u]);score=int((abs(scoring@normal-d)<.00007).sum())
            if win is None or score>win[0]:win=(score,normal,d)
        if win is None:break
        _,normal,d=win;mask=remaining&(abs(np.einsum('ijk,k->ij',P,normal)-d)<.00009)
        if mask.sum()<60:break
        pts=P[mask];origin=np.mean(pts,axis=0);_,_,V=np.linalg.svd(pts-origin,full_matrices=False);normal=V[-1];normal*=np.sign(normal@(packet['T_cell_camera'][:3,3]-origin))or 1
        d=normal@origin;mask=remaining&(abs(np.einsum('ijk,k->ij',P,normal)-d)<.00009);remaining[mask]=False
        interior=mask.copy()
        for ax in [0,1]:interior&=np.roll(mask,1,ax)&np.roll(mask,-1,ax)
        edge=mask&~interior
        basis=V[:2].T;xy=(P[edge]-origin)@basis
        if len(xy)<25:continue
        fits=circles(xy,.023 if kind=='ring'else .002,.041 if kind=='ring'else .0075,pixel_m,rng,900 if kind=='ring'else 400)
        planes.append(dict(point=origin.tolist(),normal=normal.tolist(),pixels=int(mask.sum()),circles=[dict(radius_mm=c['radius']*1000,coverage=c['coverage'],inliers=c['inliers'],center=(origin+basis@c['center']).tolist())for c in fits]))
        for fit in fits:
            rad=fit['radius'];face=origin+basis@fit['center']
            if kind=='ring':
                others=[c for c in fits if .004<rad-c['radius']<.008 and np.linalg.norm(c['center']-fit['center'])<.0015]
                if not others or fit['coverage']<15:continue
                nominal=min([.075,.062],key=lambda D:abs(2*rad-D));part='ring1'if nominal==.075 else'ring2'
                if abs(2*rad-nominal)>.002:continue
                half=.00775 if part=='ring1'else .007;center=face-normal*half;axis=normal.copy();axis*=1 if axis[2]>=0 else -1
                detections.append(dict(part=part,center_m=center.tolist(),axis=axis.tolist(),outer_diameter_m=2*rad,inner_diameter_m=2*max(others,key=lambda c:c['coverage'])['radius'],coverage_bins=fit['coverage'],support_pixels=fit['inliers'],face_center_m=face.tolist(),visible_face_normal=normal.tolist()))
            else:
                if fit['coverage']<20:continue
                # End faces identify axial sign: 10.5 mm tail at local -17.75,
                # or 5 mm tip at +17.75. Collar is not an end face.
                if abs(2*rad-.0105)<.001:axis=-normal;center=face+axis*.01775
                elif abs(2*rad-.005)<.001:axis=normal;center=face-axis*.01775
                else:continue
                detections.append(dict(part='pin',center_m=center.tolist(),axis=axis.tolist(),outer_diameter_m=2*rad,coverage_bins=fit['coverage'],support_pixels=fit['inliers'],face_center_m=face.tolist(),visible_face_normal=normal.tolist()))
    # Merge repeated face detections, favor circular angular support.
    detections.sort(key=lambda d:(d['coverage_bins'],d['support_pixels']),reverse=True);unique=[]
    for d in detections:
        if any(d['part']==b['part']and np.linalg.norm(np.array(d['center_m'])-b['center_m'])<.004 for b in unique):continue
        unique.append(d)
    return unique,dict(planes=planes,pixel_m=pixel_m,input='RGB-D pixels and calibrated K/extrinsics only',uses_semantic_ids=False,uses_simulator_transforms=False,uses_color=False)
