"""Local inner/outer ring-wall pinch using image pose and fixed CAD only.

A local rim grasp deliberately leaves the ring centre outside the finger pads.
It therefore has its own contact calculation; diameter-grasp filters stay strict.
The native finger hull, finite-force grasp and physical lift checks are unchanged.
"""
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull

_RADII={'ring1':(.0375,.031684),'ring2':(.031,.025255)}
_OPENING=.018


def _contact(c,part,G,allow_center_correction=False):
    """Expected first contact for each actual native finger, in TCP frame."""
    G=np.asarray(G,float)
    if part not in _RADII or G.shape!=(4,4) or not np.isfinite(G).all():return None
    # The closing direction crosses one radial wall. These conditions avoid
    # confusing a partial diameter grasp or axial corner pinch with this mode.
    if abs(G[0,3])>.004 or not .020<abs(G[1,3])<.040 or abs(G[2,3])>.0075:return None
    if abs(G[2,2])<.8660:return None
    cache=c.setdefault('_local_rim_geometry_cache',{})
    if 'finger_planes' not in cache:
        vertices=np.load(Path(__file__).resolve().parent/'native-official-finger.npz')['vertices']
        cache['finger_planes']=ConvexHull(vertices).equations
    estimator=c['observed_grasp_clearance'].estimator
    points=estimator.models[part]@G[:3,:3].T+G[:3,3]
    normals=estimator.normals[part]@G[:3,:3].T
    planes=cache['finger_planes'];patches=[];widths=[]
    for sign in [-1,1]:
        # Finger local y = sign*TCP_y - opening/2; root is 45 mm
        # behind TCP. Solve all hull halfspaces for the opening interval
        # in which a CAD surface sample intersects this finger.
        local=points*np.array([sign,sign,1.]);local[:,2]+=.045
        base=local@planes[:,:3].T+planes[:,3]
        coefficient=-planes[:,1]/2
        lower=np.zeros(len(points));upper=np.full(len(points),_OPENING)
        valid=np.ones(len(points),bool)
        for col,a in enumerate(coefficient):
            if a>1e-12:upper=np.minimum(upper,-base[:,col]/a)
            elif a< -1e-12:lower=np.maximum(lower,-base[:,col]/a)
            else:valid&=base[:,col]<=0
        valid&=(lower<=upper)&(upper>=0)&(lower<_OPENING)
        if not valid.any():return None
        first=float(np.max(upper[valid]))
        # A finger must be clear while open and encounter the opposed wall
        # inside its distal contact pad as it closes.
        if not .001<first<_OPENING-.001:return None
        active=valid&(upper>first-.00040)
        patch=points[active];ns=normals[active]
        if len(patch)<6 or np.any(abs(patch[:,2])>.009):return None
        ns=ns[ns[:,1]*sign>.55]
        if len(ns)<5:return None
        normal=ns.mean(0);normal/=max(np.linalg.norm(normal),1e-12)
        # The inside wall can have two symmetric contacts at the pad edges.
        # Their resultant is evaluated jointly rather than discarding them
        # because the ring COM is intentionally far from the local pinch.
        if normal[1]*sign<1/np.sqrt(1+.4**2):return None
        patches.append(dict(center_m=np.mean(patch,axis=0).tolist(),normal=normal.tolist(),samples=len(patch),x_span_m=float(np.ptp(patch[:,0]))))
        widths.append(first)
    if abs(widths[0]-widths[1])>(.004 if allow_center_correction else .0025):return None
    width=float(np.mean(widths))
    if not .002<width<.012:return None
    return dict(contact_width_m=width,jaw_contact_openings_m=widths,patches=patches,radial_center_correction_m=(widths[1]-widths[0])/4)


def rim_aperture(c,part,G):
    value=_contact(c,part,G)
    return None if value is None else value['contact_width_m']


def candidate(c,part,pose,radial_world,axial_offset_m=0.,tilt_rad=0.):
    """Build one radial local-wall pinch from a measured centre and axis.

    Caller chooses radial direction around the visible annulus, then uses the
    regular image-cloud and live fixed-obstacle checks for arm access. No loose
    object's state, identity, scene seed or privileged transform is accessed.
    """
    if part not in _RADII or not 0<=axial_offset_m<=.006 or not np.isfinite(tilt_rad) or abs(tilt_rad)>np.radians(25)+1e-9:return None
    center=np.asarray(pose['center_m'],float);normal=np.asarray(pose['axis'],float)
    normal=normal/max(np.linalg.norm(normal),1e-12);signed_normal=normal.copy();normal=normal*(1 if normal[2]>=0 else -1)
    radial=np.asarray(radial_world,float);radial=radial-normal*(radial@normal)
    if np.linalg.norm(radial)<1e-8:return None
    radial/=np.linalg.norm(radial);z=-normal;x=np.cross(radial,z);R=np.column_stack([x,radial,z])
    a=tilt_rad;R=R@np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
    ro,ri=_RADII[part];H=np.eye(4);H[:3,:3]=R;H[:3,3]=center+radial*(ro+ri)/2+normal*axial_offset_m
    part_H=np.eye(4);part_H[:3,3]=center
    # Choose any in-plane rotation: class rings are axisymmetric for this
    # contact calculation. The measured signed normal is retained.
    u=np.cross(signed_normal,np.array([0.,0.,1.]) if abs(signed_normal[2])<.9 else np.array([1.,0.,0.]));u/=np.linalg.norm(u)
    part_H[:3,:3]=np.column_stack([u,np.cross(signed_normal,u),signed_normal])
    initial=_contact(c,part,np.linalg.inv(H)@part_H,allow_center_correction=True)
    if initial is None:return None
    correction=float(initial['radial_center_correction_m'])
    if abs(correction)>.001:return None
    H[:3,3]+=radial*correction
    contact=_contact(c,part,np.linalg.inv(H)@part_H)
    if contact is None:return None
    width=contact['contact_width_m']
    return dict(tcp_world=H,opening_m=_OPENING,contact_width_m=width,command_width_m=max(.001,width-.002),release_width_m=_OPENING,grasp_type='rim',radial_center_correction_m=correction,contact_model='Class CAD against unmodified official native finger hulls; separate local inner/outer-wall pinch',patches=contact['patches'])


def contact_state(c,part,G,actual_opening_m,contact_tolerance_m=.0008):
    """Check both local wall contacts from measured RGB-D G and encoders.

    Distances are conservative convex-hull halfspace clearances, not forces.
    This gate must be combined with stopped finger motion and a physical lift
    check. The 0.8 mm allowance includes the authored 0.5 mm contact offset.
    """
    G=np.asarray(G,float);opening=float(actual_opening_m)
    if part not in _RADII or G.shape!=(4,4) or not np.isfinite(G).all() or not np.isfinite(opening) or not 0<=opening<=.08:
        raise ValueError('Invalid image pose or native gripper opening')
    if not .0001<=contact_tolerance_m<=.0015:raise ValueError('Invalid contact tolerance')
    cache=c.setdefault('_local_rim_geometry_cache',{})
    if 'finger_planes' not in cache:
        vertices=np.load(Path(__file__).resolve().parent/'native-official-finger.npz')['vertices']
        cache['finger_planes']=ConvexHull(vertices).equations
    estimator=c['observed_grasp_clearance'].estimator
    points=estimator.models[part]@G[:3,:3].T+G[:3,3]
    normals=estimator.normals[part]@G[:3,:3].T
    planes=cache['finger_planes'];jaws=[]
    for sign in [-1,1]:
        local=points*np.array([sign,sign,1.]);local[:,1]-=opening/2;local[:,2]+=.045
        signed=np.max(local@planes[:,:3].T+planes[:,3],axis=1)
        # The reported shape distance uses all model samples. Contact credit
        # additionally requires the selected distal wall patch, not the cap,
        # an edge of the finger base, or the opposite side of the ring.
        close=signed<=contact_tolerance_m
        distal=close&(abs(points[:,2])<.009)&(abs(points[:,0])<.011)
        correct=distal&(normals[:,1]*sign>.55)
        count=int(distal.sum());fraction=float(correct.sum()/count) if count else 0.
        jaws.append(dict(sign=sign,minimum_hull_clearance_m=float(signed.min()),near_hull_samples=int(close.sum()),distal_contact_samples=count,opposed_wall_samples=int(correct.sum()),opposed_wall_normal_fraction=fraction,contact_compatible=bool(correct.sum()>=5 and fraction>=.80)))
    geometry=_contact(c,part,G)
    return dict(opposed_contact_compatible=bool(geometry is not None and all(j['contact_compatible'] for j in jaws)),actual_opening_m=opening,contact_tolerance_m=float(contact_tolerance_m),expected_aperture_m=None if geometry is None else geometry['contact_width_m'],jaws=jaws,source='RGB-D measured ring-to-TCP transform + actual finger encoder opening + fixed CAD',uses_free_body_poses=False,force_contact_measured=False)
