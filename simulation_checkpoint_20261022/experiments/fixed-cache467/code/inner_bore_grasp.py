"""Centered native-hand bore expansion from RGB-D pose and class CAD.

The unmodified fingers enter collapsed, then expand against opposite bore
walls with finite actuator effort. This is a distinct mode from local rim
pinching and from an outside-diameter grasp. All object transforms supplied
here come from camera measurements, never free-body scene state.
"""
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
from rgbd_geometry import axis_frame


def _reserve(c):
    # Geometry threshold only. Physical materials are never modified here.
    value=float(c.get('config',{}).get('inner_bore_friction_reserve_mu',.4))
    if not np.isfinite(value) or not .2<=value<=.6:
        raise ValueError('Inner-bore geometric friction reserve must be 0.2--0.6')
    return value


def _center_limit(c):
    value=float(c.get('config',{}).get('inner_bore_center_correction_limit_m',.001))
    if not np.isfinite(value) or not .001<=value<=.0025:
        raise ValueError('Inner-bore centering limit must be 1--2.5 mm')
    return value


def _model(c,part,G):
    G=np.asarray(G,float)
    if part not in ['ring1','ring2'] or G.shape!=(4,4) or not np.isfinite(G).all():return None
    if np.linalg.norm(G[:2,3])>.003 or abs(G[2,3])>.008 or abs(G[2,2])<.8660:return None
    cache=c.setdefault('_inner_bore_geometry_cache',{})
    if 'planes' not in cache:
        vertices=np.load(Path(__file__).resolve().parent/'native-official-finger.npz')['vertices']
        cache['planes']=ConvexHull(vertices).equations
    estimator=c['observed_grasp_clearance'].estimator
    return (estimator.models[part]@G[:3,:3].T+G[:3,3],estimator.normals[part]@G[:3,:3].T,cache['planes'])


def _contact(c,part,G,allow_center_correction=False):
    model=_model(c,part,G)
    if model is None:return None
    points,normals,planes=model;widths=[];patches=[];mu=_reserve(c)
    for sign in [-1,1]:
        local=points*np.array([sign,sign,1.]);local[:,2]+=.045
        base=local@planes[:,:3].T+planes[:,3];coefficient=-planes[:,1]/2
        lower=np.full(len(points),.001);upper=np.full(len(points),.08);valid=np.ones(len(points),bool)
        for col,a in enumerate(coefficient):
            if a>1e-12:upper=np.minimum(upper,-base[:,col]/a)
            elif a< -1e-12:lower=np.maximum(lower,-base[:,col]/a)
            else:valid&=base[:,col]<=0
        valid&=(lower<=upper)&(upper>=.001)&(lower<=.08)
        if not valid.any():return None
        first=float(np.min(lower[valid]))
        if not .003<first<.075:return None
        active=valid&(lower<first+.0008);patch=points[active];ns=normals[active]
        if len(patch)<6 or np.any(abs(patch[:,2])>.009):return None
        inward=ns[:,1]*sign<-.55
        if inward.sum()<5 or inward.mean()<.60:return None
        # Include every active normal: mixed cap/cylinder edge samples may
        # not be silently discarded to manufacture a favorable resultant.
        normal=ns.mean(0);normal/=max(np.linalg.norm(normal),1e-12)
        if normal[1]*sign> -1/np.sqrt(1+mu**2):return None
        widths.append(first);patches.append(dict(sign=sign,center_m=patch.mean(0).tolist(),normal=normal.tolist(),samples=len(patch),inward_wall_normal_fraction=float(inward.mean())))
    if abs(widths[0]-widths[1])>(4*_center_limit(c) if allow_center_correction else .002):return None
    return dict(contact_width_m=float(np.mean(widths)),jaw_contact_openings_m=widths,center_correction_m=(widths[1]-widths[0])/4,patches=patches)


def bore_aperture(c,part,G):
    value=_contact(c,part,G)
    return None if value is None else value['contact_width_m']


def candidate(c,part,pose,closing_world,axial_offset_m=0.,tilt_rad=0.,roll_rad=0.):
    if not 0<=axial_offset_m<=.006 or not np.isfinite(tilt_rad) or abs(tilt_rad)>np.radians(25)+1e-9 or not np.isfinite(roll_rad) or abs(roll_rad)>np.radians(15)+1e-9:return None
    center=np.asarray(pose['center_m'],float);normal=np.asarray(pose['axis'],float)
    normal/=max(np.linalg.norm(normal),1e-12);up=normal*(1 if normal[2]>=0 else -1)
    y=np.asarray(closing_world,float);y=y-up*(y@up)
    if np.linalg.norm(y)<1e-8:return None
    y/=np.linalg.norm(y);z=-up;R=np.column_stack([np.cross(y,z),y,z]);b=roll_rad;R=R@np.array([[1,0,0],[0,np.cos(b),-np.sin(b)],[0,np.sin(b),np.cos(b)]])
    a=tilt_rad;R=R@np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
    H=np.eye(4);H[:3,:3]=R;H[:3,3]=center+up*axial_offset_m
    P=np.eye(4);P[:3,:3]=axis_frame(normal);P[:3,3]=center
    first=_contact(c,part,np.linalg.inv(H)@P,allow_center_correction=True)
    if first is None or abs(first['center_correction_m'])>_center_limit(c):return None
    H[:3,3]+=R[:,1]*first['center_correction_m']
    contact=_contact(c,part,np.linalg.inv(H)@P)
    if contact is None:return None
    width=contact['contact_width_m']
    return dict(tcp_world=H,opening_m=.001,contact_width_m=width,command_width_m=min(.08,width+.003),release_width_m=.001,grasp_type='inner',tilt_rad=float(tilt_rad),roll_rad=float(roll_rad),geometric_friction_reserve_mu=_reserve(c),center_correction_m=first['center_correction_m'],patches=contact['patches'],contact_model='Centered bore expansion against unmodified official native finger hulls')


def contact_state(c,part,G,actual_opening_m,contact_tolerance_m=.0008):
    opening=float(actual_opening_m)
    if not np.isfinite(opening) or not 0<=opening<=.08 or not .0001<=contact_tolerance_m<=.0015:raise ValueError('Invalid native aperture/contact tolerance')
    model=_model(c,part,G)
    if model is None:return dict(opposed_contact_compatible=False,expected_aperture_m=None,jaws=[],actual_opening_m=opening,uses_free_body_poses=False)
    points,normals,planes=model;jaws=[];mu=_reserve(c)
    for sign in [-1,1]:
        local=points*np.array([sign,sign,1.]);local[:,1]-=opening/2;local[:,2]+=.045
        distance=np.max(local@planes[:,:3].T+planes[:,3],axis=1)
        close=distance<=contact_tolerance_m;distal=close&(abs(points[:,2])<.009)&(abs(points[:,0])<.011)
        inward=distal&(normals[:,1]*sign<-.55);count=int(distal.sum());fraction=float(inward.sum()/count) if count else 0.
        ns=normals[distal];normal=ns.mean(0) if count else np.zeros(3);normal/=max(np.linalg.norm(normal),1e-12)
        valid=bool(inward.sum()>=5 and fraction>=.60 and normal[1]*sign<=-1/np.sqrt(1+mu**2))
        jaws.append(dict(sign=sign,minimum_hull_clearance_m=float(distance.min()),distal_contact_samples=count,opposed_wall_samples=int(inward.sum()),opposed_wall_normal_fraction=fraction,all_contact_normal=normal.tolist(),contact_compatible=valid))
    expected=bore_aperture(c,part,G)
    return dict(opposed_contact_compatible=bool(expected is not None and all(j['contact_compatible'] for j in jaws)),actual_opening_m=opening,expected_aperture_m=expected,contact_tolerance_m=float(contact_tolerance_m),geometric_friction_reserve_mu=mu,jaws=jaws,source='RGB-D ring-to-TCP measurement + native finger encoders + fixed CAD',uses_free_body_poses=False,force_contact_measured=False)
