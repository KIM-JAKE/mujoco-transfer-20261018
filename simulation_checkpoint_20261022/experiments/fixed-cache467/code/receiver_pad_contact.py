"""Receiver contact compatibility from fresh camera pose and hand encoders.

No part-centre heuristic, simulator actor state or force assertion is used.
This reports CAD surface proximity to each unmodified native finger hull and
whether the corresponding surface normals support an opposed closing grip.
"""
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull


def contact_state(c,part,G,actual_opening_m,contact_tolerance_m=.0008):
    """Report both pad contacts; caller must separately verify finger settling."""
    G=np.asarray(G,float);opening=float(actual_opening_m)
    if G.shape!=(4,4) or not np.isfinite(G).all() or not np.isfinite(opening) or not 0<=opening<=.08:
        raise ValueError('Invalid camera ring-to-receiver pose or native opening')
    if not .0001<=contact_tolerance_m<=.0015:raise ValueError('Invalid contact tolerance')
    cache=c.setdefault('_receiver_hull_contact_cache',{})
    if 'planes' not in cache:
        vertices=np.load(Path(__file__).resolve().parent/'native-official-finger.npz')['vertices']
        cache['planes']=ConvexHull(vertices).equations
    estimator=c['observed_grasp_clearance'].estimator
    points=estimator.models[part]@G[:3,:3].T+G[:3,3]
    normals=estimator.normals[part]@G[:3,:3].T
    planes=cache['planes'];jaws=[]
    for sign in [-1,1]:
        flip=np.array([sign,sign,1.]);local=points*flip
        local[:,1]-=opening/2;local[:,2]+=.045
        halfspace=local@planes[:,:3].T+planes[:,3]
        distance=halfspace.max(axis=1)
        near=distance<=contact_tolerance_m
        distal=near&(abs(points[:,0])<.011)&(abs(points[:,2])<.009)
        count=int(distal.sum());opposed=distal&(normals[:,1]*sign>.55)
        fraction=float(opposed.sum()/count) if count else 0.
        ns=normals[distal]
        resultant=ns.mean(0) if count else np.zeros(3)
        resultant/=max(np.linalg.norm(resultant),1e-12)
        # Closest native hull facet must face the part surface. Mean over ALL
        # nearby distal samples; no cap/corner normals are silently dropped.
        facets=np.argmax(halfspace[distal],axis=1)
        finger_out=planes[facets,:3]*flip
        opposition=-np.sum(finger_out*ns,axis=1)
        mean_opposition=float(opposition.mean()) if count else 0.
        opposed_facet_fraction=float((opposition>.75).mean()) if count else 0.
        compatible=bool(count>=5 and opposed.sum()>=5 and fraction>=.80 and resultant[1]*sign>=1/np.sqrt(1+.4**2) and mean_opposition>=.80 and opposed_facet_fraction>=.75)
        cap_evidence={}
        if c.get('config',{}).get('receiver_axial_only') and (part=='ring1' or not c.get('config',{}).get('large_double_handover_full_task')) and abs(G[1,2])>.95:
            # A flat cap is the first surface encountered by a closing pad.
            # Nearby countersink/bevel samples are not additional loaded
            # contacts merely because they lie within the 0.8 mm proximity
            # allowance. Require an actual broad, coplanar, opposed cap patch
            # and reject any non-cap surface that protrudes ahead of it.
            all_facets=np.argmax(halfspace,axis=1)
            all_finger_out=planes[all_facets,:3]*flip
            alignment=-np.sum(all_finger_out*normals,axis=1)
            cap=distal&(normals[:,1]*sign>.98)&(alignment>.95)
            cap_points=points[cap];cap_count=int(cap.sum())
            cap_min=float(distance[cap].min()) if cap_count else float('inf')
            noncap=distal&~cap
            noncap_min=float(distance[noncap].min()) if noncap.any() else float('inf')
            span=np.ptp(cap_points,axis=0) if cap_count else np.zeros(3)
            compatible=bool(cap_count>=10 and span[0]>=.005 and span[2]>=.001 and span[1]<=.00015 and cap_min<=contact_tolerance_m and noncap_min>=cap_min-.0001 and resultant[1]*sign>=1/np.sqrt(1+.4**2))
            cap_evidence=dict(contact_mode='axial_flat_caps',cap_support_samples=cap_count,cap_patch_span_m=span.tolist(),minimum_cap_clearance_m=cap_min if np.isfinite(cap_min) else None,nearest_noncap_clearance_m=noncap_min if np.isfinite(noncap_min) else None)
            if c.get('config',{}).get('axial_cap_first_contact_patch'):
                # The broad proximity allowance locates candidate cap surfaces;
                # it does not make every near point a loaded contact patch.
                # Keep the original strict patch dimensions/planarity and all
                # non-cap/normal rules, applied to the FIRST100um hull band.
                legacy_compatible=compatible
                band=float(c.get('config',{}).get('axial_cap_contact_band_m',.0001))
                if not .0001<=band<=.00015:raise ValueError('Axial first-contact band must remain100–150micrometres')
                active_cap=cap & (distance<=cap_min+band)
                active_points=points[active_cap];active_count=int(active_cap.sum())
                active_span=np.ptp(active_points,axis=0) if active_count else np.zeros(3)
                compatible=bool(active_count>=10 and active_span[0]>=.005 and active_span[2]>=.001 and active_span[1]<=.00015 and cap_min<=contact_tolerance_m and noncap_min>=cap_min-.0001 and resultant[1]*sign>=1/np.sqrt(1+.4**2))
                cap_evidence.update(legacy_all_near_cap_contact_compatible=bool(legacy_compatible),all_near_cap_support_samples=cap_count,all_near_cap_patch_span_m=span.tolist(),cap_support_samples=active_count,cap_patch_span_m=active_span.tolist(),first_contact_hull_band_m=band,cap_patch_model='first_contact_native_hull_band',force_contact_measured=False)
        jaws.append(dict(**cap_evidence,sign=sign,minimum_hull_clearance_m=float(distance.min()),near_hull_samples=int(near.sum()),distal_contact_samples=count,opposed_wall_samples=int(opposed.sum()),opposed_wall_normal_fraction=fraction,all_contact_normal=resultant.tolist(),mean_finger_surface_normal_opposition=mean_opposition,opposed_finger_facet_fraction=opposed_facet_fraction,contact_compatible=compatible))
    return dict(opposed_contact_compatible=bool(all(j['contact_compatible'] for j in jaws)),both_hulls_near_CAD=bool(all(j['distal_contact_samples']>=5 for j in jaws)),actual_opening_m=opening,contact_tolerance_m=float(contact_tolerance_m),jaws=jaws,source='Fresh RGB-D ring-to-receiver pose + actual finger encoder opening + fixed native finger CAD',uses_free_body_poses=False,force_contact_measured=False)
