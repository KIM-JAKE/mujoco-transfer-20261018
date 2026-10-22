"""Native pin contact proof in the RGB-D physical symmetry-axis frame."""
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial import ConvexHull
from receiver_pad_contact import contact_state

BODY_TO_AXIS_M=np.array([0.,-.00025,0.])

def axis_to_body_grasp(axis_G):
    """Map CAD BODY points with an RGB-D physical-axis grasp transform."""
    G=np.asarray(axis_G,float).copy()
    if G.shape!=(4,4)or not np.isfinite(G).all():raise ValueError('Invalid physical-axis grasp')
    G[:3,3]-=G[:3,:3]@BODY_TO_AXIS_M
    return G

def native_support(c,G,opening=.0105):
    """Finite hull proximity and flat patch certificate; no force assertion."""
    h=Path(__file__).resolve().parent;G=np.asarray(G,float);opening=float(opening)
    if G.shape!=(4,4)or not np.isfinite(G).all()or not 0<=opening<=.08:raise ValueError('Invalid measured native pin grasp/opening')
    if '_pin_axis_contact_context'not in c:
        model=np.load(h.parents[1]/'fieldkit/assets/part-models.npz');E=SimpleNamespace(models={'pin':model['pin_points']-BODY_TO_AXIS_M},normals={'pin':model['pin_normals']});c['_pin_axis_contact_context']=dict(config={},observed_grasp_clearance=SimpleNamespace(estimator=E));c['_pin_native_hull_planes']=ConvexHull(np.load(h/'native-official-finger.npz')['vertices']).equations
    local_c=c['_pin_axis_contact_context'];E=local_c['observed_grasp_clearance'].estimator;planes=c['_pin_native_hull_planes'];pts=E.models['pin']@G[:3,:3].T+G[:3,3];jaws=[]
    for sign in [-1,1]:
        flip=np.array([sign,sign,1.]);local=pts*flip;local[:,1]-=opening/2;local[:,2]+=.045;half=local@planes[:,:3].T+planes[:,3];dist=half.max(1);distal=(abs(pts[:,0])<.011)&(abs(pts[:,2])<.009);first=float(dist.min());active=distal&(dist<=first+.00015)&(dist<=.0008);facet=planes[np.argmax(half[active],axis=1),:3]*flip;flat=abs(facet[:,1])>=.999;support=pts[active][flat];along=(support-G[:3,3])@G[:3,2];span=float(np.ptp(along))if len(along)else 0.;jaws.append(dict(sign=sign,first_clearance_m=first,active_flat_samples=len(support),flat_pin_axis_span_m=span))
    contact=contact_state(local_c,'pin',G,opening);valid=contact['opposed_contact_compatible']and all(-.0008<=j['first_clearance_m']<=.0008 and j['active_flat_samples']>=5 and j['flat_pin_axis_span_m']>=.004 for j in jaws)
    return dict(admitted=bool(valid),minimum_flat_pin_axis_span_m=min(j['flat_pin_axis_span_m']for j in jaws),opening_m=opening,jaws=jaws,native_opposed_contact=contact,force_contact_measured=False,uses_free_body_poses=False,pose_origin='RGB-D physical symmetry-axis midpoint',CAD_body_to_axis_offset_m=BODY_TO_AXIS_M.tolist())
