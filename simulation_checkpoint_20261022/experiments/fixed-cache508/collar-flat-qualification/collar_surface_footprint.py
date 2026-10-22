exec(open('/tmp/pin-exact-field-handoff/profile_numeric.py').read().split("z=np.load('/tmp/pin481-terminal-public/")[0])
from scipy.spatial import ConvexHull
from isaac_kinematics import frame
# Original32-sided cylinder COLLAR lateral faces, sampled by interpolation on
# those originalfaces; no newcollider, margins or radii used.
cat=json.load(open(R/'isaac_port/runs/native-roas-table-scatter-stable-v251-corrected-pin-catalog/pin-native-grasp-catalog.json'))['candidates'][0];nom=np.asarray(cat['grasp_transform']);a=np.arange(32)*2*np.pi/32;vertices=np.asarray([[.0065*np.cos(x),.0065*np.sin(x),z]for z in[-.0065,-.0035]for x in a]).astype(np.float32).astype(float);collar=np.vstack([vertices[:32]*(1-u)+vertices[32:]*u for u in np.linspace(0,1,61)]);planes=ConvexHull(np.load(H/'native-official-finger.npz')['vertices'].astype(np.float32)).equations;out=[]
for shift in[0.,-.001,-.002,-.003]:
 G=nom.copy();G[2,3]+=shift;points=collar@G[:3,:3].T+G[:3,3];jaws=[]
 for sign in[-1,1]:
  flip=np.array([sign,sign,1]);p=points*flip;p[:,1]-=.0127/2;p[:,2]+=.045;h=p@planes[:,:3].T+planes[:,3];d=h.max(1);first=d.min();near=(d<=first+.0001);facets=planes[np.argmax(h[near],axis=1),:3];flat=abs(facets[:,1])>=.999;active=points[near][flat];along=(active-G[:3,3])@G[:3,2];jaws.append(dict(sign=sign,closest_collidable_collar_clearance_m=float(first),first100um_samples=int(near.sum()),first100um_flat_samples=int(flat.sum()),first100um_flat_fraction=float(flat.mean()),flat_collar_axis_span_m=float(np.ptp(along))if len(along)else 0.))
 out.append(dict(Hand_contact_z_shift_m=shift,jaws=jaws))
Path('/tmp/fixed496-pin-diagnosis/collar-surface-footprint-model.json').write_text(json.dumps(dict(scope='ORIGINAL32-sided physicalcollarfaces/nativeconvexfinger interpolatedfootprint model only; not force or dynamic qualification',rows=out),indent=2));print(json.dumps(out,indent=2))
