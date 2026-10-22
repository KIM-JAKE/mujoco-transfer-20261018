exec(open('/tmp/pin-exact-field-handoff/profile_numeric.py').read().split("z=np.load('/tmp/pin481-terminal-public/")[0])
import fcl
from fcl_transport import convex
from own_payload_clearance import _proximal_hull
from pin_native_contact import native_support
from isaac_kinematics import frame
r=next((R/'isaac_port/runs').glob('*v495-*'));pin=json.loads((r/'pipeline-results.json').read_text())['scattered_pick']['pin'];observed=np.asarray(pin['measured_grasp']);cat=json.loads((R/'isaac_port/runs/native-roas-table-scatter-stable-v251-corrected-pin-catalog/pin-native-grasp-catalog.json').read_text())['candidates'][0];nominal=np.asarray(cat['grasp_transform']);asset=np.load(H/'native-official-finger.npz');finger=convex(asset['vertices'].astype(np.float32).astype(float));specs=[('tail',.00525,-.01775,-.0065),('collar',.0065,-.0065,-.0035),('shaft',.0038,-.0035,.0135),('tip',.0025,.0135,.01775)];pieces=[]
for name,radius,z0,z1 in specs:
 a=np.arange(32)*2*np.pi/32;v=np.asarray([[radius*np.cos(x),radius*np.sin(x),z]for z in[z0,z1]for x in a]);v=v.astype(np.float32).astype(float);pieces.append((name,convex(v),v))
local=np.load(R/'.local/render-current/pair432-exact-locals.npz');proximal=[]
for i,(body,src)in enumerate(zip(local['bodies'],local['sources'])):
 if '/right_arm_base/'not in body:continue
 if str(body).endswith('_NativeHand'):proximal.append((str(body),convex(local['v'+str(i)]),np.eye(4)))
 elif str(body).endswith('_finger'):
  v=local['v'+str(i)];v=_proximal_hull(v)if str(src).endswith('/tip')else v;sign=1 if str(body).endswith('right_left_finger')else-1;proximal.append((str(body),convex(v),(sign,str(src))))
def all_gaps(G,width,prox=False):
 root=frame([0,0,.1034],np.eye(3))@G;pinobjs=[(n,fcl.CollisionObject(m,fcl.Transform(root[:3,:3],root[:3,3])))for n,m,v in pieces];pairs=[]
 if prox:
  native=[]
  for n,m,x in proximal:
   T=frame([0,x[0]*width/2,.0584],np.eye(3)if x[0]==1 else np.diag([-1.,-1.,1.]))if isinstance(x,tuple)else x;native.append((n,fcl.CollisionObject(m,fcl.Transform(T[:3,:3],T[:3,3]))))
 else:
  native=[(str(sign),fcl.CollisionObject(finger,fcl.Transform(np.eye(3)if sign==1 else np.diag([-1.,-1.,1.]),[0,sign*width/2,.0584])))for sign in[-1,1]]
 for name,obj in pinobjs:
  for n,other in native:pairs.append((float(fcl.distance(obj,other,fcl.DistanceRequest(),fcl.DistanceResult())),name,n))
 return sorted(pairs)
out=[]
for shift in[0.,-.001,-.002,-.003]:
 G=nominal.copy();G[2,3]+=shift;lo=.005;hi=.018
 for _ in range(35):
  mid=(lo+hi)/2
  if all_gaps(G,mid)[0][0]<=0:lo=mid
  else:hi=mid
 contact=all_gaps(G,hi);support=native_support({},G,.0127);prox=all_gaps(G,.0127,True)
 out.append(dict(positive_contact_z_shift_m=shift,G=G.tolist(),zero_contact_first_aperture_m=hi,first_contacts=contact[:4],native_proximal_minimum_m=prox[0][0],proximal_closest=prox[0],model_sample_support=support))
Path('/tmp/fixed496-pin-diagnosis/physics-collar-flat-negative-offset-model.json').write_text(json.dumps(dict(scope='Exact original32-sided four-cylinder contact meshes/native finger convex CAD; FCL proposal only, not full scene/retention/force proof',rows=out),indent=2));print(json.dumps([dict(shift_mm=a['positive_contact_z_shift_m']*1e3,gap_mm=a['zero_contact_first_aperture_m']*1e3,first=a['first_contacts'][:2],prox_mm=a['native_proximal_minimum_m']*1e3,sample_support=a['model_sample_support']['minimum_flat_pin_axis_span_m'],sample_admitted=a['model_sample_support']['admitted'])for a in out],indent=2))
