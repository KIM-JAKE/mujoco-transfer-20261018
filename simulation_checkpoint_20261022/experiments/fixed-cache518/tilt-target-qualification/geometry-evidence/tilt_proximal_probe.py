exec(open('/tmp/fixed504-pin-collar-flat/probe_collar_flat.py').read().split('out=[]')[0])
from scipy.spatial.transform import Rotation
out=[]
for deg in(-20,-25,-30,-35,-40,-45,-50):
 G=np.eye(4);G[:3,:3]=Rotation.from_rotvec([0,np.deg2rad(deg),0]).as_matrix();G[:3,3]=G[:3,2]*.0155+[0,0,-.001]
 prox=all_gaps(G,.0127,True);lo=.005;hi=.018
 for _ in range(30):
  mid=(lo+hi)/2
  if all_gaps(G,mid)[0][0]<=0:lo=mid
  else:hi=mid
 out.append(dict(tilt_deg=deg,original_proximal_minimum_m=prox[0][0],proximal_closest=prox[0],first_contact_aperture_m=hi,first_contact=all_gaps(G,hi)[:4]))
Path('/tmp/fixed515-pin-depth-diagnosis/tilt-native-proximal-grid.json').write_text(json.dumps(dict(scope='Exactoriginalclassfour32-sidedcylinders, Handpalm/proximal-finger CAD; proposal-only no force/fullscene proof',rows=out),indent=2)+'\n')
for a in out:print(a['tilt_deg'],'proxmm',a['original_proximal_minimum_m']*1e3,'gapmm',a['first_contact_aperture_m']*1e3)
