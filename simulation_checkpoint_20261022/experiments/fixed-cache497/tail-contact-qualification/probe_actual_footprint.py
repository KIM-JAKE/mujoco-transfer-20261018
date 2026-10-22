exec(open('/tmp/pin-exact-field-handoff/profile_numeric.py').read().split("z=np.load('/tmp/pin481-terminal-public/")[0])
from pin_native_contact import native_support
from scipy.spatial.transform import Rotation
x=json.load(open('/tmp/fixed497-pin-tail-flat/actual497-failed.json'));rows=x['observations'];out=[]
for row,gap in[(a,.010064689442515373)for a in rows if a['work']in['observation-063-held','observation-064-held']]+[(rows[-2],.009892938658595085),(rows[-1],.009852118790149689)]:
 # Preserve the original catalogue axial gauge via shortest axis rotation.
 c=np.asarray(x['events'][0]['data']['proposed_grasp']);a=c[:3,2];b=np.asarray(row['G_axis']);cross=np.cross(a,b);n=np.linalg.norm(cross);g=c.copy();g[:3,:3]=Rotation.from_rotvec(cross/max(n,1e-12)*np.arctan2(n,a@b)).as_matrix()@c[:3,:3];g[:3,3]=row['G_center_m'];support=native_support({},g,gap);out.append(dict(work=row['work'],gap=gap,G=g.tolist(),support=support));print(row['work'],json.dumps(support))
Path('/tmp/fixed497-pin-tail-flat/actual497-footprint.json').write_text(json.dumps(out,indent=2))
