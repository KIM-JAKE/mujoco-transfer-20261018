from pathlib import Path
import numpy as np,json,ast,re,fcl
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
R=Path('/home/gpuadmin/workspace/jake/mujoco');H=R/'isaac_port/native_hand_layout/roas-table';O=Path(__file__).parent
cat=json.loads((R/'isaac_port/runs/native-roas-table-scatter-stable-v251-corrected-pin-catalog/pin-native-grasp-catalog.json').read_text())['candidates'][0];G0=np.array(cat['grasp_transform']);G0[2,3]-=.001
angles=np.arange(32)*2*np.pi/32;verts=np.array([[.0065*np.cos(x),.0065*np.sin(x),z]for z in[-.0065,-.0035]for x in angles]).astype(np.float32).astype(float);sample=np.vstack([verts[:32]*(1-u)+verts[32:]*u for u in np.linspace(0,1,121)]);fv=np.load(H/'native-official-finger.npz')['vertices'].astype(np.float32).astype(float);hull=ConvexHull(fv);planes=hull.equations
tri=hull.simplices.copy();normals=np.cross(fv[tri[:,1]]-fv[tri[:,0]],fv[tri[:,2]]-fv[tri[:,0]]);flip=np.einsum('ij,ij->i',normals,hull.equations[:,:3])<0;tri[flip]=tri[flip][:,[0,2,1]];finger=fcl.Convex(fv,len(tri),np.c_[np.full(len(tri),3),tri].ravel())
# Both exact baked original fixturetrianglemeshes, no collider approximation.
scene=(R/'isaac_port/runs/native-roas-table-bin-retimed14/scene-authored.usda').read_text();fixtures={}
for name in('instance_0554','instance_0519'):
 start=scene.index('def Mesh "'+name+'"');end=scene.find('\n            def ',start+20);block=scene[start:end]
 def array(k):return np.array(ast.literal_eval(re.search(r'\b'+k+r' = (\[[^\n]+\])',block).group(1)))
 v=array('points').astype(np.float32).astype(float);f=array('faceVertexIndices').reshape(-1,3).astype(np.int32)
 if 'xformOpOrder = []'not in block:
  quat=np.array(ast.literal_eval(re.search(r'xformOp:orient = (\([^\n]+\))',block).group(1)));translation=np.array(ast.literal_eval(re.search(r'xformOp:translate = (\([^\n]+\))',block).group(1)));v=v@Rotation.from_quat(quat[[1,2,3,0]]).as_matrix().T+translation
 mesh=fcl.BVHModel();mesh.beginModel(len(v),len(f));mesh.addSubModel(v,f);mesh.endModel();fixtures[name]=fcl.CollisionObject(mesh)
public=json.loads((O/'public-depth-comparison.json').read_text());case=next(x for x in public['rows']if x['run']==514);origH=np.array(case['capture_rows'][1]['command_H']);part=origH@G0;pickup=np.array(cat['pickup_H']);oldG=np.array(cat['grasp_transform']);pickup0=(pickup@oldG)@np.linalg.inv(G0);pickupPart=pickup@oldG;width=.0127;rows=[]
# Hand/proximal exact proxies from native432 meshes; include fullNativeHand
# and BOTH full36v fingertip meshes. Other arm links require runtime guards.
loc=np.load(R/'.local/render-current/pair432-exact-locals.npz');handrows=[]
for i,(body,src)in enumerate(zip(loc['bodies'],loc['sources'])):
 if '/right_arm_base/'in str(body)and str(body).endswith('_NativeHand'):
  v=loc['v'+str(i)];h=ConvexHull(v);f=h.simplices.copy();norm=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]]);flip=np.einsum('ij,ij->i',norm,h.equations[:,:3])<0;f[flip]=f[flip][:,[0,2,1]];m=fcl.Convex(v,len(f),np.c_[np.full(len(f),3),f].ravel());handrows.append((str(src),m))
for deg in(-20,-25,-30,-35,-40,-45,-50):
 G=np.eye(4);G[:3,:3]=Rotation.from_rotvec([0,np.deg2rad(deg),0]).as_matrix();G[:3,3]=G[:3,2]*.0155+[0,0,-.001];points=sample@G[:3,:3].T+G[:3,3];jaws=[]
 for sign in(-1,1):
  p=points*np.array([sign,sign,1]);p[:,1]-=width/2;p[:,2]+=.045;h=p@planes[:,:3].T+planes[:,3];d=h.max(1);first=d.min();near=d<=first+.0001;ns=planes[np.argmax(h[near],axis=1),:3];flat=abs(ns[:,1])>=.999;active=points[near][flat];along=(active-G[:3,3])@G[:3,2];jaws.append(dict(sign=sign,first100um_flat_fraction=float(flat.mean()),first100um_flat_samples=int(flat.sum()),first100um_samples=int(near.sum()),flat_collar_axis_span_m=float(np.ptp(along))if len(along)else 0.,minimum_surface_clearance_m=float(first)))
 T=part@np.linalg.inv(G);P=pickupPart@np.linalg.inv(G);native=[]
 for sign in(1,-1):
  L=np.eye(4);L[:3,:3]=np.eye(3)if sign==1 else np.diag([-1,-1,1]);L[:3,3]=[0,sign*width/2,-.045];F=T@L;native.append(('jaw'+str(sign),finger,F))
 for name,m in handrows:
  L=np.eye(4);L[2,3]=-.1034;native.append((name,m,T@L))
 gaps=[]
 for fixture,obstacle in fixtures.items():
  for name,geom,F in native:
   obj=fcl.CollisionObject(geom,fcl.Transform(F[:3,:3],F[:3,3]));dist=float(fcl.distance(obj,obstacle,fcl.DistanceRequest(),fcl.DistanceResult()));gaps.append(dict(fixture=fixture,native=name,distance_m=dist))
 rows.append(dict(tilt_deg=deg,tail_contact_class_point_m=[0,0,-.0155],contact_Hand_point_m=[0,0,-.001],G=G.tolist(),jaws=jaws,fixture_Hand_world_delta_m=(T[:3,3]-origH[:3,3]).tolist(),pickup_world_translation_delta_m=(P[:3,3]-pickup0[:3,3]).tolist(),pickup_rotation_delta_rad=float(Rotation.from_matrix(P[:3,:3]@pickup0[:3,:3].T).magnitude()),fixture_native_subset_gaps=gaps,whole_scene_and_holder_and_IK_qualified=False))
result=dict(scope='Read-only sameclass physical tailpoint mapped to sameHandcontact, recomputedGtranslation; original32-sidedcollar/nativeCAD and0554+0519triangle subsets ONLY. Runtimefullscene/IK/force/graspnotqualified.',rows=rows)
(O/'tilt-collar-native-subset-grid.json').write_text(json.dumps(result,indent=2)+'\n')
for r in rows:print(r['tilt_deg'],'flatspanmm',[round(x['flat_collar_axis_span_m']*1e3,3)for x in r['jaws']],'fraction',[round(x['first100um_flat_fraction'],3)for x in r['jaws']],'0554/0519minmm',[round(min(x['distance_m']for x in r['fixture_native_subset_gaps']if x['fixture']==f)*1e3,3)for f in('instance_0554','instance_0519')],'deltaPickupmm',np.round(np.array(r['pickup_world_translation_delta_m'])*1e3,4))
