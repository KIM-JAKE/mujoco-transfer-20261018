from pathlib import Path
import ast,json,types,hashlib
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
R=Path('/home/gpuadmin/workspace/jake/mujoco');H=R/'isaac_port/native_hand_layout/roas-table';RUN=R/'isaac_port/runs/native-roas-table-scatter-stable-v393-current-pair-visual-descent-all3-450';SRC=Path('/tmp/visual-descent393/overrides/grasp_transfer_feasibility.py');tree=ast.parse(SRC.read_text());fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef)and n.name=='large_donor_pad_support');ns=dict(np=np,__file__=str(H/'grasp_transfer_feasibility.py'));exec(compile(ast.Module(body=[fn],type_ignores=[]),str(SRC),'exec'),ns)
z=np.load(R/'isaac_port/fieldkit/assets/part-models.npz');est=types.SimpleNamespace(models={'ring1':z['ring1_points']});ctx=dict(observed_grasp_clearance=types.SimpleNamespace(estimator=est));data=json.loads(Path('/tmp/prealign393-evaluator/results.json').read_text());observations=json.loads((RUN/'rgb-observations.json').read_text());trace=np.load(RUN/'physx-sequence-trace.npz');dofnames=json.loads((RUN/'pipeline-results.json').read_text())['dof_names'];fi=[i for i,n in enumerate(dofnames)if n.startswith('left_native_')]
planes=ConvexHull(np.load(H/'native-official-finger.npz')['vertices']).equations

def axis_frame(n):
 n=np.array(n,float);n/=np.linalg.norm(n);x=np.array([1.,0,0]);x-=n*(x@n)
 if np.linalg.norm(x)<.1:x=np.array([0.,1.,0.]);x-=n*(x@n)
 x/=np.linalg.norm(x);return np.column_stack([x,np.cross(n,x),n])
def proper(R):u,_,v=np.linalg.svd(R);return u@v
def contact_line(G,width):
 points=est.models['ring1']@G[:3,:3].T+G[:3,3];jaws=[]
 for sign in [-1,1]:
  flip=np.array([sign,sign,1.]);local=points*flip;local[:,1]-=width/2;local[:,2]+=.045;hs=local@planes[:,:3].T+planes[:,3];distance=hs.max(1);distal=(abs(points[:,0])<.011)&(abs(points[:,2])<.009);first=distance[distal].min();active=distal&(distance<=first+.0003)&(distance<=.0008);facets=planes[np.argmax(hs[active],axis=1),:3]*flip;flat=abs(facets[:,1])>=.999;support=points[active][flat]
  jaws.append(dict(sign=sign,centroid_m=support.mean(0).tolist()if len(support)else None,bounds_m=[support.min(0).tolist(),support.max(0).tolist()]if len(support)else None,flat_count=len(support)))
 if any(j['centroid_m']is None for j in jaws):return dict(jaws=jaws,lever_m=None)
 a,b=[np.array(j['centroid_m'])for j in jaws];axis=b-a;axis/=np.linalg.norm(axis);com=G[:3,3];foot=a+axis*((com-a)@axis);return dict(jaws=jaws,jaw_line_axis_hand=axis.tolist(),COM_hand_m=com.tolist(),COM_minus_nearest_contact_line_m=(com-foot).tolist(),lever_m=float(np.linalg.norm(com-foot)))
rows=[]
for i,row in enumerate(data['rows']):
 ti=row['trace_index'];width=float(trace['q'][ti,fi].sum());G=np.array(row['physical_G']);obs=observations[row['observation_index']];pose=obs['result']['pose'];P=np.eye(4);P[:3,:3]=axis_frame(pose['axis']);P[:3,3]=pose['center_m'];imageG=np.linalg.inv(np.array(obs['hand_TCP_world']))@P
 r=dict(observation_index=row['observation_index'],actual_left_width_m=width,physical_support=ns['large_donor_pad_support'](ctx,G,width),image_support=ns['large_donor_pad_support'](ctx,imageG,width),physical_contact_line=contact_line(G,width),image_contact_line=contact_line(imageG,width))
 if i:
  old=np.array(data['rows'][i-1]['physical_G']);rv=Rotation.from_matrix(proper(G[:3,:3])@proper(old[:3,:3]).T).as_rotvec();r['physical_relative_rotation_vector_hand_rad']=rv.tolist();r['physical_relative_rotation_axis_hand']=(rv/np.linalg.norm(rv)).tolist();r['physical_relative_rotation_magnitude_rad']=float(np.linalg.norm(rv))
  oldaxis=np.array(data['rows'][i-1]['image_G_axis']);newaxis=np.array(row['image_G_axis']);v=np.cross(oldaxis,newaxis);ang=np.arctan2(np.linalg.norm(v),oldaxis@newaxis);r['image_minimum_axis_rotation_vector_hand_rad']=(v/max(np.linalg.norm(v),1e-15)*ang).tolist()
 rows.append(r)
# Compare only the requested neighboring mechanical depth, not a yaw/depth grid.
G=np.array(data['rows'][-1]['physical_G']);imageG= np.linalg.inv(np.asarray(observations[data['rows'][-1]['observation_index']]['hand_TCP_world']))@P
alternatives=[]
for depth,dz in [(.005,0.),(.004,.001)]:
 altered=G.copy();altered[2,3]+=dz
 alternatives.append(dict(receiver_depth_parameter_m=depth,meaning='Fixed-CAD proposal only: physical new target TCP must shift by -1mm localZ for depth4; not alteration of actual heldpart'if dz else'Currentmeasuredgeometry',nominal_delta_Gz_m=dz,physical_reference_support=ns['large_donor_pad_support'](ctx,altered,width),contact_line=contact_line(altered,width)))
result=dict(scope='Evaluation-only mechanical evidence; original fixed-CAD support function applied offline, no source/actor/force/friction changes or hardware motion',source_function_sha256=hashlib.sha256(ast.get_source_segment(SRC.read_text(),fn).encode()).hexdigest(),class_model_sha256=hashlib.sha256((R/'isaac_port/fieldkit/assets/part-models.npz').read_bytes()).hexdigest(),finger_model_sha256=hashlib.sha256((H/'native-official-finger.npz').read_bytes()).hexdigest(),rows=rows,depth_comparison_only=alternatives)
Path('/tmp/prealign393-evaluator/mechanics.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
