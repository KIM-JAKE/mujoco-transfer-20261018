from pathlib import Path
import json,numpy as np,fcl
from scipy.spatial.transform import Rotation
from field_demo import collision
T=Path('/tmp/field-oblique-floor587');R=Path('/home/gpuadmin/workspace/jake/mujoco');H=R/'isaac_port/native_hand_layout/roas-table';D=R/'.local/field-floor587'
z=np.load(T/'unknown-voxels.npz');cells=z['unknown_centers_cell_m'];spacing=z['spacing_m'];bounds=z['bounds_cell_m'];states=z['states'];packet=dict(np.load(D/'fresh-overhead-rgbd.npz'));depth=packet['depth_m'];K=packet['K'];CT=packet['T_cell_camera']
basket=json.load(open(H/'bin-picking/asset-audit.json'))['baskets'][0];b=basket['original_row'];cad=np.load(R/'isaac_port/generated/contact_task/collision.npz');v=cad[b['mesh']+'_vertices'];faces=cad[b['mesh']+'_faces'];q=np.asarray(b['quaternion']);rot=Rotation.from_quat(q[[1,2,3,0]]).as_matrix();world=v@rot.T+np.asarray(b['position']);mesh=fcl.BVHModel();mesh.beginModel(len(world),len(faces));mesh.addSubModel(world,faces.astype(np.int32));mesh.endModel();ob=fcl.CollisionObject(mesh)
def surface_gap(p):
 sphere=fcl.CollisionObject(fcl.Sphere(1e-9),fcl.Transform(np.eye(3),np.asarray(p)));ans=fcl.DistanceResult();dist=fcl.distance(sphere,ob,fcl.DistanceRequest(enable_nearest_points=True),ans);return dict(distance_m=float(dist+1e-9),nearest_basket_surface_cell_m=np.asarray(ans.nearest_points[1]).tolist())
output=[]
for p in sorted(T.glob('*.json')):
 if not(p.stem.startswith('original-')or p.stem.startswith('swap-')):continue
 r=json.load(open(p));x=r['first_rejection_context'];cap=x.get('capsule');obstacle=x.get('obstacle')
 if not cap:continue
 a=np.asarray(cap['a']);bb=np.asarray(cap['b']);boxT=np.asarray(obstacle['T_cell_box']);half=np.asarray(obstacle['half_extents_m']);candidate=cells[np.all(abs(cells-boxT[:3,3])<=half+1e-10,axis=1)]
 distances=[]
 for center in candidate:
  X=np.eye(4);X[:3,3]=center;box=dict(T_cell_box=X.tolist(),half_extents_m=(spacing/2).tolist());dist=float(collision.segment_box_distance(a,bb,box)[0]);distances.append(dist)
 j=int(np.argmin(distances));center=candidate[j];distance=distances[j];index=np.floor((center-bounds[0])/spacing).astype(int);corners=center+np.asarray([[aa,bbb,cc]for aa in[-1,1]for bbb in[-1,1]for cc in[-1,1]])*spacing/2;cam=(corners-CT[:3,3])@CT[:3,:3];project=cam[:,:2]/cam[:,2:];project=project*[K[0,0],K[1,1]]+[K[0,2],K[1,2]];lo=np.floor(project.min(0)-.5).astype(int);hi=np.ceil(project.max(0)+.5).astype(int);inside=bool((lo>=0).all()and(hi<[depth.shape[1],depth.shape[0]]).all());foot=depth[lo[1]:hi[1]+1,lo[0]:hi[0]+1]if inside else np.array([]);valid=np.isfinite(foot)&(foot>.05)&(foot<5)
 raw=None
 if valid.any():
  zz=float(foot[valid].min());arg=np.argwhere(valid&(foot==zz))[0];px,py=int(lo[0]+arg[1]),int(lo[1]+arg[0]);surf=np.array([(px-K[0,2])/K[0,0]*zz,(py-K[1,2])/K[1,1]*zz,zz])@CT[:3,:3].T+CT[:3,3];raw=dict(pixel=[px,py],depth_m=zz,surface_cell_m=surf.tolist(),fixed_basket_CAD_evaluation=surface_gap(surf),free_ray_deficit_m=float(cam[:,2].max()+.003-zz))
 merged=float(collision.segment_box_distance(a,bb,obstacle)[0]);output.append(dict(case=p.name,tracking_bound_rad=r['tracking_bound_rad'],first_body=cap['name'],reason=x['reason'],occupancy_kind=obstacle['occupancy_kind'],capsule_radius_m=cap['r'],nominal_radius_m=cap['nominal_radius_m'],tracking_time_extra_m=cap['r']-cap['nominal_radius_m'],margin_m=x['margin_m'],closest_TRUE_unknown_distance_m=distance,merged_box_distance_m=merged,coalescing_false_positive=abs(distance-merged)>1e-9,index=index.tolist(),center_cell_m=center.tolist(),half_cell_m=(spacing/2).tolist(),inside_FOV=inside,footprint_low=lo.tolist(),footprint_high=hi.tolist(),valid_pixels=int(valid.sum()),footprint_pixels=int(foot.size),voxel_farthest_camera_depth_m=float(cam[:,2].max()),nearest_observed=raw,unknown_center_fixed_basket_CAD_evaluation=surface_gap(center),private_pose_control_used=False,interpretation='Fixed basket CAD distance is evaluation-only; no UNKNOWN cell was changed or declared free. Raw minimum-depth point on known basket surface means this first rejection need not represent unseen loose stock.'))
(T/'unknown-root-cause.json').write_text(json.dumps(dict(cases=output,hardware_tested=False,proof_approval=False,all_unknown_preserved=True),indent=2)+'\n');print(json.dumps(output,indent=2))
