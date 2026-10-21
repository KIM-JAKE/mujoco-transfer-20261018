from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
T=Path('/tmp/field-oblique-floor587');R=Path('/home/gpuadmin/workspace/jake/mujoco');b=json.load(open(R/'isaac_port/native_hand_layout/roas-table/bin-picking/asset-audit.json'))['baskets'][0];row=b['original_row'];z=np.load(R/'isaac_port/generated/contact_task/collision.npz');q=np.asarray(row['quaternion']);rot=Rotation.from_quat(q[[1,2,3,0]]).as_matrix();v=z[row['mesh']+'_vertices']@rot.T+row['position'];tri=v[z[row['mesh']+'_faces']];A,B,C=tri[:,0],tri[:,1],tri[:,2];e0=B-A;e1=C-A;normal=np.cross(e0,e1);norm2=(normal*normal).sum(1)
def closest(p):
 signed=((p-A)*normal).sum(1)/np.maximum(norm2,1e-30);proj=p-signed[:,None]*normal;vp=proj-A;d00=(e0*e0).sum(1);d01=(e0*e1).sum(1);d11=(e1*e1).sum(1);d20=(vp*e0).sum(1);d21=(vp*e1).sum(1);den=d00*d11-d01*d01;u=(d11*d20-d01*d21)/np.maximum(den,1e-30);vv=(d00*d21-d01*d20)/np.maximum(den,1e-30);inside=(u>=0)&(vv>=0)&(u+vv<=1)&(norm2>1e-20);candidates=[proj[inside]]
 for aa,bb in[(A,B),(B,C),(C,A)]:
  d=bb-aa;t=np.clip(((p-aa)*d).sum(1)/np.maximum((d*d).sum(1),1e-30),0,1);candidates.append(aa+t[:,None]*d)
 points=np.concatenate(candidates);dist=np.linalg.norm(points-p,axis=1);i=int(np.argmin(dist));return float(dist[i]),points[i]
def winding(p):
 a,b,c=A-p,B-p,C-p;la=np.linalg.norm(a,axis=1);lb=np.linalg.norm(b,axis=1);lc=np.linalg.norm(c,axis=1);numer=np.einsum('ij,ij->i',a,np.cross(b,c));den=la*lb*lc+np.einsum('ij,ij->i',a,b)*lc+np.einsum('ij,ij->i',b,c)*la+np.einsum('ij,ij->i',c,a)*lb;return float((2*np.arctan2(numer,den)).sum()/(4*np.pi))
report=[]
for c in json.load(open(T/'unknown-root-cause.json'))['cases']:
 if c['case']not in ['original-0.json','swap-0.json']:continue
 p=np.asarray(c['center_cell_m']);half=np.asarray(c['half_cell_m']);corners=p+np.array([[a,b,c]for a in[-1,1]for b in[-1,1]for c in[-1,1]])*half;d,near=closest(p);corner_dist=[closest(p)[0]for p in corners];w=[winding(p)for p in[p,*corners]];report.append(dict(case=c['case'],voxel_center_cell_m=p.tolist(),exact_nearest_calibrated_basket_triangle_point_m=near.tolist(),center_surface_distance_m=d,center_to_surface_vector_m=(near-p).tolist(),corner_surface_distance_range_m=[min(corner_dist),max(corner_dist)],center_and_corner_signed_winding_numbers=w,inside_material_corners=sum(abs(v)>.5 for v in w[1:]),raw_surface_is_known_fixed_basket=True,source_body_pose_control=False))
(T/'postfloor-nominal-triangle-proof.json').write_text(json.dumps(dict(scope='Fixed authored CAD evaluation only, no object truth/controller input or occupancy mutation',cases=report),indent=2)+'\n');print(json.dumps(report,indent=2))
