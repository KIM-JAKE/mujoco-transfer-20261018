"""RGB-only calibrated multi-view ring pose from SAM masks and image edges.

No depth channel, simulator API or body transform input. Known CAD dimensions
are object models; stereo scale is also fit continuously to classify ring size.
Robot encoder pose supplies a broad tracking seed, never the object answer.
"""
import numpy as np,cv2
from scipy.optimize import least_squares
from scipy.ndimage import distance_transform_edt
from scipy.spatial.transform import Rotation

def project(P,packet):
 T=packet['T_cell_camera'];p=(P-T[:3,3])@T[:3,:3];K=packet['K'];return p[:,:2]/p[:,2,None]*np.array([K[0,0],K[1,1]])+K[:2,2]

def axis_frame(n):
 n=np.array(n,float);n/=np.linalg.norm(n);x=np.array([1.,0,0]);x-=n*(x@n)
 if np.linalg.norm(x)<.1:x=np.array([0.,1.,0.]);x-=n*(x@n)
 x/=np.linalg.norm(x);return np.column_stack([x,np.cross(n,x),n])

def prepare(packet,masks,seed,part):
 rgb=packet['rgb'];gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);canny=cv2.Canny(gray,18,45)
 uv=project(np.asarray(seed).reshape(1,3),packet)[0]
 choices=[]
 for m in masks:
  mask=m['mask'];v,u=np.nonzero(mask)
  if len(u)<40:continue
  dist=np.linalg.norm(np.array([np.mean(u),np.mean(v)])-uv)
  if dist<min(rgb.shape[:2])*.45:choices.append((dist,m))
 if not choices:raise RuntimeError('SAM did not identify the encoder-predicted work region')
 m=min(choices,key=lambda a:a[0])[1];mask=m['mask'].astype(np.uint8);near=cv2.dilate(mask,np.ones((9,9),np.uint8))>0
 edges=(canny>0)&near
 # Exclude robot edges inside the hole unless close to a segmented boundary.
 boundary=cv2.morphologyEx(mask,cv2.MORPH_GRADIENT,np.ones((7,7),np.uint8))>0
 edges&=cv2.dilate(boundary.astype(np.uint8),np.ones((9,9),np.uint8))>0
 dt=distance_transform_edt(~edges);gy,gx=np.gradient(dt)
 return dict(packet=packet,mask=mask,edges=edges,dt=dt,dx=gx,dy=gy,score=m['score'])

def interpolate(a,uv):
 h,w=a.shape;x=np.clip(uv[:,0],0,w-1.001);y=np.clip(uv[:,1],0,h-1.001);ix=x.astype(int);iy=y.astype(int);u=x-ix;v=y-iy
 return a[iy,ix]*(1-u)*(1-v)+a[iy,ix+1]*u*(1-v)+a[iy+1,ix]*(1-u)*v+a[iy+1,ix+1]*u*v

def ring_points(part,scale=1.):
 # CAD visible annular end rims and roundover boundary. Radial circles are
 # sampled rather than accepting rendered labels or 3D surface hits.
 ro,ri,h=(.0375,.031684,.00775)if part=='ring1'else(.031,.025255,.007)
 theta=np.arange(2048)*2*np.pi/2048;cs=np.c_[np.cos(theta),np.sin(theta)]
 # Inner hole runs full depth; outer cylindrical wall joins rounded ends.
 points=[];labels=[]
 for r,z in [(ri,-h),(ri,h),(ro,-h+.001),(ro,h-.001)]:
  points.append(np.c_[cs*r*scale,np.full(len(cs),z*scale)]);labels.extend([len(points)-1]*len(cs))
 return np.concatenate(points),np.array(labels)

def estimate(packets,masks,seed,axis,part,scale_fit=False):
 views=[prepare(p,m,seed,part)for p,m in zip(packets,masks)];base=axis_frame(axis);local,labels=ring_points(part)
 def points(x):
  R=Rotation.from_rotvec(np.r_[x[3:5],0.]).as_matrix()@base;return local*x[5]@R.T+x[:3],R
 def residual(x):
  P,R=points(x);rows=[]
  for view in views:
   uv=project(P,view['packet']);d=interpolate(view['dt'],uv)
   # Occlusions: only the best 65% of each rim contribute. A successful
   # detection still needs enough support in both calibrated RGB views.
   for label in range(4):
    r=d[labels==label];chosen=np.argsort(r)[:int(.65*len(r))];rows.extend(np.minimum(r[chosen],12.))
  return np.array(rows)
 x0=np.r_[seed,0.,0.,1.];low=x0-np.r_[.025,.025,.025,.35,.35,.15 if scale_fit else .000001];high=x0+np.r_[.025,.025,.025,.35,.35,.15 if scale_fit else .000001]
 fit=least_squares(residual,x0,bounds=(low,high),diff_step=np.array([.00003,.00003,.00003,.0005,.0005,.0001]),max_nfev=140,loss='soft_l1',f_scale=1.)
 P,R=points(fit.x);err=residual(fit.x);return dict(part=part,center_m=fit.x[:3].tolist(),axis=R[:,2].tolist(),scale=float(fit.x[5]),outer_diameter_m=float((.075 if part=='ring1'else .062)*fit.x[5]),pixel_residual_median=float(np.median(err)),pixel_residual_rms=float(np.sqrt(np.mean(err**2))),optimizer_evaluations=fit.nfev,SAM_scores=[v['score']for v in views],uses_depth=False,uses_simulator_body_poses=False),views

def estimate_edges(packets,masks,seed,axis,part,scale_fit=False):
 from scipy.spatial import cKDTree
 views=[prepare(p,m,seed,part)for p,m in zip(packets,masks)];base=axis_frame(axis);local,labels=ring_points(part)
 observations=[]
 for view in views:
  v,u=np.nonzero(view['edges']);uv=np.c_[u,v].astype(float);uv=uv[::max(1,len(uv)//500)];observations.append(uv)
 def model(x):
  R=Rotation.from_rotvec(np.r_[x[3:5],0.]).as_matrix()@base;return local*x[5]@R.T+x[:3],R
 def residual(x):
  P,R=model(x);rows=[]
  for view,uv in zip(views,observations):
   pred=project(P,view['packet']);d,idx=cKDTree(pred).query(uv);rows.extend((uv-pred[idx]).reshape(-1))
  # Axial yaw is unobservable. Do not introduce an object-pose prior residual.
  return np.asarray(rows)
 x0=np.r_[seed,0.,0.,1.];low=x0-np.r_[.025,.025,.025,.4,.4,.26 if scale_fit else .000001];high=x0+np.r_[.025,.025,.025,.4,.4,.26 if scale_fit else .000001]
 candidates=[]
 for offset in [0,-.012,.012]:
  start=x0.copy();start[:3]+=np.asarray(axis)*offset
  fit=least_squares(residual,start,bounds=(low,high),max_nfev=300,loss='cauchy',f_scale=1.,diff_step=.0001,x_scale=[.01,.01,.01,.1,.1,.1],xtol=1e-12)
  err=residual(fit.x);candidates.append((np.mean(np.minimum(err**2,25)),fit))
 score,fit=min(candidates,key=lambda c:c[0]);P,R=model(fit.x);err=residual(fit.x)
 return dict(part=part,center_m=fit.x[:3].tolist(),axis=R[:,2].tolist(),scale=float(fit.x[5]),outer_diameter_m=float((.075 if part=='ring1'else .062)*fit.x[5]),pixel_residual_rms=float(np.sqrt(np.mean(err**2))),optimizer_evaluations=fit.nfev,SAM_scores=[v['score']for v in views],uses_depth=False,uses_simulator_body_poses=False),views

def refine_hole(packets,masks,rough,part):
 from scipy.spatial import cKDTree
 seed=np.array(rough['center_m']);base=axis_frame(rough['axis']);local,labels=ring_points(part);inner=local[labels<2];outer=local[labels>=2];views=[prepare(p,m,seed,part)for p,m in zip(packets,masks)];observations=[]
 for view in views:
  v,u=np.nonzero(view['edges']);uv=np.c_[u,v].astype(float);pred=project(inner@base.T+seed,view['packet']);d,idx=cKDTree(pred).query(uv);do,_=cKDTree(project(outer@base.T+seed,view['packet'])).query(uv)
  keep=(d<4)&(d<do);uv=uv[keep];uv=uv[::max(1,len(uv)//500)];observations.append(uv)
 def model(x):
  R=Rotation.from_rotvec(np.r_[x[3:5],0.]).as_matrix()@base;return inner@R.T+x[:3],R
 def residual(x):
  P,R=model(x);rows=[]
  for view,uv in zip(views,observations):
   pred=project(P,view['packet']);d,idx=cKDTree(pred).query(uv);rows.extend((uv-pred[idx]).reshape(-1))
  return np.array(rows)
 x0=np.r_[seed,0.,0.];low=x0-[.005,.005,.005,.15,.15];high=x0+[.005,.005,.005,.15,.15]
 fit=least_squares(residual,x0,bounds=(low,high),max_nfev=200,loss='cauchy',f_scale=.6,x_scale=[.005,.005,.005,.1,.1],xtol=1e-12,diff_step=.0001);P,R=model(fit.x)
 result=dict(rough);result.update(center_m=fit.x[:3].tolist(),axis=R[:,2].tolist(),hole_edge_refinement=True,hole_edge_points=[len(u)for u in observations],hole_residual_rms=float(np.sqrt(np.mean(residual(fit.x)**2))))
 return result
