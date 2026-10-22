"""Fit the visible axial pin end plane/circle from RGB-masked depth pixels.

Reports the physical symmetry-axis midpoint, independent of unobservable yaw.
No CAD/body transform lookup; the known end-to-midpoint distance is 17.75 mm.
"""
import numpy as np
from scipy.spatial import ConvexHull
from rgbd_geometry import axis_frame

def _robust_cap_circle(xy,expected_radius_m):
 """Fit only a sufficiently visible circular CAD cap edge, not its chord.

 Circular RANSAC uses measured plane pixels and the known end-radius class.
 Hidden-circle centers cannot be supplied by a pose prior or simulator state.
 """
 from itertools import combinations
 from scipy.optimize import least_squares
 edge=xy[ConvexHull(xy).vertices]
 if len(edge)<12:raise RuntimeError('Pin cap has insufficient circular edge support')
 triples=np.asarray(list(combinations(range(len(edge)),3))) if len(edge)<=24 else np.asarray([np.random.default_rng(2118+i).choice(len(edge),3,replace=False)for i in range(2000)])
 p,q,r=edge[triples[:,0]],edge[triples[:,1]],edge[triples[:,2]]
 a=2*(q-p);b=2*(r-p);d=(q*q).sum(1)-(p*p).sum(1);e=(r*r).sum(1)-(p*p).sum(1);det=a[:,0]*b[:,1]-a[:,1]*b[:,0]
 good=abs(det)>1e-10;a,b,d,e,det,p=a[good],b[good],d[good],e[good],det[good],p[good]
 centers=np.c_[(d*b[:,1]-a[:,1]*e)/det,(a[:,0]*e-d*b[:,0])/det]
 radii=np.linalg.norm(p-centers,axis=1);good=abs(radii-expected_radius_m)<=.00015;centers,radii=centers[good],radii[good]
 best=None
 for center,radius in zip(centers,radii):
  residual=abs(np.linalg.norm(edge-center,axis=1)-radius);inliers=residual<.00009
  if inliers.sum()<12 or inliers.mean()<.75:continue
  angles=np.sort(np.arctan2(*(edge[inliers]-center).T[::-1]));coverage=2*np.pi-max(np.diff(np.r_[angles,angles[0]+2*np.pi]))
  if coverage<np.deg2rad(240):continue
  score=coverage+float(inliers.sum())*.003
  if best is None or score>best[0]:best=(score,center,radius,inliers)
 if best is None:raise RuntimeError('Pin circular cap is occluded: no class-radius arc spanning240degrees')
 _,center,radius,inliers=best
 solution=least_squares(lambda x:np.linalg.norm(edge[inliers]-x[:2],axis=1)-x[2],np.r_[center,radius])
 center,radius=solution.x[:2],float(solution.x[2]);residual=abs(np.linalg.norm(edge-center,axis=1)-radius);inliers=residual<.00009
 angles=np.sort(np.arctan2(*(edge[inliers]-center).T[::-1]));coverage=2*np.pi-max(np.diff(np.r_[angles,angles[0]+2*np.pi]));rms=float(np.sqrt(np.mean(residual[inliers]**2)))
 if abs(radius-expected_radius_m)>.00015 or inliers.sum()<12 or inliers.mean()<.75 or coverage<np.deg2rad(240) or rms>.00006:
  raise RuntimeError('Refined pin cap failed class-radius/full-arc confidence checks')
 return center,radius,dict(cap_edge_coverage_deg=float(np.rad2deg(coverage)),cap_circular_edge_inliers=int(inliers.sum()),cap_total_hull_edge_points=len(edge),cap_circular_edge_rms_m=rms,cap_expected_radius_m=float(expected_radius_m),robust_cap_circle_from_pixels=True)

def fit_pin(points,seed,prior_axis,initial=False,measured_axis=None,robust_circle=False):
 pts=np.asarray(points,float);pts=pts[np.isfinite(pts).all(1)];prior=np.array(prior_axis,float);prior/=np.linalg.norm(prior);rng=np.random.default_rng(822)
 if len(pts)<100:raise RuntimeError('Insufficient image depth on pin')
 sampling=pts[::max(1,len(pts)//3500)];best=None
 if measured_axis is not None:
  n=np.array(measured_axis,float)
 else:
  for _ in range(500):
   a,b,c=sampling[rng.choice(len(sampling),3,replace=False)];n=np.cross(b-a,c-a);length=np.linalg.norm(n)
   if length<1e-10:continue
   n/=length;n*=1 if n@prior>=0 else -1
   if n@prior<.7:continue
   mask=abs((sampling-a)@n)<.00004;count=int(mask.sum());score=count*float(n@prior)**2
   if best is None or score>best[0]:best=(score,n,mask)
  if best is None or best[2].sum()<60:raise RuntimeError('No visible pin axial face in depth')
  _,n,mask=best;face=sampling[mask];center=face.mean(0);_,_,V=np.linalg.svd(face-center,full_matrices=False);n=V[-1];n*=1 if n@prior>=0 else -1
 axial=pts@n;level=float(np.quantile(axial,.002 if initial else .998));face=pts[abs(axial-level)<.00010]
 if len(face)<20:raise RuntimeError('Pin end-cap depth region is incomplete')
 R=axis_frame(n);origin=face.mean(0);xy=(face-origin)@R[:,:2]
 circle_evidence={}
 if robust_circle:
  center2,radius,circle_evidence=_robust_cap_circle(xy,.00525 if initial else .0025);cap=origin+R[:,:2]@center2;part_center=cap-n*(-.01775 if initial else .01775)
 else:
  hull=ConvexHull(xy);edge=xy[hull.vertices];A=np.c_[2*edge,np.ones(len(edge))];coef=np.linalg.lstsq(A,(edge*edge).sum(1),rcond=None)[0];cap=origin+R[:,:2]@coef[:2];radius=float(np.sqrt(max(0,coef[2]+coef[:2]@coef[:2])));part_center=cap-n*(-.01775 if initial else .01775)
 expected_radius=.00525 if initial else .0025;lateral_predicted=bool(abs(radius-expected_radius)>.00045)
 if lateral_predicted:part_center=np.asarray(seed,float)+n*(float(n@part_center)-float(n@np.asarray(seed,float)))
 if np.linalg.norm(part_center-np.asarray(seed,float))>.006:raise RuntimeError('Pin axial face inconsistent with prior image/encoder grasp state')
 if not lateral_predicted and not .0015<radius<.008:raise RuntimeError('Pin end radius outside CAD range')
 return dict(**circle_evidence,part='pin',center_m=part_center.tolist(),axis=n.tolist(),pose_origin='Physical pin symmetry-axis midpoint',visible_end_radius_m=radius,cap_points=len(face),lateral_from_previous_RGBD_grasp_and_encoders=lateral_predicted,axial_depth_updated_from_current_RGBD=True,depth_points=len(pts),plane_rms_m=float(np.sqrt(np.mean(((face-cap)@n)**2))),uses_depth_sensor=True,uses_simulator_body_poses=False,method='RGB-D axial end plane; visible end-circle XY fit, encoder-propagated image grasp XY when the circle is occluded')
