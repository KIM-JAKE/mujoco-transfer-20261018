"""RGB-only pin end-face metrology with calibrated views and known pin CAD.

Detect image ellipses (no depth or simulator lookup), jointly fit the known end
circle, then offset by half the known 35.5 mm length to the part frame.
"""
import itertools,numpy as np,cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from rgb_pose import axis_frame,project

def candidates(packet,seed_face,radius):
 gray=cv2.cvtColor(packet['rgb'],cv2.COLOR_RGB2GRAY);edge=cv2.Canny(gray,8,24);contours,_=cv2.findContours(edge,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE);contours=list(contours);rgb=packet['rgb'];neutral=(rgb.max(2)-rgb.min(2))<45
 # Uniform synthetic face colors provide RGB connected regions in addition to edges.
 for level in np.unique(gray[neutral]):
  if level<25 or level>190:continue
  region=(neutral&(abs(gray.astype(int)-int(level))<=3)).astype(np.uint8);region=cv2.morphologyEx(region,cv2.MORPH_OPEN,np.ones((3,3),np.uint8));cc,_=cv2.findContours(region,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE);contours.extend(cc)
 uv=project(np.array(seed_face).reshape(1,3),packet)[0];T=packet['T_cell_camera'];z=((np.array(seed_face)-T[:3,3])@T[:3,:3])[2];diam=2*radius*packet['K'][0,0]/z;found=[]
 for c in contours:
  if len(c)<15:continue
  ell=cv2.fitEllipse(c);center=np.array(ell[0]);axes=np.array(ell[1])/2
  if min(axes)<2 or max(axes)>diam*1. or max(axes)<diam*.28 or min(axes)/max(axes)<.40:continue
  shift=np.linalg.norm(center-uv)
  if shift>max(110,diam*4.):continue
  angle=np.radians(ell[2]);R=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]);q=(c[:,0,:]-center)@R;res=(np.sqrt(((q/axes)**2).sum(1))-1)*min(axes);rms=np.sqrt(np.mean(res**2))
  if rms>1.5:continue
  if any(np.linalg.norm(center-e['center'])<2 and abs(max(axes)-max(e['axes']))<2 for e in found):continue
  found.append(dict(center=center,axes=axes,R=R,rms=rms,score=rms+shift*.025+abs(2*max(axes)-diam)*.03))
 return sorted(found,key=lambda a:a['score'])[:5]

def estimate_pin(packets,seed,axis,initial=False):
 axis=np.array(axis,float);axis/=np.linalg.norm(axis);base=axis_frame(axis);radius=.00525 if initial else .0025;offset=-.01775 if initial else .01775;seedface=np.array(seed)+axis*offset;theta=np.arange(256)*2*np.pi/256;local=np.c_[radius*np.cos(theta),radius*np.sin(theta),np.zeros(len(theta))];groups=[candidates(p,seedface,radius)for p in packets]
 if any(not g for g in groups):raise RuntimeError('RGB pin end ellipse missing in calibrated view: '+str([len(g)for g in groups]))
 fits=[]
 for combo in itertools.product(*[g[:3]for g in groups]):
  def model(x):
   R=Rotation.from_rotvec(np.r_[x[3:5],0.]).as_matrix()@base;return local*x[5]@R.T+x[:3],R
  def residual(x):
   P,R=model(x);rows=[]
   for p,e in zip(packets,combo):
    uv=project(P,p);q=(uv-e['center'])@e['R'];rows.extend((np.sqrt(((q/e['axes'])**2).sum(1))-1)*min(e['axes']))
   return np.array(rows)
  x0=np.r_[seedface,0.,0.,1.];fit=least_squares(residual,x0,bounds=(x0-[.025,.025,.03,.95,.95,.4],x0+[.025,.025,.03,.95,.95,.4]),max_nfev=120,x_scale=[.01,.01,.01,.5,.5,.3],loss='soft_l1',f_scale=.5);P,R=model(fit.x);err=float(np.sqrt(np.mean(residual(fit.x)**2)));center=fit.x[:3]-R[:,2]*offset;score=err+.03*sum(e['score']for e in combo)
  fits.append((score,center,R[:,2],err,combo))
 score,center,axis,err,combo=min(fits,key=lambda v:v[0])
 if err>1.5:raise RuntimeError('RGB pin multiview ellipse reprojection failed: '+str(err))
 return dict(part='pin',center_m=center.tolist(),axis=axis.tolist(),pixel_residual_rms=err,known_end_diameter_m=2*radius,source='Calibrated RGB ellipses + known 35.5 mm pin length',uses_depth=False,uses_simulator_body_poses=False,ellipses=[dict(center=e['center'].tolist(),axes=e['axes'].tolist(),rms=float(e['rms']))for e in combo])
