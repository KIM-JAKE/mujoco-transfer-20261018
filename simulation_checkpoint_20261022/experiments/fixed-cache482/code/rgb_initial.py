"""Select accessible rings by RGB masks and calibrated basket plane (seed only)."""
import numpy as np,cv2
from rgb_pose import estimate_edges,refine_hole,project

def seeds(packet,masks,floor,normal):
 K=packet['K'];T=packet['T_cell_camera'];out=[]
 for m in sorted(masks,key=lambda a:-a['score']):
  mask=m['mask'];v,u=np.nonzero(mask)
  if len(u)<40 or len(u)>mask.size*.25:continue
  if any((mask&old['mask']).sum()/max(1,(mask|old['mask']).sum())>.45 for old in out):continue
  cont,_=cv2.findContours(mask.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE);c=max(cont,key=len)
  if len(c)<10:continue
  ell=cv2.fitEllipse(c);uv=np.array(ell[0]);ray=T[:3,:3]@np.r_[(uv-K[:2,2])/np.array([K[0,0],K[1,1]]),1.];O=T[:3,3];point=O+ray*((np.array(floor)+normal*.0075-O)@normal)/(ray@normal)
  if not(.44<point[0]<.67 and -.88<point[1]<-.55 and 1.07<point[2]<1.17):continue
  depth=((point-T[:3,3])@T[:3,:3])[2];diameter=max(ell[1])*depth/K[0,0]
  out.append(dict(center=point,mask=mask,score=m['score'],measured_diameter_m=float(diameter)))
 return out

def detect_initial(packets,masks,calibration):
 floor=np.array(calibration['floor_origin_world_m']);normal=np.array(calibration['basket_to_world_R'])[:,2]
 candidates=[]
 for p,mm in zip(packets,masks):
  for s in seeds(p,mm,floor,normal):
   if not any(np.linalg.norm(s['center']-c['center'])<.025 for c in candidates):candidates.append(s)
 # Favor rings nearest the free left side, where the native hand has clearance.
 candidates.sort(key=lambda c:c['center'][0]);chosen=[]
 for c in candidates:
  if not any(np.linalg.norm(c['center']-a['center'])<.04 for a in chosen):chosen.append(c)
  if len(chosen)==2:break
 if len(chosen)<2:raise RuntimeError('Two accessible RGB ring instances not detected')
 answers=[]
 for c in chosen:
  measured=c['measured_diameter_m'];part='ring1'if abs(measured-.075)<abs(measured-.062)else'ring2'
  fit2,_=estimate_edges(packets,masks,c['center'],normal,part)
  fit2=refine_hole(packets,masks,fit2,part);fit2.update(measured_classification_diameter_m=measured,size_classification='RGB ellipse major diameter with calibrated bin plane; nearest nominal 75/62 mm; color unused')
  answers.append(fit2)
 return answers
