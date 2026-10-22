"""Small five-cycle storage: indexed ring columns and passive pin shoulder forks.
The geometric concept is independent of the M80; actuator housing is an envelope.
"""
from pathlib import Path
import json,copy,math,os
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent;PORT=HERE.parent

def ring_specs(part,P,R,top=1.1):
 P=np.array(P);radius=.0375 if part=='ring1'else .031;half=.00775 if part=='ring1'else .007;pitch=2*half;cross=R[:,1].copy();cross[2]=0;cross/=np.linalg.norm(cross);angle=np.arctan2(cross[1],cross[0]);base=P+[0,0,-4*pitch-half-.105];spec=[]
 def box(name,p,size):spec.append(dict(name=part+'_'+name,type='box',pos=np.array(p).tolist(),size=size,R=np.eye(3).tolist()))
 # Motor/lead-screw envelope remains below the ring stack, away from the hand.
 box('lift_housing',base+[0,0,.05],[.042,.042,.10]);box('cassette_base',base,[2*radius+.02,2*radius+.02,.008]);spec.append(dict(name=part+'_post',type='capsule',a=(base+[0,0,-.004]).tolist(),b=[P[0],P[1],top+.004],radius=.008))
 z0=P[2]-4*pitch-half-.006;z1=P[2]+half+.005
 for k,offset in enumerate([45,135,270]):
  theta=angle+np.deg2rad(offset);xy=P[:2]+(radius+.0025)*np.array([np.cos(theta),np.sin(theta)]);spec.append(dict(name=part+'_guide_'+str(k),type='capsule',a=[*xy,z0],b=[*xy,z1],radius=.002,friction=.2))
 # Conservative collision envelope includes all elevator plate positions and
 # the stored rings below the exposed pickup level, including after depletion.
 low=P[2]-4*pitch-half-.008;high=P[2]-half
 envelope=dict(name=part+'_cassette_keepout',type='cylinder',pos=[P[0],P[1],(low+high)/2],R=np.eye(3).tolist(),radius=radius+.007,length=high-low)
 return spec,envelope,dict(capacity=5,pitch_m=pitch,lift_stroke_m=4*pitch,plate_initial_center=(P+[0,0,-4*pitch-half-.003]).tolist(),plate_radius_m=radius+.001,plate_thickness_m=.006,top_part_center_m=P.tolist(),motor_force_limit_n=20,index_duration_s=.4)

def passive_pin_specs(P,R,pitch=.04):
 P=np.array(P);row=R[:,1].copy();row[2]=0;row/=np.linalg.norm(row);axis=R[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis)
 if axis[1]>0:axis=-axis
 B=np.column_stack([axis,row,[0,0,1]])
 if np.linalg.det(B)<0:B[:,1]*=-1
 spec=[];centers=[]
 for i in range(5):
  C=P+row*(i-2)*pitch;centers.append(C.tolist())
  for sign in [-1,1]:spec.append(dict(name='pin_neck_guide_'+str(i)+'_'+str(sign),type='box',pos=(C-axis*.006+row*sign*.0055+[0,0,.004]).tolist(),size=[.028,.002,.003],R=B.tolist()))
  for sign in [-1,1]:spec.append(dict(name='pin_passive_detent_'+str(i)+'_'+str(sign),type='sphere',pos=(C+axis*.003+row*sign*.0042+[0,0,.004]).tolist(),radius=.0015,passive_gate=True,slide_axis=(row*sign).tolist(),spring_n_m=200))
  spec.append(dict(name='pin_rear_locating_ball_'+str(i),type='sphere',pos=(C-axis*.0055+[0,0,.004]).tolist(),radius=.0015))
  spec.append(dict(name='pin_axial_datum_'+str(i),type='cylinder',pos=(C+[0,-.00025,-.01875]).tolist(),R=np.eye(3).tolist(),radius=.0024,length=.002))
  end=C-axis*.023+[0,0,-.01975]
  for label,a,b,rad in [('datum_link',C+[0,-.00025,-.01975],end,.0015),('rear_upright',end,C-axis*.023+[0,0,.004],.002)]:spec.append(dict(name='pin_'+label+'_'+str(i),type='capsule',a=np.array(a).tolist(),b=np.array(b).tolist(),radius=rad))
 beam=P-axis*.023+[0,0,.004];spec.append(dict(name='pin_shared_backbeam',type='box',pos=beam.tolist(),size=[.008,4*pitch+.022,.006],R=B.tolist()));spec.append(dict(name='pin_post',type='capsule',a=(beam+[0,0,-.003]).tolist(),b=[beam[0],beam[1],1.104],radius=.008))
 return spec,centers,dict(capacity=5,pitch_m=pitch,flange_supported=False,axial_end_datum=True,passive_neck_guide=True,active_stock_clips=False,pre_extraction_lift_m=.003,extraction_axis=axis.tolist(),extraction_distance_m=.03)

def part_pose(recipe):
 R=np.array(recipe['R']);Rp=R@np.array(recipe['localR']).T;P=np.array(recipe['tcp'])-Rp@np.array(recipe['localP']);return P,Rp


def clamped_pin_specs(P,R,pitch=None):
 if pitch is None:pitch=float(os.environ.get('M80_PIN_PITCH','.065'))
 P=np.array(P);row=R[:,1].copy();row[2]=0;row/=np.linalg.norm(row);axis=R[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis)
 if axis[1]>0:axis=-axis
 B=np.column_stack([axis,row,[0,0,1]])
 if np.linalg.det(B)<0:B[:,1]*=-1
 spec=[];centers=[]
 for i in range(5):
  C=P+row*(i-2)*pitch;centers.append(C.tolist())
  for sign in [-1,1]:
   spec.append(dict(name='pin_active_jaw_sweep_'+str(i)+'_'+str(sign),type='box',pos=(C+[sign*.0088,-.00025,.004]).tolist(),size=[.010,.008,.008],R=np.eye(3).tolist(),collision_only=True))
   spec.append(dict(name='pin_active_jaw_'+str(i)+'_'+str(sign),type='box',pos=(C+[sign*.0048,-.00025,.004]).tolist(),size=[.002,.008,.008],R=np.eye(3).tolist(),render_only=True))
   spec.append(dict(name='pin_actuator_housing_'+str(i)+'_'+str(sign),type='box',pos=(C+[sign*.023,0,.014]).tolist(),size=[.014,.012,.012],R=np.eye(3).tolist()))
  spec.append(dict(name='pin_roof_'+str(i),type='box',pos=(C+[0,0,.025]).tolist(),size=[.060,.012,.006],R=np.eye(3).tolist()))
  spec.append(dict(name='pin_top_stop_sweep_'+str(i),type='box',pos=(C+[0,-.00025,.01875]).tolist(),size=[.022,.0048,.002],R=np.eye(3).tolist(),collision_only=True))
  for sign in [-1,1]:
   spec.append(dict(name='pin_top_stop_'+str(i)+'_'+str(sign),type='box',pos=(C+[sign*.002,-.00025,.01875]).tolist(),size=[.002,.0048,.002],R=np.eye(3).tolist(),render_only=True))
   for corner in [-1,1]:
    spec.append(dict(name='pin_vjaw_tip_'+str(i)+'_'+str(sign)+'_'+str(corner),type='sphere',pos=(C+[sign*.004,corner*.004-.00025,.004]).tolist(),radius=.0015,render_only=True))
    spec.append(dict(name='pin_vjaw_sweep_'+str(i)+'_'+str(sign)+'_'+str(corner),type='box',pos=(C+[sign*.008,corner*.004-.00025,.004]).tolist(),size=[.011,.003,.003],R=np.eye(3).tolist(),collision_only=True))
  datum=C+[0,-.00025,-.01875];spec.append(dict(name='pin_datum_'+str(i),type='cylinder',pos=datum.tolist(),R=np.eye(3).tolist(),radius=.0024,length=.002))
  end=C-axis*.020+[0,0,-.01975]
  for label,a,b,rad in [('datum_link',datum+[0,0,-.001],end,.0015),('rear_upright',end,C-axis*.020+[0,0,.03],.002),('roof_link',C+[0,0,.03],C-axis*.020+[0,0,.03],.003)]:spec.append(dict(name='pin_'+label+'_'+str(i),type='capsule',a=np.array(a).tolist(),b=np.array(b).tolist(),radius=rad))
 beam=P-axis*.020+[0,0,.030];spec.append(dict(name='pin_shared_backbeam',type='box',pos=beam.tolist(),size=[.008,4*pitch+.064,.008],R=B.tolist()))
 # Put the single support in the centre gap behind the pickup direction.
 # A post directly behind the middle pin obstructed the small-ring fingertip.
 post=P+[.030,.070,.030];spec.append(dict(name='pin_beam_to_post',type='capsule',a=beam.tolist(),b=post.tolist(),radius=.004));spec.append(dict(name='pin_post',type='capsule',a=post.tolist(),b=[post[0],post[1],1.104],radius=.008))
 return spec,centers,dict(capacity=5,pitch_m=pitch,active_stock_clips=True,clip_force_cap_n=20,clip_open_wait_s=.15,pre_extraction_lift_m=.003,extraction_axis=axis.tolist(),extraction_distance_m=.10)

def socket_pin_specs(P,R,pitch=None):
 """Five bottom-bearing slotted sockets; no suspended pins or overhead clamp.

 Opposed finger windows leave the native lower grasp band accessible. The
 bottom and two walls form a U pocket into which the pin is seated by 6 mm.
 """
 if pitch is None:pitch=float(os.environ.get('M80_PIN_PITCH','.035'))
 P=np.asarray(P);R=np.asarray(R);row=R[:,1].copy();row[2]=0;row/=np.linalg.norm(row);other=np.cross([0,0,1],row);B=np.column_stack([row,other,[0,0,1]])
 back=R[:,2].copy();back[2]=0;back/=np.linalg.norm(back)
 band=json.load(open(HERE/'compact-pin-band.json'));L=np.asarray(band['localR']);Rp=R@L.T
 vertices=np.load(PORT/'generated/contact_task/precision-pin-main.npz')['instance_0557_vertices'];relative=vertices@Rp.T@B
 section=relative[(relative[:,2]>=-.01775)&(relative[:,2]<=-.01175)]
 low=float(section[:,1].min())-.00035;high=float(section[:,1].max())+.00035
 spec=[];centers=[]
 def box(name,C,delta,size):spec.append(dict(name=name,type='box',pos=(C+B@np.asarray(delta)).tolist(),size=size,R=B.tolist()))
 for i in range(5):
  C=P+row*(i-2)*pitch;centers.append(C.tolist())
  box('pin_socket_floor_'+str(i),C,[0,(low+high)/2,-.01875],[.006,high-low+.003,.002])
  for side,y in [('front',low-.00075),('back',high+.00075)]:box('pin_socket_wall_'+side+'_'+str(i),C,[0,y,-.01475],[.006,.0015,.006])
  end=C+back*.025+[0,0,-.030]
  spec.append(dict(name='pin_socket_stem_'+str(i),type='capsule',a=(C+[0,0,-.01975]).tolist(),b=end.tolist(),radius=.0015))
 beam=P+back*.025+[0,0,-.030];spec.append(dict(name='pin_socket_backbeam',type='box',pos=beam.tolist(),size=[4*pitch+.018,.008,.008],R=B.tolist()))
 post=P+back*.065+row*.030+[0,0,-.030]
 spec.append(dict(name='pin_socket_beam_link',type='capsule',a=beam.tolist(),b=post.tolist(),radius=.004))
 spec.append(dict(name='pin_post',type='capsule',a=post.tolist(),b=[post[0],post[1],1.104],radius=.008))
 return spec,centers,dict(capacity=5,pitch_m=pitch,bottom_supported=True,slotted_socket=True,socket_depth_m=.006,socket_finger_window=True,active_stock_clips=False,pre_extraction_lift_m=.010,extraction_axis=(-back).tolist(),extraction_distance_m=.04,concept_only=True)

def pin_specs(P,R,pitch=None):
 return socket_pin_specs(P,R,pitch)if os.environ.get('M80_PIN_STORAGE')=='socket'else clamped_pin_specs(P,R,pitch)
