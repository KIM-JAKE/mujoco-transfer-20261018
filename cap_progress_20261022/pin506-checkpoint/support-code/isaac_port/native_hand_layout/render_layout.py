"""Actual MuJoCo static CAD renders of the proposed independent supply stand."""
from pathlib import Path
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,sys,xml.etree.ElementTree as E
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw,ImageFont
HERE=Path(__file__).resolve().parent;PORT=HERE.parent;OUT=HERE/'renders';OUT.mkdir(exist_ok=True)
FONT=PORT/'assets/NotoSansKR.ttf'
def font(n):return ImageFont.truetype(str(FONT),n)
def vec(x):return ' '.join(map(str,np.asarray(x).ravel()))
def quat(R):return vec(Rotation.from_matrix(R).as_quat()[[3,0,1,2]])
choice=json.load(open(HERE/'selected-layout.json'));recipe=choice['recipes'];frames=choice['support_frames'];root=E.parse(PORT/'native_hand_feasibility/rendered_poses/native-hand-scene.xml').getroot();world=root.find('worldbody')
# Remove the superseded assumed stock fixtures, preserving all M80 geometry.
parents={c:p for p in root.iter()for c in p}
for b in list(root.findall('.//body')):
 if b.get('name','')=='cap_supply_fixture' or b.get('name','').startswith('cap_stock_'):
  if b in parents and b in list(parents[b]):parents[b].remove(b)
colors={'ring1':'.98 .48 .06 1','ring2':'.05 .58 .95 1','pin':'.90 .22 .10 1'};teal='.03 .65 .60 1';dark='.04 .28 .30 1'
for frame in frames:
 part=frame['part'];r=recipe[part];R=np.array(r['R']);Rp=R@np.array(r['localR']).T;P=np.array(r['tcp'])-Rp@np.array(r['localP']);best=frame['best'];nodes=np.array(best['nodes'])
 for i,(a,b)in enumerate(zip(nodes[:-1],nodes[1:])):
  if np.linalg.norm(a-b)<1e-6:continue
  rad=.02 if i==len(nodes)-2 else .004 if np.linalg.norm(a-nodes[0])<.25 else .0125
  E.SubElement(world,'geom',name=part+'_independent_tube_'+str(i),type='capsule',fromto=vec(np.r_[a,b]),size=str(rad),rgba=teal,contype='0',conaffinity='0')
 anchor=np.array(best['anchor']);# A shared weighted floor plate is drawn below all branch masts.
 # Head color denotes the part; the frame remains a separate floor-supported body.
 if part.startswith('ring'):
  axis=R[:,0].copy();axis[2]=0;axis/=np.linalg.norm(axis);angle=best.get('fork_angle_deg') or 0;axis=Rotation.from_euler('z',angle,degrees=True).apply(axis);cross=np.cross([0,0,1],axis);basis=np.column_stack([axis,cross,[0,0,1]]);rad=.0375 if part=='ring1'else .031;half=.00775 if part=='ring1'else .007
  for sign in [-1,1]:
   p=P-axis*.02+cross*(sign*rad*.7)+[0,0,-half-.001];E.SubElement(world,'geom',type='box',pos=vec(p),quat=quat(basis),size=vec([rad+.025,.003,.001]),rgba=teal,contype='0',conaffinity='0')
  backbone=P-axis*(rad+.045)+[0,0,-half-.001];E.SubElement(world,'geom',type='box',pos=vec(backbone),quat=quat(basis),size=vec([.004,rad*.7+.004,.002]),rgba=teal,contype='0',conaffinity='0');E.SubElement(world,'geom',type='capsule',fromto=vec(np.r_[nodes[0],backbone]),size='.003',rgba=teal,contype='0',conaffinity='0')
 else:
  for sign in [-1,1]:
   E.SubElement(world,'geom',type='box',pos=vec(P+Rp@np.array([sign*.0048,-.00025,.004])),quat=quat(Rp),size='.001 .004 .004',rgba=teal,contype='0',conaffinity='0')
  E.SubElement(world,'geom',type='cylinder',pos=vec(P+Rp@np.array([0,-.00025,-.01875])),quat=quat(Rp),size='.0024 .001',rgba=teal,contype='0',conaffinity='0')
  E.SubElement(world,'geom',type='box',pos=vec(P+[0,0,.033]),size='.015 .008 .008',rgba=teal,contype='0',conaffinity='0')
anchors=np.array([f['best']['anchor'] for f in frames]);xmin=anchors[:,0].min()-.175;xmax=anchors[:,0].max()+.175
E.SubElement(world,'geom',name='shared_weighted_floor_base',type='box',pos=vec([(xmin+xmax)/2,-1.15,.02]),size=vec([(xmax-xmin)/2,.175,.02]),rgba=dark,contype='0',conaffinity='0')
visual=root.find('visual');visual.find('global').set('offwidth','1800');visual.find('global').set('offheight','1200');path=OUT/'independent-supply-scene.xml';E.ElementTree(root).write(path,encoding='unicode');m=mujoco.MjModel.from_xml_path(str(path));d=mujoco.MjData(m)
for side in ['left','right']:
 for j,val in enumerate(choice['common_start_q'][side],1):d.qpos[m.joint(f'{side}_arm_fr3v2_joint{j}').qposadr[0]]=val
 for finger in ['left','right']:d.qpos[m.joint(side+'_native_finger_'+finger).qposadr[0]]=.04
for part,r in recipe.items():
 R=np.array(r['R']);Rp=R@np.array(r['localR']).T;P=np.array(r['tcp'])-Rp@np.array(r['localP']);mid=m.body('workpiece_'+part).mocapid[0];d.mocap_pos[mid]=P;d.mocap_quat[mid]=Rotation.from_matrix(Rp).as_quat()[[3,0,1,2]]
mujoco.mj_forward(m,d);renderer=mujoco.Renderer(m,height=1200,width=1800,max_geom=18000);option=mujoco.MjvOption();option.geomgroup[3]=0;option.geomgroup[4]=1
m.vis.headlight.ambient[:]=.3;m.vis.headlight.diffuse[:]=.25;m.vis.headlight.specular[:]=.08

def project(p):
 cam=renderer.scene.camera[0];v=np.array(p)-cam.pos;forward=cam.forward;up=cam.up;right=np.cross(forward,up);depth=float(v@forward)
 if depth<=0:return None
 fy=1200/2*cam.frustum_near/cam.frustum_top;return (int(900+fy*(v@right)/depth),int(600-fy*(v@up)/depth))
def render(name,look,distance,az,elev,title,labels=True):
 cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE;cam.lookat[:]=look;cam.distance=distance;cam.azimuth=az;cam.elevation=elev;renderer.update_scene(d,camera=cam,scene_option=option);im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im);draw.rectangle((0,0,1800,110),fill=(20,29,42));draw.text((30,14),title,font=font(34),fill='white');draw.text((32,64),'Franka Hand 기본 손가락 · 설비에 부착하지 않는 바닥 독립형 공급 거치대',font=font(23),fill=(185,225,220))
 if labels:
  for part,r in recipe.items():
   R=np.array(r['R']);Rp=R@np.array(r['localR']).T;P=np.array(r['tcp'])-Rp@np.array(r['localP']);xy=project(P)
   if xy:
    x,y=xy;label={'ring1':'① 큰 링 · 외경 75mm','ring2':'② 작은 링 · 외경 62mm','pin':'③ 핀 · 위치결정 클립'}[part];color={'ring1':(255,164,64),'ring2':(72,182,255),'pin':(255,120,90)}[part];offset={'ring1':(-350,-90),'ring2':(-350,80),'pin':(100,100)}[part];tx=int(np.clip(x+offset[0],25,1400));ty=int(np.clip(y+offset[1],125,1050));draw.line((x,y,tx+20,ty+20),fill=color,width=3);draw.ellipse((x-5,y-5,x+5,y+5),fill=color);draw.rounded_rectangle((tx,ty,tx+350,ty+43),radius=7,fill=(22,32,45));draw.text((tx+10,ty+5),label,font=font(23),fill=color)
 draw.rectangle((0,1145,1800,1200),fill=(20,29,42));draw.text((28,1156),'배치·형상 검토용 정지 렌더 | 시간은 경로 시뮬레이션 예측값 · 전체 물리 조립/실기 성공 미검증',font=font(23),fill=(245,215,155));im.save(OUT/(name+'.png'));return im
render('cell-overview',[-.25,-.35,1.25],3.65,112,-23,'독립 공급 거치대 · 전체 셀 배치')
render('supply-closeup',[-.20,-.04,1.43],1.15,110,-24,'링 2개 + 핀 1개 · 막대 없이 꺼내는 공급부')
# Show support construction with a transparent M80, without moving its geometry.
original=m.geom_rgba.copy();machine=m.body('m80').id;m.geom_rgba[m.geom_bodyid==machine,3]=.10
for side in ['left','right']:
 for i in range(m.ngeom):
  if (mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,int(m.geom_bodyid[i]))or'').startswith(side+'_'):m.geom_rgba[i,3]=.13
render('independent-frame-cutaway',[-.25,-.45,1.20],3.25,120,-20,'지지 구조 확인 · 설비와 분리된 바닥 베이스',False);m.geom_rgba[:]=original
# Isolate each head with the actual CAD workpiece and native hand pose.
for part,r in recipe.items():
 for side in ['left','right']:
  for j,val in enumerate(choice['common_start_q'][side],1):d.qpos[m.joint(f'{side}_arm_fr3v2_joint{j}').qposadr[0]]=val
 side=r['side']
 for j,val in enumerate(r['q_pick'],1):d.qpos[m.joint(f'{side}_arm_fr3v2_joint{j}').qposadr[0]]=val
 for finger in ['left','right']:d.qpos[m.joint(side+'_native_finger_'+finger).qposadr[0]]=r['gap']/2
 mujoco.mj_forward(m,d);render(part+'-head',np.array(r['tcp'])+[0,0,-.025],.35 if part!='pin'else .22,120,-20,{'ring1':'큰 링 · 얇은 2열 포크 받침','ring2':'작은 링 · 얇은 2열 포크 받침','pin':'핀 · 머리 클립 + 끝단 위치결정'}[part],False)
renderer.close();(OUT/'render-validation.json').write_text(json.dumps(dict(renderer='MuJoCo EGL',static_pose_render=True,physics_stepped=False,source_world_coordinates_m=True,machine_geometry_moved=False,old_assumed_supply_fixture_removed=True,selected_layout=choice['label']),indent=2));print('RENDERED',OUT,flush=True)
