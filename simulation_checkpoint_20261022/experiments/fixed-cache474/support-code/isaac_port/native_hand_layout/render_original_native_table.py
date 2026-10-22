"""CPU static views of the unchanged archived supply and independent table."""
import os
os.environ.setdefault('MUJOCO_GL','osmesa')
from pathlib import Path
import json,hashlib,xml.etree.ElementTree as E
import numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont

HERE=Path(__file__).resolve().parent;PORT=HERE.parent
OUT=HERE/'original-9s-native-table';OUT.mkdir(exist_ok=True)
source=PORT/'native_hand_feasibility/rendered_poses/native-hand-scene.xml'
root=E.parse(source).getroot();world=root.find('worldbody')
E.SubElement(world,'geom',name='independent_table_top',type='box',pos='-.315 -.28 1.0275',size='.4 .175 .0125',rgba='.08 .25 .29 1',contype='0',conaffinity='0')
for x in [-.69,.06]:
 for y in [-.43,-.13]:
  E.SubElement(world,'geom',type='capsule',fromto=f'{x} {y} .015 {x} {y} 1.015',size='.015',rgba='.08 .25 .29 1',contype='0',conaffinity='0')
E.SubElement(world,'geom',name='independent_pin_bank_post',type='box',pos='-.603 -.16 1.157125',size='.006 .006 .117125',rgba='.05 .65 .55 1',contype='0',conaffinity='0')
root.find('./visual/global').set('offwidth','1600');root.find('./visual/global').set('offheight','1000')
for m in root.findall('./asset/mesh'):
 p=Path(m.get('file',''))
 if not p.is_absolute() and m.get('file'):m.set('file',str((source.parent/p).resolve()))
path=OUT/'original-native-table-scene.xml';E.ElementTree(root).write(path,encoding='unicode')
m=mujoco.MjModel.from_xml_path(str(path));d=mujoco.MjData(m)
old=np.load(PORT/'runs/contact-trim-03/arm-posture-fr3.npz')
for side in ['left','right']:
 for j,q in enumerate(old[side+'_qpos'][0],1):d.qpos[m.joint(f'{side}_arm_fr3v2_joint{j}').qposadr[0]]=q
 for f in ['left','right']:d.qpos[m.joint(side+'_native_finger_'+f).qposadr[0]]=.04
mujoco.mj_forward(m,d)
option=mujoco.MjvOption();option.geomgroup[3]=0;option.geomgroup[4]=1
m.vis.headlight.ambient[:]=.35;m.vis.headlight.diffuse[:]=.25
renderer=mujoco.Renderer(m,height=1000,width=1600,max_geom=18000)
fontfile=PORT/'assets/NotoSansKR.ttf'
def font(n):return ImageFont.truetype(str(fontfile),n)
def render(name,look,dist,az,el,title,subtitle):
 cam=mujoco.MjvCamera();cam.lookat[:]=look;cam.distance=dist;cam.azimuth=az;cam.elevation=el
 renderer.update_scene(d,camera=cam,scene_option=option)
 im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im)
 draw.rectangle((0,0,1600,100),fill=(20,29,42));draw.text((25,12),title,font=font(29),fill='white');draw.text((27,58),subtitle,font=font(21),fill=(180,220,215))
 draw.rectangle((0,955,1600,1000),fill=(20,29,42));draw.text((25,964),'실제 CAD 정지 렌더 · 기존 부품 위치 유지 · 전체 동적 조립 성공/실기 시간 미검증',font=font(21),fill=(245,215,155))
 im.save(OUT/(name+'.png'))
render('original-native-table-overview',[-.28,-.12,1.25],3.3,112,-23,'9.47초 버전의 공급 위치 + Franka Hand','독립 책상 800×350mm · 상판 높이 1.04m · 링/핀 기준 위치 높이 1.30m')
original=m.geom_rgba.copy()
for i in range(m.ngeom):
 name=m.body(int(m.geom_bodyid[i])).name
 if name=='m80' or name.startswith(('left_','right_','official_FR3_Duo')):m.geom_rgba[i,3]=0.
render('original-native-table-supply',[-.32,-.16,1.22],1.3,100,-20,'원래 거치 형태: 링은 수직 봉, 핀은 소켓','링 공급 위치 간격 430mm · 원래 받침 유지 · 아래 책상과 핀 받침만 추가')
m.geom_rgba[:]=original
renderer.close()
(OUT/'render-metadata.json').write_text(json.dumps(dict(static_render=True,physics_stepped=False,source_native_scene=str(source),source_native_scene_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),archived_supply_positions_preserved=True,franka_standard_hand=True,table_dimensions_m=[.8,.35,1.04],machine_attachment=False),indent=2))
print('ORIGINAL_NATIVE_RENDER_DONE',OUT,flush=True)
