"""Actual recorded contact-bench states, without integrating or editing poses."""
from pathlib import Path
import os,json
os.environ.setdefault('MUJOCO_GL','osmesa')
import mujoco,numpy as np
from PIL import Image,ImageDraw,ImageFont
HERE=Path(__file__).resolve().parent
font=lambda n:ImageFont.truetype(str(HERE.parent/'assets/NotoSansKR.ttf'),n)
canvas=Image.new('RGB',(1800,1120),(20,29,42));draw=ImageDraw.Draw(canvas)
draw.text((25,12),'5회분 저장 · 기본 Franka Hand로 15개 개별 인출',font=font(32),fill='white')
draw.text((25,62),'MuJoCo 접촉 시험 기록 · 위: 첫 번째 파지 직전 / 아래: 다섯 번째 인출 후',font=font(23),fill=(182,226,220))
for col,(part,folder,title) in enumerate([('ring1','five-cycle-converged-ring1','큰 링 5/5'),('ring2','five-cycle-converged-ring2','작은 링 5/5'),('pin','five-cycle-socket-pin','포켓형 핀 5/5 · 간격 35mm')]):
 j=json.load(open(HERE/folder/(part+'-five-picks.json')));m=mujoco.MjModel.from_xml_path(j['scene']);d=mujoco.MjData(m)
 m.vis.global_.offwidth=600;m.vis.global_.offheight=440
 renderer=mujoco.Renderer(m,height=440,width=600);m.vis.headlight.ambient[:]=.4
 qadr=m.joint('stored_joint_0').qposadr[0];center=m.qpos0[qadr:qadr+3].copy()
 if part=='pin':center=np.mean([m.qpos0[m.joint('stored_joint_'+str(i)).qposadr[0]:m.joint('stored_joint_'+str(i)).qposadr[0]+3]for i in range(5)],axis=0)
 for row,(index,t) in enumerate([(0,.65),(4,2.25 if part=='pin'else 1.9)]):
  s=min(j['picks'][index]['states'],key=lambda v:abs(v['t']-t));d.qpos[:]=s['qpos'];mujoco.mj_forward(m,d)
  cam=mujoco.MjvCamera();cam.lookat[:]=center+[0,0,.01 if part=='pin'else .025];cam.distance=.65 if part=='pin'else .44;cam.azimuth=115;cam.elevation=-20
  renderer.update_scene(d,camera=cam);im=Image.fromarray(renderer.render());canvas.paste(im,(col*600,120+row*470));draw.text((col*600+16,125+row*470),title+(' · 시작'if row==0 else' · 마지막 인출'),font=font(24),fill=(255,220,170))
 renderer.close()
draw.rectangle((0,1060,1800,1120),fill=(20,29,42));draw.text((25,1071),'저장/인출만 검증 · 팔 전체 동역학 및 M80 삽입 미검증 · 핀 인출 후 회전오차 약 6.2°',font=font(24),fill=(245,214,152))
canvas.save(HERE/'five-storage-contact-evidence.png');print('STORAGE_EVIDENCE_RENDERED',flush=True)
