"""Render actual M80/native Hand CAD and the small external-table proposal.
Uses an allocated GL context or explicit CPU OSMesa, never an implicit GPU lease.
"""
from pathlib import Path
import os,json,copy,xml.etree.ElementTree as E
os.environ.setdefault('MUJOCO_GL','osmesa')
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw,ImageFont
HERE=Path(__file__).resolve().parent;PORT=HERE.parent;OUT=Path(os.environ.get('M80_RENDER_OUT',str(HERE/'compact-renders')));OUT.mkdir(exist_ok=True)
choice=json.load(open(os.environ.get('M80_LAYOUT_JSON',str(HERE/'compact-selected.json'))));recipes=choice['recipes'];root=E.parse(PORT/'native_hand_feasibility/rendered_poses/native-hand-scene.xml').getroot();world=root.find('worldbody');parents={c:p for p in root.iter()for c in p}
for b in list(root.findall('.//body')):
 if b.get('name','')=='cap_supply_fixture'or b.get('name','').startswith('cap_stock_'):
  if b in parents and b in list(parents[b]):parents[b].remove(b)
for stock in choice.get('stock_parts',[]):
 if stock['index']==0:continue
 original=world.find("body[@name='workpiece_"+stock['part']+"']");clone=copy.deepcopy(original)
 for node in clone.iter():
  if node.get('name'):node.set('name',node.get('name')+'_'+str(stock['index']))
 world.append(clone)
def vec(x):return ' '.join(map(str,np.asarray(x).ravel()))
def quat(R):return vec(Rotation.from_matrix(R).as_quat()[[3,0,1,2]])
geom_counter=0
def geometry(spec):
 global geom_counter
 geom_counter+=1
 kw=dict(name=spec['name']+'_'+str(geom_counter),type=spec['type'],rgba=spec.get('rgba','.03 .60 .54 1'),contype='0',conaffinity='0')
 if spec['type']=='capsule':kw.update(fromto=vec(np.r_[spec['a'],spec['b']]),size=str(spec['radius']))
 elif spec['type']=='sphere':kw.update(pos=vec(spec['pos']),size=str(spec['radius']))
 else:
  kw.update(pos=vec(spec['pos']),quat=quat(np.array(spec.get('R',np.eye(3)))))
  kw['size']=vec(np.array(spec['size'])/2)if spec['type']=='box'else vec([spec['radius'],spec['length']/2])
 E.SubElement(world,'geom',**kw)
for s in choice['geometry']:
 if not s.get('collision_only'):geometry(s)
cx,cy,z=choice['table_center_m'];w,depth=choice['table_size_m']
geometry(dict(name='external_table_top',type='box',pos=[cx,cy,z-.0125],size=[w,depth,.025],rgba='.14 .25 .32 1'))
for sx in [-1,1]:
 for sy in [-1,1]:
  px=cx+sx*(w/2-.025);py=cy+sy*(depth/2-.025);geometry(dict(name='external_table_leg_'+str(sx)+'_'+str(sy),type='capsule',a=[px,py,.015],b=[px,py,z-.025],radius=.015,rgba='.13 .20 .24 1'))
# A separate base plate connects only the three holder posts, on the new table.
if choice.get('base_plate'):
 b=choice['base_plate'];geometry(dict(name='supply_base_plate',type='box',pos=b['pos'],size=b['size'],rgba='.05 .29 .29 1'))
visual=root.find('visual');visual.find('global').set('offwidth','1800');visual.find('global').set('offheight','1200');path=OUT/'compact-supply-scene.xml';E.ElementTree(root).write(path,encoding='unicode');m=mujoco.MjModel.from_xml_path(str(path));d=mujoco.MjData(m)
for side in ['left','right']:
 for j,q in enumerate(choice['common_start_q'][side],1):d.qpos[m.joint(f'{side}_arm_fr3v2_joint{j}').qposadr[0]]=q
 for finger in ['left','right']:d.qpos[m.joint(side+'_native_finger_'+finger).qposadr[0]]=.04
centers={}
for part,r in recipes.items():
 R=np.array(r['R']);Rp=R@np.array(r['localR']).T;P=np.array(r['tcp'])-Rp@np.array(r['localP']);centers[part]=P;mid=m.body('workpiece_'+part).mocapid[0];d.mocap_pos[mid]=P;d.mocap_quat[mid]=Rotation.from_matrix(Rp).as_quat()[[3,0,1,2]]
for stock in choice.get('stock_parts',[]):
 name='workpiece_'+stock['part']+('_'+str(stock['index'])if stock['index']else'');mid=m.body(name).mocapid[0];d.mocap_pos[mid]=stock['P'];d.mocap_quat[mid]=Rotation.from_matrix(np.array(stock['R'])).as_quat()[[3,0,1,2]]
for part,P in choice.get('label_centers',{}).items():centers[part]=np.array(P)
mujoco.mj_forward(m,d);renderer=mujoco.Renderer(m,height=1200,width=1800,max_geom=18000);option=mujoco.MjvOption();option.geomgroup[3]=0;option.geomgroup[4]=1;m.vis.headlight.ambient[:]=.4;m.vis.headlight.diffuse[:]=.4
FONT=PORT/'assets/NotoSansKR.ttf'
def font(n):return ImageFont.truetype(str(FONT),n)
def project(p):
 cam=renderer.scene.camera[0];v=np.array(p)-cam.pos;forward=cam.forward;up=cam.up;right=np.cross(forward,up);depth=float(v@forward)
 if depth<=0:return None
 fy=600*cam.frustum_near/cam.frustum_top;return int(900+fy*(v@right)/depth),int(600-fy*(v@up)/depth)
def render(name,look,distance,az,elev,title,subtitle,labels=True):
 cam=mujoco.MjvCamera();cam.lookat[:]=look;cam.distance=distance;cam.azimuth=az;cam.elevation=elev;renderer.update_scene(d,camera=cam,scene_option=option);im=Image.fromarray(renderer.render());dr=ImageDraw.Draw(im);dr.rectangle((0,0,1800,110),fill=(20,29,42));dr.text((30,12),title,font=font(34),fill='white');dr.text((32,65),subtitle,font=font(23),fill=(182,226,220))
 if labels:
  for part,P in centers.items():
   xy=project(P)
   if not xy:continue
   x,y=xy;offset={'ring1':(-360,-100),'ring2':(-400,65),'pin':(95,90)}[part];tx=int(np.clip(x+offset[0],20,1410));ty=int(np.clip(y+offset[1],120,1050));text=({'ring1':'① 큰 링 ×5 · 승강 매거진','ring2':'② 작은 링 ×5 · 승강 매거진','pin':'③ 핀 ×5 · 바닥 받침 포켓'if choice.get('storage_metadata',{}).get('pin',{}).get('slotted_socket')else'③ 핀 ×5 · 대칭 클램프'}if choice.get('storage_capacity')else{'ring1':'① 큰 링 · 75mm','ring2':'② 작은 링 · 62mm','pin':'③ 핀 · 분리형 클립'})[part];color={'ring1':(255,172,65),'ring2':(79,189,255),'pin':(255,124,88)}[part];dr.line((x,y,tx+20,ty+20),fill=color,width=3);dr.ellipse((x-5,y-5,x+5,y+5),fill=color);dr.rounded_rectangle((tx,ty,tx+350,ty+44),radius=8,fill=(20,29,42));dr.text((tx+12,ty+5),text,font=font(23),fill=color)
 dr.rectangle((0,1145,1800,1200),fill=(20,29,42));dr.text((28,1157),'실제 CAD 정지 렌더 · 표기 시간은 경로 시뮬레이션 예측 · 전체 물리 삽입/실기 미검증',font=font(24),fill=(245,214,152));im.save(OUT/(name+'.png'));return im
render('compact-cell-overview',[-.25,-.15,1.10],3.7,112,-20,'소형 독립 책상 · 설비 앞 공급 배치','책상 500 × 350mm · M80 부착 없음 · Franka Hand 기본 손가락')
render('compact-fixture-context',[-.27,-.20,1.35],1.2,115,-20,'5회분 · 링 10개 + 핀 5개'if choice.get('storage_capacity')else'링 2개 + 핀 1개 · 소형 받침 배치',('승강 매거진 2개 + 핀이 꽂히는 바닥 받침 포켓 5칸'if choice.get('storage_metadata',{}).get('pin',{}).get('slotted_socket')else'승강 매거진 2개 + 5칸 클램프 랙')if choice.get('storage_capacity')else'긴 막대 대신 얇은 포크 받침 · 빈 거치대와 팔 경로를 함께 검사')
original=m.geom_rgba.copy()
# Isolate the actual table/holders/workpieces, preserving their exact world poses.
for i in range(m.ngeom):
 name=m.geom(i).name or'';body=m.body(int(m.geom_bodyid[i])).name or''
 if not(name.startswith(('ring1_','ring2_','pin_','external_table_','supply_base'))or body.startswith('workpiece_')):m.geom_rgba[i,3]=0
render('compact-fixture-isolated',[cx,cy,z+.15],1.05,115,-25,'거치대 형태 · 책상 위에 독립 설치',choice.get('size_caption','책상 상판 500 × 350mm · 받침 3개'),True)
# Top and side views are useful for engineering placement review.
render('compact-fixture-top',[cx,cy,z+.15],.9,90,-89,'공급부 평면 배치 · 치수 검토',choice.get('size_caption','상판 500 × 350mm'),True)
if choice.get('storage_metadata',{}).get('pin',{}).get('slotted_socket'):
 render('compact-pin-sockets',np.asarray(centers['pin'])+[0,0,-.008],.34,115,-18,'핀 5개 · 아래에서 받치는 삽입형 포켓','하단 6mm를 포켓에 꽂음 · 양옆 손가락 창 · 먼저 10mm 들어 올린 뒤 이송',False)
m.geom_rgba[:]=original
# Show a pickup pose with the real native hand, without changing the fixture.
for part,r in (recipes.items()if choice.get('render_grasp',True)else[]):
 side=r['side']
 for s in ['left','right']:
  for j,q in enumerate(choice['common_start_q'][s],1):d.qpos[m.joint(f'{s}_arm_fr3v2_joint{j}').qposadr[0]]=q
 for j,q in enumerate(r['q_pick'],1):d.qpos[m.joint(f'{side}_arm_fr3v2_joint{j}').qposadr[0]]=q
 for finger in ['left','right']:d.qpos[m.joint(side+'_native_finger_'+finger).qposadr[0]]=r['gap']/2
 # Keep the actual Hand visible while removing occluding arm/machine meshes
 # from this detail view. The separate cell overview retains all geometry.
 for i in range(m.ngeom):
  name=m.geom(i).name or'';body=m.body(int(m.geom_bodyid[i])).name or''
  if not(name.startswith(('ring1_','ring2_','pin_','external_table_','supply_base'))or body.startswith(('workpiece_',side+'_native_'))):m.geom_rgba[i,3]=0
 mujoco.mj_forward(m,d);render('compact-'+part+'-grasp',np.array(r['tcp'])+[0,0,-.03],.40 if part!='pin'else .28,115,-22,{'ring1':'큰 링 · 기본 손가락으로 외경 파지','ring2':'작은 링 · 기본 손가락으로 외경 파지','pin':'핀 · 포켓 측면 파지 후 위로 인출'if choice.get('storage_metadata',{}).get('pin',{}).get('slotted_socket')else'핀 · 아래쪽 파지 후 클립 해제'}[part],'손가락 접촉은 개별 동역학 시험 통과 · 이 이미지는 정지 포즈',False)
 m.geom_rgba[:]=original
renderer.close();(OUT/'render-metadata.json').write_text(json.dumps(dict(renderer=os.environ['MUJOCO_GL'],cpu_software_render=True,static_render=True,machine_geometry_moved=False,fixtures_attached_to_machine=False,source='actual source M80 CAD / official Franka visuals',label=choice['label']),indent=2));print('COMPACT_RENDERED',OUT,flush=True)
