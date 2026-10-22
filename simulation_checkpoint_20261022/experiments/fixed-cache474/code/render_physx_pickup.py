"""CPU replay of measured Isaac PhysX states. No MuJoCo physics steps.

Uses the existing CAD display model only as a software renderer. The recorded
PhysX joint/part poses determine every frame; this is not an RTX recording.
"""
from pathlib import Path
import json,os,xml.etree.ElementTree as E,subprocess
os.environ.setdefault('MUJOCO_GL','osmesa')
import numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
HERE=Path(__file__).resolve().parent;PORT=HERE.parents[1];RUN=PORT/'runs/native-roas-table-03'
report=json.load(open(RUN/'physx-pickup-results.json'));spec=report['placement'];z=np.load(RUN/'physx-pickup-trace.npz')
source=PORT/'native_hand_feasibility/rendered_poses/native-hand-scene.xml';root=E.parse(source).getroot();world=root.find('worldbody')
for parent in root.iter():
    for child in list(parent):
        if child.get('name','').startswith('duo_common_'):parent.remove(child)
duo=root.find(".//body[@name='official_FR3_Duo']");duo.set('pos',' '.join(map(str,spec['duo_world_position'])))
root.find('./visual/global').set('offwidth','1280');root.find('./visual/global').set('offheight','800')
for mesh in root.findall('./asset/mesh'):
    p=Path(mesh.get('file',''))
    if mesh.get('file')and not p.is_absolute():mesh.set('file',str((source.parent/p).resolve()))
def box(name,p,size,rgba='.24 .28 .31 1'):
    E.SubElement(world,'geom',name=name,type='box',pos=' '.join(map(str,p)),size=' '.join(str(v/2)for v in size),rgba=rgba,contype='0',conaffinity='0')
cx,cy=spec['duo_world_position'][:2]
for i,sx in enumerate([-1,1]):
    for j,sy in enumerate([-1,1]):
        x=cx+sx*.31;y=cy+sy*.30;box(f'roas_post{i}{j}',[x,y,.36],[.06,.06,.6])
        E.SubElement(world,'geom',name=f'caster{i}{j}',type='cylinder',pos=f'{x} {y} .025',euler='0 1.5707963268 0',size='.025 .0125',rgba='.08 .08 .09 1',contype='0',conaffinity='0')
        box(f'caster_mount{i}{j}',[x,y,.0525],[.045,.045,.015])
for k,h in enumerate([.09,.63]):
    for i,s in enumerate([-1,1]):
        box(f'roas_x{k}{i}',[cx,cy+s*.30,h],[.56,.06,.06]);box(f'roas_y{k}{i}',[cx+s*.31,cy,h],[.06,.54,.06])
for i in range(9):
    box(f'roas_top{i}',[cx,cy-.24+i*.06,.63],[.56,.06,.06])
    if i!=4:box(f'roas_bottom{i}',[cx,cy-.24+i*.06,.09],[.56,.06,.06])
for i,s in enumerate([-1,1]):
    box(f'roas_doorx{i}',[cx,cy+s*.323,.36],[.56,.003,.47],'.76 .78 .8 1');box(f'roas_doory{i}',[cx+s*.333,cy,.36],[.003,.54,.47],'.76 .78 .8 1')
sx,sy,sz=spec['supply_table']['center'];box('supply_top',[sx,sy,sz-.0125],[.8,.15,.025],'.05 .24 .27 1')
for i,dx in enumerate([-.375,.375]):
    for j,dy in enumerate([-.05,.05]):box(f'supply_leg{i}{j}',[sx+dx,sy+dy,(sz-.025)/2],[.025,.025,sz-.025],'.05 .24 .27 1')
for i,x in enumerate([-.02,-.45]):box(f'head_extension{i}',[x,-.16,(sz+1.04)/2],[.024,.024,1.04-sz],'.1 .6 .55 1')
box('pin_post',[-.603,-.16,(sz+1.26625)/2],[.012,.012,1.26625-sz],'.05 .6 .5 1');box('pin_crossbar',[-.603,-.16,1.27025],[.030,.036,.008],'.05 .6 .5 1')
path=RUN/'physx-replay-display.xml';E.ElementTree(root).write(path,encoding='unicode')
m=mujoco.MjModel.from_xml_path(str(path));d=mujoco.MjData(m);names=report['dof_names']
indices=[]
for n in names:
    if '_arm_'in n:mn=n
    else:
        side,label=n.split('_native_');mn=side+'_native_finger_'+label
    indices.append(m.joint(mn).qposadr[0])
def assign(k):
    d.qpos[indices]=z['q'][k]
    for j,p in enumerate(['ring1','ring2','pin']):
        mid=m.body('workpiece_'+p).mocapid[0];tr=z['parts'][k,j];d.mocap_pos[mid]=tr[:3];d.mocap_quat[mid]=tr[[6,3,4,5]]
    mujoco.mj_kinematics(m,d);mujoco.mj_comPos(m,d);mujoco.mj_camlight(m,d)
validation=[]
for k in [0,len(z['time_s'])//2,len(z['time_s'])-1]:
    assign(k)
    for j,n in enumerate(report['link_names']):
        mn=n.replace('_NativeHand','_native_hand')
        if mn.endswith('_finger'):mn=mn.removesuffix('_finger').replace('_left','_native_left').replace('_right','_native_right')
        bid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,mn)
        if bid>=0:validation.append(float(np.linalg.norm(d.xpos[bid]-z['links'][k,j,:3])))
assert max(validation)<.001,('Replay differs from measured PhysX link poses',max(validation))
renderer=mujoco.Renderer(m,width=1280,height=800,max_geom=18000);opt=mujoco.MjvOption();opt.geomgroup[3]=0;opt.geomgroup[4]=1
m.vis.headlight.ambient[:]=.35;m.vis.headlight.diffuse[:]=.25
fontfile=PORT/'assets/NotoSansKR.ttf';font=lambda n:ImageFont.truetype(str(fontfile),n)
def render(k,detail=False):
    assign(k);cam=mujoco.MjvCamera();cam.lookat[:]=[-.25,-.12,1.27]if detail else[-.25,-.10,.95];cam.distance=1.22 if detail else 2.75;cam.azimuth=108;cam.elevation=-22
    renderer.update_scene(d,camera=cam,scene_option=opt);im=Image.fromarray(renderer.render());dr=ImageDraw.Draw(im)
    dr.rectangle((0,0,1280,98),fill=(20,29,42));dr.text((24,9),'Isaac PhysX 측정 결과 · Franka Hand 양손 링 파지',font=font(28),fill='white')
    dr.text((24,53),f"{z['time_s'][k]:.2f}s  · {z['phase'][k]}  · 25mm 상승 시험",font=font(20),fill=(200,226,230))
    dr.rectangle((0,751,1280,800),fill=(20,29,42));dr.text((22,763),'PhysX 기록의 CPU 시각화 · 실제 속도 1× · 전체 삽입/핀/버튼 시험은 별도',font=font(20),fill=(255,218,152));return im
ffmpeg=Path('/home/gpuadmin/workspace/jake/.isaac/lab3.0.0-rc1/packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2')
if not ffmpeg.exists():ffmpeg=Path('/usr/bin/ffmpeg')
video=RUN/'physx-native-hand-pickup-1x.mp4';fps=12
with (RUN/'replay-ffmpeg.log').open('w')as log:
    writer=subprocess.Popen([str(ffmpeg),'-y','-f','rawvideo','-pix_fmt','rgb24','-s','1280x800','-r',str(fps),'-i','-','-an','-c:v','libx264','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(video)],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=log)
    for ti in np.arange(z['time_s'][0],z['time_s'][-1],1/fps):
        k=int(np.argmin(abs(z['time_s']-ti)));writer.stdin.write(np.asarray(render(k,True)).tobytes())
    writer.stdin.close();assert writer.wait()==0
render(len(z['time_s'])-1).save(RUN/'physx-pickup-overview.png');render(len(z['time_s'])-1,True).save(RUN/'physx-pickup-detail.png');renderer.close()
(RUN/'replay-validation.json').write_text(json.dumps(dict(physics_engine='Isaac Sim Omni PhysX CPU',renderer='MuJoCo OSMesa display only',mujoco_physics_steps=0,measured_state_replay=True,max_link_position_reprojection_error_m=max(validation),speed_multiplier=1,full_task_pass=False,video=str(video)),indent=2));print('PHYSX_REPLAY_RENDERED',video,flush=True)
