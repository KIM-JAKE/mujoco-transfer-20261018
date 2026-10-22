"""Render the original three-part/button recording as native Isaac Sim USD animation.

Physics is disabled intentionally: this is an auditable trajectory replay, not
an insertion/contact-controller test. Original MuJoCo inputs remain immutable.
"""
from __future__ import annotations
import argparse,hashlib,json,math,os,sys,time,traceback
from pathlib import Path
import numpy as np

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,required=True)
args,_=parser.parse_known_args()
here=Path(__file__).resolve().parent
root=here/'generated/cap_button'
out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
status=out/'status.json';started=time.monotonic()

def record(**values):
    tmp=status.with_suffix('.tmp');tmp.write_text(json.dumps(values,indent=2)+'\n');tmp.replace(status)

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()

record(state='running',phase='app_start')
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'multi_gpu':False,'anti_aliasing':0,'width':1344,'height':720})
code=0;writer=None
try:
    import omni.usd,omni.timeline
    import omni.replicator.core as rep
    from pxr import Usd,UsdGeom,UsdLux,UsdPhysics,Gf,Sdf
    import imageio.v2 as imageio
    from PIL import Image,ImageDraw,ImageFont
    metadata=json.loads((root/'replay-source.json').read_text())
    conversion=json.loads((root/'conversion-status.json').read_text())
    contract=json.loads((root/'source.json').read_text())
    assert metadata['state']=='passed' and conversion['state']=='passed'
    assert conversion['prepared_sha256']==contract['prepared_sha256']
    assert conversion['asset_digest']==contract['asset_digest']
    assert digest(root/'replay.npz')==metadata['replay_sha256']
    workspace=here.parent.parent
    for name,sha in metadata['source_sha256'].items():
        assert digest(workspace/metadata['source_run']/name)==sha,('Source recording changed',name)
    assert digest(workspace/metadata['source_scene'])==contract['source_sha256']
    data=np.load(root/'replay.npz',allow_pickle=False)
    positions=data['body_positions'];quats=data['body_quaternions'];times=data['time_s']
    T=float(times[-1]);rate=60;fps=30
    scene_path=out/'replay.usda'
    stage=Usd.Stage.CreateNew(str(scene_path))
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage,'/World').GetPrim())
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1)
    stage.SetTimeCodesPerSecond(rate);stage.SetFramesPerSecond(rate)
    stage.SetStartTimeCode(0);stage.SetEndTimeCode(T*rate)
    cell=UsdGeom.Xform.Define(stage,'/World/Cell').GetPrim()
    cell.GetReferences().AddReference(os.path.relpath(here/conversion['usd_path'],out))
    assert 'none' in cell.GetVariantSet('Physics').GetVariantNames()
    cell.GetVariantSet('Physics').SetVariantSelection('none')
    names=metadata['body_names'];name_indices={n:i for i,n in enumerate(names) if i}
    bindings={};hidden=[]
    for prim in list(stage.Traverse()):
        if not str(prim.GetPath()).startswith('/World/Cell/Geometry/'):continue
        name=prim.GetName()
        if name in name_indices and prim.IsA(UsdGeom.Xform):
            assert name not in bindings,('Ambiguous body',name)
            index=name_indices[name]
            xf=UsdGeom.Xformable(prim);xf.ClearXformOpOrder()
            op=xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble,'recording')
            # Source world poses avoid any joint-axis/hierarchy convention change.
            xf.SetResetXformStack(True)
            bindings[name]=(index,str(prim.GetPath()),op)
        if name in metadata['geom_visibility'] and not metadata['geom_visibility'][name]:
            UsdGeom.Imageable(prim).CreateVisibilityAttr(UsdGeom.Tokens.invisible)
            hidden.append(str(prim.GetPath()))
    assert set(bindings)==set(names[1:]),('Missing source bodies',set(names[1:])-set(bindings))
    record(state='running',phase='author_animation',bodies=len(bindings))
    def matrix(position,quat):
        result=Gf.Matrix4d().SetRotate(Gf.Quatd(float(quat[0]),Gf.Vec3d(*map(float,quat[1:]))))
        result.SetTranslateOnly(Gf.Vec3d(*map(float,position)))
        return result
    for name,(i,path,op) in bindings.items():
        op.Set(matrix(positions[0,i],quats[0,i]))
        for f,t in enumerate(times):op.Set(matrix(positions[f,i],quats[f,i]),float(t)*rate)
    # No physics engine is advanced in this replay.
    assert not any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in stage.Traverse())
    sun=UsdLux.DistantLight.Define(stage,'/ReplaySun');sun.CreateIntensityAttr(1000)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-35,20,30))
    UsdLux.DomeLight.Define(stage,'/ReplayDome').CreateIntensityAttr(600)
    camera_specs=[('Overview',[-.06,-.18,1.34],3.3,55,-20,(896,640)),
                  ('Button',[.356,-.235,1.527],.47,130,-10,(448,320)),
                  ('Jigs',[-.235,.255,1.52],.64,90,-12,(448,320))]
    for name,look,distance,az,el,resolution in camera_specs:
        az=math.radians(az);el=math.radians(el)
        eye=np.asarray(look)-distance*np.array([math.cos(az)*math.cos(el),math.sin(az)*math.cos(el),math.sin(el)])
        camera=UsdGeom.Camera.Define(stage,'/Replay'+name)
        camera.CreateVerticalApertureAttr(20.25);camera.CreateHorizontalApertureAttr(20.25*1.4)
        camera.CreateFocalLengthAttr(20.25/(2*math.tan(math.radians(22.5))))
        camera.CreateClippingRangeAttr(Gf.Vec2f(.001,100))
        UsdGeom.Xformable(camera).AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*look),Gf.Vec3d(0,0,1)).GetInverse())
    stage.GetRootLayer().Save()
    # Check every recorded body pose at every source sample in the authored USD.
    max_position=0.;max_angle=0.
    for f,t in enumerate(times):
        cache=UsdGeom.XformCache(float(t)*rate)
        for name,(i,path,_) in bindings.items():
            m=cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))
            p=np.asarray(m.ExtractTranslation());q=m.ExtractRotationQuat()
            quaternion=np.r_[q.GetReal(),q.GetImaginary()]
            max_position=max(max_position,float(np.linalg.norm(p-positions[f,i])))
            max_angle=max(max_angle,float(2*np.arccos(np.clip(abs(np.dot(quaternion,quats[f,i])),0,1))))
    assert max_position<1e-8 and max_angle<1e-6,(max_position,max_angle)
    assert omni.usd.get_context().open_stage(str(scene_path))
    stage=omni.usd.get_context().get_stage()
    timeline=omni.timeline.get_timeline_interface();timeline.stop()
    timeline.set_current_time(0);app.update()
    rep.orchestrator.set_capture_on_play(False)
    products=[];annotators=[]
    for name,*_,resolution in camera_specs:
        product=rep.create.render_product('/Replay'+name,resolution)
        rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(product)
        products.append(product);annotators.append(rgb)
    font_path=str(workspace/'isaac-visuals/NotoSansKR.ttf')
    font=ImageFont.truetype(font_path,16);large=ImageFont.truetype(font_path,23)
    partial=out/'replay.partial.mp4'
    writer=imageio.get_writer(str(partial),fps=fps,codec='libx264',quality=8,macro_block_size=2,ffmpeg_params=['-movflags','+faststart','-threads','4'])
    indices=np.unique(np.r_[np.arange(0,len(times),2),len(times)-1])
    save_targets=[0,3.25,5.75,7.303,7.93,T]
    save_indices={int(indices[np.argmin(np.abs(times[indices]-t))])for t in save_targets}
    saved=[];raw_first=None;max_frame_delta=0.
    events=metadata['events']
    for j,f in enumerate(indices):
        t=float(times[f]);timeline.set_current_time(t);app.update()
        rep.orchestrator.step(delta_time=0,rt_subframes=2,pause_timeline=True)
        frames=[np.ascontiguousarray(rgb.get_data()[:,:,:3]) for rgb in annotators]
        for attempt in range(3):
            if all(a.size for a in frames):break
            rep.orchestrator.step(delta_time=0,rt_subframes=2,pause_timeline=True)
            frames=[np.ascontiguousarray(rgb.get_data()[:,:,:3]) for rgb in annotators]
        assert all(a.shape==(resolution[1],resolution[0],3) for a,(*_,resolution) in zip(frames,camera_specs)),[(a.shape,a.std() if a.size else None) for a in frames]
        # An arm can fill a close-up with a single color. The overview must
        # remain nonuniform; a valid occluded detail is still recorded honestly.
        assert frames[0].std()>1,('Blank overview',j,t)
        if raw_first is None:raw_first=frames[0].copy()
        max_frame_delta=max(max_frame_delta,float(np.mean(np.abs(frames[0].astype(float)-raw_first))))
        canvas=Image.new('RGB',(1344,720),(18,24,32))
        for a,xy in zip(frames,[(0,48),(896,48),(896,368)]):canvas.paste(Image.fromarray(a),xy)
        draw=ImageDraw.Draw(canvas)
        green=t>=T-1e-9
        installed=sum(t>=v-1e-9 for v in metadata['release_times_s'].values())
        phase='GREEN (RECORDED EVENT)' if green else 'BUTTON PRESS / HOLD' if t>=events['button_first_contact'] else '3/3 RELEASED -> BUTTON' if t>=events['all_three_installed'] else 'PICK / TRANSFER / INSERT'
        draw.text((14,10),'ISAAC SIM | M80: 3 PARTS -> BUTTON',font=large,fill=(225,234,245))
        draw.text((785,14),phase,font=font,fill=(80,255,120)if green else'white')
        for y,label in [(52,'BUTTON | RECORDED 2 mm TRAVEL'),(372,f'ASSEMBLY JIGS | {installed}/3 RELEASED')]:
            draw.rectangle((900,y,1340,y+28),fill=(18,24,32));draw.text((908,y+4),label,font=font,fill='white')
        if green:draw.ellipse((1313,88,1335,110),fill=(35,245,95))
        draw.rectangle((0,688,1344,720),fill=(18,24,32))
        draw.text((14,696),'1x ORIGINAL KINEMATIC REPLAY | RTX render | insertion forces / physical switch unverified',font=font,fill=(220,226,233))
        draw.text((1190,693),f'{t:.3f} s',font=large,fill='white')
        writer.append_data(np.asarray(canvas))
        if f in save_indices:
            path=out/f'frame-{t:.3f}s.png';canvas.save(path);saved.append(path)
        if j%15==0:
            record(state='running',phase='rendering',frame=j,frames=len(indices),time_s=t)
            print('REPLAY_FRAME',j,len(indices),t,flush=True)
    assert max_frame_delta>1,('Renderer did not visibly animate',max_frame_delta)
    for _ in range(fps):writer.append_data(np.asarray(canvas))
    writer.close();writer=None;partial.replace(out/'m80-three-parts-button-isaac.mp4')
    for a,p in zip(annotators,products):a.detach(p);p.destroy()
    sheet=Image.new('RGB',(1344,math.ceil(len(saved)/3)*240),(18,24,32))
    for j,p in enumerate(saved):sheet.paste(Image.open(p).resize((448,240)),((j%3)*448,(j//3)*240))
    sheet.save(out/'contact-sheet.png')
    frames_decoded=0
    reader=imageio.get_reader(str(out/'m80-three-parts-button-isaac.mp4'))
    video_metadata=reader.get_meta_data()
    for frame in reader:assert frame.shape==(720,1344,3);frames_decoded+=1
    reader.close();assert frames_decoded==len(indices)+fps
    record(state='passed',mode='kinematic_recording_replay',renderer='Isaac Sim RTX',physics_variant='none',
        source_run=metadata['source_run'],source_sha256=metadata['source_sha256'],replay_sha256=metadata['replay_sha256'],
        source_frames=len(times),rendered_cycle_frames=len(indices),decoded_frames=frames_decoded,fps=fps,
        duration_s=T,video_duration_s=frames_decoded/fps,final_hold_s=1,events=events,release_times_s=metadata['release_times_s'],
        pose_audit={'samples':len(times)*len(bindings),'body_count':len(bindings),'max_position_error_m':max_position,'max_rotation_error_rad':max_angle,'passed':True},
        hidden_source_geom_count=len(hidden),rendered_max_frame_delta=max_frame_delta,
        video_sha256=digest(out/'m80-three-parts-button-isaac.mp4'),physical_assembly_pass=False,physical_switch_actuation_verified=False,
        scope=metadata['scope'],seconds=time.monotonic()-started)
    print('ISAAC_BUTTON_REPLAY_PASS',status,flush=True)
except Exception as exc:
    traceback.print_exc();record(state='failed',error=repr(exc),seconds=time.monotonic()-started);code=1
finally:
    if writer is not None:writer.close()
    app.close(exit_code=code)
sys.exit(code)
