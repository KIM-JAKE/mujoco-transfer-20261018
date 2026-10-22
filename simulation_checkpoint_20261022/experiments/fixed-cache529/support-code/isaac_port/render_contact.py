"""RTX rendering of measured PhysX states, including free parts and fixture jaws."""
from pathlib import Path
import argparse,json,math,hashlib,time,traceback,sys,os
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--stride',type=int,default=1);p.add_argument('--output',type=Path);p.add_argument('--xray',action='store_true');p.add_argument('--cycle-only',action='store_true');p.add_argument('--preview-time',type=float);a=p.parse_args()
run=a.run.resolve();out=a.output.resolve()if a.output else run/'render';out.mkdir(exist_ok=True);start=time.monotonic()
def save(**v):(out/'status.json').write_text(json.dumps(v,indent=2)+'\n')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
save(state='running',phase='startup')
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'multi_gpu':False,'anti_aliasing':0,'width':1344,'height':720});writer=None;code=0
try:
 import omni.usd,omni.timeline,omni.replicator.core as rep
 from pxr import Usd,UsdGeom,UsdLux,UsdPhysics,UsdShade,Gf,Sdf
 import imageio.v2 as imageio
 from PIL import Image,ImageDraw,ImageFont
 tape=np.load(run/'physics-tape.npz');times=tape['time_s'];positions=tape['body_positions'];quats=tape['body_quaternions'];names=json.loads((run/'tape-bodies.json').read_text());paths=json.loads((run/'body-paths.json').read_text())
 trace=json.loads((run/'trace.json').read_text());accept=json.loads((run/'acceptance.json').read_text())if(run/'acceptance.json').exists()else{'physical_simulation_pass':False,'state':'incomplete'}
 source=json.loads((Path(__file__).parent/'generated/cap_button/replay-source.json').read_text())
 manifest=json.loads((run/'run-manifest.json').read_text());button_view_start=7.4;release2=3.2444;release1=5.7504
 if manifest['arguments'].get('fast'):
  schedule=json.loads((run/'recording/independent_schedule.json').read_text())
  def release_time(side,label):return next(row['end_s']for row in schedule['lanes'][side]if row['stage']['label']==label)
  release2=release_time('left','release_two_parts');release1=release_time('right','release_two_parts');button_view_start=release_time('left','release_last_pin')+.03
 if manifest['arguments'].get('retimed'):
  from task_timing import LEFT_RELEASED
  button_view_start=LEFT_RELEASED+.05
 def focus_for_row(row):
  refs=row.get('source_times')
  if refs is not None:
   if refs['left']<release2:return 'ring2'
   if refs['right']<release1:return 'ring1'
   if refs['left']<button_view_start-.03:return 'pin'
   return 'button'
  ref=row['source_t']
  return ('ring2'if ref<release2 else'ring1'if ref<release1 else'pin')if ref<button_view_start else'button'
 assert len(trace)==len(times)==len(positions)
 stage=Usd.Stage.CreateNew(str(out/'measured.usda'));stage.GetRootLayer().subLayerPaths=[os.path.relpath(run/'configuration.usda',out),os.path.relpath(run/'scene.usda',out)]
 UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1);stage.SetTimeCodesPerSecond(30);stage.SetStartTimeCode(0);stage.SetEndTimeCode(float(times[-1])*30)
 for prim in stage.Traverse():
  if prim.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
  if prim.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
  if prim.IsA(UsdPhysics.Joint):UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
  if prim.GetName() in source['geom_visibility'] and not source['geom_visibility'][prim.GetName()]:UsdGeom.Imageable(prim).CreateVisibilityAttr('invisible')
  if str(prim.GetPath())=='/World/Cell/Geometry/m80/instance_0159':UsdGeom.Imageable(prim).CreateVisibilityAttr('invisible')
 bindings=[]
 for i,n in enumerate(names):
  path='/World/Button'if n=='__button_plunger'else n if n.startswith('/World/Clamps/')else paths[n]
  prim=stage.GetPrimAtPath(path);assert prim,(n,path)
  xf=UsdGeom.Xformable(prim);xf.ClearXformOpOrder();xf.SetResetXformStack(True);op=xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble,'measured')
  for f,t in enumerate(times):
   q=quats[f,i];m=Gf.Matrix4d().SetRotate(Gf.Quatd(float(q[0]),Gf.Vec3d(*map(float,q[1:]))));m.SetTranslateOnly(Gf.Vec3d(*map(float,positions[f,i])));op.Set(m,float(t)*30)
  bindings.append((path,i))
 # Colors identify the three dynamic CAD parts; geometry and measured
 # transforms are unchanged. Static stock and fixtures retain CAD materials.
 colors={'ring1':(.08,.38,.85),'ring2':(.95,.38,.055),'pin':(.12,.72,.22)}
 for name,color in colors.items():
  material=UsdShade.Material.Define(stage,'/RenderMaterials/'+name);shader=UsdShade.Shader.Define(stage,'/RenderMaterials/'+name+'/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(0.0);shader.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*[v*.5 for v in color]));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.32);material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
  for part_prim in Usd.PrimRange(stage.GetPrimAtPath(paths['workpiece_'+name])):
   if part_prim.IsA(UsdGeom.Imageable):UsdGeom.Imageable(part_prim).CreatePurposeAttr('default')
   UsdShade.MaterialBindingAPI.Apply(part_prim).Bind(material,UsdShade.Tokens.strongerThanDescendants)
 if a.xray:
  ghost=UsdShade.Material.Define(stage,'/RenderMaterials/TransparentFixture');shader=UsdShade.Shader.Define(stage,'/RenderMaterials/TransparentFixture/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.65,.72,.78));shader.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(.12);shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.5);ghost.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
  for prim in stage.Traverse():
   path=str(prim.GetPath())
   if prim.IsA(UsdGeom.Mesh)and ('/Geometry/m80/'in path or 'official_FR3_Duo'in path):UsdShade.MaterialBindingAPI.Apply(prim).Bind(ghost,UsdShade.Tokens.strongerThanDescendants)
 # Visible replacement bezel surrounds the measured, spring-loaded cap.
 bezel=UsdGeom.Cylinder.Define(stage,'/World/ButtonBezel');bezel.CreateRadiusAttr(.018);bezel.CreateHeightAttr(.004);bezel.CreateDisplayColorAttr([Gf.Vec3f(.12,.14,.17)])
 xf=UsdGeom.Xformable(bezel);xf.AddTranslateOp().Set(Gf.Vec3d(.355969,-.243,1.527));xf.AddRotateXOp().Set(90)
 UsdLux.DomeLight.Define(stage,'/RenderDome').CreateIntensityAttr(600);sun=UsdLux.DistantLight.Define(stage,'/RenderSun');sun.CreateIntensityAttr(1000);UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-35,20,30))
 cameras=[('Overview',[-.06,-.18,1.34],3.3,55,-20,(896,640)),('Button',[.356,-.245,1.527],.32,115,-8,(448,320)),('Jigs',[-.225,.255,1.527],.34,55,18,(448,320))]
 for n,look,dist,az,el,res in cameras:
  az=math.radians(az);el=math.radians(el);eye=np.array(look)-dist*np.array([math.cos(az)*math.cos(el),math.sin(az)*math.cos(el),math.sin(el)])
  c=UsdGeom.Camera.Define(stage,'/Render'+n);c.CreateVerticalApertureAttr(20.25);c.CreateHorizontalApertureAttr(28.35);c.CreateFocalLengthAttr(20.25/(2*math.tan(math.radians(22.5))));c.CreateClippingRangeAttr(Gf.Vec2f(.001,100));UsdGeom.Xformable(c).AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*look),Gf.Vec3d(0,0,1)).GetInverse())
 # Follow the currently manipulated physical workpiece before the button.
 camera=UsdGeom.Xformable(stage.GetPrimAtPath('/RenderButton'));op=camera.GetOrderedXformOps()[0]
 for f,t in enumerate(times):
  ref=trace[f]['source_t']
  name=focus_for_row(trace[f])
  if name=='button':continue
  i=names.index('workpiece_'+name);look=positions[f,i]
  if manifest['arguments'].get('retimed')and 5.7504<=ref<7.4:look=positions[f,names.index('right_grip_base')]+np.array([-.06,0,.05])
  dz=.10-.18*float(np.clip((look[2]-1.43)/.07,0,1));eye=look+np.array([-.08,-.11,.065])if a.xray else look+np.array([-.15,-.20,dz])
  op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*map(float,eye)),Gf.Vec3d(*map(float,look)),Gf.Vec3d(0,0,1)).GetInverse(),float(t)*30)
 # A final camera key restores the fixed physical button view at the transition.
 if focus_for_row(trace[-1])=='button':
  first_button=next(f for f,r in enumerate(trace)if focus_for_row(r)=='button');n,look,dist,az,el,res=cameras[1];az=math.radians(az);el=math.radians(el);eye=np.array(look)-dist*np.array([math.cos(az)*math.cos(el),math.sin(az)*math.cos(el),math.sin(el)])
  op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*look),Gf.Vec3d(0,0,1)).GetInverse(),float(times[first_button])*30)
 stage.GetRootLayer().Save()
 maxerr=0
 for f in np.unique(np.linspace(0,len(times)-1,min(50,len(times))).astype(int)):
  cache=UsdGeom.XformCache(float(times[f])*30)
  for path,i in bindings:maxerr=max(maxerr,float(np.linalg.norm(np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path)).ExtractTranslation())-positions[f,i])))
 assert maxerr<1e-8,maxerr
 omni.usd.get_context().open_stage(str(out/'measured.usda'));timeline=omni.timeline.get_timeline_interface();timeline.stop();timeline.set_current_time(0);app.update();rep.orchestrator.set_capture_on_play(False)
 products=[];rgbs=[]
 for n,*_,res in cameras:
  product=rep.create.render_product('/Render'+n,res);rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(product);products.append(product);rgbs.append(rgb)
 font=ImageFont.truetype(str(Path(__file__).resolve().parent/'assets/NotoSansKR.ttf'),16);big=ImageFont.truetype(str(Path(__file__).resolve().parent/'assets/NotoSansKR.ttf'),23)
 fps=30/a.stride;indices=list(range(0,len(times),a.stride));saved=[];partial=out/'video.partial.mp4'
 if a.cycle_only:
  start_t=manifest['arguments']['settle'];end_t=accept.get('button',{}).get('actuation_time_s')
  indices=[f for f in indices if times[f]>=start_t-1e-8 and (end_t is None or times[f]<=end_t+.15)]
 if a.preview_time is not None:indices=[int(np.argmin(abs(times-a.preview_time)))]
 writer=imageio.get_writer(str(partial),fps=fps,codec='libx264',quality=8,macro_block_size=2,ffmpeg_params=['-movflags','+faststart','-threads','4'])
 checkpoints=set(int(v)for v in np.linspace(0,len(indices)-1,9));first=None;delta=0
 for j,f in enumerate(indices):
  t=float(times[f]);timeline.set_current_time(t);app.update();rep.orchestrator.step(delta_time=0,rt_subframes=2,pause_timeline=True)
  frames=[np.ascontiguousarray(r.get_data()[:,:,:3])for r in rgbs]
  for attempt in range(3):
   if all(v.size for v in frames):break
   rep.orchestrator.step(delta_time=0,rt_subframes=2,pause_timeline=True);frames=[np.ascontiguousarray(r.get_data()[:,:,:3])for r in rgbs]
  assert all(v.shape==(res[1],res[0],3)for v,(*_,res)in zip(frames,cameras));assert frames[0].std()>1
  if first is None:first=frames[0].copy()
  delta=max(delta,float(np.mean(abs(frames[0].astype(float)-first))))
  canvas=Image.new('RGB',(1344,720),(18,24,32))
  for frame,xy in zip(frames,[(0,48),(896,48),(896,368)]):canvas.paste(Image.fromarray(frame),xy)
  draw=ImageDraw.Draw(canvas);row=trace[f];clamped=sum(row.get('clamp_verified',{}).values());stroke=1000*row['button_travel_m']
  passed=bool(accept['physical_simulation_pass']);act=accept.get('button',{}).get('actuation_time_s');green=passed and act is not None and t>=act
  title='PHYSX | M80: X-RAY PART VIEW'if a.xray else'PHYSX | M80: GRASP -> INSERT -> CLAMP -> BUTTON'
  if manifest['arguments'].get('fr3_limits'):title='FR3 LIMITED | '+('X-RAY PART VIEW'if a.xray else'M80: GRASP -> INSERT -> CLAMP -> BUTTON')
  if manifest['arguments'].get('efficient_timing'):title='FR3 PLAN 95% | '+('X-RAY PART VIEW'if a.xray else'GRASP -> INSERT -> CLAMP -> BUTTON')
  draw.text((14,9),title,font=big,fill='white')
  draw.text((1080,13),f'{t-(manifest["arguments"]["settle"]if a.cycle_only else 0):.2f} s | 1x',font=font,fill='white')
  ref=row['source_t'];focus=focus_for_row(row);part=focus.upper();detail=f'FOLLOW CAMERA | {part} | MEASURED POSE'if focus!='button' else f'ACTUAL BUTTON STROKE: {stroke:.2f} mm'
  if manifest['arguments'].get('retimed')and 5.7504<=ref<7.4:detail='RIGHT HAND RETURN | PIN HELD CLEAR'
  for y,label in [(52,detail),(372,f'CONTACT-VERIFIED CLAMPS: {clamped}/3')]:draw.rectangle((900,y,1340,y+28),fill=(18,24,32));draw.text((906,y+3),label,font=font,fill='white')
  if green:draw.ellipse((1307,88,1334,115),fill=(35,245,95))
  draw.rectangle((10,648,337,681),fill=(18,24,32))
  for name,x,color in [('RING1',18,(40,130,255)),('RING2',127,(255,125,35)),('PIN',236,(65,230,90))]:draw.ellipse((x,657,x+10,667),fill=color);draw.text((x+16,652),name,font=font,fill='white')
  layout_label='LEGACY SHIFTED SENSOR EXPERIMENT'if manifest.get('layout')=='trial_rail_offsets'else'ORIGINAL CAD MOUNTS | optical-axis display excluded'if manifest.get('sensor_geometry')=='physical_head_tail'else'ORIGINAL CAD GEOMETRY'
  draw.rectangle((0,688,1344,720),fill=(18,24,32));draw.text((12,695),'Measured PhysX states | assumed finite-force clamps | '+layout_label,font=font,fill=(225,234,245))
  if not passed:
   functional=all(v['passed']for v in accept.get('parts',{}).values())and accept.get('button',{}).get('passed',False)and accept.get('fr3_kinematic_limits_pass',False)
   draw.text((900,650),'ASSEMBLY PASS | TIME FAIL'if functional else'INCOMPLETE / FAILED TEST',font=big,fill=(255,130,90))
  writer.append_data(np.array(canvas))
  if a.preview_time is not None:canvas.save(out/'preview.png')
  if j in checkpoints:
   file=out/f'frame-{t:.3f}s.png';canvas.save(file);saved.append(file)
  if j%30==0:save(state='running',phase='rendering',frame=j,frames=len(indices),time_s=t);print('FRAME',j,len(indices),t,flush=True)
 writer.close();writer=None;partial.replace(out/'m80-physx-contact.mp4')
 for rgb,product in zip(rgbs,products):rgb.detach(product);product.destroy()
 sheet=Image.new('RGB',(1344,math.ceil(len(saved)/3)*240))
 for j,file in enumerate(saved):sheet.paste(Image.open(file).resize((448,240)),((j%3)*448,(j//3)*240))
 sheet.save(out/'contact-sheet.png');count=0;reader=imageio.get_reader(str(out/'m80-physx-contact.mp4'))
 for frame in reader:assert frame.shape==(720,1344,3);count+=1
 reader.close();assert count==len(indices)and(delta>1 or a.preview_time is not None)
 playback=float((times[indices[-1]]-times[indices[0]])*fps/(count-1))if count>1 else 1.
 assert abs(playback-1)<1e-6,playback
 save(state='passed',mode='measured_physx_state_render',playback_speed=playback,physical_simulation_pass=passed,fr3_kinematic_limits_pass=accept.get('fr3_kinematic_limits_pass'),physics_tape_sha256=sha(run/'physics-tape.npz'),acceptance_sha256=sha(run/'acceptance.json')if(run/'acceptance.json').exists()else None,video_sha256=sha(out/'m80-physx-contact.mp4'),cycle_only=a.cycle_only,first_physics_time_s=float(times[indices[0]]),last_physics_time_s=float(times[indices[-1]]),decoded_frames=count,fps=fps,duration_s=count/fps,max_pose_error_m=maxerr,max_frame_change=delta,seconds=time.monotonic()-start)
except Exception as exc:traceback.print_exc();save(state='failed',error=repr(exc));code=1
finally:
 if writer is not None:writer.close()
 app.close(wait_for_replicator=False,exit_code=code)
sys.exit(code)
