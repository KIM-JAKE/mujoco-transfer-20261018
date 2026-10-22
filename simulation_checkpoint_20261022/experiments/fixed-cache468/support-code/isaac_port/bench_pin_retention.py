"""Three-body clamp/retention diagnostic; never counted as a full task pass."""
from pathlib import Path
import argparse,json,math,time,sys,cProfile,numpy as np
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--velocity-iterations',type=int,required=True);p.add_argument('--seconds',type=float,default=3);p.add_argument('--gil-interval',type=float,default=.005);p.add_argument('--max-depenetration',type=float);p.add_argument('--compliance',type=float,default=0);a=p.parse_args();out=a.output.resolve();out.mkdir(exist_ok=True);run=a.run.resolve()
sys.setswitchinterval(a.gil_interval)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'multi_gpu':False,'anti_aliasing':0});code=0
try:
 import omni.usd,omni.timeline,omni.physx,carb
 from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf,PhysicsSchemaTools
 from isaacsim.core.simulation_manager import SimulationManager as SM
 from isaacsim.core.experimental.prims import RigidPrim
 paths=json.loads((run/'body-paths.json').read_text());names=json.loads((run/'tape-bodies.json').read_text());tape=np.load(run/'physics-tape.npz');pin=paths['workpiece_pin'];jaws=['/World/Clamps/pin/jaw_0','/World/Clamps/pin/jaw_1'];keep=[pin]+jaws
 stage=Usd.Stage.CreateNew(str(out/'scene.usda'));stage.GetRootLayer().subLayerPaths=[str(run/'configuration.usda'),str(run/'scene.usda')]
 for prim in stage.Traverse():
  path=str(prim.GetPath())
  if prim.HasAPI(UsdPhysics.ArticulationRootAPI):prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
  if prim.IsA(UsdPhysics.Joint)and path not in [j+'/Actuator'for j in jaws]:UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
  if prim.HasAPI(UsdPhysics.RigidBodyAPI):
   UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(path in keep)
   if path in keep:
    rb=PhysxSchema.PhysxRigidBodyAPI.Apply(prim);rb.CreateSolverPositionIterationCountAttr(64);rb.CreateSolverVelocityIterationCountAttr(a.velocity_iterations);rb.CreateSleepThresholdAttr(0)
    if a.max_depenetration is not None:rb.CreateMaxDepenetrationVelocityAttr(a.max_depenetration)
  if prim.HasAPI(UsdPhysics.CollisionAPI)and ('official_FR3_Duo'in path or 'workpiece_ring'in path or path.startswith(('/World/Clamps/ring1/','/World/Clamps/ring2/'))):UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
 for path in keep:
  name='workpiece_pin'if path==pin else path;i=names.index(name);pos=tape['body_positions'][-1,i];q=tape['body_quaternions'][-1,i]
  prim=stage.GetPrimAtPath(path);xf=UsdGeom.Xformable(prim);xf.ClearXformOpOrder();xf.SetResetXformStack(True);xf.AddTranslateOp().Set(Gf.Vec3d(*map(float,pos)));xf.AddOrientOp().Set(Gf.Quatf(float(q[0]),Gf.Vec3f(*map(float,q[1:]))))
  if path in jaws:
   joint=UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(path+'/Actuator'));origin=np.array(joint.GetLocalPos0Attr().Get());rot=Gf.Rotation(joint.GetLocalRot0Attr().Get());direction=np.array(rot.TransformDir(Gf.Vec3d(1,0,0)));displacement=float(np.dot(pos-origin,direction));PhysxSchema.JointStateAPI.Apply(joint.GetPrim(),'linear').CreatePositionAttr(displacement);UsdPhysics.DriveAPI(joint.GetPrim(),'linear').GetTargetPositionAttr().Set(-.0047)
 if a.compliance:
  material=stage.GetPrimAtPath('/World/Materials/steel');cm=PhysxSchema.PhysxMaterialAPI.Apply(material);cm.CreateCompliantContactStiffnessAttr(a.compliance);cm.CreateCompliantContactDampingAttr(100)
 stage.GetRootLayer().Save();assert omni.usd.get_context().open_stage(str(out/'scene.usda'));SM.setup_simulation(dt=1/480,device='cpu')
 settings=carb.settings.get_settings();settings.set_bool('/physics/updateToUsd',False);settings.set_bool('/physics/updateVelocitiesToUsd',False);settings.set_string('/log/fileFlushLevel','error');settings.set_bool('/log/async',True)
 timeline=omni.timeline.get_timeline_interface();timeline.play();app.update();timeline.pause();view=RigidPrim(keep);forces={j:0. for j in jaws};clock=[0.]
 def contact(headers,points):
  for h in headers:
   aa=str(PhysicsSchemaTools.intToSdfPath(h.actor0));bb=str(PhysicsSchemaTools.intToSdfPath(h.actor1))
   if pin not in [aa,bb]:continue
   other=bb if aa==pin else aa
   if other in forces:forces[other]+=sum(float(np.linalg.norm(points[i].impulse))*480 for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data))
 sub=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(contact);rows=[];start=time.monotonic()
 profile=cProfile.Profile();profile.enable()
 for i in range(round(a.seconds*480)):
  for key in forces:forces[key]=0.
  clock[0]=i/480;SM.step()
  if i%4==0:
   pos,q=view.get_world_poses();v,w=view.get_velocities();rows.append(dict(t=i/480,pos=pos.numpy()[0].tolist(),velocity=v.numpy()[0].tolist(),forces=dict(forces)))
 profile.disable();profile.dump_stats(str(out/'physics.prof'))
 result=dict(gil_interval=a.gil_interval,max_depenetration=a.max_depenetration,scope='isolated pin and two finite-force jaws; other robot/parts disabled',velocity_iterations=a.velocity_iterations,position_iterations=64,seconds=time.monotonic()-start,samples=len(rows),max_retained_speed_m_s=max(np.linalg.norm(r['velocity'])for r in rows if r['t']>=1),final_contacting_jaws=sum(v>.5 for v in rows[-1]['forces'].values()),max_position_drift_m=max(np.linalg.norm(np.array(r['pos'])-rows[0]['pos'])for r in rows if r['t']>=1))
 result['diagnostic_pass']=bool(result['max_retained_speed_m_s']<.005 and result['final_contacting_jaws']==2 and result['max_position_drift_m']<.0008)
 (out/'trace.json').write_text(json.dumps(rows,indent=2));(out/'result.json').write_text(json.dumps(result,indent=2));print('RESULT',result,flush=True)
except Exception as exc:
 import traceback;traceback.print_exc();(out/'error.json').write_text(json.dumps(dict(error=repr(exc))));code=1
finally:app.close(wait_for_replicator=False,exit_code=code)
