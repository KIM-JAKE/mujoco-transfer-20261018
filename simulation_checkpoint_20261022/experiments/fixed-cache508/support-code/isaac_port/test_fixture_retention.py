"""Isolated gravity retention of original ring/jig CAD; no attachment or pose servo."""
from pathlib import Path
import json,math,time,sys
import numpy as np
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'multi_gpu':False,'anti_aliasing':0});code=0
try:
 import omni.usd,omni.timeline,carb
 from pxr import UsdGeom,UsdPhysics,UsdShade,PhysxSchema,Gf
 from isaacsim.core.simulation_manager import SimulationManager as SM
 from isaacsim.core.experimental.prims import RigidPrim
 root=Path(__file__).resolve().parent;out=root/'runs/retention-bench';out.mkdir(exist_ok=True)
 arrays=np.load(root/'generated/contact_task/precision-cad.npz');rows=[]
 for compliant in [0,1e6]:
  for preload in [0,.0001,.0003]:
   omni.usd.get_context().new_stage();stage=omni.usd.get_context().get_stage();UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1)
   stage.SetTimeCodesPerSecond(480);stage.SetStartTimeCode(0);stage.SetEndTimeCode(4800)
   material=UsdShade.Material.Define(stage,'/Steel');ma=UsdPhysics.MaterialAPI.Apply(material.GetPrim());ma.CreateStaticFrictionAttr(.6);ma.CreateDynamicFrictionAttr(.51)
   if compliant:
    cm=PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim());cm.CreateCompliantContactStiffnessAttr(compliant);cm.CreateCompliantContactDampingAttr(20)
   for identifier in [545,546,547,548,549]:
    moving=identifier==549
    key=f'instance_{identifier:04d}';path='/Ring'if moving else'/Jig/'+key
    body=UsdGeom.Xform.Define(stage,path)
    if moving:
     body.AddTranslateOp().Set(Gf.Vec3d(-.23903095783836364,.28424999618794844,1.532+preload));body.AddOrientOp().Set(Gf.Quatf(math.cos(math.pi/6),Gf.Vec3f(0,0,math.sin(math.pi/6))));UsdPhysics.RigidBodyAPI.Apply(body.GetPrim());mass=UsdPhysics.MassAPI.Apply(body.GetPrim());mass.CreateMassAttr(.0717855);mass.CreateDiagonalInertiaAttr(Gf.Vec3f(3.136e-5,3.136e-5,6.064e-5))
    mesh=UsdGeom.Mesh.Define(stage,path+'/Shape');mesh.CreatePointsAttr(arrays[key+'_vertices'].astype(np.float32));f=arrays[key+'_faces'];mesh.CreateFaceVertexCountsAttr([3]*len(f));mesh.CreateFaceVertexIndicesAttr(f.reshape(-1));mesh.CreateSubdivisionSchemeAttr('none')
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('sdf'if moving else'none')
    if moving:
     sdf=PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(mesh.GetPrim());sdf.CreateSdfResolutionAttr(768);sdf.CreateSdfSubgridResolutionAttr(6)
    ca=PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim());ca.CreateContactOffsetAttr(.0001);ca.CreateRestOffsetAttr(0)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material,UsdShade.Tokens.weakerThanDescendants,'physics')
   sc=UsdPhysics.Scene.Define(stage,'/Physics');sc.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));sc.CreateGravityMagnitudeAttr(9.81);PhysxSchema.PhysxSceneAPI.Apply(sc.GetPrim()).CreateSolverTypeAttr('TGS')
   SM.setup_simulation(dt=1/480,device='cpu');carb.settings.get_settings().set_bool('/physics/updateToUsd',False)
   tl=omni.timeline.get_timeline_interface();tl.play();app.update();tl.pause()
   part=RigidPrim('/Ring');initial=part.get_world_poses()[0].numpy()[0];history=[]
   for step in range(960):
    SM.step()
    if step%48==0:
     p=part.get_world_poses()[0].numpy()[0];history.append(p.tolist())
     if p[2]<1:break
   p=part.get_world_poses()[0].numpy()[0];velocity=part.get_velocities()[0].numpy()[0]
   row=dict(stiffness_n_m=compliant,preload_m=preload,initial=initial.tolist(),final=p.tolist(),velocity=velocity.tolist(),held=bool(np.linalg.norm(p-initial)<.0008 and np.linalg.norm(velocity)<.005),history=history)
   rows.append(row);(out/'results.json').write_text(json.dumps(rows,indent=2)+'\n');print('RETENTION_RESULT',json.dumps({k:v for k,v in row.items()if k!='history'}),flush=True)
   tl.stop();app.update();del part
except Exception:
 import traceback;traceback.print_exc();code=1
finally:app.close(wait_for_replicator=False,exit_code=code)
sys.exit(code)
