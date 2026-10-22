"""A real CPU PhysX drop test in Isaac Sim's installed runtime, without RTX."""
import json
from pathlib import Path
import omni.kit.app, omni.physx,carb
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf,UsdUtils

OUT=Path('/workspace/project/mujoco/isaac_port/native_hand_layout/roas-table')
try:
    stage=Usd.Stage.CreateInMemory();UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    scene=UsdPhysics.Scene.Define(stage,'/PhysicsScene');scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));scene.CreateGravityMagnitudeAttr(9.81)
    api=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim());api.CreateEnableGPUDynamicsAttr(False);api.CreateBroadphaseTypeAttr('MBP');api.CreateTimeStepsPerSecondAttr(240)
    floor=UsdGeom.Cube.Define(stage,'/Floor');floor.CreateSizeAttr(1);floor.AddTranslateOp().Set(Gf.Vec3d(0,0,-.05));floor.AddScaleOp().Set(Gf.Vec3f(2,2,.1));UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    cube=UsdGeom.Cube.Define(stage,'/Cube');cube.CreateSizeAttr(.1);cube.AddTranslateOp().Set(Gf.Vec3d(0,0,1));UsdPhysics.CollisionAPI.Apply(cube.GetPrim());UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim());UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(.1)
    cache=UsdUtils.StageCache.Get();ident=cache.Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ident)
    positions=[]
    for i in range(480):
        sim.simulate(1/240,i/240);sim.fetch_results()
        if i%24==0:positions.append(UsdGeom.Xformable(cube).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[2])
    z=float(UsdGeom.Xformable(cube).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()[2])
    report=dict(engine='Isaac Sim bundled Omni PhysX',rendering_enabled=False,gpu_devices_exposed=False,cpu_physics=True,final_height_m=z,positions_m=positions,passed=abs(z-.05)<.005)
    stage.GetRootLayer().Export(str(OUT/'cpu-smoke.usda'));(OUT/'cpu-smoke.json').write_text(json.dumps(report,indent=2));print('CPU_PHYSX_SMOKE',json.dumps(report),flush=True)
    sim.detach_stage();omni.kit.app.get_app().post_quit(0 if report['passed']else 2)
except Exception:
    import traceback
    (OUT/'cpu-smoke-error.txt').write_text(traceback.format_exc());traceback.print_exc();omni.kit.app.get_app().post_quit(1)
