"""Isolate the new physical Hand joints from the imported robot articulation."""
from pathlib import Path
import sys,json,math,traceback
sys.path.insert(0,str(Path(__file__).resolve().parent))
from native_roas_physx import native_hand,pose
import omni.physx,omni.kit.app
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,PhysxSchema,UsdUtils,Gf
import numpy as np
HERE=Path(__file__).resolve().parent
try:
    stage=Usd.Stage.CreateInMemory();UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    sc=UsdPhysics.Scene.Define(stage,'/PhysicsScene');sc.CreateGravityMagnitudeAttr(9.81);sc.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));sa=PhysxSchema.PhysxSceneAPI.Apply(sc.GetPrim());sa.CreateEnableGPUDynamicsAttr(False);sa.CreateBroadphaseTypeAttr('MBP')
    mat=UsdShade.Material.Define(stage,'/World/Materials/rubber');UsdPhysics.MaterialAPI.Apply(mat.GetPrim()).CreateStaticFrictionAttr(.8)
    base=UsdGeom.Xform.Define(stage,'/World/Base').GetPrim();pose(base,[0,0,.7]);UsdPhysics.RigidBodyAPI.Apply(base);ma=UsdPhysics.MassAPI.Apply(base);ma.CreateMassAttr(1);ma.CreateDiagonalInertiaAttr(Gf.Vec3f(.1))
    root=UsdPhysics.FixedJoint.Define(stage,'/World/RootJoint');root.CreateBody1Rel().SetTargets(['/World/Base']);root.CreateLocalPos0Attr(Gf.Vec3f(0,0,.7));UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    hand,_=native_hand(stage,'right',dict(right_arm_fr3v2_link8='/World/Base',right_arm_fr3v2_link7='/World/Base'))
    bodies=[p for p in stage.Traverse()if p.HasAPI(UsdPhysics.RigidBodyAPI)];mats=[UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())for p in bodies]
    for p,m in zip(bodies,mats):
        xf=UsdGeom.Xformable(p);xf.ClearXformOpOrder();xf.SetResetXformStack(True);xf.AddTransformOp().Set(m)
    cache=UsdUtils.StageCache.Get();id=cache.Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(id)
    for i in range(480):sim.simulate(1/480,i/480);sim.fetch_results()
    world=UsdGeom.Xformable(stage.GetPrimAtPath(hand)).ComputeLocalToWorldTransform(Usd.TimeCode.Default());position=list(world.ExtractTranslation());report=dict(engine='Isaac Sim CPU PhysX',hand_position_m=position,passed=bool(np.isfinite(position).all()and np.linalg.norm(np.array(position)-[0,0,.7])<.001))
    (HERE/'native-hand-physics-smoke.json').write_text(json.dumps(report,indent=2));print('NATIVE_HAND_SMOKE',report,flush=True);sim.detach_stage();omni.kit.app.get_app().post_quit(0 if report['passed']else 2)
except Exception:
    (HERE/'native-hand-physics-smoke-error.txt').write_text(traceback.format_exc());traceback.print_exc();omni.kit.app.get_app().post_quit(1)
