"""Render actual recorded Isaac states using visible CAD triangle meshes."""
from pathlib import Path
import os,sys,json,traceback
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf,UsdUtils
import omni.physx,omni.kit.app
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE));PORT=HERE.parents[1];OUT=PORT/'runs'/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True)
try:
    request_path=HERE/'bin-picking/quality-request.json';request=json.load(open(request_path))if request_path.exists()else{}
    source=PORT/'runs'/request.get('source_run','native-roas-table-bin-pin-simple02')
    stage=Usd.Stage.Open(str(source/'scene-authored.usda'));result_file=source/'physx-pickup-results.json';result=json.load(open(result_file if result_file.exists()else source/'pipeline-results.json'));paths=json.load(open(source/'body-paths.json'));tape=np.load(source/'physx-sequence-trace.npz');j=int(np.searchsorted(tape['time_s'],request['sim_time_s']))if 'sim_time_s'in request else -1;j=min(j,len(tape['time_s'])-1)
    def set_pose(path,value):
        pr=stage.GetPrimAtPath(path)
        if not pr:return
        x,y,z,w=map(float,value[3:]);M=Gf.Matrix4d().SetRotate(Gf.Quatd(w,Gf.Vec3d(x,y,z)));M.SetTranslateOnly(Gf.Vec3d(*map(float,value[:3])));xf=UsdGeom.Xformable(pr);xf.ClearXformOpOrder();xf.SetResetXformStack(True);xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(M)
    bodies={p.GetName():str(p.GetPath())for p in stage.Traverse()if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    for name,value in zip(result['link_names'],tape['links'][j]):
        if name in bodies:set_pose(bodies[name],value)
    for p,value in zip(tape['part_names'].tolist(),tape['parts'][j]):set_pose(paths['workpiece_'+p],value)
    count=0
    for p in list(stage.Traverse()):
        if p.IsA(UsdPhysics.Joint):p.SetActive(False);continue
        for api in [UsdPhysics.ArticulationRootAPI,PhysxSchema.PhysxArticulationAPI,UsdPhysics.RigidBodyAPI]:
            if p.HasAPI(api):p.RemoveAPI(api)
        if p.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)
        if p.IsA(UsdGeom.Mesh):
            mesh=UsdGeom.Mesh(p)
            if mesh.ComputeVisibility()=='invisible' or 'PlanningQueries'in str(p.GetPath()):continue
            if not len(mesh.GetPointsAttr().Get()or[]):continue
            UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(True);UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr('none');count+=1
        elif p.IsA(UsdGeom.Cube)or p.IsA(UsdGeom.Cylinder):
            if UsdGeom.Imageable(p).ComputeVisibility()!='invisible':UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(True)
    # Hide only the unmeasured surrogate clamp drawing; physical trial unchanged.
    clamps=stage.GetPrimAtPath('/World/Clamps')
    if clamps:clamps.SetActive(False)
    for name,eye,target,focal in [('Assembly',[-.53,-.02,1.42],[-.22,.25,1.535],45),('AssemblyFront',[-.22,-.35,1.4],[-.22,.25,1.535],50),('Global',[-1.5,-1.85,2.1],[.05,-.15,1.12],35),('Basket',[.70,-1.1,1.65],[.55,-.725,1.10],48)]:
        c=UsdGeom.Camera.Define(stage,'/World/Cameras/Quality'+name);c.CreateFocalLengthAttr(focal);c.CreateHorizontalApertureAttr(32);xf=UsdGeom.Xformable(c);xf.ClearXformOpOrder();xf.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse())
    ident=UsdUtils.StageCache.Get().Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ident);sim.simulate(1/60,0);sim.fetch_results();print('QUALITY_VISUAL_MESHES',count,flush=True)
    from quality_physx_camera import render
    for name in request.get('views',['Assembly','Global']):
        rgb=render(stage,'/World/Cameras/Quality'+name,960,720)
        with(OUT/(name+'.ppm')).open('wb')as f:f.write(b'P6\n960 720\n255\n'+rgb.tobytes())
        print('QUALITY_FRAME',name,flush=True)
    (OUT/'render-metadata.json').write_text(json.dumps(dict(source_run=source.name,renderer='Isaac CPU visual triangle ray queries with shadows',RTX=False,visible_mesh_count=count,resolution=[960,720],same_ring_material=True,replay_only=True,source_continuous_cycle_pass=result.get('continuous_cycle_pass',False)),indent=2))
    sim.detach_stage();omni.kit.app.get_app().post_quit(0)
except Exception:
    (OUT/'error.txt').write_text(traceback.format_exc());traceback.print_exc();omni.kit.app.get_app().post_quit(1)
