"""Development snapshots: replay states only create camera inputs, never labels."""
from pathlib import Path
import os,sys,json,traceback
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf,UsdUtils
import omni.kit.app,omni.physx
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE));PORT=HERE.parents[1];OUT=PORT/'runs'/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True)
from rgb_sensor import capture
try:
    src=PORT/'runs/native-roas-table-bin-OD49';stage=Usd.Stage.Open(str(src/'scene-authored.usda'));paths=json.load(open(src/'body-paths.json'));res=json.load(open(src/'pipeline-results.json'));trace=np.load(src/'physx-sequence-trace.npz');commands=np.load(src/'executed-actuator-commands.npz')
    primnames={p.GetName():str(p.GetPath())for p in stage.Traverse()if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    def pose(path,tr):
        p=stage.GetPrimAtPath(path);xf=UsdGeom.Xformable(p);xf.ClearXformOpOrder();xf.SetResetXformStack(True);x,y,z,w=map(float,tr[3:]);M=Gf.Matrix4d().SetRotate(Gf.Quatd(w,Gf.Vec3d(x,y,z)));M.SetTranslateOnly(Gf.Vec3d(*map(float,tr[:3])));xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(M)
    for p in list(stage.Traverse()):
        if p.IsA(UsdPhysics.Joint):p.SetActive(False)
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if p.HasAPI(PhysxSchema.PhysxArticulationAPI):p.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
        if p.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(p).CreateKinematicEnabledAttr(True)
    ident=UsdUtils.StageCache.Get().Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ident)
    for label,st in [('initial',0),('ring1',20.95),('ring2',46.4896),('pin',64.858)]:
        k=int(np.argmin(abs(commands['right_source_time_s']-st)));j=int(np.argmin(abs(trace['time_s']-commands['time_s'][k])))
        for n,tr in zip(res['link_names'],trace['links'][j]):
            if n in primnames:pose(primnames[n],tr)
        for n,tr in zip(trace['part_names'],trace['parts'][j]):pose(paths['workpiece_'+n],tr)
        sim.flush_changes();sim.simulate(1/480,0);sim.fetch_results()
        if label in ['initial','drag_before','drag_after']:
            
            for i,eye in enumerate([[.49,-1.3,1.65],[.75,-1.15,1.6],[.49,-.3,1.55]]):capture(stage,eye,[.51,-.70,1.10],.28,width=256,height=256,out=OUT/(label+'-basket'+str(i)+'.npz'))
            if label=='initial':capture(stage,[.19,-.79,1.35],[.19,-.79,.916],.12,width=256,height=256,out=OUT/'initial-pin.npz')
        else:
            hand=trace['links'][j,res['link_names'].index('right_NativeHand')]
            from native_pickup_probe import rotation
            target=hand[:3]+rotation(hand[3:])@np.array([0,0,.1034])
            for i,delta in enumerate([[-.25,.12,-.2],[-.25,-.12,-.2],[0,-.3,-.1],[.25,-.15,-.15]]):capture(stage,target+delta,target,.16,width=320,height=320,out=OUT/(label+'-held'+str(i)+'.npz'))
        print('RGBD_PROBE',label,flush=True)
    sim.detach_stage();omni.kit.app.get_app().post_quit(0)
except Exception:
    traceback.print_exc();omni.kit.app.get_app().post_quit(1)
