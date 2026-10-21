"""Development snapshots: replay states only create camera inputs, never labels."""
from pathlib import Path
import os,sys,json,traceback
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf,UsdUtils
import omni.kit.app,omni.physx
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE));PORT=HERE.parents[1];OUT=PORT/'runs'/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True)
from rgbd_sensor import capture
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
    for label,st in [('initial',0),('ring1',20.95),('drag_before',33.889),('drag_after',42.6),('ring2',46.4896),('pin',64.858)]:
        k=int(np.argmin(abs(commands['right_source_time_s']-st)));j=int(np.argmin(abs(trace['time_s']-commands['time_s'][k])))
        for n,tr in zip(res['link_names'],trace['links'][j]):
            if n in primnames:pose(primnames[n],tr)
        for n,tr in zip(trace['part_names'],trace['parts'][j]):pose(paths['workpiece_'+n],tr)
        sim.flush_changes();sim.simulate(1/480,0);sim.fetch_results()
        if label in ['initial','drag_before','drag_after']:
            capture([.50,-.71,1.80],[.53,-.71,1.10],.39,out=OUT/(label+'-basket.npz'))
            if label=='initial':capture([.19,-.79,1.35],[.19,-.79,.916],.12,out=OUT/'initial-pin.npz')
        else:
            hand=trace['links'][j,res['link_names'].index('right_NativeHand')]
            from native_pickup_probe import rotation
            target=hand[:3]+rotation(hand[3:])@np.array([0,0,.1034])
            for i,delta in enumerate([[0,-.2,-.3],[.25,0,-.25],[-.25,.12,-.2]]):capture(target+delta,target,.16,out=OUT/(label+'-held'+str(i)+'.npz'))
        print('RGBD_PROBE',label,flush=True)
    sim.detach_stage();omni.kit.app.get_app().post_quit(0)
except Exception:
    traceback.print_exc();omni.kit.app.get_app().post_quit(1)
