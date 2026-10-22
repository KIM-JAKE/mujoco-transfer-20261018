"""Planning-only checkpoint. Never counted as a physical task completion."""
from pathlib import Path
import os,sys,json,traceback
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf,UsdUtils
import omni.physx,omni.kit.app
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE));PORT=HERE.parents[1];OUT=PORT/'runs'/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True)
sys.path.insert(0,str(PORT))
try:
    source=PORT/'runs'/'native-roas-table-bin-pipeline13'
    stage=Usd.Stage.Open(str(source/'scene-authored.usda'));result=json.load(open(source/'pipeline-results.json'));paths=json.load(open(source/'body-paths.json'));trace=np.load(source/'physx-sequence-trace.npz');j=-1
    def pose(path,value):
        x,y,z,w=map(float,value[3:]);M=Gf.Matrix4d().SetRotate(Gf.Quatd(w,Gf.Vec3d(x,y,z)));M.SetTranslateOnly(Gf.Vec3d(*map(float,value[:3])));xf=UsdGeom.Xformable(stage.GetPrimAtPath(path));xf.ClearXformOpOrder();xf.SetResetXformStack(True);xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(M)
    bodies={p.GetName():str(p.GetPath())for p in stage.Traverse()if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    for name,value in zip(result['link_names'],trace['links'][j]):
        if name in bodies:pose(bodies[name],value)
    for p,value in zip(trace['part_names'].tolist(),trace['parts'][j]):pose(paths['workpiece_'+p],value)
    drives={}
    for p in list(stage.Traverse()):
        if p.IsA(UsdPhysics.Joint):
            UsdPhysics.Joint(p).CreateJointEnabledAttr(False)
            if '_arm_fr3v2_joint'in p.GetName():drives[p.GetName()]=UsdPhysics.DriveAPI(p,'angular')
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if p.HasAPI(PhysxSchema.PhysxArticulationAPI):p.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
        if p.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(p).CreateKinematicEnabledAttr(True)
    ident=UsdUtils.StageCache.Get().Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ident);sim.simulate(1/480,0);sim.fetch_results()
    from isaac_kinematics import Arm
    from isaac_collision_queries import Checker
    from recipient_grasp_search import plan
    from basket_pipeline import hermite
    from native_pickup_probe import rotation
    tape=dict(np.load(PORT/'runs/native-roas-table-bin-quick07/physx-pickup-trace.npz'));names=result['dof_names'];idx=[names.index(f'left_arm_fr3v2_joint{k}')for k in range(1,8)];arm=Arm(stage,'left',drives,trace['q'][j,idx]);checker=Checker(stage,arm,'left')
    ring=trace['parts'][j,1];report=dict(planning_only=True,continuous_cycle_pass=False,checkpoint_time_s=float(trace['time_s'][j]),candidates=[])
    ri=[names.index(f'right_arm_fr3v2_joint{k}')for k in range(1,8)];giver=Arm(stage,'right',drives,trace['q'][j,ri]);oldH=giver.fk(trace['q'][j,ri]);opening=float(sum(trace['q'][j,-2:]));original_tape={k:v.copy()for k,v in tape.items()}
    for offset in [[-.12,-.15,0],[0,0,.08],[0,0,-.1],[-.08,-.08,-.1],[0,-.15,-.1]]:
        H=oldH.copy();H[:3,3]+=offset;q=giver.solve(H[:3,3],H[:3,:3],trace['q'][j,ri])
        if q is None:report['candidates'].append(dict(offset=offset,error='donor IK'));continue
        for path,T in giver.bodies(q,opening).items():
            xf=UsdGeom.Xformable(stage.GetPrimAtPath(path));xf.ClearXformOpOrder();xf.SetResetXformStack(True);xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Matrix4d(*T.T.ravel().tolist()))
        moved=ring.copy();moved[:3]+=offset;pose(paths['workpiece_ring2'],moved);sim.flush_changes();sim.simulate(1/480,0);sim.fetch_results()
        checker=Checker(stage,arm,'left');tape={k:v.copy()for k,v in original_tape.items()};entry=dict(offset=offset)
        try:
            entry['grasp']=plan(arm,checker,tape,idx,ring[:3],rotation(ring[3:])[:,2],paths['workpiece_ring2'],61.3229166667,offset)
            report['grasp']=entry['grasp'];report['offset']=offset
        except Exception:entry['error']=traceback.format_exc();entry['checks']=getattr(checker,'recipient_checks',[])
        report['candidates'].append(entry);(OUT/'query-results.json').write_text(json.dumps(report,indent=2));print('OFFSET_QUERY',offset,'PASS'if'grasp'in entry else'failed',flush=True)
        if 'grasp'in entry:break
    (OUT/'query-results.json').write_text(json.dumps(report,indent=2));print('HANDOVER_QUERY_RESULT',report,flush=True);sim.detach_stage();omni.kit.app.get_app().post_quit(0)
except Exception:
    (OUT/'error.txt').write_text(traceback.format_exc());traceback.print_exc();omni.kit.app.get_app().post_quit(1)
