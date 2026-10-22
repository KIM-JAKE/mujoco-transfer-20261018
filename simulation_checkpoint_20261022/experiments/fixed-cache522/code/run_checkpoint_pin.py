"""Independent pin trial from the recorded two-ring checkpoint, never a full cycle."""
from pathlib import Path
import os,sys,json,shutil,traceback,math
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,UsdShade,Gf
import omni.kit.app
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE));PORT=HERE.parents[1];OUT=PORT/'runs'/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True);source=PORT/'runs'/'native-roas-table-bin-quick07';sys.path.insert(0,str(PORT))
try:
    stage=Usd.Stage.Open(str(source/'scene-authored.usda'));paths=json.load(open(source/'body-paths.json'));hands=paths.pop('native_hands');initial=json.load(open(source/'initial-robot-pose.json'));plan=json.load(open(source/'arm-plan.json'));old=json.load(open(source/'physx-pickup-results.json'));plan['parts']={'pin':dict(side='right')};plan['initial_apertures_m']={'left':.08,'right':.022};plan['handoffs']['pin']['insert_q_seeds'].insert(0,[-.1869964,1.0797545,1.1027995,-.9652948,1.5667019,2.1647879,-.2077391]);plan['live_demo_control']=True
    def pose(path,p,q):
        xf=UsdGeom.Xformable(stage.GetPrimAtPath(path));xf.ClearXformOpOrder();xf.SetResetXformStack(True);x,y,z,w=map(float,q);M=Gf.Matrix4d().SetRotate(Gf.Quatd(w,Gf.Vec3d(x,y,z)));M.SetTranslateOnly(Gf.Vec3d(*map(float,p)));xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(M)
    tape=np.load(source/'physx-sequence-trace.npz');names=tape['part_names'].tolist()
    for p in ['ring1','ring2']:
        v=tape['parts'][-1,names.index(p)];pose(paths['workpiece_'+p],v[:3],v[3:])
    pose(paths['workpiece_pin'],[.19,-.79,.91605],[1,0,0,0])
    for row in old.get('clutter_settled_objects',[]):
        if row['path']not in [paths['workpiece_ring1'],paths['workpiece_ring2']]:pose(row['path'],row['transform'][:3],row['transform'][3:])
    asset=np.load(HERE/'native-official-finger.npz')
    for side in ['left','right']:
        for label in ['left','right']:
            path=hands[side]+'/'+side+'_'+label+'_finger'
            for name in ['mount','sledge','diagonal','tip']:stage.RemovePrim(path+'/'+name)
            m=UsdGeom.Mesh.Define(stage,path+'/tip');m.CreatePointsAttr(asset['vertices'].astype(np.float32));f=asset['faces'];m.CreateFaceVertexCountsAttr([3]*len(f));m.CreateFaceVertexIndicesAttr(f.ravel());m.CreateSubdivisionSchemeAttr('none');m.CreateDisplayColorAttr([Gf.Vec3f(.08,.08,.10)]);UsdPhysics.CollisionAPI.Apply(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull');UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(UsdShade.Material.Get(stage,'/World/Materials/rubber'),UsdShade.Tokens.strongerThanDescendants,'physics');pc=PhysxSchema.PhysxCollisionAPI.Apply(m.GetPrim());pc.CreateContactOffsetAttr(.00005);pc.CreateRestOffsetAttr(0.);pc.CreateTorsionalPatchRadiusAttr(.003);pc.CreateMinTorsionalPatchRadiusAttr(.002)
    for prim in stage.Traverse():
        path=str(prim.GetPath())
        if not path.startswith('/World/Clamps/')or not prim.IsA(UsdPhysics.PrismaticJoint):continue
        j=UsdPhysics.PrismaticJoint(prim);pin=path.startswith('/World/Clamps/pin/');UsdPhysics.DriveAPI(prim,'linear').GetTargetPositionAttr().Set(0. if pin else j.GetUpperLimitAttr().Get())
        if pin:
            a=j.GetLocalPos0Attr().Get();j.GetLocalPos0Attr().Set(Gf.Vec3f(a[0],a[1],1.525));body=str(j.GetBody1Rel().GetTargets()[0]);bp=UsdGeom.Xformable(stage.GetPrimAtPath(body)).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation();q=[0,0,0,1]if body.endswith('0')else[0,0,1,0];pose(body,[bp[0],bp[1],1.525],q)
    from simple_pin_collision import apply as simplify_pin
    simple_pin_model=simplify_pin(stage,paths['workpiece_pin'])
    drives={};qcurrent={}
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint)and '_arm_fr3v2_joint'in prim.GetName():
            n=prim.GetName();qcurrent[n]=initial['ready_q'][n.split('_')[0]][int(n[-1])-1];drives[n]=UsdPhysics.DriveAPI(prim,'angular')
    report=dict(engine='Isaac Sim 6.1 Omni PhysX CPU',placement=old['placement'],checkpoint_source_run=source.name,starts_with_rings_already_installed=True,scope='Independent physical pin trial; earlier ring pickup and insertion evidence in checkpoint source',free_workpieces=True,pin_collision_approximation=simple_pin_model,part_attachments=False,full_task_pass=False,hardware_tested=False,vision_end_to_end_pass=False)
    (OUT/'arm-plan.json').write_text(json.dumps(plan,indent=2));(OUT/'body-paths.json').write_text(json.dumps(dict(**paths,native_hands=hands),indent=2));shutil.copyfile(source/'initial-robot-pose.json',OUT/'initial-robot-pose.json');stage.GetRootLayer().Export(str(OUT/'scene-authored.usda'))
    frozen=OUT/'code_snapshot';frozen.mkdir(exist_ok=True)
    for name in ['run_checkpoint_pin.py','pin_finish_physx.py','checkpoint_pin_probe.py','isaac_collision_queries.py','isaac_motion.py','isaac_kinematics.py','simple_pin_collision.py','quick_pin_episode.py','quick_pin_finish_accept.py','quick-pin-pusher-recipe.json']:shutil.copyfile(HERE/name,frozen/name)
    from checkpoint_pin_probe import execute
    execute(stage,paths,hands,drives,qcurrent,initial,report,OUT,HERE,OUT/'arm-plan.json');omni.kit.app.get_app().post_quit(0)
except Exception:
    (OUT/'error.txt').write_text(traceback.format_exc());traceback.print_exc();omni.kit.app.get_app().post_quit(1)
