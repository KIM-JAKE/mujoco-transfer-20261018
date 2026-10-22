"""Isolated physical ring repositioning; never assembly evidence."""
from pathlib import Path
import os,sys,json,traceback
import numpy as np
from pxr import Usd,UsdGeom,UsdUtils,UsdPhysics,PhysxSchema,Gf
import omni.kit.app,omni.physx,omni.physics.tensors as tensors
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parents[1])]
from isaac_kinematics import Arm
from isaac_motion import Audit,duration
from native_pickup_probe import rotation
RUNS=HERE.parents[1]/'runs';OUT=RUNS/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True)
try:
    source=RUNS/'native-roas-table-bin-retimed14';stage=Usd.Stage.Open(str(source/'scene-authored.usda'));paths=json.load(open(source/'body-paths.json'));reference=np.load(RUNS/'native-roas-table-bin-under30-capture01/full-command-trace.npz');meta=json.load(open(RUNS/'native-roas-table-bin-under30-capture01/pipeline-results.json'));goals=json.load(open(source/'arm-plan.json'))['goals']
    # Diagnostic initial condition: the first ring is already absent from the bin.
    xf=UsdGeom.Xformable(stage.GetPrimAtPath(paths['workpiece_ring1']));xf.ClearXformOpOrder();xf.SetResetXformStack(True);xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Matrix4d().SetTranslate(Gf.Vec3d(*goals['ring1']['P'])))
    drives={p.GetName():UsdPhysics.DriveAPI(p,'angular')for p in stage.Traverse()if p.IsA(UsdPhysics.RevoluteJoint)and '_arm_fr3v2_joint'in p.GetName()}
    for side in ['left','right']:
        gear=PhysxSchema.PhysxMimicJointAPI.Apply(stage.GetPrimAtPath('/World/NativeJoints/'+side+'_native_right'),'rotX');gear.CreateReferenceJointRel().SetTargets(['/World/NativeJoints/'+side+'_native_left']);gear.CreateGearingAttr(-1.);gear.CreateOffsetAttr(0.);gear.CreateNaturalFrequencyAttr(1000.);gear.CreateDampingRatioAttr(1.2)
    ident=UsdUtils.StageCache.Get().Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ident);dt=1/480;sim.simulate(dt,0);sim.fetch_results();view=tensors.create_simulation_view('numpy',ident);art=view.create_articulation_view('/World/RobotMountJoint');names=list(art.shared_metatype.dof_names);ai={s:[names.index(f'{s}_arm_fr3v2_joint{j}')for j in range(1,8)]for s in ['left','right']};idx=np.array([0],np.uint32)
    cmd=reference['command'][np.searchsorted(reference['time_s'],33.889):np.searchsorted(reference['time_s'],33.889)+1].copy();cmd[0,ai['left']]=reference['command'][np.searchsorted(reference['time_s'],73.),ai['left']];cmd[0,-4:-2]=.04;cmd[0,-2:]=.008;art.set_dof_positions(cmd,idx);art.set_dof_velocities(np.zeros_like(cmd),idx);art.set_dof_position_targets(cmd,idx);art.set_dof_stiffnesses(np.array([[12000 if n in drives else 5000 for n in names]],np.float32),idx);art.set_dof_dampings(np.array([[600 if n in drives else 100 for n in names]],np.float32),idx);art.set_dof_max_forces(np.array([[(87 if int(n[-1])<=4 else 12)*.95 if n in drives else 30 for n in names]],np.float32),idx)
    parts={p:view.create_rigid_body_view(paths['workpiece_'+p])for p in ['ring1','ring2','pin']};audit=Audit(cmd,ai,dt);t=0.;rows=[];phase='settle'
    def step(new):
        global cmd,t
        old=cmd.copy();cmd[:]=new;audit.update(cmd);art.set_dof_position_targets(cmd,idx);art.set_dof_velocity_targets((cmd-old)/dt,idx);t+=dt;sim.simulate(dt,t);sim.fetch_results()
        if round(t/dt)%8==0:rows.append(dict(t=t,q=art.get_dof_positions()[0].copy(),links=art.get_link_transforms()[0].copy(),parts=np.array([v.get_transforms()[0]for v in parts.values()]),phase=phase))
    def wait(seconds):
        for _ in range(round(seconds/dt)):step(cmd.copy())
    def move(goal,minimum,label):
        global phase
        phase=label;start=cmd[0,ai['right']].copy();N=int(np.ceil(duration([start,goal],minimum)/dt))
        for j in range(N):
            u=(j+1)/N;h=u**3*(10-15*u+6*u*u);new=cmd.copy();new[0,ai['right']]=start+(goal-start)*h;step(new)
    def fingers(width):
        global phase
        phase='finger_close';goal=width/2
        while abs(cmd[0,-1]-goal)>1e-7:
            new=cmd.copy();new[0,-2:]+=np.clip(goal-new[0,-2:],-.047*dt,.047*dt);step(new)
    wait(.2);initial=parts['ring2'].get_transforms()[0].copy();contact=reference['command'][np.searchsorted(reference['time_s'],36.9),ai['right']];move(contact,.9,'rim_approach');wait(.12);fingers(.0035);wait(.25);before=parts['ring2'].get_transforms()[0].copy();arm=Arm(stage,'right',drives,art.get_dof_positions()[0,ai['right']]);H=arm.fk(cmd[0,ai['right']]);goal=arm.solve(H[:3,3]+np.array([0,-.08,0]),H[:3,:3],cmd[0,ai['right']]);assert goal is not None
    move(goal,1.0,'planar_drag');wait(.2);final=parts['ring2'].get_transforms()[0].copy();delta=final[:3]-before[:3];normal0=rotation(before[3:])[:,2];normal1=rotation(final[3:])[:,2];axis=float(np.arccos(np.clip(normal0@normal1,-1,1)));passed=abs(delta[1]+.08)<.008 and np.linalg.norm(delta[[0,2]])<.008 and axis<.15
    result=dict(planar_reposition_pass=bool(passed),full_task_pass=False,initial_first_ring_already_installed=True,workpiece_pose_writes_after_initialization=0,part_attachments=False,delta_m=delta.tolist(),axis_change_rad=axis,before=before.tolist(),after=final.tolist(),physics_time_s=t,command_limit_audit=audit.report());(OUT/'drag-result.json').write_text(json.dumps(result,indent=2));np.savez_compressed(OUT/'physx-sequence-trace.npz',time_s=[r['t']for r in rows],q=[r['q']for r in rows],links=[r['links']for r in rows],parts=[r['parts']for r in rows],phase=[r['phase']for r in rows],part_names=list(parts));print('PLANAR_DRAG_RESULT',result,flush=True);sim.detach_stage();omni.kit.app.get_app().post_quit(0)
except Exception:
    (OUT/'error.txt').write_text(traceback.format_exc());traceback.print_exc();omni.kit.app.get_app().post_quit(1)
