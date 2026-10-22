"""Run the converted scene with PhysX and exercise measured robot joint control."""
from __future__ import annotations
import argparse
import json
import math
import os
import sys
import time
import traceback
from pathlib import Path
import numpy as np

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--scene',choices=['exchange','m80'],default='exchange')
parser.add_argument('--steps',type=int,default=240)
parser.add_argument('--probe-only',action='store_true')
parser.add_argument('--output',type=Path)
parser.add_argument('--video',action='store_true')
args,_=parser.parse_known_args()
here=Path(__file__).resolve().parent
root=here/'generated'/args.scene
out=args.output.resolve() if args.output else here/'runs'/(args.scene+'-physics')
out.mkdir(parents=True,exist_ok=True)
status=out/'status.json'

def record(**items):
    tmp=status.with_suffix('.tmp');tmp.write_text(json.dumps(items,indent=2)+'\n');tmp.replace(status)

record(state='running',phase='app_start')
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'multi_gpu':False,'anti_aliasing':0,'width':1280,'height':720})
code=0
writer=None
started=time.monotonic()
try:
    import omni.usd
    import omni.timeline
    import omni.kit.app
    from pxr import Usd,UsdGeom,UsdLux,UsdPhysics,PhysxSchema,Gf,Sdf
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.experimental.prims import Articulation,RigidPrim
    import isaacsim.core.experimental.utils.app as app_utils
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate('isaacsim.asset.importer.mjcf',True)
    import usdex.core
    converted=json.loads((root/'conversion-status.json').read_text())
    source=json.loads((root/'source.json').read_text())
    assert converted['state']=='passed'
    assert converted['prepared_sha256']==source['prepared_sha256'],'Converted asset is stale; run conversion again'
    assert converted['asset_digest']==source['asset_digest'],'Source mesh/texture changed; run conversion again'
    usd=here/converted['usd_path']
    record(state='running',phase='author_physx_scene')
    scene_path=out/'scene.usda'
    stage=Usd.Stage.CreateNew(str(scene_path))
    world=UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.0)
    cell=UsdGeom.Xform.Define(stage,'/World/Cell').GetPrim()
    cell.GetReferences().AddReference(os.path.relpath(usd,out))
    cell.GetVariantSet('Physics').SetVariantSelection('physx')
    joint_names=[j['name'] for j in source['joints']]
    valid_names=usdex.core.getValidChildNames(cell,joint_names)
    source_by_usd=dict(zip(valid_names,joint_names))
    body_names=source['body_names'][1:]
    body_by_usd=dict(zip(usdex.core.getValidChildNames(cell,body_names),body_names))
    joint_prims={source_by_usd.get(p.GetName(),p.GetName()):p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)}
    body_data={b['name']:b for b in source['bodies']}
    adaptations={'d6_outside_articulation':[],'d6_locked_axes':[],'restored_mimic_joints':[],'massless_fixed_frames':[], 'gravity_compensated_bodies':[],
                 'mass_properties_from_compiled_mujoco':[]}
    # The converter put mimic API tags in the shared layer but left the
    # relationship in the MuJoCo-only variant. Restore the source linear rule.
    for equality in source['equalities']:
        if equality['type']!='joint':continue
        coefficients=[float(v) for v in equality.get('polycoef','0 1 0 0 0').split()]
        assert coefficients[2:]==[0,0,0],'Nonlinear source joint equality requires a dedicated controller'
        prim=joint_prims[equality['joint1']]
        target=joint_prims[equality['joint2']]
        prim.CreateRelationship('newton:mimicJoint').SetTargets([target.GetPath()])
        prim.CreateAttribute('newton:mimicEnabled',Sdf.ValueTypeNames.Bool).Set(True)
        prim.CreateAttribute('newton:mimicCoef0',Sdf.ValueTypeNames.Double).Set(math.degrees(coefficients[0]))
        prim.CreateAttribute('newton:mimicCoef1',Sdf.ValueTypeNames.Double).Set(coefficients[1])
        adaptations['restored_mimic_joints'].append(equality['joint1'])
    for prim in stage.Traverse():
        name=body_by_usd.get(prim.GetName(),prim.GetName())
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI) or name not in body_data:continue
        body=body_data[name]
        if body['mass']==0:
            # PhysX requires positive inertia even for massless fixed frames.
            # Keep this numerical regularization explicit in the report.
            mass=UsdPhysics.MassAPI.Apply(prim)
            mass.CreateMassAttr(1e-6);mass.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-9))
            adaptations['massless_fixed_frames'].append(name)
        else:
            mass=UsdPhysics.MassAPI.Apply(prim)
            mass.CreateMassAttr(body['mass'])
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*body['inertia']))
            mass.CreateCenterOfMassAttr(Gf.Vec3f(*body['center_of_mass']))
            q=body['inertial_quaternion']
            mass.CreatePrincipalAxesAttr(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
            adaptations['mass_properties_from_compiled_mujoco'].append(name)
        if body['gravity_compensation']==1:
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
            adaptations['gravity_compensated_bodies'].append(name)
        else:
            assert body['gravity_compensation']==0,'Partial gravity compensation requires an explicit force controller'
    scene=UsdPhysics.Scene.Define(stage,'/PortPhysicsScene')
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));scene.CreateGravityMagnitudeAttr(9.81)
    physics=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physics.CreateTimeStepsPerSecondAttr(120)
    physics.CreateEnableGPUDynamicsAttr(False)
    physics.CreateSolverTypeAttr('TGS')
    # The planar base is a regular D6 constraint on the floating G2 root.
    # PhysX reduced-coordinate articulation joints cannot represent a D6.
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint) and prim.GetName()=='base_x' and prim.GetTypeName()=='PhysicsJoint':
            UsdPhysics.Joint(prim).CreateExcludeFromArticulationAttr(True)
            adaptations['d6_outside_articulation'].append(str(prim.GetPath()))
            # A generic D6 defaults to six free axes. The source has only
            # planar x/y/yaw joints, so explicitly restore its other constraints.
            for axis in ['transZ','rotX','rotY']:
                limit=UsdPhysics.LimitAPI.Apply(prim,axis)
                limit.CreateLowAttr(1.0);limit.CreateHighAttr(-1.0)
                adaptations['d6_locked_axes'].append(axis)
            for axis,name in [('transX','base_x'),('transY','base_y'),('rotZ','base_yaw')]:
                target=next(j['seed_position'] for j in source['joints'] if j['name']==name)
                if axis.startswith('rot'):target=math.degrees(target)
                UsdPhysics.DriveAPI.Apply(prim,axis).CreateTargetPositionAttr(target)
    UsdLux.DomeLight.Define(stage,'/PortDome').CreateIntensityAttr(600)
    light=UsdLux.DistantLight.Define(stage,'/PortSun');light.CreateIntensityAttr(1200)
    UsdGeom.Xformable(light).AddRotateXYZOp().Set(Gf.Vec3f(-35,25,30))
    camera=UsdGeom.Camera.Define(stage,'/PortCamera')
    camera.CreateFocalLengthAttr(34);camera.CreateHorizontalApertureAttr(36);camera.CreateVerticalApertureAttr(20.25)
    look=Gf.Matrix4d().SetLookAt(Gf.Vec3d(5,-8,5),Gf.Vec3d(-.8,-.7,1.25),Gf.Vec3d(0,0,1))
    UsdGeom.Xformable(camera).AddTransformOp().Set(look.GetInverse())
    reference=np.load(root/'reference.npz')
    body_index={name:i for i,name in enumerate(source['body_names'])}
    cache=UsdGeom.XformCache()
    pose_errors=[]
    for prim in stage.Traverse():
        name=body_by_usd.get(prim.GetName(),prim.GetName())
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI) or name not in body_index:continue
        index=body_index[name];matrix=cache.GetLocalToWorldTransform(prim)
        position=np.asarray(matrix.ExtractTranslation())
        q=matrix.ExtractRotationQuat();quat=np.r_[q.GetReal(),q.GetImaginary()]
        angle=2*math.acos(np.clip(abs(np.dot(quat,reference['body_quaternions'][index])),0,1))
        pose_errors.append((name,float(np.linalg.norm(position-reference['body_positions'][index])),angle))
    assert len(pose_errors)==source['nbody']-1
    max_position_error=max(row[1] for row in pose_errors)
    max_angle_error=max(row[2] for row in pose_errors)
    assert max_position_error<1e-5,(max_position_error,sorted(pose_errors,key=lambda v:v[1])[-3:])
    assert max_angle_error<1e-3,(max_angle_error,sorted(pose_errors,key=lambda v:v[2])[-3:])
    pose_audit={'body_count':len(pose_errors),'max_position_error_m':max_position_error,'max_angle_error_rad':max_angle_error,'passed':True}
    (out/'initial-pose-audit.json').write_text(json.dumps(pose_audit,indent=2)+'\n')
    stage.GetRootLayer().Save()
    assert omni.usd.get_context().open_stage(str(scene_path))
    stage=omni.usd.get_context().get_stage()
    # Live measurements and body reset poses are transient; the authored scene
    # above remains a reproducible reference to the converted asset.
    stage.SetEditTarget(stage.GetSessionLayer())
    for _ in range(3):app.update()
    SimulationManager.setup_simulation(dt=1/120,device='cpu')
    assert SimulationManager.get_active_physics_engine()=='physx'
    timeline=omni.timeline.get_timeline_interface()
    timeline.set_play_every_frame(True);timeline.set_target_framerate(120);stage.SetTimeCodesPerSecond(120)
    record(state='running',phase='physics_initialization')
    app_utils.play()
    app.update()
    timeline.pause()
    root_paths=[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    print('ARTICULATION_ROOTS',root_paths,flush=True)
    joints={j['name']:j for j in source['joints']}
    articulations=[];inventory=[]
    for path in root_paths:
        art=Articulation(path)
        names=art.dof_names
        print('ARTICULATION',path,'DOFS',names,flush=True)
        inventory.append({'path':path,'num_dofs':art.num_dofs,'dof_names':names})
        positions=art.get_dof_positions().numpy()
        for i,name in enumerate(names):
            source_name=source_by_usd.get(name,name)
            if source_name in joints and joints[source_name]['seed_position'] is not None:
                positions[0,i]=joints[source_name]['seed_position']
        if art.num_dofs:
            art.set_dof_positions(positions)
            art.set_dof_velocities(np.zeros_like(positions))
            art.set_dof_position_targets(positions)
        # Initialize floating magazine roots from the recorded source state.
        prim=stage.GetPrimAtPath(path)
        body_name=body_by_usd.get(prim.GetName(),prim.GetName())
        if body_name in body_index:
            index=body_index[body_name]
            art.set_world_poses(positions=reference['seed_body_positions'][index:index+1],
                                orientations=reference['seed_body_quaternions'][index:index+1])
            art.set_velocities(linear_velocities=[0,0,0],angular_velocities=[0,0,0])
        articulations.append((art,positions.copy()))
    (out/'articulations.json').write_text(json.dumps(inventory,indent=2)+'\n')
    # Free workpieces that are not articulation roots need the same seed pose.
    free_bodies={j['body_name'] for j in joints.values() if j['type']==0}
    for prim in stage.Traverse():
        name=body_by_usd.get(prim.GetName(),prim.GetName())
        if name in free_bodies and not prim.HasAPI(UsdPhysics.ArticulationRootAPI) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            index=body_index[name]
            RigidPrim(str(prim.GetPath())).set_world_poses(positions=reference['seed_body_positions'][index:index+1],
                    orientations=reference['seed_body_quaternions'][index:index+1])
    # Publish the tensor reset poses to the renderer before the first capture.
    timeline.play();app.update();timeline.pause()
    if args.probe_only:
        record(state='passed',phase='probe',inventory=inventory,adaptations=adaptations)
    else:
        controls=[('left_arm_fr3v2_joint1',.12),('right_arm_fr3v2_joint1',-.12),
                  ('idx21_arm_l_joint1',.08),('idx61_arm_r_joint1',-.08)]
        control_bindings=[]
        for name,amplitude in controls:
            art,initial=next(pair for pair in articulations if name in pair[0].dof_names)
            control_bindings.append((name,amplitude,art,art.dof_names.index(name),float(initial[0,art.dof_names.index(name)])))
        g2=next(art for art,_ in articulations if 'idx21_arm_l_joint1' in art.dof_names)
        # The source G2 is mounted with a 90-degree yaw: its base_x coordinate
        # moves along world +Y. Validate the source Jacobian axis, not the label.
        base_axis=np.asarray(source['base_axes']['base_x']['world_linear_axis'])
        np.testing.assert_allclose(np.linalg.norm(base_axis),1.0,atol=1e-6)
        base_start=float(np.dot(g2.get_world_poses()[0].numpy()[0],base_axis))
        base_seed=joints['base_x']['seed_position']
        base_height=float(reference['seed_body_positions'][body_index['base_link'],2])
        max_height_error=0.0;max_tilt=0.0
        base_drive=UsdPhysics.DriveAPI(stage.GetPrimAtPath(joint_prims['base_x'].GetPath()),'transX')
        max_mimic_errors={e['joint1']:0.0 for e in source['equalities'] if e['type']=='joint'}
        readings=[];writer=None;rgb=None;product=None
        if args.video:
            import omni.replicator.core as rep
            import imageio.v2 as imageio
            from PIL import Image
            rep.orchestrator.set_capture_on_play(False)
            product=rep.create.render_product('/PortCamera',(1280,720))
            rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(product)
            writer=imageio.get_writer(str(out/'physx-joint-control.mp4'),fps=30,codec='libx264',quality=8,
                                     macro_block_size=2,ffmpeg_params=['-movflags','+faststart'])
            def capture():
                rep.orchestrator.step(delta_time=0,rt_subframes=1,pause_timeline=True)
                frame=np.ascontiguousarray(rgb.get_data()[:,:,:3])
                assert frame.size and frame.std()>1
                return frame
            first=capture();Image.fromarray(first).save(out/'initial.png')
            for _ in range(30):writer.append_data(first)
        record(state='running',phase='joint_control',inventory=inventory)
        for step in range(args.steps):
            fraction=(step+1)/args.steps
            fraction_shape=math.sin(math.pi*fraction)**2
            target_values=[]
            for art,initial in articulations:
                targets=initial.copy()
                for name,amplitude,bound,index,initial_position in control_bindings:
                    if art is bound:targets[0,index]=initial_position+amplitude*fraction_shape
                if art.num_dofs:art.set_dof_position_targets(targets)
            for name,amplitude,art,index,initial_position in control_bindings:
                target_values.append(initial_position+amplitude*fraction_shape)
            base_target=base_start+.08*fraction_shape
            base_drive.GetTargetPositionAttr().Set(base_seed+.08*fraction_shape)
            timeline.play();app.update();timeline.pause()
            actual_values=[float(art.get_dof_positions().numpy()[0,index]) for _,_,art,index,_ in control_bindings]
            base_position,base_quaternion=g2.get_world_poses()
            base_position=base_position.numpy()[0];base_quaternion=base_quaternion.numpy()[0]
            base_actual=float(np.dot(base_position,base_axis))
            max_height_error=max(max_height_error,abs(float(base_position[2])-base_height))
            tilt=math.acos(np.clip(1-2*float(base_quaternion[1]**2+base_quaternion[2]**2),-1,1))
            max_tilt=max(max_tilt,tilt)
            assert max_height_error<.005 and max_tilt<.01,('Base left source plane',max_height_error,max_tilt)
            readings.append([step/120,*target_values,*actual_values,base_target,base_actual])
            for other,_ in articulations:
                assert np.isfinite(other.get_dof_positions().numpy()).all()
                p,q=other.get_world_poses()
                assert np.isfinite(p.numpy()).all() and np.isfinite(q.numpy()).all()
            for equality in source['equalities']:
                if equality['type']!='joint':continue
                mimic=next(art for art,_ in articulations if equality['joint1'] in art.dof_names)
                q=mimic.get_dof_positions().numpy()[0]
                a,b=[mimic.dof_names.index(equality[name]) for name in ['joint1','joint2']]
                coefficients=[float(v) for v in equality.get('polycoef','0 1 0 0 0').split()]
                error=abs(float(q[a])-coefficients[0]-coefficients[1]*float(q[b]))
                max_mimic_errors[equality['joint1']]=max(max_mimic_errors[equality['joint1']],error)
            if writer is not None and step%4==0:
                frame=capture();writer.append_data(frame)
            if step%30==0: print('PHYSX_STEP',step,'targets',target_values,'measured',actual_values,flush=True)
        readings=np.asarray(readings)
        if writer is not None:
            Image.fromarray(frame).save(out/'final.png')
            for _ in range(30):writer.append_data(frame)
            writer.close();rgb.detach(product);product.destroy()
        np.save(out/'joint-trace.npy',readings)
        movements=np.ptp(readings[:,5:9],axis=0)
        errors=np.max(np.abs(readings[:,1:5]-readings[:,5:9]),axis=0)
        tracking={name:{'measured_motion_rad':float(motion),'max_tracking_error_rad':float(error)} for (name,_),motion,error in zip(controls,movements,errors)}
        assert np.all(movements>.02),f'Joint failed to move: {tracking}'
        assert np.all(errors<.03),f'Joint tracking diverged: {tracking}'
        base_motion=float(np.ptp(readings[:,10]));base_error=float(np.max(np.abs(readings[:,9]-readings[:,10])))
        assert base_motion>.02 and base_error<.03,('Planar base failed to track',base_motion,base_error)
        assert max(max_mimic_errors.values())<.01,max_mimic_errors
        record(state='passed',phase='joint_control',physics_engine='physx',physics_device='cpu',
               steps=args.steps,dt=1/120,tracking=tracking,base_tracking={'measured_motion_m':base_motion,'max_error_m':base_error,'world_axis':base_axis.tolist()},
               planar_constraint={'max_height_error_m':max_height_error,'max_tilt_rad':max_tilt,'passed':True},
               mimic_max_errors_rad=max_mimic_errors,
               trace_columns=['time_s']+[name+'_target_rad' for name,_ in controls]+[name+'_measured_rad' for name,_ in controls]+['base_x_target_m','base_x_measured_m'],
               initial_pose_audit=pose_audit,inventory=inventory,adaptations=adaptations,seconds=time.monotonic()-started,
               physical_assembly_pass=False,
               scope='PhysX initialization and joint-drive smoke test; no grasp, transfer, insertion or full physical task validation')
        print('PORT_PHYSICS_PASS',tracking,flush=True)
except Exception as exc:
    traceback.print_exc();record(state='failed',error=repr(exc),seconds=time.monotonic()-started);code=1
finally:
    if writer is not None:
        writer.close()
    app.close(exit_code=code)
sys.exit(code)
