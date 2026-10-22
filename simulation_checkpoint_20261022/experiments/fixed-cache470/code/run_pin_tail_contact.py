"""Independent native-Hand tail-grasp and one-stroke pin insertion trial.

Two rings start installed solely to reproduce insertion obstacles. This is not
an end-to-end three-part timing result. Live control uses RGB-D and encoders.
"""
from pathlib import Path
import os,sys,time,json,traceback,numpy as np
from pxr import Usd,Sdf,UsdUtils,UsdPhysics,PhysxSchema,UsdGeom,Gf
import carb,omni.physx,omni.physics.tensors as tensors,omni.kit.app
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE/'bin-picking/realtime-deps-python312'),str(HERE),str(HERE.parents[1]),str(HERE.parents[2])];OUT=HERE.parents[1]/'runs'/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True)
from isaac_kinematics import Arm,frame,exp,log
from rgbd_geometry import axis_frame
from process_vision import Vision,CachedGeometryCamera
from isaac_collision_queries import Checker
os.environ['M80_ROAS_RGB_ONLY']='1'
from fr3_motion_control import position_velocity_bounds
from scipy.interpolate import CubicSpline
report=dict(scope='Independent pin contact trial with both rings initially installed; not a full cycle',full_task_pass=False,vision_end_to_end_pass=False,pin_single_stroke_pass=False,hardware_tested=False,part_attachments=False,workpiece_teleports_after_initialization=0,control_input='Fresh RGB-D and robot encoders',events=[]);DT=1/480;tape=[];issued=[];physical=0.;cfg=json.loads((OUT/'trial-config.json').read_text())if(OUT/'trial-config.json').exists()else{};candidate_index=int(cfg.get('candidate',0))
try:
    prep=time.perf_counter();settings=carb.settings.get_settings();settings.set_int('/persistent/physics/numThreads',0);settings.set_bool('/physics/physxDispatcher',True);base=HERE.parents[1]/'runs/native-roas-table-bin-retimed14';stage=Usd.Stage.Open(str(base/'scene-authored.usda'));paths=json.load(open(base/'body-paths.json'));drives={p.GetName():UsdPhysics.DriveAPI(p,'angular')for p in stage.Traverse()if p.IsA(UsdPhysics.RevoluteJoint)and '_arm_fr3v2_joint'in p.GetName()}
    for prim in stage.Traverse():
        if prim.HasAPI(PhysxSchema.PhysxSceneAPI):PhysxSchema.PhysxSceneAPI(prim).CreateEnableSceneQuerySupportAttr(True);PhysxSchema.PhysxSceneAPI(prim).CreateEnableCCDAttr(True)
        if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):PhysxSchema.PhysxRigidBodyAPI(prim).CreateSolverPositionIterationCountAttr(16);PhysxSchema.PhysxRigidBodyAPI(prim).CreateSolverVelocityIterationCountAttr(4)
        if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):PhysxSchema.PhysxArticulationAPI(prim).CreateSolverPositionIterationCountAttr(16);PhysxSchema.PhysxArticulationAPI(prim).CreateSolverVelocityIterationCountAttr(4)
    if cfg.get('pin_clamp_center_z_m') is not None:
        height=float(cfg['pin_clamp_center_z_m'])
        for sideidx in range(2):
            pth='/World/Clamps/pin/jaw_'+str(sideidx);body=stage.GetPrimAtPath(pth);joint=UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(pth+'/Actuator'));v=joint.GetLocalPos0Attr().Get();joint.GetLocalPos0Attr().Set(Gf.Vec3f(v[0],v[1],height))
            for op in UsdGeom.Xformable(body).GetOrderedXformOps():
                if op.GetOpType()==UsdGeom.XformOp.TypeTransform:
                    M=op.Get();v=M.ExtractTranslation();M.SetTranslateOnly(Gf.Vec3d(v[0],v[1],height));op.Set(M)
        report['pin_clamp_assumed_center_z_m']=height;report['pin_clamp_hardware_calibrated']=False
    for side in ['left','right']:
        master=stage.GetPrimAtPath('/World/NativeJoints/'+side+'_native_left');follower=stage.GetPrimAtPath('/World/NativeJoints/'+side+'_native_right');gear=PhysxSchema.PhysxMimicJointAPI.Apply(follower,'rotX');gear.CreateReferenceJointRel().SetTargets([master.GetPath()]);gear.CreateReferenceJointAxisAttr('rotX');gear.CreateGearingAttr(-1.);gear.CreateOffsetAttr(0.);gear.CreateNaturalFrequencyAttr(1000.);gear.CreateDampingRatioAttr(1.2)
    sid=UsdUtils.StageCache.Get().Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(sid);sim.simulate(DT,0);sim.fetch_results();view=tensors.create_simulation_view('numpy',sid);art=view.create_articulation_view('/World/RobotMountJoint');names=list(art.shared_metatype.dof_names);links=list(art.shared_metatype.link_names);ids=[names.index('right_arm_fr3v2_joint'+str(j))for j in range(1,8)];fi=[i for i,n in enumerate(names)if n.startswith('right_native')];idx=np.array([0],np.uint32);initial=np.load(HERE.parents[1]/'runs/native-roas-table-rgbd-control05/executed-actuator-commands.npz')['command'][:1].copy();cmd=initial.copy();art.set_dof_positions(cmd,idx);art.set_dof_velocities(np.zeros_like(cmd),idx);art.set_dof_position_targets(cmd,idx);art.set_dof_velocity_targets(np.zeros_like(cmd),idx);stiff=np.array([[12000 if n in drives else 5000 for n in names]],np.float32);damping=np.array([[600 if n in drives else 100 for n in names]],np.float32);force=np.array([[(87 if int(n[-1])<=4 else 12)*.95 if n in drives else 30 for n in names]],np.float32);art.set_dof_stiffnesses(stiff,idx);art.set_dof_dampings(damping,idx);art.set_dof_max_forces(force,idx);sim.simulate(DT,DT);sim.fetch_results();arm=Arm(stage,'right',drives,art.get_dof_positions()[0,ids]);cand=json.load(open(HERE.parents[1]/'runs/native-roas-table-pin-tail01/pin-direct-probe.json'))['candidates'][candidate_index];G=np.array(cand['grasp_transform']);goals=json.load(open(base/'arm-plan.json'))['goals'];goal=np.array(goals['pin']['P']);pickupH=np.array(cand['pickup_H']);preH=pickupH.copy();preH[:3,3]+=[0,0,.07];preq=arm.solve(preH[:3,3],preH[:3,:3],cand['pickup_q']);assert preq is not None
    old=np.load(HERE.parents[1]/'runs/native-roas-table-wall-live51/physx-sequence-trace.npz');oldr=json.load(open(HERE.parents[1]/'runs/native-roas-table-wall-live51/pipeline-results.json'));row=np.argmin(abs(old['phase_time_s']-22.));cmd[:]=old['q'][row];cmd[0,ids]=old['q'][np.argmin(abs(old['phase_time_s']-16.5)),ids];cmd[0,fi]=.014/2;art.set_dof_positions(cmd,idx);art.set_dof_velocities(np.zeros_like(cmd),idx);art.set_dof_position_targets(cmd,idx);art.set_dof_velocity_targets(np.zeros_like(cmd),idx)
    for path,tr in zip(old['camera_body_paths'].tolist(),old['camera_bodies'][row]):
        if '/official_FR3_Duo'in path or path==paths['workpiece_pin']:continue
        b=view.create_rigid_body_view(path);b.set_transforms(tr[None,:].copy(),idx);b.set_velocities(np.zeros((1,6),np.float32),idx)
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.PrismaticJoint)and str(prim.GetPath()).startswith('/World/Clamps/'):
            joint=UsdPhysics.PrismaticJoint(prim);UsdPhysics.DriveAPI(prim,'linear').GetTargetPositionAttr().Set(0. if '/pin/'in str(prim.GetPath())else joint.GetUpperLimitAttr().Get())
    sim.flush_changes()
    for _ in range(60):sim.simulate(DT,0);sim.fetch_results()
    camera=CachedGeometryCamera(view,HERE.parents[1]/'runs/native-roas-table-wall-live13/camera-local-models.npz');vision=Vision(camera,json.load(open(HERE/'bin-picking/asset-audit.json'))['baskets'][0],OUT);truth={p:view.create_rigid_body_view(paths['workpiece_'+p])for p in ['ring1','ring2','pin']};checker=Checker(stage,arm,'right');checker.ignored_future_actors={paths['workpiece_pin']};report.update(candidate=cand,physics_hz=480,starts_with_rings_already_installed=True,solver_iterations=dict(position=16,velocity=4),dof_names=names,link_names=links,standby_preparation_wall_s=time.perf_counter()-prep);settings.set_bool('/physics/updateToUsd',False);settings.set_bool('/physics/updateVelocitiesToUsd',False);started=time.perf_counter();vision.start=started;phase='first_capture';prev_cmd=cmd.copy()
    def step():
        global physical,prev_cmd
        tick=time.perf_counter();art.set_dof_position_targets(cmd,idx);art.set_dof_velocity_targets((cmd-prev_cmd)/DT,idx);prev_cmd=cmd.copy();sim.simulate(DT,physical);sim.fetch_results();physical+=DT
        remaining=tick+DT-time.perf_counter()
        if remaining>.0003:time.sleep(remaining-.0003)
        while time.perf_counter()<tick+DT:pass
        issued.append(dict(t=time.perf_counter()-started,q=cmd[0].copy(),phase=phase))
        if round(physical/DT)%4==0:tape.append(dict(t=time.perf_counter()-started,sim=physical,phase=phase,q=art.get_dof_positions()[0].copy(),links=art.get_link_transforms()[0].copy(),camera=camera.all_bodies.get_transforms().copy(),parts=np.array([v.get_transforms()[0]for v in truth.values()])))
    def wait(seconds,label):
        global phase
        phase=label
        for _ in range(round(seconds/DT)):step()
    def observe(label,grasp=None):
        e=dict(mode='pin_initial'if grasp is None else'held',part='pin',label=label)
        if grasp is not None:e.update(tracked_grasp=grasp.tolist(),camera_eye_offset=([-.22,0,.12] if label=="pin_lift_RGBD" else [-.22,-.15,-.12]))
        H=arm.fk(art.get_dof_positions()[0,ids]);f=vision.request(e,H);deadline=time.perf_counter()+1.
        while not f.done():
            vision.poll();step()
            if time.perf_counter()>deadline:raise RuntimeError('RGB-D response timeout')
        value=f.result();e['delivered_wall_s']=time.perf_counter()-started
        if value['result'].get('error'):raise RuntimeError(value['result']['error'])
        report['events'].append(dict(event=label,wall_s=e['delivered_wall_s'],result=value['result']));print('PIN_CONTACT_VISION',label,e['delivered_wall_s'],flush=True);return value
    def motion(qs,requested,label):
        global phase
        qs=np.array(qs);qs[0]=cmd[0,ids];T=requested
        # Smooth joint spline sampled at the real control rate, checked against
        # position-dependent bounds, acceleration and jerk before execution.
        spline=CubicSpline(np.linspace(0,1,len(qs)),qs)
        for _ in range(15):
            N=max(2,int(np.ceil(T/DT)));u=np.linspace(0,1,N+1);h=u**3*(10-15*u+6*u*u);q=spline(h).astype(np.float32).astype(float);v=np.diff(q,axis=0,prepend=q[:1])/DT;a=np.diff(v,axis=0,prepend=np.zeros((1,7)))/DT;j=np.diff(a,axis=0,prepend=np.zeros((1,7)))/DT;lo,hi=position_velocity_bounds(q);ratio=np.max(abs(v)/np.maximum(.95*np.where(v>=0,hi,-lo),1e-8));factor=max(ratio,np.sqrt(abs(a).max()/9.5),np.cbrt(abs(j).max()/4750))
            if factor<=.98:break
            T*=max(1.04,float(factor)*1.03)
        else:raise RuntimeError('Cannot time pin segment within FR3 limits')
        phase=label
        for row in q[1:]:cmd[0,ids]=row;step()
        report['events'].append(dict(event=label,wall_s=time.perf_counter()-started,physical_duration_s=(len(q)-1)*DT));print('PIN_CONTACT_MOVE',label,time.perf_counter()-started,flush=True)
    def line(H0,H1,requested,label):
        rows=[cmd[0,ids].copy()];last=rows[0]
        for u in np.linspace(0,1,17)[1:]:
            T=H0.copy();T[:3,3]=H0[:3,3]*(1-u)+H1[:3,3]*u;q=arm.solve(T[:3,3],T[:3,:3],last)
            if q is None:raise RuntimeError('Axial IK failed '+label)
            rows.append(q);last=q
        motion(rows,requested,label)
    def fingers(width,label):
        global phase
        phase=label;start=cmd[0,fi].copy();target=np.array([width/2]*2);T=max(.14,float(abs(target-start).max())/.0475*1.875);N=int(np.ceil(T/DT))
        for k in range(1,N+1):u=k/N;h=u**3*(10-15*u+6*u*u);cmd[0,fi]=start+(target-start)*h;step()
    v=observe('pin_initial');P=np.array(v['result']['pose']['center_m']);pickupH[:3,3]+=P-np.array(json.load(open(HERE.parents[1]/'runs/native-roas-table-pin-tail01/pin-direct-probe.json'))['pickup_center_from_RGBD']);newpre=pickupH.copy();newpre[:3,3]+=[0,0,.07];newpreq=arm.solve(newpre[:3,3],newpre[:3,:3],preq);route=checker.route(cmd[0,ids],newpreq,.014,paths['workpiece_pin'],seconds=3);assert route is not None,'No pregrasp route';motion(route,1.5,'pin_tail_approach');H=arm.fk(art.get_dof_positions()[0,ids]);line(H,pickupH,.6,'pin_tail_descend');stiff[0,fi]=30000.;damping[0,fi]=300.;art.set_dof_stiffnesses(stiff,idx);art.set_dof_dampings(damping,idx);fingers(.0095,'pin_tail_close');wait(.15,'pin_tail_grasp_hold');H=arm.fk(art.get_dof_positions()[0,ids]);lift=H.copy();lift[:3,3]+=[0,0,.07];line(H,lift,.6,'pin_tail_lift');wait(.15,'pin_lift_settle');v=observe('pin_lift_RGBD',G);p=v['result']['pose'];Gobs=np.linalg.inv(v['H'])@frame(p['center_m'],axis_frame(p['axis']));axis0=G[:3,2];axis1=Gobs[:3,2];cross=np.cross(axis0,axis1);angle=np.arctan2(np.linalg.norm(cross),axis0@axis1);Gobs[:3,:3]=exp(cross/max(np.linalg.norm(cross),1e-12)*angle)@G[:3,:3];G=Gobs;report['measured_grasp']=G.tolist();R=np.array(cand['goal_part_R']);safe=frame([-.10,-.25,1.35],R)@np.linalg.inv(G);pre=frame(goal-[0,0,.045],R)@np.linalg.inv(G);end=frame(goal,R)@np.linalg.inv(G)
    for target,label in [(safe,'pin_tail_orient_free'),(pre,'pin_tail_carry_to_preinsert')]:
        q=arm.solve(target[:3,3],target[:3,:3],cand['insert_waypoints'][0] if label=='pin_tail_carry_to_preinsert' else cmd[0,ids])
        if q is None:raise RuntimeError('Measured tail grasp IK failed '+label)
        checker.attach_payload(paths['workpiece_pin'],frame([0,0,.1034],np.eye(3))@G);print('PIN_CARRY_GOAL_CLEAR',label,checker.clear(q,.0105,paths['workpiece_pin']),flush=True);route=checker.route(cmd[0,ids],q,.0105,paths['workpiece_pin'],seconds=2);checker.detach_payload()
        if route is None:raise RuntimeError('No collision-free carry '+label)
        motion(route,1.5,label)
    v=observe('pin_preinsert_RGBD',G);Pobs=np.array(v['result']['pose']['center_m']);report['preinsert_observed_center_m']=Pobs.tolist();line(pre,end,.8,'pin_single_axial_insert');wait(.12,'pin_full_depth_hold');v=observe('pin_full_depth_RGBD',G);pose=v['result']['pose'];error=float(np.linalg.norm(np.array(pose['center_m'])-goal));ae=float(np.arccos(np.clip(np.array(pose['axis'])[2],-1,1)));report['RGBD_full_depth_check']=dict(position_error_m=error,axis_error_rad=ae)
    if error>=.0015 or ae>=.05:raise RuntimeError('Full depth not confirmed by RGB-D; fingers remain closed')
    for prim in stage.Traverse():
        if cfg.get('close_pin_clamp',True) and prim.IsA(UsdPhysics.PrismaticJoint)and str(prim.GetPath()).startswith('/World/Clamps/pin/'):
            joint=UsdPhysics.PrismaticJoint(prim);UsdPhysics.DriveAPI(prim,'linear').GetTargetPositionAttr().Set(joint.GetLowerLimitAttr().Get())
    sim.flush_changes();wait(.12,'pin_clamp_hold');fingers(.018,'pin_release_after_full_depth');withdraw=end.copy();withdraw[:3,3]-=[0,0,.07];line(end,withdraw,.65,'pin_axial_withdraw');wait(.5,'pin_retention');tr=truth['pin'].get_transforms()[0];ae=float(np.arccos(np.clip(1-2*(tr[3]**2+tr[4]**2),-1,1)));err=float(np.linalg.norm(tr[:3]-goal));report.update(state='pin_single_stroke_contact_pass'if err<.0015 and ae<.05 else'pin_retention_failed',pin_single_stroke_pass=bool(err<.0015 and ae<.05),final_position_error_m=err,final_axis_error_rad=ae,pin_trial_wall_s=time.perf_counter()-started)
except Exception:report.update(state='failed',error=traceback.format_exc());traceback.print_exc()
finally:
    if 'started'in globals():report['pin_trial_wall_s']=time.perf_counter()-started
    if tape:np.savez_compressed(OUT/'physx-sequence-trace.npz',time_s=[v['t']for v in tape],simulation_time_s=[v['sim']for v in tape],phase=[v['phase']for v in tape],q=[v['q']for v in tape],links=[v['links']for v in tape],camera_body_paths=camera.all_bodies.prim_paths,camera_bodies=[v['camera']for v in tape],parts=[v['parts']for v in tape],part_names=list(truth))
    if issued:np.savez_compressed(OUT/'executed-actuator-commands.npz',time_s=[v['t']for v in issued],command=[v['q']for v in issued],phase=[v['phase']for v in issued])
    if 'vision'in globals():
        try:vision.save(OUT);vision.close()
        except Exception:report['camera_close_error']=traceback.format_exc()
    (OUT/'pin-contact-results.json').write_text(json.dumps(report,indent=2));print('PIN_CONTACT_RESULT',report['state'],report.get('pin_trial_wall_s'),flush=True)
    if 'sim'in globals():sim.detach_stage()
    if not os.environ.get('M80_REALTIME_KEEPALIVE'):omni.kit.app.get_app().post_quit(0 if report.get('pin_single_stroke_pass')else 2)
