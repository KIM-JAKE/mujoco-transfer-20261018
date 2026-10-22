"""Isaac PhysX geometric probe: direct axial insertion with camera-measured grasp.

This is an independent checkpoint feasibility probe, never a full task success.
Recorded rigid body poses initialize the scene only. The grasp transform is
computed from RGB-D estimates plus robot encoders, not workpiece ground truth.
"""
from pathlib import Path
import os,sys,json,time,traceback,collections,numpy as np
from pxr import Usd,UsdUtils,UsdPhysics,PhysxSchema,Sdf
import carb,omni.physx,omni.physics.tensors as tensors,omni.kit.app
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parents[1]),str(HERE.parents[2])]
from isaac_kinematics import Arm,frame,exp
from rgbd_geometry import axis_frame
from isaac_collision_queries import Checker
OUT=HERE.parents[1]/'runs'/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True);source=HERE.parents[1]/'runs/native-roas-table-random17';base=HERE.parents[1]/'runs/native-roas-table-bin-retimed14';result=dict(full_task_pass=False,vision_end_to_end_pass=False,hardware_tested=False,scope='Independent geometric probe of one uninterrupted axial pin insertion; no physical execution',grasp_input='Recorded RGB-D pose plus robot encoders',fixtures_removed=False,part_attachments=False)
try:
    settings=carb.settings.get_settings();settings.set_int('/persistent/physics/numThreads',0);settings.set_bool('/physics/physxDispatcher',True)
    stage=Usd.Stage.Open(str(base/'scene-authored.usda'));paths=json.load(open(base/'body-paths.json'));drives={p.GetName():UsdPhysics.DriveAPI(p,'angular')for p in stage.Traverse()if p.IsA(UsdPhysics.RevoluteJoint)and '_arm_fr3v2_joint'in p.GetName()}
    for prim in stage.Traverse():
        if prim.HasAPI(PhysxSchema.PhysxSceneAPI):PhysxSchema.PhysxSceneAPI(prim).CreateEnableSceneQuerySupportAttr(True)
    sid=UsdUtils.StageCache.Get().Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(sid);sim.simulate(1/480,0);sim.fetch_results();view=tensors.create_simulation_view('numpy',sid);art=view.create_articulation_view('/World/RobotMountJoint');names=list(art.shared_metatype.dof_names);ids=[names.index('right_arm_fr3v2_joint'+str(j))for j in range(1,8)];fi=[i for i,n in enumerate(names)if n.startswith('right_native')];idx=np.array([0],np.uint32);initial=np.load(HERE.parents[1]/'runs/native-roas-table-rgbd-control05/executed-actuator-commands.npz')['command'][:1].copy();art.set_dof_positions(initial,idx);art.set_dof_velocities(np.zeros_like(initial),idx);art.set_dof_position_targets(initial,idx);art.set_dof_velocity_targets(np.zeros_like(initial),idx);sim.simulate(1/480,1/480);sim.fetch_results();arm=Arm(stage,'right',drives,art.get_dof_positions()[0,ids]);trace=np.load(source/'physx-sequence-trace.npz');j=int(np.argmin(abs(trace['phase_time_s']-22.9)));cp=trace['q'][j:j+1].copy();art.set_dof_positions(cp,idx);art.set_dof_velocities(np.zeros_like(cp),idx);art.set_dof_position_targets(cp,idx);art.set_dof_velocity_targets(np.zeros_like(cp),idx)
    for path,tr in zip(trace['camera_body_paths'].tolist(),trace['camera_bodies'][j]):
        if '/official_FR3_Duo'in path:continue
        b=view.create_rigid_body_view(path);b.set_transforms(tr[None,:].copy(),idx);b.set_velocities(np.zeros((1,6),np.float32),idx)
    sim.simulate(1/480,2/480);sim.fetch_results()
    obs=next(o for o in json.load(open(source/'rgb-observations.json'))if o['part']=='pin'and o['mode']=='held');qcam=np.array([np.interp(obs['captured_wall_s'],trace['time_s'],trace['q'][:,i])for i in ids]);Hcam=arm.fk(qcam);pose=obs['result']['pose'];G=np.linalg.inv(Hcam)@frame(pose['center_m'],axis_frame(pose['axis']));goal=np.array(json.load(open(source/'arm-plan.json'))['goals']['pin']['P']);opening=float(cp[0,fi].sum());checker=Checker(stage,arm,'right');checker.ignored_future_actors={paths['workpiece_pin']};attempts=[];feasible=[];hits=collections.Counter();started=time.perf_counter()
    # A circular pin permits yaw about its axis. Check the whole robot, not
    # just the pin tip. The fixture and both rings remain collision obstacles.
    for yaw_deg in [0,15,-15,30,-30,45,-45,60,-60,90,-90,120,-120,150,-150,180]:
        R=exp(np.array([0,0,np.radians(yaw_deg)]));T=frame(goal,R)@np.linalg.inv(G);seed=cp[0,ids];q=arm.solve(T[:3,3],T[:3,:3],seed);row=dict(yaw_deg=yaw_deg,IK=q is not None,goal_robot_clear=False,straight_line_clear=False)
        if q is not None:
            clear,why=checker.clear(q,opening,paths['workpiece_pin']);row.update(goal_robot_clear=bool(clear),goal_collision=why)
            if why:hits[why['collider']]+=1
            if clear:
                route=[q];last=q;line_ok=True
                for dz in np.linspace(0,-.045,16)[1:]:
                    H=frame(goal+[0,0,dz],R)@np.linalg.inv(G);v=arm.solve(H[:3,3],H[:3,:3],last)
                    if v is None:row['line_failure']='IK';line_ok=False;break
                    ok,why=checker.clear(v,opening,paths['workpiece_pin'])
                    if not ok:row['line_failure']=why;hits[why['collider']]+=1;line_ok=False;break
                    route.append(v);last=v
                row['straight_line_clear']=line_ok
                if line_ok:feasible.append(dict(yaw_deg=yaw_deg,waypoints=np.array(route[::-1]).tolist()))
        attempts.append(row);print('PIN_DIRECT_CANDIDATE',yaw_deg,row['IK'],row['goal_robot_clear'],row['straight_line_clear'],flush=True)
    result.update(state='direct_geometric_candidate_found'if feasible else'no_clear_direct_candidate_in_sample',probe_wall_s=time.perf_counter()-started,source_run=source.name,goal_pin_center_m=goal.tolist(),line_length_m=.045,grasp_transform_from_RGBD=G.tolist(),opening_m=opening,attempts=attempts,candidates=feasible,collision_counts=dict(hits),physical_insertion_tested=False,collision_scope='Full right robot against live PhysX fixtures, other arm and stock. Own arm and currently carried pin excluded; intended pin/bore contact not evaluated by this robot-only geometric probe.')
except Exception:result.update(state='probe_error',error=traceback.format_exc());traceback.print_exc()
finally:
    (OUT/'pin-direct-probe.json').write_text(json.dumps(result,indent=2));print('PIN_DIRECT_RESULT',result['state'],flush=True)
    if 'sim'in globals():sim.detach_stage()
    omni.kit.app.get_app().post_quit(0 if result['state']!='probe_error'else 1)
