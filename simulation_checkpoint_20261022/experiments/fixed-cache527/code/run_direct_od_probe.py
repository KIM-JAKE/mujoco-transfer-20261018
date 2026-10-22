"""Isaac-native feasibility search: OD grasp and direct insertion by right arm."""
from pathlib import Path
import os,sys,json,traceback
import numpy as np
from pxr import Usd,UsdUtils,UsdPhysics
import omni.kit.app,omni.physx,omni.physics.tensors as tensors
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parents[1])]
from isaac_kinematics import Arm,frame,exp
from native_pickup_probe import rotation
from isaac_collision_queries import Checker
ROOT=HERE.parents[1]/'runs';OUT=ROOT/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True)
try:
    source=ROOT/'native-roas-table-bin-retimed14';stage=Usd.Stage.Open(str(source/'scene-authored.usda'));paths=json.load(open(source/'body-paths.json'));capture=ROOT/'native-roas-table-bin-under30-capture01';trace=np.load(capture/'physx-sequence-trace.npz');meta=json.load(open(capture/'pipeline-results.json'));goals=json.load(open(source/'arm-plan.json'))['goals'];drives={p.GetName():UsdPhysics.DriveAPI(p,'angular')for p in stage.Traverse()if p.IsA(UsdPhysics.RevoluteJoint)and '_arm_fr3v2_joint'in p.GetName()}
    ident=UsdUtils.StageCache.Get().Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ident);dt=1/480;sim.simulate(dt,0);sim.fetch_results();view=tensors.create_simulation_view('numpy',ident);art=view.create_articulation_view('/World/RobotMountJoint');names=list(art.shared_metatype.dof_names);ai={s:[names.index(f'{s}_arm_fr3v2_joint{j}')for j in range(1,8)]for s in ['left','right']};index=np.array([0],np.uint32);cmd=art.get_dof_positions();k=np.argmin(abs(trace['time_s']-73.));j=np.argmin(abs(trace['time_s']-6.3));cmd[0,ai['left']]=trace['q'][k,ai['left']];cmd[0,ai['right']]=trace['q'][j,ai['right']];cmd[0,-4:]=.04;art.set_dof_positions(cmd,index);art.set_dof_velocities(np.zeros_like(cmd),index);art.set_dof_position_targets(cmd,index);sim.simulate(dt,dt);sim.fetch_results()
    arm=Arm(stage,'right',drives,art.get_dof_positions()[0,ai['right']]);checker=Checker(stage,arm,'right');parts={p:view.create_rigid_body_view(paths['workpiece_'+p])for p in ['ring1','ring2']};rows=[];best={};rng=np.random.default_rng(20261020)
    for part,contact_time in [('ring1',38.8),('ring2',67.)]:
        k=np.argmin(abs(trace['time_s']-contact_time));pidx=trace['part_names'].tolist().index(part);hidx=meta['link_names'].index('left_NativeHand');oldhand=trace['links'][k,hidx];oldpart=trace['parts'][k,pidx];H=frame(oldhand[:3],rotation(oldhand[3:]))@frame([0,0,.1034],np.eye(3));G=np.linalg.inv(H)@frame(oldpart[:3],rotation(oldpart[3:]));observed=parts[part].get_transforms()[0];P=frame(observed[:3],rotation(observed[3:]));goal=frame(goals[part]['P'],np.eye(3));found=False
        for yaw in [0.,np.pi/2,-np.pi/2,np.pi,np.pi/4,-np.pi/4,3*np.pi/4,-3*np.pi/4]:
            for height in [0.,.002,.004,.006]:
                B=P@frame([0,0,0],exp(np.array([0.,0.,yaw])))@np.linalg.inv(G);B[:3,3]+=[0,0,height];grip=np.linalg.inv(B)@P;target=goal@np.linalg.inv(grip)
                pick=arm.solve(B[:3,3],B[:3,:3],cmd[0,ai['right']])
                if pick is None:continue
                clear,why=checker.clear(pick,.08,paths['workpiece_'+part])
                if not clear:rows.append(dict(part=part,yaw=yaw,height=height,stage='pickup_collision',reason=why));continue
                insertion=None
                seeds=[pick,trace['q'][np.argmin(abs(trace['time_s']-81.)),ai['right']]]+[rng.uniform(arm.bounds[:,0],arm.bounds[:,1])for _ in range(6)]
                for seed in seeds:
                    candidate=arm.solve(target[:3,3],target[:3,:3],seed)
                    if candidate is not None and checker.clear(candidate,.065 if part=='ring1'else .053,paths['workpiece_'+part])[0]:insertion=candidate;break
                row=dict(part=part,yaw=yaw,height=height,stage='both_poses_clear'if insertion is not None else'insertion_IK_or_collision_failed',pickup_q=pick.tolist(),insertion_q=insertion.tolist()if insertion is not None else None,grasp_TCP_relative=grip.tolist(),pickup_TCP=B.tolist(),insert_TCP=target.tolist());rows.append(row)
                if insertion is not None:best[part]=row;found=True;print('DIRECT_OD_POSES_CLEAR',part,row,flush=True);break
            if found:break
        print('DIRECT_OD_PART',part,found,flush=True)
    result=dict(full_task_pass=False,physics='Isaac PhysX CPU static geometry query',grasp_or_motion_not_yet_tested=True,poses=best,candidates=rows)
    (OUT/'od-probe.json').write_text(json.dumps(result,indent=2));sim.detach_stage();omni.kit.app.get_app().post_quit(0)
except Exception:
    (OUT/'error.txt').write_text(traceback.format_exc());traceback.print_exc();omni.kit.app.get_app().post_quit(1)
