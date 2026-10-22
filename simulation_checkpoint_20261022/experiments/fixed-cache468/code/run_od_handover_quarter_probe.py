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
    arm=Arm(stage,'right',drives,art.get_dof_positions()[0,ai['right']]);checker=Checker(stage,arm,'right',allow_loose_fingertips=True);parts={p:view.create_rigid_body_view(paths['workpiece_'+p])for p in ['ring1','ring2']};rows=[];best={};rng=np.random.default_rng(20261020)
    park_left=trace['q'][np.argmin(abs(trace['time_s']-73.)),ai['left']].copy()
    step_time=.01
    def set_left(q):
        global step_time
        pose=art.get_dof_positions();pose[0,ai['left']]=q;pose[0,-4:]=.04;art.set_dof_positions(pose,index);art.set_dof_velocities(np.zeros_like(pose),index);art.set_dof_position_targets(pose,index);sim.simulate(dt,step_time);sim.fetch_results();step_time+=dt
    for part,contact_time,donor_time,precontact_time in [('ring1',26.91,20.95,25.),('ring2',51.4,46.49,49.2)]:
        pidx=trace['part_names'].tolist().index(part);left_contact=trace['links'][np.argmin(abs(trace['time_s']-contact_time)),meta['link_names'].index('left_NativeHand')];donor_part=trace['parts'][np.argmin(abs(trace['time_s']-donor_time)),pidx];Tpart=frame(donor_part[:3],rotation(donor_part[3:]));normal=Tpart[:3,2];yl=rotation(left_contact[3:])[:,1];yr=np.cross(normal,yl);yr/=np.linalg.norm(yr);zr=-normal;xr=np.cross(yr,zr);Rbase=np.column_stack([xr,yr,zr]);found=False
        for yaw in [0.,np.pi,np.pi/4,-np.pi/4,np.pi/12,-np.pi/12,np.pi+np.pi/4]:
            for height in [.003,.005,.007]:
                set_left(park_left);observed=parts[part].get_transforms()[0];P=frame(observed[:3],rotation(observed[3:]));target=frame(Tpart[:3,3]+normal*height,Rbase@exp(np.array([0.,0.,yaw])));grip=np.linalg.inv(target)@Tpart@frame([0,0,0],exp(np.array([0.,0.,np.pi/2])));B=P@np.linalg.inv(grip)
                pick=arm.solve(B[:3,3],B[:3,:3],cmd[0,ai['right']])
                if pick is None:continue
                clear,why=checker.clear(pick,.08,paths['workpiece_'+part])
                if not clear:rows.append(dict(part=part,yaw=yaw,height=height,stage='pickup_collision',reason=why));continue
                set_left(trace['q'][np.argmin(abs(trace['time_s']-precontact_time)),ai['left']]);donor=None;seeds=[pick,cmd[0,ai['right']]]+[rng.uniform(arm.bounds[:,0],arm.bounds[:,1])for _ in range(6)]
                for seed in seeds:
                    candidate=arm.solve(target[:3,3],target[:3,:3],seed)
                    if candidate is not None and checker.clear(candidate,.08,paths['workpiece_'+part])[0]:donor=candidate;break
                row=dict(part=part,yaw=yaw,height=height,stage='both_poses_clear'if donor is not None else'donor_IK_or_collision_failed',pickup_q=pick.tolist(),donor_q=donor.tolist()if donor is not None else None,donor_part_yaw_rad=float(np.pi/2),grasp_TCP_relative=grip.tolist(),pickup_TCP=B.tolist(),donor_TCP=target.tolist());rows.append(row)
                if donor is not None:best[part]=row;found=True;print('OD_HANDOVER_POSES_CLEAR',part,row,flush=True);break
            if found:break
        print('OD_HANDOVER_PART',part,found,flush=True)
    result=dict(full_task_pass=False,physics='Isaac PhysX CPU static geometry query',grasp_or_motion_not_yet_tested=True,poses=best,candidates=rows)
    (OUT/'od-handover-probe.json').write_text(json.dumps(result,indent=2));sim.detach_stage();omni.kit.app.get_app().post_quit(0)
except Exception:
    (OUT/'error.txt').write_text(traceback.format_exc());traceback.print_exc();omni.kit.app.get_app().post_quit(1)
