"""Search free-space dual-arm handover poses against the actual Isaac cell."""
from pathlib import Path
import os,sys,json,traceback,time
import numpy as np
from pxr import Usd,UsdUtils,UsdPhysics
import omni.kit.app,omni.physx,omni.physics.tensors as tensors
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parents[1])]
from isaac_kinematics import Arm,frame,exp
from isaac_collision_queries import Checker
from isaac_motion import duration
from native_pickup_probe import rotation
from fr3_motion_control import position_velocity_bounds
RUNS=HERE.parents[1]/'runs';OUT=RUNS/os.environ['M80_ROAS_RUN'];OUT.mkdir(exist_ok=True)
try:
    source=RUNS/'native-roas-table-bin-retimed14';stage=Usd.Stage.Open(str(source/'scene-authored.usda'));paths=json.load(open(source/'body-paths.json'));ref=np.load(RUNS/'native-roas-table-bin-under30-capture01/full-command-trace.npz');trace=np.load(RUNS/'native-roas-table-bin-under30-capture01/physx-sequence-trace.npz');meta=json.load(open(RUNS/'native-roas-table-bin-under30-capture01/pipeline-results.json'));tt=ref['time_s'];cq=ref['command'];ai={s:[meta['dof_names'].index(f'{s}_arm_fr3v2_joint{j}')for j in range(1,8)]for s in ['left','right']};drives={p.GetName():UsdPhysics.DriveAPI(p,'angular')for p in stage.Traverse()if p.IsA(UsdPhysics.RevoluteJoint)and '_arm_fr3v2_joint'in p.GetName()}
    ident=UsdUtils.StageCache.Get().Insert(stage).ToLongInt();sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ident);dt=1/480;sim.simulate(dt,0);sim.fetch_results();view=tensors.create_simulation_view('numpy',ident);art=view.create_articulation_view('/World/RobotMountJoint');idx=np.array([0],np.uint32);cmd=cq[np.searchsorted(tt,6.3):np.searchsorted(tt,6.3)+1].copy();cmd[0,ai['left']]=cq[np.searchsorted(tt,73.),ai['left']];cmd[0,-4:]=.04;art.set_dof_positions(cmd,idx);art.set_dof_velocities(np.zeros_like(cmd),idx);art.set_dof_position_targets(cmd,idx);sim.simulate(dt,dt);sim.fetch_results();arms={s:Arm(stage,s,drives,art.get_dof_positions()[0,ai[s]])for s in ai};check={s:Checker(stage,arms[s],s,allow_loose_fingertips=True)for s in ai};time_s=.01;rng=np.random.default_rng(20261020);result=dict(full_task_pass=False,planning_only=True,parts={})
    poses=json.load(open(RUNS/'native-roas-table-od-plan03/od-plan.json'))['poses']
    for part,contact_time,entry_time,diam in [('ring1',27.85,35.542,.074895),('ring2',52.2,63.265,.061913)]:
        record=poses[part];j=trace['part_names'].tolist().index(part);k=int(np.argmin(abs(trace['time_s']-contact_time)));old_part=trace['parts'][k,j];old_hand=trace['links'][k,meta['link_names'].index('left_NativeHand')];partT=frame(old_part[:3],rotation(old_part[3:]));leftH=frame(old_hand[:3],rotation(old_hand[3:]))@frame([0,0,.1034],np.eye(3));Gright=np.array(record['grasp_TCP_relative']);left_seed=cq[np.searchsorted(tt,contact_time),ai['left']];right_seed=cq[np.searchsorted(tt,20.95 if part=='ring1'else 46.49),ai['right']];pickup=np.array(record['pickup_q']);entry=cq[np.searchsorted(tt,entry_time),ai['left']];pinpre=cq[np.searchsorted(tt,58.089),ai['right']];rows=[]
        for z in [1.3,1.2,1.4]:
            for y in [-.15,0.,-.30]:
                for x in [-.25,-.15,-.35,-.05]:
                    P=np.array([x,y,z]);offset=P-partT[:3,3];LH=leftH.copy();LH[:3,3]+=offset;RP=partT.copy();RP[:3,3]=P;RH=RP@frame([0,0,0],exp(np.array([0,0,record['donor_part_yaw_rad']])))@np.linalg.inv(Gright)
                    sols={}
                    for side,H,seeds in [('left',LH,[left_seed,cmd[0,ai['left']]]),('right',RH,[right_seed,pickup,pinpre])]:
                        options=[]
                        for seed in seeds:
                            sol=arms[side].solve(H[:3,3],H[:3,:3],seed)
                            if sol is None:continue
                            lo,hi=position_velocity_bounds(sol);cap=float(np.minimum(-lo,hi).min())
                            if cap<.35:continue
                            clear,why=check[side].clear(sol,diam,paths['workpiece_'+part])
                            if clear:options.append((sol,cap))
                        if not options:break
                        if side=='left':chosen=min(options,key=lambda item:duration([item[0],entry],.1))
                        else:chosen=min(options,key=lambda item:duration([pickup,item[0]],.1)+.3*duration([item[0],pinpre],.1))
                        sols[side]=chosen
                    if len(sols)!=2:continue
                    # Test both complete arms together at the actual contact pose.
                    qpose=art.get_dof_positions();qpose[0,ai['left']]=sols['left'][0];qpose[0,-4:-2]=diam/2;art.set_dof_positions(qpose,idx);art.set_dof_velocities(np.zeros_like(qpose),idx);art.set_dof_position_targets(qpose,idx);sim.simulate(dt,time_s);sim.fetch_results();time_s+=dt
                    clear,why=check['right'].clear(sols['right'][0],diam,paths['workpiece_'+part])
                    if not clear:continue
                    costs=dict(right_pick_to_handover_s=duration([pickup,sols['right'][0]],.1),left_handover_to_entry_s=duration([sols['left'][0],entry],.1),right_to_pin_seed_s=duration([sols['right'][0],pinpre],.1));score=costs['right_pick_to_handover_s']+costs['left_handover_to_entry_s']+.2*costs['right_to_pin_seed_s'];row=dict(P=P.tolist(),offset_m=offset.tolist(),left_q=sols['left'][0].tolist(),right_q=sols['right'][0].tolist(),left_TCP=LH.tolist(),right_TCP=RH.tolist(),min_velocity_caps_rad_s={s:v[1]for s,v in sols.items()},costs=costs,score=score);rows.append(row);print('HANDOVER_CANDIDATE',part,P.tolist(),round(score,3),flush=True)
        rows.sort(key=lambda row:row['score']);result['parts'][part]=dict(candidates=rows,best=rows[0]if rows else None)
        (OUT/'handover-search.json').write_text(json.dumps(result,indent=2));print('HANDOVER_SEARCH_PART',part,len(rows),rows[0]if rows else None,flush=True)
    sim.detach_stage();omni.kit.app.get_app().post_quit(0)
except Exception:
    (OUT/'error.txt').write_text(traceback.format_exc());traceback.print_exc();omni.kit.app.get_app().post_quit(1)
