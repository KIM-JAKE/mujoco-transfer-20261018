"""Independent artifact check: physics retention, full wall clock and commands."""
from pathlib import Path
import sys,json,numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE.parents[1]));from fr3_motion_control import position_velocity_bounds,LOWER,UPPER
root=HERE.parents[1]/'runs'/sys.argv[1];r=json.load(open(root/'pipeline-results.json'));assert r['full_task_pass']and r['vision_end_to_end_pass'],'No completed physical task'
z=np.load(root/'physx-sequence-trace.npz');t=z['time_s'];physical=z['simulation_time_s'];assert np.all(np.diff(t)>0)and np.all(np.diff(physical)>0)
clock_margin=np.diff(t)-np.diff(physical);assert clock_margin.min()>-.00001,'Recorded motion was locally fast-forwarded'
assert abs(t[-1]-r['wall_verified_elapsed_s'])<.05
from audit_pin_insertion import audit as audit_pin_procedure
pin_procedure=audit_pin_procedure(root);assert pin_procedure['procedure_pass'],'Pin procedure failed: '+str(pin_procedure['violations'])
if '--require-under20'in sys.argv:assert r.get('end_to_end_under20') and r['wall_verified_elapsed_s']<20
if '--require-under30'in sys.argv:assert r['end_to_end_under30']and r['wall_verified_elapsed_s']<30
pnames=z['part_names'].tolist();goals=json.load(open(root/'arm-plan.json'))['goals'];ln=r['link_names'];roots=[ln.index(s+'_NativeHand')for s in ['left','right']];positions=z['parts'][:,:,:3];quat=z['parts'][:,:,3:];goal=np.array([goals[n]['P']for n in pnames]);errors=np.linalg.norm(positions-goal[None,:,:],axis=2);angles=np.arccos(np.clip(1-2*(quat[:,:,0]**2+quat[:,:,1]**2),-1,1));hands=z['links'][:,roots,:3];clear=np.linalg.norm(positions[:,:,None,:]-hands[:,None,:,:],axis=3).min(2)>.16;ok=(errors<.0015).all(1)&(angles<.05).all(1)&clear.all(1)
start=len(ok)-1
while start>0 and ok[start-1]:start-=1
assert ok[-1]and physical[-1]-physical[start]>=.5-1e-6 and t[-1]-t[start]>=.5-1e-6,'Insufficient independently verified retention'
c=np.load(root/'executed-actuator-commands.npz');names=r['dof_names'];ids=[names.index(f'{s}_arm_fr3v2_joint{j}')for s in ['left','right']for j in range(1,8)];fi=[i for i,n in enumerate(names)if '_native_'in n];base=np.load(HERE.parents[1]/'runs/native-roas-table-rgbd-control05/executed-actuator-commands.npz')['command'][0].astype(float);
if r.get('prepared_grasp_wrist_rotation_deg'):
    with np.load(root/'corrected-skill-field.npz')as taught:base=taught['command'][0].astype(np.float32).astype(float)
if (root/'initial-actuator-command.npz').exists():base=np.load(root/'initial-actuator-command.npz')['command'].astype(float)
q=c['command'][:,ids].astype(float);assert np.all(q>=np.tile(LOWER,2))and np.all(q<=np.tile(UPPER,2));dt=1/480;v=np.diff(q,axis=0,prepend=base[ids][None,:])/dt;a=np.diff(v,axis=0,prepend=np.zeros((1,14)))/dt;j=np.diff(a,axis=0,prepend=np.zeros((1,14)))/dt;lo,hi=position_velocity_bounds(q.reshape(-1,2,7));ratio=float(np.max(abs(v.reshape(-1,2,7))/np.maximum(.95*np.where(v.reshape(-1,2,7)>=0,hi,-lo),1e-8)));amax=float(np.max(abs(a)));jmax=float(np.max(abs(j)));assert ratio<=1.00001 and amax<=9.5001 and jmax<=4750.1
fv=np.diff(c['command'][:,fi].astype(float),axis=0,prepend=base[fi][None,:])/dt;finger_peak=float(np.max(abs(fv)));assert finger_peak<.0476
obs=json.load(open(root/'rgb-observations.json'));latencies=[]
for o in obs:
    assert not o['result'].get('error');assert o['result'].get('simulator_body_pose_input')is False;captured=o['captured_wall_s'];ready=o['ready_wall_s'];delivered=o['delivered_wall_s'];assert 0<=captured<=ready<=delivered<=r['wall_verified_elapsed_s'];latencies.append(delivered-captured)
    work=root/'rgb-perception'/Path(o['work']).name
    with np.load(work/'rgbd.npz')as packet:assert set(packet.files)<={'rgb','depth_m','K','T_cell_camera','points_camera_m','fixture_depth_m'}
actual_speed={}
if '--require-under20'in sys.argv:
    setup=json.load(open(root/'randomization-evidence.json'));assert setup['config'].get('randomization_mode')=='full_bin_drop' and setup['ring_count']==10 and not setup['target_body_privileged']
    assert r.get('encoder_motor_speed_audit',{}).get('enforced'),'Actual motor speed guard was not enabled'
    aq=c['actual_dof_position'][:,ids].astype(float).reshape(-1,2,7);av=c['actual_dof_velocity'][:,ids].astype(float).reshape(-1,2,7);assert np.isfinite(aq).all() and np.isfinite(av).all();lo,hi=position_velocity_bounds(aq);fraction=float(np.max(abs(av)/np.maximum(.95*np.where(av>=0,hi,-lo),1e-8)));finger_fraction=float(np.max(abs(c['actual_dof_velocity'][:,fi]))/.0475);assert fraction<=1.001 and finger_fraction<=1.01
    actual_speed=dict(recorded_encoder_velocity_fraction_of_95_percent_PBV=fraction,recorded_finger_velocity_fraction_of_47p5_mm_s=finger_fraction,full_bin_drop_setup_checked=True)
result=dict(source_run=root.name,physical_task_pass=True,end_to_end_under20=bool(r.get('end_to_end_under20')),actual_motor_audit=actual_speed,end_to_end_under30=bool(r['end_to_end_under30']),wall_verified_elapsed_s=r['wall_verified_elapsed_s'],motion_physics_s=float(physical[-1]),minimum_recorded_wall_minus_physics_interval_s=float(clock_margin.min()),no_local_motion_fast_forward=True,retention_wall_s=float(t[-1]-t[start]),retention_physics_s=float(physical[-1]-physical[start]),position_errors_mm=(errors[-1]*1000).tolist(),axis_errors_rad=angles[-1].tolist(),fresh_RGBD_observations=len(obs),capture_to_controller_latency_range_ms=[1000*min(latencies),1000*max(latencies)],capture_to_controller_latency_median_ms=1000*float(np.median(latencies)),command_velocity_fraction_of_95_percent_PBV=ratio,max_command_acceleration_rad_s2=amax,max_command_jerk_rad_s3=jmax,max_finger_position_target_speed_m_s=finger_peak,perception_packet_contains_object_poses=False,hardware_tested=False,timing_basis=r['timing_basis']);(root/'end-to-end-evidence.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
