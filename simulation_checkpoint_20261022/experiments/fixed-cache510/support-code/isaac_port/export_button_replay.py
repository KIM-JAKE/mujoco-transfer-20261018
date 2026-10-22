"""Export the immutable 8.060 s MuJoCo recording for native USD animation."""
from pathlib import Path
import hashlib,json,sys
import mujoco
import numpy as np
from prepare import HERE,SOURCES,prepare

DUO=SOURCES['cap_button'].parent.parent
SOURCE=DUO/'cap_button_parallel_alt_20261016'
OUT=HERE/'generated/cap_button'

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def export():
    prepare('cap_button')
    sys.path.insert(0,str(DUO))
    from kinematics import gripper_state
    report=json.loads((SOURCE/'timing_report.json').read_text())
    events=json.loads((SOURCE/'button_events.json').read_text())
    assert json.loads((SOURCE/'validation_manifest.json').read_text())['state']=='complete'
    assert report['events']['all_three_installed'] < events['first_contact_s'] < events['green_s']
    trajectory=np.load(SOURCE/'trajectory.npz',allow_pickle=False)
    times=trajectory['time_s']
    assert np.all(np.diff(times)>0)
    np.testing.assert_allclose(times[[0,-1]],[0,report['duration_s']],atol=1e-10)
    model=mujoco.MjModel.from_xml_path(str(SOURCES['cap_button']))
    data=mujoco.MjData(model)
    positions=[];quaternions=[];gripper_delta=0.
    for i,t in enumerate(times):
        data.qpos[:]=trajectory['qpos'][i]
        data.mocap_pos[:]=trajectory['mocap_pos'][i]
        data.mocap_quat[:]=trajectory['mocap_quat'][i]
        for k,side in enumerate(['left','right']):
            gripper_state(model,data,side,float(trajectory['gaps'][i,k]))
        gripper_delta=max(gripper_delta,float(np.max(np.abs(data.qpos-trajectory['qpos'][i]))))
        mujoco.mj_forward(model,data)
        positions.append(data.xpos.copy());quaternions.append(data.xquat.copy())
    positions=np.asarray(positions);quaternions=np.asarray(quaternions)
    assert np.isfinite(positions).all() and np.isfinite(quaternions).all()
    np.testing.assert_allclose(np.linalg.norm(quaternions,axis=-1),1,atol=1e-8)
    np.savez_compressed(OUT/'replay.npz',time_s=times,body_positions=positions,body_quaternions=quaternions)
    schedule=json.loads((SOURCE/'independent_schedule.json').read_text())
    config=json.loads((SOURCE/'config.json').read_text())
    goal_errors={part['name']:float(np.linalg.norm(positions[-1,model.body('workpiece_'+part['name']).id]-part['goal_origin_m']))
                 for part in config['parts']}
    assert len(goal_errors)==3 and max(goal_errors.values())<1e-6,goal_errors
    record=dict(state='passed',source_run=str(SOURCE.relative_to(HERE.parent.parent)),
        source_scene=str(SOURCES['cap_button'].relative_to(HERE.parent.parent)),
        source_sha256={n:sha(SOURCE/n)for n in ['trajectory.npz','timing_report.json','button_events.json','config.json','independent_schedule.json','frame_labels.json']},
        replay_sha256=sha(OUT/'replay.npz'),frames=len(times),duration_s=float(times[-1]),source_fps=report['fps'],
        events=report['events'],release_times_s=schedule['release_times_s'],
        body_names=[model.body(i).name for i in range(model.nbody)],
        geom_visibility={model.geom(i).name:bool(model.geom_group[i] not in [3,5])for i in range(model.ngeom)},
        gripper_reconstruction_max_delta_rad=gripper_delta,
        recorded_final_part_goal_errors_m=goal_errors,
        source_geometry_audit=json.loads((SOURCE/'robot_path_audit.json').read_text()),
        button_assumptions=report['button_assumptions'],
        physical_assembly_pass=False,physical_switch_actuation_verified=False,
        scope='Original recorded kinematic states, FK and mocap stock poses. No PhysX task execution or force/contact validation.')
    (OUT/'replay-source.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps({k:record[k] for k in ['state','frames','duration_s','events','gripper_reconstruction_max_delta_rad']}))

if __name__=='__main__':export()
