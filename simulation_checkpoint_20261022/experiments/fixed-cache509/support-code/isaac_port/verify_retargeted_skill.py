"""Independent FK and fixed-goal checks for an observed-pose pickup plan."""
from pathlib import Path
import argparse,json
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from source_model import load_model
from fieldkit.perception import PARTS,pose
from task_timing_fast import source_time
from task_path_offsets import pin_clearance_z


def verify(base,candidate,output):
    root=Path(__file__).resolve().parent;recording=base if(base/'trajectory.npz').exists()else base/'recording'
    old=np.load(base/'arm-posture-fr3.npz');new=np.load(candidate/'arm-posture-fr3.npz');a=np.load(recording/'trajectory.npz');b=np.load(candidate/'trajectory.npz')
    ts=a['time_s'];assert np.array_equal(ts,b['time_s']);assert np.array_equal(a['qpos'],b['qpos'])
    schedule=json.loads((candidate/'independent_schedule.json').read_text());assert schedule==json.loads((recording/'independent_schedule.json').read_text())
    assert json.loads((candidate/'config.json').read_text())==json.loads((recording/'config.json').read_text())
    source=json.loads((root/'generated/contact_task/contact-source.json').read_text());observations=json.loads((candidate/'observations.json').read_text())
    model=load_model();d0=mujoco.MjData(model);d1=mujoco.MjData(model);addresses={s:np.array([int(model.joint(f'{s}_arm_fr3v2_joint{j}').qposadr[0])for j in range(1,8)])for s in ['left','right']}
    allowed={s:np.zeros(len(ts),dtype=bool)for s in addresses};reports=[]
    for name,side,approach,grasp,lift in [('ring2','left','approach_two_parts','grasp_two_parts','lift_two_parts_30mm'),('ring1','right','approach_two_parts','grasp_two_parts','lift_two_parts_30mm'),('pin','left','approach_last_pin','grasp_last_pin','lift_last_pin_30mm')]:
        lane=schedule['lanes'][side];at=lambda label,key:next(r[key]for r in lane if r['stage']['label']==label)
        begin,end=at(approach,'start_s'),at(lift,'end_s');allowed[side]|=(ts>begin)&(ts<end)
        index=PARTS.index(name);after=ts>=end
        assert np.array_equal(a['mocap_pos'][after,index],b['mocap_pos'][after,index]),'Fixed insertion position changed'
        # Quaternion conversion can change signs/roundoff; compare rotations.
        rotations_a=Rotation.from_quat(a['mocap_quat'][after,index][:,[1,2,3,0]])
        rotations_b=Rotation.from_quat(b['mocap_quat'][after,index][:,[1,2,3,0]])
        assert np.max((rotations_b*rotations_a.inv()).magnitude())<1e-12,'Fixed insertion orientation changed'
        k=int(np.argmin(abs(ts-at(grasp,'end_s'))));d0.qpos[:]=a['qpos'][k];d1.qpos[:]=a['qpos'][k]
        for s in addresses:d0.qpos[addresses[s]]=old[s+'_qpos'][k];d1.qpos[addresses[s]]=new[s+'_qpos'][k]
        mujoco.mj_kinematics(model,d0);mujoco.mj_kinematics(model,d1)
        prior=pose(source['parts'][name]['position'],source['parts'][name]['quaternion']);observed=np.asarray(observations['parts'][name]['T_cell_part']);delta=observed[:3,:3]@prior[:3,:3].T;shift=observed[:3,3]-prior[:3,3]
        part=a['mocap_pos'][k,index].copy()
        if name=='pin':part[2]+=pin_clearance_z(source_time('left',float(ts[k])))
        hand=int(model.body(side+'_grip_base').id);goal=part+shift+delta@(d0.xpos[hand]-part)
        position_error=float(np.linalg.norm(goal-d1.xpos[hand]));rotation_error=float(Rotation.from_matrix(d1.xmat[hand].reshape(3,3)@(delta@d0.xmat[hand].reshape(3,3)).T).magnitude())
        assert position_error<2e-6 and rotation_error<2e-6,'Pickup FK does not match observation'
        reports.append(dict(part=name,pickup_fk_position_error_m=position_error,pickup_fk_orientation_error_rad=rotation_error,baseline_handoff_reference=end,fixed_insertion_reference_unchanged=True))
    for s in addresses:
        assert np.array_equal(old[s+'_qpos'][~allowed[s]],new[s+'_qpos'][~allowed[s]]),'Optimized path changed outside pickup windows'
        assert np.array_equal(old[s+'_qpos'][0],new[s+'_qpos'][0]),'Initial robot pose changed'
    result=dict(pickup_adaptation_contract_pass=True,hardware_ready=False,parts=reports,initial_robot_pose_unchanged=True,gripper_and_schedule_unchanged=True,scope='Independent FK and stored goal comparison; not collision, grasp or physical camera validation')
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,required=True);p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();verify(a.base,a.candidate,a.output)
