"""Retime the two arms separately, preserving each arm's spatial path."""
from pathlib import Path
from types import SimpleNamespace
import argparse
import copy
import hashlib
import json
import numpy as np
from scipy.interpolate import CubicSpline
from fr3_reference_profile import reference_fingerprint
from prepare_fr3_timing import retime_segment, local_time_allocation
from optimization_clock import require_optimization_window


def prepare(base, output):
    require_optimization_window(120)
    names = json.loads((base/'dof-names.json').read_text())
    src = json.loads((Path(__file__).parent/'generated/contact_task/contact-source.json').read_text())
    lookup = {j['name']: j['qpos_address'] for j in src['joints']}
    z = np.load(base/'recording/trajectory.npz')
    q = z['qpos'][:, [lookup[n] for n in names]].copy()
    plan = np.load(base/'arm-posture-fr3.npz')
    indices = [names.index(f'{side}_arm_fr3v2_joint{j}') for side in ['left', 'right'] for j in range(1,8)]
    for i, name in enumerate(names):
        if '_arm_' in name:
            q[:,i] = plan[name.split('_')[0]+'_qpos'][:,int(name[-1])-1]
    schedule = json.loads((base/'recording/independent_schedule.json').read_text())
    data = dict(fingerprint=np.array(reference_fingerprint(z['time_s'],q,names,schedule)))
    records = {}
    for side in ['left', 'right']:
        lane = schedule['lanes'][side]
        boundaries = [0., float(z['time_s'][-1])]
        for row in lane:
            if row['stage']['label'] in ['grasp_two_parts','grasp_last_pin',
                                        'release_two_parts','release_last_pin',
                                        'insert_last_pin_approach','insert_last_pin_last5mm',
                                        'press_button']:
                boundaries.extend([row['start_s'],row['end_s']])
        bounds = np.unique(boundaries)
        if side=='right':
            release=next(r['end_s'] for r in schedule['lanes']['left']if r['stage']['label']=='release_last_pin')
            bounds=np.unique(np.r_[bounds,release])
        masked = q.copy()
        other = 'right' if side == 'left' else 'left'
        for i, name in enumerate(names):
            if name.startswith(other+'_'):
                masked[:, i] = masked[0, i]
        local_schedule = copy.deepcopy(schedule)
        for row in local_schedule['lanes'][other]:
            row['start_s'] += 1000
            row['end_s'] += 1000
        profile = SimpleNamespace(path=CubicSpline(z['time_s'],masked[:,indices],axis=0),
                                  full_path=CubicSpline(z['time_s'],masked,axis=0),
                                  driver_indices=[names.index(s+'_grip_left_driver_joint') for s in ['left','right']])
        prefix = side + '_'
        data[prefix+'bounds'] = bounds
        records[side] = []
        for i, (begin,end) in enumerate(zip(bounds[:-1],bounds[1:])):
            require_optimization_window(30)
            candidates = [retime_segment(profile,begin,end,local_schedule,sigma,True)
                          for sigma in [.002,.006,.012]]
            clock,factor,report = min(candidates,key=lambda item:item[2]['duration_s'])
            if end-begin>.15:
                for sigma in [.015,.04,.07]:
                    candidate = local_time_allocation(profile,clock,begin,end,local_schedule,sigma)
                    if candidate[2]['duration_s']<report['duration_s']:
                        clock,factor,report = candidate
            data[f'{prefix}clock_{i}_x']=clock.x
            data[f'{prefix}clock_{i}_c']=clock.c
            data[f'{prefix}clock_{i}_scale']=np.array(factor)
            records[side].append(report)
            print(side,i,round(begin,3),round(end,3),round(report['duration_s'],3),flush=True)
        data[prefix+'durations']=np.array([r['duration_s'] for r in records[side]])
    output.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(output,**data)
    report=dict(state='planned_unvalidated_in_PhysX',hardware_ready=False,
                planning_fraction=.95,cycle_target_s=10.,segments=records,
                side_nominal_s={side:float(data[side+'_durations'].sum()) for side in records},
                independent_clocks=True,simultaneous_path_requires_new_collision_check=True,
                archive_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
                input_fingerprint=str(data['fingerprint']))
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report['side_nominal_s']),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',type=Path,default=Path(__file__).parent/'runs/contact-under15-07')
    p.add_argument('--output',type=Path,default=Path(__file__).parent/'generated/contact_task/fr3-concurrent-profile.npz')
    a=p.parse_args();prepare(a.base,a.output)
