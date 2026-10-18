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


def prepare(base, output, aggressive_contact=False, coordinated=False,
            right_crossing_stop=2.1, left_ring_clear=2.2, right_ring_clear=4., left_pin_time_reserve=1., right_return_time_reserve=1.1, respect_cutoff=False, model_torque_rate=None, contact_minimums=None):
    from optimization_clock import install_cutoff_alarm
    if respect_cutoff:
        require_optimization_window(120)
        install_cutoff_alarm()
    import prepare_fr3_timing as timing
    if aggressive_contact:
        timing.CONTACT_MINIMUMS.update(insert_last_pin_approach=.65,insert_last_pin_last5mm=.5,press_button=.3)
        timing.GRIPPER_TRAVEL_CAP=1.
        timing.PIN_EARLY_CLOSE_CAP=.8
        timing.MAX_PHASE_RATE=30.
        timing.MINIMUM_TIME_OBJECTIVE=True
    for key,value in (contact_minimums or {}).items():
        if value is not None:
            if not np.isfinite(value) or value<=0:raise ValueError('Contact duration must be positive')
            timing.CONTACT_MINIMUMS[key]=value
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
    if any(not np.isfinite(value) or value<1. for value in [left_pin_time_reserve,right_return_time_reserve]):
        raise ValueError('Time reserve may only slow a segment')
    rules=[dict(side='right',reference=right_crossing_stop,wait_for_side='left',minimum_reference=left_ring_clear),
           dict(side='left',reference=3.357916666666667,wait_for_side='right',minimum_reference=right_ring_clear)]if coordinated else[]
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
        bounds=np.unique(np.r_[bounds,[r['reference']for r in rules if r['side']==side]])
        if side=='left':
            # The remaining shared reference is an idle left arm while the
            # right arm presses the button. Hold the final left state instead
            # of assigning a numerically ill-conditioned clock to this idle leg.
            final_release=next(r['end_s'] for r in lane if r['stage']['label']=='release_last_pin')
            bounds=bounds[bounds<=final_release+1e-8]
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
        if model_torque_rate is not None:
            from model_torque_screen import ModelTorqueScreen
            profile.model_torque_screen=ModelTorqueScreen(profile,names,float(bounds[0]),float(bounds[-1]),rate_limit=model_torque_rate)
        prefix = side + '_'
        data[prefix+'bounds'] = bounds
        records[side] = []
        for i, (begin,end) in enumerate(zip(bounds[:-1],bounds[1:])):
            if respect_cutoff:require_optimization_window(30)
            candidates = [retime_segment(profile,begin,end,local_schedule,sigma,True)
                          for sigma in [.002,.006,.012]]
            clock,factor,report = min(candidates,key=lambda item:item[2]['duration_s'])
            if end-begin>.15:
                for sigma in [.015,.04,.07]:
                    candidate = local_time_allocation(profile,clock,begin,end,local_schedule,sigma)
                    if candidate[2]['duration_s']<report['duration_s']:
                        clock,factor,report = candidate
            reserve=left_pin_time_reserve if side=='left' and abs(begin-3.357916666666667)<1e-7 else right_return_time_reserve if side=='right' and 3.89<begin<3.90 else 1.
            if reserve!=1.:
                factor*=reserve
                report=dict(report)
                report['duration_s']*=reserve
                report['time_scale']*=reserve
                report['peak_acceleration_rad_s2']/=reserve**2
                report['peak_jerk_rad_s3']/=reserve**3
                report['dynamic_time_reserve']=reserve
            data[f'{prefix}clock_{i}_x']=clock.x
            data[f'{prefix}clock_{i}_c']=clock.c
            data[f'{prefix}clock_{i}_scale']=np.array(factor)
            records[side].append(report)
            if report['minimum_phase_rate']<-.0001:
                raise RuntimeError(f'Nonmonotonic {side} phase clock: {begin}..{end}, {report}')
            print(side,i,round(begin,3),round(end,3),round(report['duration_s'],3),flush=True)
        data[prefix+'durations']=np.array([r['duration_s'] for r in records[side]])
    output.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(output,**data)
    report=dict(state='planned_unvalidated_in_PhysX',hardware_ready=False,
                planning_fraction=.95,cycle_target_s=10.,segments=records,
                side_nominal_s={side:float(data[side+'_durations'].sum()) for side in records},
                independent_clocks=True,simultaneous_path_requires_new_collision_check=True,
                experimental_contact_timing=aggressive_contact,
                contact_minimum_s=dict(timing.CONTACT_MINIMUMS),
                gripper_travel_driver_cap_rad_s=timing.GRIPPER_TRAVEL_CAP,
                pin_early_close_driver_cap_rad_s=timing.PIN_EARLY_CLOSE_CAP,
                maximum_source_phase_rate=timing.MAX_PHASE_RATE,
                actual_time_objective=timing.MINIMUM_TIME_OBJECTIVE,
                coordination_rules=rules,
                left_pin_time_reserve=left_pin_time_reserve,
                right_return_time_reserve=right_return_time_reserve,
                model_torque_rate_planning_limit_nm_s=model_torque_rate,
                model_torque_scope='Imported arm/gripper, no carried payload, contact or friction; not physical robot certification'if model_torque_rate else None,
                archive_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
                input_fingerprint=str(data['fingerprint']))
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report['side_nominal_s']),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',type=Path,default=Path(__file__).parent/'runs/contact-under15-07')
    p.add_argument('--output',type=Path,default=Path(__file__).parent/'generated/contact_task/fr3-concurrent-profile.npz')
    p.add_argument('--aggressive-contact-timing',action='store_true',help='Experimental timings; requires a new native physics/contact audit')
    p.add_argument('--coordinated',action='store_true',help='Add explicit crossing-zone stop points and dependency gates')
    p.add_argument('--right-crossing-stop',type=float,default=2.1)
    p.add_argument('--left-ring-clear',type=float,default=2.2)
    p.add_argument('--right-ring-clear',type=float,default=4.)
    p.add_argument('--left-pin-time-reserve',type=float,default=1.)
    p.add_argument('--right-return-time-reserve',type=float,default=1.1)
    p.add_argument('--respect-cutoff',action='store_true',help='Enforce the optimization deadline; omit for later reproduction')
    p.add_argument('--model-torque-rate',type=float,help='Additional imported-model gravity-excluded torque-rate planning ceiling in Nm/s (<=1000)')
    p.add_argument('--pin-approach-min',type=float)
    p.add_argument('--pin-fine-min',type=float)
    p.add_argument('--button-min',type=float)
    a=p.parse_args();prepare(a.base,a.output,a.aggressive_contact_timing,a.coordinated,
                            a.right_crossing_stop,a.left_ring_clear,a.right_ring_clear,a.left_pin_time_reserve,a.right_return_time_reserve,a.respect_cutoff,a.model_torque_rate,
                            dict(insert_last_pin_approach=a.pin_approach_min,insert_last_pin_last5mm=a.pin_fine_min,press_button=a.button_min))
