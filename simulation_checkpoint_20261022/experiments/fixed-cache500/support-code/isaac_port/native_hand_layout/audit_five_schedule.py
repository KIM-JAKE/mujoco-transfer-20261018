"""Independent time sampling of coordinated five-capacity research schedules.

This checks commanded kinematics and inter-arm/payload separation. It does not
claim force-based insertion, a calibrated dynamics model, or hardware execution.
"""
from compact_compare import *
from fr3_motion_control import VMAX
import sys
from fast_environment import clear as world_clear,set_env,FixedScene
from audit_five_geometry import obstacles

_SWEEP_SCENES={}

def stationary_world_sweeps(row,cycle):
    failures=[]
    for side,lane in cycle['lanes'].items():
        for segment in lane:
            if segment['label']not in ('grasp','release','pin_preshape','close_empty_hand_for_button'):
                continue
            recipe=cycle['recipes'].get(segment['part'])
            contact=segment['label']in ('grasp','release')
            base=obstacles(row,recipe if contact else None)
            # The selected workpiece is the intended fingertip contact. All
            # other parts, machine CAD and support geometry remain obstacles.
            excluded=('stored_'+recipe['part']+'_'+str(cycle['slot']-1 if recipe['part']=='pin'else 0))if segment['label']=='grasp'else None
            token=(base.cache_token,'stationary_width_sweep',excluded)
            if token not in _SWEEP_SCENES:_SWEEP_SCENES[token]=FixedScene([(n,o)for n,o in base if n!=excluded],token)
            set_env(_SWEEP_SCENES[token]);q=np.asarray(segment['a'])
            first=segment.get('gap_start_m',segment['gap_m']);last=segment.get('gap_finish_m',segment['gap_m'])
            for width in np.linspace(first,last,41):
                okay,why=world_clear(side,q,float(width))
                if not okay:
                    failures.append(dict(side=side,label=segment['label'],part=segment['part'],width_m=float(width),why=why));break
    set_env(obstacles(row))
    for side,start in [('left',.02),('right',0.)]:
        for width in np.linspace(start,.08,41):
            okay,why=world_clear(side,READY[side],float(width))
            if not okay:
                failures.append(dict(side=side,label='reopen_at_READY',width_m=float(width),why=why));break
    return failures


def limits(lanes):
    reports = []
    for side, lane in lanes.items():
        previous = None
        for segment in lane:
            a, b = np.array(segment['a']), np.array(segment['b'])
            T = segment['duration_s']
            u = np.linspace(0, 1, max(1001, int(np.ceil(T / .001)) + 1))[:, None]
            delta = b - a
            q = a + u**3 * (10 - 15*u + 6*u*u) * delta
            v = 30*u*u*(1-u)**2*delta/T
            acc = 60*u*(1-u)*(1-2*u)*delta/T**2
            jerk = (60-360*u+360*u*u)*delta/T**3
            lo, hi = position_velocity_bounds(q)
            discontinuity = 0. if previous is None else float(np.max(abs(previous-a)))
            okay = (np.all(q >= LOWER-1e-9) and np.all(q <= UPPER+1e-9)
                    and np.all(v >= .95*lo-1e-7) and np.all(v <= .95*hi+1e-7)
                    and np.max(abs(acc)) <= 9.5+1e-6
                    and np.max(abs(jerk)) <= 4750+1e-5
                    and discontinuity < 1e-6)
            reports.append(dict(side=side, label=segment['label'], part=segment['part'],
                                passed=bool(okay), start_discontinuity_rad=discontinuity,
                                peak_velocity_fraction=float(np.max(abs(v)/VMAX)),
                                peak_acceleration_rad_s2=float(np.max(abs(acc))),
                                peak_jerk_rad_s3=float(np.max(abs(jerk)))))
            previous = b
    return reports


def dense_cross(lanes, schedule, initial=None, dt=.01):
    state = {s: (np.array(lanes[s][0]['a']),
                 (initial or {}).get(s, .08)) for s in AD}
    payload = {s: None for s in AD}
    failures, samples = [], 0
    for step in schedule['steps']:
        active = {s: lanes[s][step[s+'_index']]
                  if step[s+'_index'] is not None else None for s in AD}
        for t in np.linspace(0, step['duration_s'],
                             max(3, int(np.ceil(step['duration_s']/dt))+1)):
            current = {s: state_at(active[s], t) if active[s] else state[s] for s in AD}
            held = {s: active[s].get('payload') if active[s] else payload[s] for s in AD}
            if not arm_pair_clear(current['left'], current['right'],
                                  left_payload=held['left'], right_payload=held['right']):
                failures.append(dict(time_s=float(step['start_s']+t),
                                     left_index=step['left_index'], right_index=step['right_index']))
                if len(failures) >= 20:
                    return failures, samples+1, state
            samples += 1
        for side, segment in active.items():
            if segment:
                state[side] = state_at(segment, segment['duration_s'])
                payload[side] = segment.get('payload')
    return failures, samples, state


def audit(path):
    row = json.load(open(path))
    if not row.get('eligible') or row.get('provisional_only'):
        raise ValueError('A complete eligible comparison is required.')
    records = []
    for cycle in row['cycles']:
        sweep_failures=stationary_world_sweeps(row,cycle)
        traces = limits(cycle['lanes']) + limits(cycle['return_lanes'])
        traces += limits(dict(right=cycle['button_finish']))
        failures, samples, last = dense_cross(cycle['lanes'], cycle['coordination'])
        for segment in cycle['button_finish']:
            for t in np.linspace(0, segment['duration_s'], max(3, int(np.ceil(segment['duration_s']/.01))+1)):
                if not arm_pair_clear(last['left'], state_at(segment, t)):
                    failures.append(dict(button_label=segment['label'], local_time_s=float(t)))
                samples += 1
        reset, reset_samples, _ = dense_cross(cycle['return_lanes'], cycle['return_coordination'],
                                              dict(left=.02, right=0.))
        failures += [dict(return_motion=True, **x) for x in reset]
        records.append(dict(slot=cycle['slot'], cross_arm_failures=failures,
                            stationary_gripper_world_failures=sweep_failures,
                            samples=samples+reset_samples, joint_limits=traces,
                            passed=not failures and not sweep_failures and all(x['passed'] for x in traces)))
        out = path.with_name(path.stem+'-dense-audit.json')
        report = dict(label=row['label'], complete=len(records)==len(row['cycles']),
                      passed=len(records)==len(row['cycles']) and all(x['passed'] for x in records),
                      collision_sample_max_dt_s=.01, joint_sample_max_dt_s=.001,
                      scope='Command kinematics, stationary gripper width sweeps against static world, and cross-arm separation including nominal rigid CAD payloads; static-world free transfers audited separately',
                      full_physx_assembly_pass=False, real_robot_tested=False, cycles=records)
        out.write_text(json.dumps(report, indent=2))
        print('FIVE_DENSE_SLOT', row['label'], cycle['slot'], records[-1]['passed'], samples+reset_samples, flush=True)
    return report


if __name__ == '__main__':
    for argument in sys.argv[1:]:
        result = audit(Path(argument))
        print('FIVE_DENSE_DONE', result['label'], result['passed'], flush=True)
