"""Keep validated clock segments when only other stage boundaries changed."""
from pathlib import Path
import argparse,hashlib,json
import numpy as np


def reuse(source,target):
    if (target/'run-manifest.json').exists():raise ValueError('Do not alter a measured run')
    original=source/'fr3-concurrent-profile.npz';destination=target/'fr3-concurrent-profile.npz'
    with np.load(original)as z:old={n:z[n].copy()for n in z.files}
    with np.load(destination)as z:new={n:z[n].copy()for n in z.files}
    if str(old['fingerprint'])!=str(new['fingerprint']):raise ValueError('Spatial path/driver/schedule fingerprint differs')
    a=json.loads(original.with_suffix('.json').read_text());b=json.loads(destination.with_suffix('.json').read_text())
    for key in ['planning_fraction','gripper_travel_driver_cap_rad_s','pin_early_close_driver_cap_rad_s','maximum_source_phase_rate','model_torque_rate_planning_limit_nm_s']:
        if a.get(key)!=b.get(key):raise ValueError('Different constraints: '+key)
    if a.get('path_acceleration_limit_rad_s2',9.1)!=b.get('path_acceleration_limit_rad_s2',9.1):raise ValueError('Different path acceleration allocation')
    records=[]
    for side in ['left','right']:
        bounds=new[side+'_bounds'];old_bounds=old[side+'_bounds']
        for k,(begin,end)in enumerate(zip(bounds[:-1],bounds[1:])):
            matches=[j for j,(lo,hi)in enumerate(zip(old_bounds[:-1],old_bounds[1:]))if abs(lo-begin)<1e-8 and abs(hi-end)<1e-8]
            if not matches:continue
            j=matches[0];report=a['segments'][side][j]
            if b['segments'][side][k].get('dynamic_time_reserve',1.)>report.get('dynamic_time_reserve',1.)+1e-12:continue
            # A stricter task-time heuristic needs freshly generated clocks.
            guarded=[('insert_last_pin_approach',5.257916666666667),('insert_last_pin_last5mm',5.5634722222222255),('press_button',6.654992690058481)]
            if any(begin-1e-8<=point<end-1e-8 and b['contact_minimum_s'][name]>a['contact_minimum_s'][name]+1e-12 for name,point in guarded):continue
            for suffix in ['c','x','scale']:new[f'{side}_clock_{k}_{suffix}']=old[f'{side}_clock_{j}_{suffix}'].copy()
            new[side+'_durations'][k]=old[side+'_durations'][j];b['segments'][side][k]=report
            records.append(dict(side=side,begin=float(begin),end=float(end),source_segment=j,target_segment=k))
    np.savez_compressed(destination,**new)
    b['side_nominal_s']={s:float(new[s+'_durations'].sum())for s in ['left','right']}
    b['archive_sha256']=hashlib.sha256(destination.read_bytes()).hexdigest()
    b['reused_clocks']=dict(source_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),segments=records,scope='Identical path and common constraint settings; native dynamics and changed simultaneous timing still require new acceptance')
    destination.with_suffix('.json').write_text(json.dumps(b,indent=2)+'\n');print(json.dumps(dict(reused=len(records),nominal=b['side_nominal_s'])))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--target',type=Path,required=True);a=p.parse_args();reuse(a.source,a.target)
