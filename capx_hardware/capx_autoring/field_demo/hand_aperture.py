"""Actual cached SDK width intervals for stopped Hand geometry/arm monitoring.

These are conditional sampled state guards. They never certify mechanical
retention, unobserved aperture motion or a physical stopping distance.
"""
from datetime import datetime,timezone
import copy
import numpy as np
from .calibration import utc_stamp

TOLERANCE_M=.0005

def sample(row,*,now=None):
    now=now or datetime.now(timezone.utc)
    if (not isinstance(row,dict)or row.get('completed')is not True or type(row.get('is_grasped'))is not bool or
            type(row.get('robot_time_s'))not in(int,float)or not np.isfinite(row['robot_time_s'])or row['robot_time_s']<0):
        raise ValueError('Actual completed timestamped native Hand SDK sample required')
    age=(now-utc_stamp(row['captured_at_utc'])).total_seconds()
    width=row.get('width_m');maximum=row.get('max_width_m')
    if not 0<=age<=.2:raise RuntimeError('Actual Hand SDK aperture sample is stale/future')
    if (type(width)not in(int,float)or not np.isfinite(width)or not 0<=width<=.0805 or
            type(maximum)not in(int,float)or not np.isfinite(maximum)or not width<=maximum<=.0805):
        raise ValueError('Actual bounded native Hand SDK width/max_width required')
    return float(width)

def binding(samples,*,sweep_side=None,target_width_m=None,epsilon_inner_m=0.,epsilon_outer_m=0.,now=None):
    now=now or datetime.now(timezone.utc);intervals={}
    if set(samples)!={'left','right'}:raise ValueError('Both actual Hand SDK aperture states required')
    for side,row in samples.items():
        width=sample(row,now=now);low=width-TOLERANCE_M;high=width+TOLERANCE_M
        if side==sweep_side:
            if type(target_width_m)not in(int,float)or not np.isfinite(target_width_m)or not 0<=target_width_m<=.08:raise ValueError('Actual bounded native open/close target required')
            for epsilon in [epsilon_inner_m,epsilon_outer_m]:
                if type(epsilon)not in(int,float)or not np.isfinite(epsilon)or not 0<=epsilon<=.002:raise ValueError('Actual bounded native grasp epsilon required')
            low=min(low,target_width_m-epsilon_inner_m-TOLERANCE_M);high=max(high,target_width_m+epsilon_outer_m+TOLERANCE_M)
        intervals[side]=[max(0.,low),min(.081,high)]
    return dict(schema_version=1,source='actual_cached_franka_Hand_SDK',created_at_utc=now.isoformat(),
        samples=copy.deepcopy(samples),intervals_m=intervals,width_guard_tolerance_m=TOLERANCE_M,
        actual_sweep_side=sweep_side,physical_unsampled_width_bound_certified=False)

def validate(value):
    if value.get('schema_version')!=1 or value.get('source')!='actual_cached_franka_Hand_SDK'or value.get('width_guard_tolerance_m')!=TOLERANCE_M:
        raise ValueError('Bound actual SDK aperture interval record required')
    initial=utc_stamp(value['created_at_utc'])
    for side in ['left','right']:
        width=sample(value['samples'][side],now=initial);interval=np.asarray(value['intervals_m'][side],float)
        if interval.shape!=(2,)or not np.isfinite(interval).all()or not 0<=interval[0]<=width<=interval[1]<=.081:
            raise ValueError('Actual aperture must lie inside its checked geometry interval')
        if interval[1]-interval[0]>TOLERANCE_M*2+1e-10 and value.get('actual_sweep_side')!=side:raise ValueError('Only actual commanded Hand may use an opening/closing sweep')
    return value['intervals_m']

def current_within(value,samples,*,now=None):
    intervals=validate(value);now=now or datetime.now(timezone.utc)
    for side in ['left','right']:
        width=sample(samples[side],now=now);old=value['samples'][side]
        if samples[side]['robot_time_s']<old['robot_time_s']:raise RuntimeError('Actual SDK Hand clock regressed')
        low,high=intervals[side]
        if not low<=width<=high:raise RuntimeError('Actual Hand aperture left the checked collision envelope: '+side)
    return intervals

def tools(envelope,interval):
    """Same named official geometry columns, at the complete checked aperture."""
    from .model_preset import ASSETS,tool_columns
    from .calibration import rigid
    spec=envelope['native_hand_aperture_geometry'];low,high=np.asarray(interval,float)
    if not 0<=low<=high<=.081:raise ValueError('Bounded physical Hand aperture geometry required')
    T=rigid(spec['T_tcp_hand']);pad=float(spec['cable_radial_allowance_m'])
    with np.load(ASSETS/'collision-models.npz',allow_pickle=False)as z:v0=z['finger_vertices'].copy()
    result=copy.deepcopy(envelope['tool_capsules_tcp_m'][:spec['palm_column_count']])
    middle=(low+high)/2;sweep=(high-low)/4
    for sign,label in [(1,'left_finger'),(-1,'right_finger')]:
        v=v0.copy();v[:,:2]*=sign;v[:,1]+=sign*middle/2;v[:,2]+=.0584
        rows=tool_columns(v,T,label,padding=pad,y_sweep=sweep)
        for row in rows:row['geometry_role']='finger'
        result+=rows
    if [x['name']for x in result]!=[x['name']for x in envelope['tool_capsules_tcp_m']]:raise ValueError('Width-aware native geometry changed capsule identity/order')
    return result
