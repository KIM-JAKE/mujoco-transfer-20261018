"""Official fixed Hand surfaces placed ONLY with actual SDK frame/width.

Camera self-exclusion is a surface match, not a broad enclosing volume. A
measured flange-to-Hand mount is required; the Desk TCP offset is never assumed.
"""
from functools import lru_cache
from pathlib import Path
import numpy as np
from .calibration import rigid

@lru_cache(maxsize=1)
def surfaces():
    p=Path(__file__).resolve().parents[1]/'isaac_port/fieldkit/assets/native-hand/surfaces.npz'
    with np.load(p,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}

def actual_hand_points(T_cell_tcp,F_T_EE,T_flange_hand,width_m):
    if type(width_m)not in (int,float)or not np.isfinite(width_m)or not 0<=width_m<=.0805:raise ValueError('Actual SDK native Hand width required')
    T=rigid(T_cell_tcp)@np.linalg.inv(rigid(F_T_EE))@rigid(T_flange_hand);cloud=surfaces();points=[cloud['palm_points']]
    for sign in [1,-1]:
        f=cloud['finger_points'].copy();f[:,:2]*=sign;f[:,1]+=sign*width_m/2;f[:,2]+=.0584;points.append(f)
    points=np.concatenate(points);return points@T[:3,:3].T+T[:3,3]
