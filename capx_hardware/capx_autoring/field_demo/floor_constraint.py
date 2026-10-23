"""Known measured support-plane constraint on RGB-D CAD inference only.

No object truth or actual object pose write. Elevated rings stay elevated;
only a small explicitly bounded inferred penetration is projected upward.
"""
from functools import lru_cache
from pathlib import Path
import copy
import numpy as np
from .collision import finite,transform

@lru_cache(maxsize=2)
def support_vertices(part):
    if part not in ('ring1','ring2'):raise ValueError('Fixed ring CAD required')
    p=Path(__file__).resolve().parents[1]/'isaac_port/fieldkit/assets/ring-support-vertices.npz'
    with np.load(p,allow_pickle=False)as z:v=z[part].copy()
    v.setflags(write=False);return v

def project_inferred_ring_above_floor(T_cell_part,part,floor_origin_cell_m,up_axis_cell,*,maximum_correction_m,reported_uncertainty_m):
    T=transform(T_cell_part).copy();floor=finite(floor_origin_cell_m,(3,));up=finite(up_axis_cell,(3,))
    if abs(np.linalg.norm(up)-1)>1e-6:raise ValueError('Measured unit support normal required')
    if any(type(x)not in (int,float)or not np.isfinite(x)or not 0<x<=.005 for x in [maximum_correction_m,reported_uncertainty_m]):raise ValueError('Explicit positive <=5mm correction and reported depth/fit bound required')
    limit=min(maximum_correction_m,reported_uncertainty_m);v=support_vertices(part);minimum=float(np.min(v@(T[:3,:3].T@up)+(T[:3,3]-floor)@up));shift=max(0.,-minimum)
    if shift>limit:raise ValueError('Inferred CAD penetration exceeds actual reported floor-fit/depth bound')
    T[:3,3]+=shift*up
    return T,dict(source='RGBD_CAD_fit_plus_measured_support_plane',uses_object_truth=False,actual_object_pose_written=False,raw_minimum_floor_height_m=minimum,upward_inference_correction_m=shift,maximum_allowed_correction_m=limit,reported_uncertainty_m=reported_uncertainty_m,elevated_ring_forced_down=False)
