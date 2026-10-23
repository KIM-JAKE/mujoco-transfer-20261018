"""Fixed base-to-mount support contact, proved with complete pinned base CAD.

This is not a general obstacle exemption. Only static link0 capsule skin may
cross one actual measured support top plane; every original base vertex must
remain above it. All moving links/Hand/cables/other obstacles stay checked.
"""
import numpy as np
from .collision import box,transform,finite

def certify(spec,obstacle,T_cell_base,vertices,*,cad_tessellation_tolerance_m=2e-5):
    if spec['obstacle_name']!=obstacle.get('name'):raise ValueError('Named actual mounting support required')
    n=finite(spec['normal_cell'],(3,));origin=finite(spec['origin_cell_m'],(3,))
    if abs(np.linalg.norm(n)-1)>1e-7:raise ValueError('Unit actual mounting support normal required')
    T,half=box(obstacle);directions=np.abs(T[:3,:3].T@n)
    if abs(directions.max()-1)>1e-7:raise ValueError('Mounting support must have a measured planar OBB top face')
    maximum=float((T[:3,3]-origin)@n+half@directions)
    if abs(maximum)>1e-6:raise ValueError('Support obstacle top does not match actual mounting plane')
    base=transform(T_cell_base);v=finite(vertices)
    heights=(v@base[:3,:3].T+base[:3,3]-origin)@n
    if heights.min() < -cad_tessellation_tolerance_m:raise ValueError('Actual mounted fixed-base CAD penetrates the support plane')
    # Cable allowance may extend to all sides of moving-body envelopes, but
    # cannot pretend a below-table cable is part of the original base CAD.
    if spec.get('no_base_cables_below_support')is not True:raise ValueError('Actual fixed-base cables must remain above support or be separately modeled')
    return dict(kind='fixed_base_known_mount_contact',obstacle_name=obstacle['name'],minimum_CAD_height_m=float(heights.min()),cad_tessellation_tolerance_m=cad_tessellation_tolerance_m,all_original_base_vertices_checked=len(v))
