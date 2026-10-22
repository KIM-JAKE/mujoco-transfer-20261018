"""Initial test scene on the EXISTING M80 front shelf; no online pose writes.

The shelf is original CAD instance_0641. The green three-hole holder was already
an explicitly proposed photo-derived proxy; its dimensions are not field verified.
Online perception receives only calibrated ROI/camera information, never these
initialized free-body poses. Ring signed faces are estimated from RGB-D.
"""
import json
from pathlib import Path
import numpy as np
from pxr import UsdGeom,Gf
SHELF_Z=.884313439
R_DOWN=np.diag([1.,-1.,-1.])
PARTS={'ring1':[.230,-.575,SHELF_Z+.00775+.0001],
       'ring2':[.200,-.715,SHELF_Z+.007+.0001],
       'pin':[.230,-.855,SHELF_Z+.03175]}
HOLDER_ORIGIN=[.170,-.8725,SHELF_Z]
def _pose(prim,P,R):
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=P
    x=UsdGeom.Xformable(prim);x.ClearXformOpOrder();x.AddTransformOp().Set(Gf.Matrix4d(*T.T.ravel().tolist()));x.SetResetXformStack(True)
def _local_mesh_vertices(prim):
    # Setup-only original USD geometry, never passed across sensor boundary.
    from pxr import Usd
    cache=UsdGeom.XformCache();inverse=cache.GetLocalToWorldTransform(prim).GetInverse();values=[]
    for child in Usd.PrimRange(prim):
        if child.IsA(UsdGeom.Mesh):
            points=np.asarray(UsdGeom.Mesh(child).GetPointsAttr().Get(),float)
            if not len(points):continue
            relative=np.asarray(cache.GetLocalToWorldTransform(child)*inverse,float)
            values.append((np.c_[points,np.ones(len(points))]@relative)[:,:3])
    if not values:raise RuntimeError('Fixed variation requires original part mesh support')
    return np.concatenate(values)
def _variation_pose(prim,part,P,R,variation):
    value=variation.get(part)
    if value is None:return np.asarray(P,float),R,None
    if set(value)-{'translation_xy_m','tilt_xy_rad'}:raise ValueError('Only initialization XY/tilt variation is supported')
    xy=np.asarray(value.get('translation_xy_m',[0.,0.]),float);tilt=np.asarray(value.get('tilt_xy_rad',[0.,0.]),float)
    if xy.shape!=(2,)or tilt.shape!=(2,)or not np.isfinite(np.r_[xy,tilt]).all()or np.max(abs(xy))>.01000000001 or np.linalg.norm(tilt)>.04000000001:raise ValueError('Fixed test variation is limited toXY±10mm and total tilt0.04rad')
    # Rodrigues in world coordinates. The part may settle normally afterward;
    # neither the pose nor orientation is held/pinned during simulation.
    vector=np.r_[tilt,0.];angle=float(np.linalg.norm(vector));K=np.array([[0.,-vector[2],vector[1]],[vector[2],0.,-vector[0]],[-vector[1],vector[0],0.]])
    rotation=np.eye(3)+K*(np.sin(angle)/angle if angle else 1.)+K@K*((1-np.cos(angle))/angle**2 if angle else .5)
    R=rotation@R;points=_local_mesh_vertices(prim);P=np.asarray(P,float)+np.r_[xy,0.];P[2]=SHELF_Z-float(np.min((points@R.T)[:,2]))+.0001
    return P,R,dict(translation_xy_m=xy.tolist(),requested_tilt_xy_rad=tilt.tolist(),original_mesh_support_clearance_m=.0001,setup_only=True)
def author(stage,paths,out,config=None):
    variation=(config or {}).get('fixed_supply_pose_variation',{})
    if not isinstance(variation,dict) or set(variation)-{'ring1','ring2'}:raise ValueError('Only ring1/ring2 fixed initialization variations are supported')
    # Collect only direct child paths before editing activity. Deactivating a
    # parent invalidates descendant Prim handles from an earlier traversal.
    stock=stage.GetPrimAtPath('/World/BinStock')
    removed=[str(prim.GetPath()) for prim in stock.GetChildren()] if stock else []
    for path in removed:
        stage.GetPrimAtPath(path).SetActive(False)
    holder=stage.GetPrimAtPath('/World/ProposedThreePinHolder')
    if not holder:raise RuntimeError('Existing proposed green PIN holder is absent; do not invent an unreported replacement')
    _pose(holder,HOLDER_ORIGIN,np.eye(3))
    initial_centers={};initial_axes={};variation_records={}
    for part,P in PARTS.items():
        prim=stage.GetPrimAtPath(paths['workpiece_'+part]);P,R,detail=_variation_pose(prim,part,P,np.eye(3) if part=='ring2' else R_DOWN,variation)
        _pose(prim,P,R);initial_centers[part]=P.tolist();initial_axes[part]=R[:,2].tolist()
        if detail is not None:variation_records[part]=detail
    record=dict(mode='fixed_existing_front_shelf',setup_before_physics_and_first_capture=True,
        simulator_body_poses_sent_to_controller=False,online_pose_writes=0,extra_tables_or_ring_holders=0,
        shelf_original_CAD='instance_0641/mesh_248',shelf_support_z_m=SHELF_Z,
        usable_shelf_bounds_world_m=[[.16196904,-.91075,SHELF_Z],[.29766904,-.50575,SHELF_Z]],
        left_to_right_world_direction=[0,1,0],nominal_part_centers_world_m=PARTS,initial_part_centers_world_m=initial_centers,
        initial_signed_CAD_axes_world=initial_axes,ring_initial_floor_clearance_m=.0001,initialization_only_pose_variation=variation_records,original_physical_settle_unchanged=True,
        removed_extra_ring_actors=removed,ring_count=2,pin_count=1,
        holder=dict(path=str(holder.GetPath()),origin_world_m=HOLDER_ORIGIN,yaw_deg=0,
            size_m=[.120,.035,.018],status='existing_photo_derived_proxy_not_field_verified',new_holder_added=False))
    (Path(out)/'fixed-supply-initialization.json').write_text(json.dumps(record,indent=2)+'\n')
    return record
def calibration():
    return dict(floor_origin_world_m=[.23,-.70825,SHELF_Z],basket_to_world_R=np.eye(3).tolist(),
        outer_bounds_world_m=[[.155,-.915,SHELF_Z-.003],[.315,-.500,SHELF_Z+.15]],
        outer_dimensions_in_basket_frame_m=[.160,.415,.153],loose_bin_mode=True,
        fixed_supply_mode=True,pin_initial_center_world_m=[.230,-.85475,SHELF_Z+.03175],
        initial_eye_offset_m=[0,0,.20],initial_camera_width_px=768,initial_camera_height_px=768,initial_camera_span_m=.50,
        fixture_information_source='Original CAD shelf support and calibrated supply ROI; not online body poses')
