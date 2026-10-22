"""Continuous native Hand sweep before an empty-PIN pickup preshape.

Numeric SDK/FK and fixed native CAD only. Invisible query meshes never become
physical actors. Preserve original self/fixed/pair margins and selected-tip rule.
"""
import numpy as np
from scipy.spatial import ConvexHull
from pxr import UsdGeom,PhysicsSchemaTools,Sdf
from predictive_pair_clearance import at_pose,shape_overlap,proxy
from grasp_transfer_feasibility import swept_gripper_boxes


def check(ctx,target_width):
    art,cmd,checker,arm=ctx['art'],ctx['cmd'],ctx['checker'],ctx['arm'];ids,fi=ctx['ids'],ctx['fi'];selected=ctx['paths']['workpiece_pin']
    if checker.payload_path:raise RuntimeError('PIN pickup preshape requires an empty planning Hand')
    measured=art.get_dof_positions()[0].copy();actual_q=measured[ids].copy();command_q=cmd[0,ids].copy();current_width=float(measured[fi].sum());command_width=float(cmd[0,fi].sum())
    if not np.isfinite(np.r_[current_width,command_width,target_width]).all()or not 0<=target_width<=.08:raise ValueError('Native finite preshape widths required')
    low=min(current_width,command_width,target_width);high=max(current_width,command_width,target_width)
    a=swept_gripper_boxes(checker,arm.fk(actual_q),low,high);b=swept_gripper_boxes(checker,arm.fk(command_q),low,high)
    own={arm.hand_path}|{arm.hand_path+'/'+checker.side+'_'+label+'_finger'for label in ['left','right']};sources=[(body,source)for body,source,_ in checker.pair_boxes if body in own]
    if not a or len(a)!=len(b)or len(a)!=len(sources):raise RuntimeError('Missing native continuous preshape geometry')
    swept=[proxy(np.r_[x[3],y[3]])for x,y in zip(a,b)]
    other_callback=getattr(checker,'other_configuration',None)
    if not callable(other_callback):raise RuntimeError('Current opposite-arm model required for preshape')
    other,other_q,other_width=other_callback();names=list(art.shared_metatype.dof_names);oi=[names.index(other.side+'_arm_fr3v2_joint'+str(j))for j in range(1,8)];of=[j for j,n in enumerate(names)if n.startswith(other.side+'_native_')]
    other_widths=np.unique(np.linspace(min(float(measured[of].sum()),float(cmd[0,of].sum())),max(float(measured[of].sum()),float(cmd[0,of].sum())),5))
    for oq in [measured[oi],cmd[0,oi]]:
        for width in other_widths:
            for _,_,ob in at_pose(other,oq,float(width)):
                if any(shape_overlap(box,ob,margin=.0005)for box in swept):raise RuntimeError('Continuous empty PIN preshape intersects opposite arm/payload')
    for width in np.linspace(low,high,13):
        clear,why=checker.path([actual_q,command_q],float(width),selected)
        if not clear:raise RuntimeError('Full robot PIN preshape aperture guard blocked '+str(why))
    # Same continuous-query construction used by existing empty-LEFT closure.
    generation=int(getattr(checker,'_pin_empty_sweep_generation',0))+1;checker._pin_empty_sweep_generation=generation;query=[]
    for i,((body,source),box)in enumerate(zip(sources,swept)):
        vertices=box[3];hull=ConvexHull(vertices);faces=hull.simplices.copy();normals=np.cross(vertices[faces[:,1]]-vertices[faces[:,0]],vertices[faces[:,2]]-vertices[faces[:,0]]);flip=np.einsum('ij,ij->i',normals,hull.equations[:,:3])<0;faces[flip]=faces[flip][:,[0,2,1]]
        path='/World/PlanningQueries/pin_empty_width_'+str(generation)+'_'+str(i);mesh=UsdGeom.Mesh.Define(checker.stage,path);mesh.CreatePointsAttr(vertices.astype(np.float32));mesh.CreateFaceVertexCountsAttr([3]*len(faces));mesh.CreateFaceVertexIndicesAttr(faces.ravel());mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateVisibilityAttr('invisible');xf=UsdGeom.Xformable(mesh);xf.ClearXformOpOrder();op=xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble);query.append((body,source,op,PhysicsSchemaTools.encodeSdfPath(Sdf.Path(path))))
    saved=checker.rows,checker.payload_rows
    try:
        checker.rows=query;checker.payload_rows=[];clear,why=checker._clear_transforms({body:np.eye(4)for body,_ in sources},selected)
    finally:checker.rows,checker.payload_rows=saved
    if not clear:raise RuntimeError('Continuous PIN preshape sweep hits fixed structure '+str(why))
    return dict(continuous_native_CAD_sweep_clear=True,current_and_command_pose_checked=True,opposite_actual_and_command_q_checked=True,other_apertures_m=other_widths.tolist(),start_actual_opening_m=current_width,start_command_opening_m=command_width,target_opening_m=float(target_width),own_sampled_openings_m=np.linspace(low,high,13).tolist(),force_speed_contact_and_margin_unchanged=True,uses_free_body_poses=False)
