"""Keep completed, visually observed parts in future planning geometry.

Invisible FCL/SAT query shapes only. No physical actor is read for localization,
created, moved or attached. Local templates are fixed class collision CAD.
"""
import numpy as np
from predictive_pair_clearance import proxy,transform_proxy,transform_proxy_bounds,shape_overlap
try:
    import fcl
except ImportError:
    fcl=None

class PlacedPartClearance:
    def __init__(self):
        self.parts={};self.generation=0;self.payload_cache={}
        self.stats=dict(queries=0,rejected=0,registered_parts=[],uses_free_body_poses=False)
    def register(self,checker,label,template,T,uncertainty_m=.0035):
        from pxr import UsdGeom
        from isaac_kinematics import world
        T=np.asarray(T,float)
        if T.shape!=(4,4) or not np.isfinite(T).all() or not .001<=uncertainty_m<=.010:raise ValueError('Invalid observed placed-part pose/uncertainty')
        if np.max(abs(T[:3,:3].T@T[:3,:3]-np.eye(3)))>1e-5 or np.linalg.det(T[:3,:3])<.99999:raise ValueError('Invalid observed part rotation')
        base=np.linalg.inv(world(checker.stage,template));root=checker.stage.GetPrimAtPath(template+'/CPUConvexContact');shapes=[]
        for prim in root.GetChildren():
            if not prim.IsA(UsdGeom.Mesh):continue
            local=base@world(checker.stage,str(prim.GetPath()));v=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(),float);shapes.append(proxy(v@local[:3,:3].T+local[:3,3]))
        if not shapes:raise RuntimeError('Missing fixed class collision CAD for observed placed part')
        placed=[transform_proxy(g,T)for g in shapes];objects=None
        if fcl:
            from fcl_transport import convex
            objects=[fcl.CollisionObject(convex(g[3]),fcl.Transform(np.ascontiguousarray(T[:3,:3]),np.ascontiguousarray(T[:3,3])))for g in shapes]
        self.parts[label]=dict(placed=placed,objects=objects,uncertainty_m=float(uncertainty_m),bounds_cache={});self.generation+=1;self.stats['registered_parts']=sorted(self.parts)
    @staticmethod
    def bounds(shapes,margin=0.):
        centers=np.asarray([p[0]for p in shapes]);extents=np.asarray([abs(p[1])@(p[2]+margin)for p in shapes]);return centers-extents,centers+extents
    def clear(self,checker,transforms,changed_bodies=None):
        if not self.parts:return True,None
        self.stats['queries']+=1;guard=checker.same_arm_clearance;rows=list(guard.rows)
        objects=list(guard.objects)if fcl and guard.geometries is not None else None
        # A missing FCL guard falls back to complete transformed SAT geometry.
        place_proxy=transform_proxy_bounds if objects is not None else transform_proxy
        robot=[place_proxy(g,transforms[b])for b,_,g in rows]
        if checker.payload_path:
            if checker.payload_relative is None:raise RuntimeError('Missing current image-derived payload transform')
            T=transforms[checker.arm.hand_path]@checker.payload_relative
            for b,source,g in checker.payload_boxes:
                rows.append((b,source,g));robot.append(place_proxy(g,T))
                if objects is not None:
                    from fcl_transport import convex
                    key=id(g)
                    if key not in self.payload_cache:
                        if len(self.payload_cache)>512:self.payload_cache.clear()
                        geometry=convex(g[3]);self.payload_cache[key]=(g,geometry,fcl.CollisionObject(geometry))
                    payload_object=self.payload_cache[key][2]
                    payload_object.setTransform(fcl.Transform(np.ascontiguousarray(T[:3,:3]),np.ascontiguousarray(T[:3,3])))
                    objects.append(payload_object)
        rlo,rhi=self.bounds(robot,guard.margin_m)
        for label,part in self.parts.items():
            margin=2*guard.margin_m+part['uncertainty_m'];bounds_key=float(guard.margin_m)
            if bounds_key not in part['bounds_cache']:part['bounds_cache'][bounds_key]=self.bounds(part['placed'],guard.margin_m+part['uncertainty_m'])
            plo,phi=part['bounds_cache'][bounds_key];candidates=np.all(rhi[:,None,:]>=plo[None,:,:],axis=2)&np.all(phi[None,:,:]>=rlo[:,None,:],axis=2)
            for i,j in np.argwhere(candidates):
                if changed_bodies is not None and rows[i][0]not in changed_bodies:continue
                if objects is not None:
                    distance=float(fcl.distance(objects[i],part['objects'][j],fcl.DistanceRequest(),fcl.DistanceResult()));hit=not np.isfinite(distance)or distance<=margin
                else:
                    distance=None;hit=shape_overlap(robot[i],part['placed'][j],margin=guard.margin_m+part['uncertainty_m']/2)
                if hit:
                    why=dict(kind='robot_or_payload_vs_completed_part',query=rows[i][1],actor='RGBD_placed_'+label,placed_part=label,distance_m=distance,required_clearance_m=margin,uses_free_body_poses=False);self.stats['rejected']+=1;self.stats['last_rejection']=why;return False,why
        return True,None
