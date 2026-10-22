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
        self.parts={};self.generation=0;self.payload_cache={};self._local_obb_cache={}
        self.stats=dict(queries=0,rejected=0,registered_parts=[],uses_free_body_poses=False,early_broadphase_queries=0,early_broadphase_clear=0,early_broadphase_fallback=0)
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
        self.parts[label]=dict(placed=placed,objects=objects,uncertainty_m=float(uncertainty_m),bounds_cache={});self.generation+=1;self._local_obb_cache.clear();self.stats['registered_parts']=sorted(self.parts)
    @staticmethod
    def bounds(shapes,margin=0.):
        centers=np.asarray([p[0]for p in shapes]);extents=np.asarray([abs(p[1])@(p[2]+margin)for p in shapes]);return centers-extents,centers+extents
    def _packed_local_obbs(self,rows):
        # Class-local CAD is immutable throughout its owning guard's lifetime.
        # Strong references prevent recycled identities; attachment/rebuilds
        # create new row identities, and register invalidates the whole cache.
        rows=tuple(rows)
        key=(self.generation,tuple((b,s,id(g))for b,s,g in rows))
        entry=self._local_obb_cache.get(key)
        if entry is not None:return entry[1:]
        centers=[];axes=[];halves=[]
        for body,source,g in rows:
            if not isinstance(body,str)or not isinstance(source,str)or len(g)!=5:raise ValueError('Unsupported local proxy')
            center,axis,half,vertices,normals=map(lambda v:np.asarray(v,float),g)
            if center.shape!=(3,)or axis.shape!=(3,3)or half.shape!=(3,)or np.any(half<0):raise ValueError('Malformed local OBB')
            if vertices.ndim!=2 or vertices.shape[1]!=3 or len(vertices)<4 or normals.ndim!=2 or normals.shape[1]!=3 or not len(normals):raise ValueError('Malformed fixed CAD')
            if not all(np.isfinite(v).all()for v in [center,axis,half,vertices,normals]):raise ValueError('Nonfinite local CAD')
            if any(np.max(abs(v),initial=0)>10 for v in [center,half,vertices])or np.max(abs(axis))>2:raise ValueError('Unsupported local numeric domain')
            centers.append(center);axes.append(axis);halves.append(half)
        packed=(np.asarray(centers).reshape(-1,3),np.asarray(axes).reshape(-1,3,3),np.asarray(halves).reshape(-1,3))
        if len(self._local_obb_cache)>=16:self._local_obb_cache.clear()
        self._local_obb_cache[key]=(rows,*packed)
        return packed

    def _early_clear_current_obbs(self,checker,transforms):
        """Positive-only original broadphase at the CURRENT supplied q/G.

        No posed verdict survives this call. Any possible pair or unknown
        returns to the complete original method below, including FCL/SAT.
        """
        self.stats['early_broadphase_queries']+=1
        try:
            guard=checker.same_arm_clearance;margin=float(guard.margin_m)
            if not guard.rows:return False
            if not np.isfinite(margin)or margin<0:return False
            def bounds(rows,shared_transform=None):
                rows=tuple(rows);center,axes,half=self._packed_local_obbs(rows)
                if not rows:return np.empty((0,3)),np.empty((0,3))
                if shared_transform is None:T=np.asarray([transforms[b]for b,_,_ in rows],float)
                else:T=np.broadcast_to(np.asarray(shared_transform,float),(len(rows),4,4))
                if T.shape!=(len(rows),4,4)or not np.isfinite(T).all():raise ValueError('Unknown current transform')
                if np.max(abs(T[:,:3,:3]))>2 or np.max(abs(T[:,:3,3]))>10 or np.max(abs(T[:,3,:]-[0,0,0,1]))>1e-12:raise ValueError('Unsupported current affine domain')
                R=T[:,:3,:3];C=np.einsum('nij,nj->ni',R,center)+T[:,:3,3]
                A=R@axes;E=np.einsum('nij,nj->ni',abs(A),half+margin)
                low=C-E-1e-6;high=C+E+1e-6
                if not np.isfinite(low).all()or not np.isfinite(high).all()or np.any(low>high):raise ValueError('Unknown projected bounds')
                return low,high
            rlo,rhi=bounds(guard.rows)
            if checker.payload_path:
                if checker.payload_relative is None or not checker.payload_boxes:return False
                T=np.asarray(transforms[checker.arm.hand_path],float)@np.asarray(checker.payload_relative,float)
                low,high=bounds(checker.payload_boxes,T);rlo=np.concatenate((rlo,low));rhi=np.concatenate((rhi,high))
            for part in self.parts.values():
                uncertainty=float(part['uncertainty_m'])
                if not np.isfinite(uncertainty)or not .001<=uncertainty<=.010:return False
                key=float(margin)
                if key not in part['bounds_cache']:part['bounds_cache'][key]=self.bounds(part['placed'],margin+uncertainty)
                plo,phi=map(lambda v:np.asarray(v,float),part['bounds_cache'][key])
                if not len(part['placed'])or plo.ndim!=2 or plo.shape[1:]!=(3,)or phi.shape!=plo.shape or len(plo)!=len(part['placed'])or not np.isfinite(plo).all()or not np.isfinite(phi).all()or np.any(plo>phi)or np.max(abs(plo),initial=0)>100 or np.max(abs(phi),initial=0)>100:return False
                # First cull with the union of the SAME original part bounds.
                # A union can add candidates only. This avoids the large
                # per-hull matrix for robot/payload boxes far from the fixture.
                possible=np.all(rhi>=plo.min(axis=0)-1e-6,axis=1)&np.all(phi.max(axis=0)+1e-6>=rlo,axis=1)
                if not np.any(possible):continue
                # Outward reserve only adds possible pairs; never removes one.
                candidates=np.all(rhi[possible,None,:]>=plo[None,:,:]-1e-6,axis=2)&np.all(phi[None,:,:]+1e-6>=rlo[possible,None,:],axis=2)
                if np.any(candidates):return False
            self.stats['early_broadphase_clear']+=1
            return True
        except Exception:
            return False
        finally:
            # Counter only, not a reusable result or collision certificate.
            self.stats['early_broadphase_fallback']=self.stats['early_broadphase_queries']-self.stats['early_broadphase_clear']

    def clear(self,checker,transforms,changed_bodies=None):
        if not self.parts:return True,None
        self.stats['queries']+=1
        if self._early_clear_current_obbs(checker,transforms):return True,None
        guard=checker.same_arm_clearance;rows=list(guard.rows)
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
