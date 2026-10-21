"""Check image-estimated carried CAD against its carrier arm and palm.

Only the existing native distal grasp pad region can contact the carried
class model. Every proximal finger region, palm and robot link remains an
obstacle. This never changes physical actors or opposite-arm contact policy.
"""
import numpy as np
from scipy.spatial import ConvexHull
from predictive_pair_clearance import proxy, transform_proxy, shape_overlap
try:
    import fcl
except ImportError:
    fcl = None


def _proximal_hull(vertices, pad_start_z_m=.036):
    """Exact convex clipping; .036 local Z is TCP Z=-9mm on official Hand.

    The existing native pad contact certificate uses |TCP Z|<9mm. The lower
    side of that SAME distal region may touch held CAD; the complete proximal
    convex volume is retained, including edges intersecting this plane.
    """
    v=np.asarray(vertices,float);h=ConvexHull(v);edges={tuple(sorted((int(a),int(b))))
        for f in h.simplices for a,b in [(f[0],f[1]),(f[1],f[2]),(f[2],f[0])]}
    kept=[p for p in v if p[2]<=pad_start_z_m]
    for i,j in edges:
        a,b=v[i],v[j]
        if (a[2]-pad_start_z_m)*(b[2]-pad_start_z_m)<0:
            kept.append(a+(b-a)*((pad_start_z_m-a[2])/(b[2]-a[2])))
    result=np.unique(np.asarray(kept),axis=0)
    if len(result)<4:raise RuntimeError('Official native proximal finger hull is unavailable')
    return result


class OwnPayloadClearance:
    def __init__(self,guard):
        self.guard=guard;self.cache={};self.robot_rows=[]
        for body,source,geometry in guard.rows:
            if body.endswith('_finger') and source.endswith('/tip'):
                geometry=proxy(_proximal_hull(geometry[3]))
            self.robot_rows.append((body,source,geometry))
        self.robot_geometries=None
        if fcl:
            from fcl_transport import convex
            self.robot_geometries=[convex(r[2][3])for r in self.robot_rows]
            self.robot_objects=[fcl.CollisionObject(g)for g in self.robot_geometries]
        self.stats=dict(queries=0,rejected=0,distal_native_pad_contact_only=True,
                        palm_and_arm_exempt=False,uses_free_body_poses=False)

    def clear(self,checker,transforms,*,changed_bodies=None):
        if not checker.payload_path:return True,None
        if checker.payload_relative is None or not checker.payload_boxes:
            raise RuntimeError('Planned payload missing its current image-estimated CAD transform')
        self.stats['queries']+=1
        transform=transforms[self.guard.arm.hand_path]@np.asarray(checker.payload_relative,float)
        payload=checker.payload_boxes
        # Geometry caches depend only on authored local CAD, not body state.
        key=tuple((source,id(geometry))for _,source,geometry in payload)
        if key not in self.cache:
            if len(self.cache)>=16:self.cache.clear()
            if fcl:
                from fcl_transport import convex
                geometries=[convex(r[2][3])for r in payload]
                self.cache[key]=(tuple(payload),geometries,[fcl.CollisionObject(g)for g in geometries])
            else:self.cache[key]=None
        payload_objects=self.cache[key][2]if fcl else None
        placed_robot=[transform_proxy(g,transforms[b])for b,_,g in self.robot_rows]
        placed_payload=[transform_proxy(g,transform)for _,_,g in payload]
        if fcl:
            for obj,(body,_,_)in zip(self.robot_objects,self.robot_rows):
                t=transforms[body];obj.setTransform(fcl.Transform(np.ascontiguousarray(t[:3,:3]),np.ascontiguousarray(t[:3,3])))
            for obj in payload_objects:obj.setTransform(fcl.Transform(np.ascontiguousarray(transform[:3,:3]),np.ascontiguousarray(transform[:3,3])))
        margin=self.guard.margin_m
        def limits(placed):
            center=np.asarray([v[0]for v in placed]);extent=np.asarray([abs(v[1])@(v[2]+margin)for v in placed]);return center-extent,center+extent
        rlo,rhi=limits(placed_robot);plo,phi=limits(placed_payload)
        candidates=np.all(rhi[:,None,:]>=plo[None,:,:],axis=2)&np.all(phi[None,:,:]>=rlo[:,None,:],axis=2)
        for i,j in np.argwhere(candidates):
            if changed_bodies is not None and self.robot_rows[i][0]not in changed_bodies:continue
            if fcl:
                distance=float(fcl.distance(self.robot_objects[i],payload_objects[j],fcl.DistanceRequest(),fcl.DistanceResult()))
                collision=not np.isfinite(distance)or distance<=2*margin
            else:
                distance=None;collision=shape_overlap(placed_robot[i],placed_payload[j],margin=margin)
            if collision:
                why=dict(kind='carried_payload_vs_own_robot',query=payload[j][1],actor=self.robot_rows[i][0],collider=self.robot_rows[i][1],distance_m=distance,required_clearance_m=2*margin,intended_contact_exemption='official native distal pad only; proximal clipped CAD remains checked')
                self.stats['rejected']+=1;self.stats['last_rejection']=why;return False,why
        return True,None
