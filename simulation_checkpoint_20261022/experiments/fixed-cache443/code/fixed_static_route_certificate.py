"""Compile conservative static-world certificates for unchanged robot samples.

Uses the ORIGINAL PhysX mesh overlap queries and temporary NONCOLLIDING query
meshes. This never moves a physical body. Opposite-arm, same-arm, held/own robot,
completed-part, current encoder bridge, contact and motor guards are separate
and must still run. A False row means original live checking is required.
"""
from contextlib import contextmanager
from pathlib import Path
import hashlib,json,math
import numpy as np
from scipy.spatial import ConvexHull

_VERSION='original-physx-static-envelope-v1'

def rigid(T):
    T=np.asarray(T,float)
    return (T.shape==(4,4) and np.isfinite(T).all() and
            np.max(abs(T[:3,:3].T@T[:3,:3]-np.eye(3)))<1e-6 and
            abs(np.linalg.det(T[:3,:3])-1)<1e-6 and
            np.max(abs(T[3]-[0,0,0,1]))<1e-10)

def expanded_vertices(vertices,radius):
    """L-infinity cube is a conservative superset of Euclidean dilation."""
    v=np.asarray(vertices,float);r=float(radius)
    if v.ndim!=2 or v.shape[1]!=3 or not np.isfinite(v).all() or not 0<=r<=.05:raise ValueError('Invalid CAD envelope')
    offsets=np.array([[x,y,z]for x in (-r,r)for y in (-r,r)for z in (-r,r)])
    return (v[:,None,:]+offsets[None,:,:]).reshape(-1,3)

def rotation_angle(A,B):
    return math.acos(float(np.clip((np.trace(A.T@B)-1)/2,-1,1)))

def grasp_in_envelope(nominal,current,translation_m,rotation_rad):
    if not rigid(nominal) or not rigid(current):return False
    # 1nm / 1nrad inward reserve: boundary uncertainty always falls back.
    return (np.linalg.norm(np.asarray(current)[:3,3]-np.asarray(nominal)[:3,3])<=translation_m-1e-9 and
            rotation_angle(np.asarray(nominal)[:3,:3],np.asarray(current)[:3,:3])<=rotation_rad-1e-9)

def _digest_array(h,a):
    a=np.ascontiguousarray(a);h.update(a.dtype.str.encode());h.update(repr(a.shape).encode());h.update(a.tobytes())

def payload_geometry_sha256(checker):
    h=hashlib.sha256(b'original-class-payload-CAD-v1')
    for body,source,shape in checker.payload_boxes:
        h.update(source.encode())
        for value in shape:_digest_array(h,value)
    return h.hexdigest()

def exact_row_key(q):
    a=np.ascontiguousarray(q,dtype=np.float64)
    if a.shape!=(7,) or not np.isfinite(a).all():return None
    return hashlib.sha256(a.tobytes()).hexdigest()

def static_world_seal(checker):
    """Authored static geometry seal; moving fixture joints require sweeps.

    Recompute at recipe preparation and after layout/calibration/scene changes,
    never from stale USD free-workpiece or moving-arm transforms. The collision
    policy ignores free bodies only in the existing RGB-D-only task mode.
    """
    from pxr import UsdPhysics,UsdGeom
    from isaac_kinematics import world
    h=hashlib.sha256(_VERSION.encode());inventory=[];dynamic={}
    for prim in checker.stage.Traverse():
        path=str(prim.GetPath())
        if path.startswith('/World/PlanningQueries') or '/BinStock/' in path or '/workpiece_' in path:continue
        if '/left_arm_base' in path or '/right_arm_base' in path:continue
        if not prim.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:continue
        owner=prim
        while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):owner=owner.GetParent()
        body=str(owner.GetPath()) if owner else None
        h.update(path.encode());h.update(str(prim.GetTypeName()).encode());_digest_array(h,world(checker.stage,path))
        # All authored collider attributes include original geometry, cooking,
        # filters/contact offsets and primitive parameters without rounding.
        for attr in sorted(prim.GetAttributes(),key=lambda a:a.GetName()):
            name=attr.GetName()
            if name.startswith(('points','faceVertex','physics:','physxCollision:','size','radius','height','axis','xformOp')):
                h.update(name.encode());value=attr.Get()
                if name.startswith(('points','faceVertex','xformOp')) and value is not None:
                    array=np.asarray(value)
                    if array.dtype.kind in 'biufc':_digest_array(h,array)
                    else:h.update(repr(value).encode())
                else:h.update(repr(value).encode())
        inventory.append(path)
        if body:dynamic.setdefault(body,[]).append(path)
    for prim in checker.stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):continue
        joint=UsdPhysics.Joint(prim)
        touched=[str(x)for x in joint.GetBody1Rel().GetTargets() if str(x) in dynamic]
        if not touched:continue
        if not prim.IsA(UsdPhysics.PrismaticJoint) or joint.GetBody0Rel().GetTargets():raise ValueError('Static certificate cannot enclose unsupported moving fixture '+str(prim.GetPath()))
        h.update(str(prim.GetPath()).encode())
        for attr in sorted(prim.GetAttributes(),key=lambda a:a.GetName()):h.update(attr.GetName().encode());h.update(repr(attr.Get()).encode())
        for body in touched:dynamic[body]=dict(colliders=dynamic[body],joint=str(prim.GetPath()))
    if any(not isinstance(value,dict) for value in dynamic.values()):raise ValueError('Static certificate has unbounded moving fixture')
    return dict(sha256=h.hexdigest(),colliders=inventory,moving_fixture=dynamic,version=_VERSION)

def _query_row(checker,body,source,vertices,index):
    from pxr import UsdGeom,PhysicsSchemaTools,Sdf
    hull=ConvexHull(vertices);v=np.asarray(vertices)[hull.vertices]
    hull=ConvexHull(v);faces=hull.simplices.copy()
    normals=np.cross(v[faces[:,1]]-v[faces[:,0]],v[faces[:,2]]-v[faces[:,0]])
    flip=np.einsum('ij,ij->i',normals,hull.equations[:,:3])<0
    faces[flip]=faces[flip][:,[0,2,1]]
    path='/World/PlanningQueries/static_envelope_'+checker.side+'_'+str(index)
    mesh=UsdGeom.Mesh.Define(checker.stage,path);mesh.CreatePointsAttr(v.astype(np.float32));mesh.CreateFaceVertexCountsAttr([3]*len(faces));mesh.CreateFaceVertexIndicesAttr(faces.ravel());mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateVisibilityAttr('invisible')
    xf=UsdGeom.Xformable(mesh);xf.ClearXformOpOrder();op=xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble)
    return (body,source,op,PhysicsSchemaTools.encodeSdfPath(Sdf.Path(path)))

class StaticRows:
    def __init__(self,metadata,row_keys):
        self.metadata=metadata;self.row_keys=set(row_keys)
    def permits(self,q,width,current_G,world_sha,model_sha,*,payload_sha256=None):
        m=self.metadata
        if world_sha!=m['world_sha256'] or model_sha!=m['model_sha256']:return False
        if payload_sha256!=m['payload_geometry_sha256']:return False
        lo,hi=m['width_interval_m'];width=float(width)
        if not np.isfinite(width) or not lo+1e-9<=width<=hi-1e-9:return False
        if m['nominal_hand_object_T'] is not None and not grasp_in_envelope(np.asarray(m['nominal_hand_object_T']),current_G,m['translation_m'],m['rotation_rad']):return False
        return exact_row_key(q) in self.row_keys
    def save(self,path):
        Path(path).write_text(json.dumps(dict(metadata=self.metadata,row_keys=sorted(self.row_keys)),indent=2)+'\n')
    @classmethod
    def load(cls,path):
        doc=json.loads(Path(path).read_text())
        if doc['metadata'].get('version')!=_VERSION:raise ValueError('Unknown proof version')
        return cls(doc['metadata'],doc['row_keys'])

def compile_static_rows(checker,qs,width_interval_m,*,world_seal,model_sha256,
                        nominal_hand_object_T=None,translation_m=.003,
                        rotation_rad=.02,fixture_sweep_clear=None):
    """Return StaticRows plus per-row verdicts; only exact input q rows stored.

    fixture_sweep_clear(transforms) must check ALL conservative full-stroke
    fixture CAD envelopes. Required when the sealed world has moving fixtures;
    no callback means no certificate (never ignore a moving clamp silently).
    Caller owns immutable scope: no actuation/layout edits while compiling.
    """
    import os
    if os.environ.get('M80_ROAS_RGB_ONLY')!='1':raise ValueError('Free-body collision policy must use existing RGB-D-only mode')
    if checker.preview_other_boxes is None:raise ValueError('Explicit other-arm context required; pair checks remain separate')
    if any('/left_arm_base/' not in body and '/right_arm_base/' not in body for body in checker.preview_other_actors):raise ValueError('Other-arm exclusion contains a non-robot actor')
    if getattr(checker,'ignored_future_actors',set()):raise ValueError('Static certificate cannot inherit contact actor omissions')
    lo,hi=map(float,width_interval_m)
    if not 0<=lo<hi<=.080001:raise ValueError('Invalid native width interval')
    if world_seal.get('moving_fixture') and fixture_sweep_clear is None:raise ValueError('Moving fixture full-stroke envelope checker required')
    if not isinstance(model_sha256,str) or len(model_sha256)!=64:raise ValueError('Complete original FK/native CAD/source fingerprint required')
    G=None if nominal_hand_object_T is None else np.asarray(nominal_hand_object_T,float)
    if G is not None and not rigid(G):raise ValueError('Invalid nominal public RGB-D grasp')
    if bool(checker.payload_path)!=(G is not None):raise ValueError('Payload template and nominal grasp must agree')
    if not .0001<=translation_m<=.01 or not .0001<=rotation_rad<=.1:raise ValueError('Invalid bounded grasp envelope')
    rows=[];payload_rows=[];query_shapes=[];mid=(lo+hi)/2
    # Float32 query-mesh conversion receives an extra 1um outward cube guard.
    for body,source,shape in checker.pair_boxes:
        v=np.asarray(shape[3],float)
        if body.endswith('_finger'):
            d=(hi-lo)/4;v=np.concatenate([v+[0,-d,0],v+[0,d,0]])
        v=expanded_vertices(v,1e-6)
        query_shapes.append((body,source,v))
        rows.append(_query_row(checker,body,source,v,len(rows)))
    if G is not None:
        for body,source,shape in checker.payload_boxes:
            v=np.asarray(shape[3],float)
            radius=float(np.max(np.linalg.norm(v,axis=1)))
            d=translation_m+2*radius*math.sin(rotation_rad/2)+1e-6
            v=v@G[:3,:3].T+G[:3,3]
            v=expanded_vertices(v,d)
            query_shapes.append((checker.arm.hand_path,source,v))
            payload_rows.append(_query_row(checker,checker.arm.hand_path,source,v,10000+len(payload_rows)))
    if fixture_sweep_clear is not None and hasattr(fixture_sweep_clear,'bind'):
        fixture_sweep_clear.bind(query_shapes)
    original=(checker.rows,checker.payload_rows,getattr(checker,'ignored_future_actors',set()))
    checker.rows=rows;checker.payload_rows=payload_rows
    checker.ignored_future_actors=set(original[2])|set(world_seal.get('moving_fixture',{}))
    mask=[];rejections=[]
    try:
        for index,q in enumerate(qs):
            if exact_row_key(q) is None:raise ValueError('Invalid candidate q')
            transforms=checker.arm.bodies(np.asarray(q,float),mid)
            ok,why=checker._clear_transforms(transforms,None)
            if ok and fixture_sweep_clear is not None:ok,why=fixture_sweep_clear(transforms)
            mask.append(bool(ok));rejections.append(why)
            if index and index%256==0:print('STATIC_ENVELOPE_COMPILE_PROGRESS',dict(rows=index+1,certified=sum(mask)),flush=True)
    finally:checker.rows,checker.payload_rows,checker.ignored_future_actors=original
    metadata=dict(version=_VERSION,world_sha256=world_seal['sha256'],model_sha256=model_sha256,
                  payload_geometry_sha256=payload_geometry_sha256(checker),width_interval_m=[lo,hi],nominal_hand_object_T=None if G is None else G.tolist(),
                  translation_m=translation_m,rotation_rad=rotation_rad,
                  authority='original_PhysX_static_overlap_plus_conservative_fixture_sweeps',
                  dynamic_checks_required=['current_start_bridge','opposite_arm','own_payload','placed_parts','encoder_tracking','contact','FR3_limits'])
    cert=StaticRows(metadata,[exact_row_key(q)for q,ok in zip(qs,mask)if ok])
    return cert,np.asarray(mask,bool),rejections

class PrismaticFixtureSweeps:
    """Conservative full-stroke CAD envelopes for world-anchored linear jaws.

    The original authored prismatic stroke is expanded by 1mm in every axis,
    covering a stated simulation-model reserve. No moving fixture is omitted.
    Unsupported joints/geometry reject compilation. Hardware stroke compliance
    still requires the field fixture model and PLC/visual interlock.
    """
    def __init__(self,checker,world_seal,*,stroke_reserve_m=.001):
        import fcl
        from pxr import UsdPhysics
        from isaac_kinematics import world,frame,rotation
        from fcl_transport import mesh_data,convex
        self.fcl=fcl;self.convex=convex;self.objects=[];self.sources=[];self.native=None
        if not .0005<=stroke_reserve_m<=.005:raise ValueError('Invalid fixture stroke reserve')
        for body,record in world_seal['moving_fixture'].items():
            joint=UsdPhysics.PrismaticJoint(checker.stage.GetPrimAtPath(record['joint']))
            if joint.GetBody0Rel().GetTargets():raise ValueError('Moving fixture not anchored directly to world')
            axis=np.eye(3)['XYZ'.index(joint.GetAxisAttr().Get())]
            J0=frame(joint.GetLocalPos0Attr().Get(),rotation(joint.GetLocalRot0Attr().Get()))
            J1=frame(joint.GetLocalPos1Attr().Get(),rotation(joint.GetLocalRot1Attr().Get()))
            limits=np.array([joint.GetLowerLimitAttr().Get(),joint.GetUpperLimitAttr().Get()],float)
            if not np.isfinite(limits).all() or limits[1]<limits[0] or limits[1]-limits[0]>.05:raise ValueError('Unbounded fixture joint')
            Tbody=world(checker.stage,body)
            for source in record['colliders']:
                data=mesh_data(checker.stage.GetPrimAtPath(source))
                if data is None:raise ValueError('Unsupported swept fixture collider '+source)
                local=np.linalg.inv(Tbody)@world(checker.stage,source)
                v=np.asarray(data[0],float)@local[:3,:3].T+local[:3,3]
                ends=[]
                for q in limits:
                    T=J0@frame(axis*q,np.eye(3))@np.linalg.inv(J1)
                    ends.append(v@T[:3,:3].T+T[:3,3])
                geom=convex(expanded_vertices(np.concatenate(ends),stroke_reserve_m))
                self.objects.append(fcl.CollisionObject(geom));self.sources.append(source)
    def bind(self,query_shapes):
        self.native=[(body,source,self.fcl.CollisionObject(self.convex(vertices)))for body,source,vertices in query_shapes]
        return self
    def __call__(self,transforms):
        if self.native is None:raise RuntimeError('Swept-fixture checker has no exact query envelope geometry')
        for body,source,obj in self.native:
            T=transforms[body];obj.setTransform(self.fcl.Transform(np.ascontiguousarray(T[:3,:3]),np.ascontiguousarray(T[:3,3])))
            for obstacle,other in zip(self.objects,self.sources):
                distance=float(self.fcl.distance(obj,obstacle,self.fcl.DistanceRequest(),self.fcl.DistanceResult()))
                if not np.isfinite(distance) or distance<=1e-9:
                    return False,dict(kind='static_full_stroke_fixture_envelope',query=source,collider=other,distance_m=distance)
        return True,None

@contextmanager
def bound_static_rows(checker,certificate,*,current_world_sha256,current_model_sha256,diagnostics=None):
    """Bind one sealed static segment; ONLY original static queries may skip.

    Rebuild current_world_sha256 and current_model_sha256 at recipe preparation
    and after any layout/calibration/model edit. The caller must call this
    predicate after its ORIGINAL fresh self/payload/placed/other-arm checks.
    Grasp and widths are read from the current public image/encoder binding.
    No actual/cmd joint bridge is covered unless its exact q was compiled.
    """
    old=getattr(checker,'static_route_proof',None)
    payload_sha=payload_geometry_sha256(checker)
    geometry_identity=tuple((body,source,id(shape)) for body,source,shape in checker.payload_boxes)
    rows_identity=tuple((body,source,id(shape)) for body,source,shape in checker.pair_boxes)
    stats=diagnostics if diagnostics is not None else {}
    stats.update(static_certificate_hits=0,static_certificate_misses=0)
    def permits(q,width,selected=None):
        try:
            unchanged=(geometry_identity==tuple((body,source,id(shape)) for body,source,shape in checker.payload_boxes) and rows_identity==tuple((body,source,id(shape)) for body,source,shape in checker.pair_boxes))
            accepted=(unchanged and checker.preview_other_boxes is not None and
                      not getattr(checker,'ignored_future_actors',set()) and
                      certificate.permits(q,width,checker.payload_relative,current_world_sha256,current_model_sha256,payload_sha256=payload_sha))
        except Exception:accepted=False
        stats['static_certificate_hits' if accepted else 'static_certificate_misses']+=1
        return bool(accepted)
    checker.static_route_proof=permits
    try:yield stats
    finally:checker.static_route_proof=old
