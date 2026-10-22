"""CPU geometric proposal planner. Every returned path still needs PhysX checks.

Uses fixed collision CAD, robot encoders, and image-estimated carried-part CAD.
Free-stock actors are excluded exactly as in the existing PhysX planner.
This module does not advance physics or write any physical actor state.
"""
from pathlib import Path
import builtins,hashlib,time
import numpy as np
from scipy.spatial import ConvexHull
from pxr import UsdGeom,UsdPhysics
from isaac_kinematics import world
from predictive_pair_clearance import at_pose
try:
    import fcl
except ImportError:
    fcl=None


def convex(vertices):
    vertices=np.ascontiguousarray(vertices,dtype=np.float64);hull=ConvexHull(vertices)
    faces=hull.simplices.copy()
    normals=np.cross(vertices[faces[:,1]]-vertices[faces[:,0]],vertices[faces[:,2]]-vertices[faces[:,0]])
    flip=np.einsum('ij,ij->i',normals,hull.equations[:,:3])<0
    faces[flip]=faces[flip][:,[0,2,1]]
    encoded=np.c_[np.full(len(faces),3,dtype=np.int64),faces].ravel()
    return fcl.Convex(vertices,len(faces),encoded)


def mesh_data(prim):
    if prim.IsA(UsdGeom.Mesh):
        mesh=UsdGeom.Mesh(prim);v=np.asarray(mesh.GetPointsAttr().Get(),float);indices=np.asarray(mesh.GetFaceVertexIndicesAttr().Get());counts=mesh.GetFaceVertexCountsAttr().Get();faces=[];offset=0
        for count in counts:
            face=indices[offset:offset+count];offset+=count
            faces.extend((int(face[0]),int(face[i]),int(face[i+1])) for i in range(1,count-1))
        approximation=UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
        return v,np.asarray(faces,np.int32),approximation not in [None,'none']
    if prim.IsA(UsdGeom.Cube):
        half=float(UsdGeom.Cube(prim).GetSizeAttr().Get())/2
        return np.array([[x,y,z] for x in [-half,half] for y in [-half,half] for z in [-half,half]]),None,True
    if prim.IsA(UsdGeom.Cylinder):
        shape=UsdGeom.Cylinder(prim);r=float(shape.GetRadiusAttr().Get())/np.cos(np.pi/32);half=float(shape.GetHeightAttr().Get())/2;a=np.arange(32)*2*np.pi/32
        v=np.array([[r*np.cos(t),r*np.sin(t),z]for z in [-half,half]for t in a]);axis=str(shape.GetAxisAttr().Get())
        if axis=='X':v=v[:,[2,0,1]]
        elif axis=='Y':v=v[:,[0,2,1]]
        return v,None,True
    return None


class StaticWorld:
    def __init__(self,stage):
        self.objects=[];self.geometries=[];self.skipped=[];self.geom_names={};self.convex_cache={}
        for prim in stage.Traverse():
            path=str(prim.GetPath())
            if not prim.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()==False:continue
            if '/BinStock/' in path or '/workpiece_' in path:continue
            if any('/'+side+'_arm_base' in path for side in ['left','right']):continue
            T=world(stage,path)
            if prim.IsA(UsdGeom.Sphere):
                geom=fcl.Sphere(float(UsdGeom.Sphere(prim).GetRadiusAttr().Get())*float(np.linalg.svd(T[:3,:3],compute_uv=False).max()));obj=fcl.CollisionObject(geom,fcl.Transform(T[:3,3]))
            else:
                data=mesh_data(prim)
                if data is None:self.skipped.append(path);continue
                v,faces,is_convex=data
                if not len(v) or not np.isfinite(v).all():self.skipped.append(path);continue
                v=v@T[:3,:3].T+T[:3,3]
                try:
                    if is_convex:geom=convex(v)
                    else:
                        geom=fcl.BVHModel();geom.beginModel(len(v),len(faces));geom.addSubModel(v,faces);geom.endModel()
                    obj=fcl.CollisionObject(geom)
                except Exception:self.skipped.append(path);continue
            self.geometries.append(geom);self.objects.append(obj)
            self.geom_names[id(geom)]=path
        self.manager=fcl.DynamicAABBTreeCollisionManager();self.manager.registerObjects(self.objects);self.manager.setup()


class FastPlanner:
    def __init__(self,checker,static_world):
        self.checker=checker;self.world=static_world;self.geometry_cache=static_world.convex_cache
        self.stats=dict(routes=0,proposals=0,clear_calls=0,seconds=0.,static_objects=len(static_world.objects),static_skipped=len(static_world.skipped),backend='FCL CPU proposals + live PhysX verification')
    def geometry(self,vertices,padding=0.):
        canonical=np.round(vertices,12);canonical[canonical==0]=0.
        key=(padding,hashlib.sha256(canonical.tobytes()).digest())
        if key not in self.geometry_cache:
            points=vertices
            if padding:
                offsets=np.array([[x,y,z]for x in [-padding,padding]for y in [-padding,padding]for z in [-padding,padding]])
                points=(vertices[:,None,:]+offsets[None,:,:]).reshape(-1,3)
            self.geometry_cache[key]=convex(points)
        return self.geometry_cache[key]
    def route(self,start,goal,opening,seconds):
        started=time.perf_counter();self.stats['routes']+=1;c=self.checker
        rows=c.pair_boxes+c.payload_boxes;geometries=[self.geometry(row[2][3])for row in rows];objects=[fcl.CollisionObject(g)for g in geometries]
        inflated_geometries=[self.geometry(row[2][3],.0005)for row in rows];inflated_objects=[fcl.CollisionObject(g)for g in inflated_geometries]
        moving=fcl.DynamicAABBTreeCollisionManager();moving.registerObjects(objects);moving.setup()
        inflated=fcl.DynamicAABBTreeCollisionManager();inflated.registerObjects(inflated_objects);inflated.setup()
        if getattr(c,'preview_other_configuration',None) is not None:other,q,width=c.preview_other_configuration
        elif getattr(c,'other_configuration',None):
            other,q,width=c.other_configuration()
        else:other=None
        other_rows=[] if other is None else other.pair_boxes+other.payload_boxes
        other_transforms={} if other is None else other.arm.bodies(q,width)
        if other is not None and other.payload_path:other_transforms[other.payload_path]=other_transforms[other.arm.hand_path]@other.payload_relative
        other_geometries=[self.geometry(row[2][3],.0005)for row in other_rows]
        other_objects=[fcl.CollisionObject(g,fcl.Transform(other_transforms[row[0]][:3,:3],other_transforms[row[0]][:3,3])) for g,row in zip(other_geometries,other_rows)]
        obstacle=fcl.DynamicAABBTreeCollisionManager();obstacle.registerObjects(other_objects);obstacle.setup()
        request=fcl.CollisionRequest(num_max_contacts=1,enable_contact=False);cache={}
        def first_collision(a,b,data):
            hit=fcl.collide(a,b,data.request,data.result)>0
            if hit:data.result.is_collision=True;data.done=True
            return hit
        def clear(q,diagnose=False):
            key=q.tobytes()
            if key in cache:return cache[key]
            transforms=c.arm.bodies(q,opening)
            if c.payload_path:transforms[c.payload_path]=transforms[c.arm.hand_path]@c.payload_relative
            for obj,expanded,(body,_,_)in zip(objects,inflated_objects,rows):
                T=transforms[body];tf=fcl.Transform(T[:3,:3],T[:3,3]);obj.setTransform(tf);expanded.setTransform(tf)
            moving.update();inflated.update();self.stats['clear_calls']+=1
            data=fcl.CollisionData(request=request);moving.collide(self.world.manager,data,first_collision)
            result=not data.result.is_collision
            if not result and diagnose:
                data=fcl.CollisionData(request=fcl.CollisionRequest(num_max_contacts=1,enable_contact=True));moving.collide(self.world.manager,data,fcl.defaultCollisionCallback)
                names=dict(self.world.geom_names);names.update({id(g):row[1] for g,row in zip(geometries,rows)})
                self.stats['last_static_collision']=[[names.get(id(contact.o1),'unknown'),names.get(id(contact.o2),'unknown')]for contact in data.result.contacts]
            if result and other_objects:
                data=fcl.CollisionData(request=request);inflated.collide(obstacle,data,first_collision);result=not data.result.is_collision
                if not result and diagnose:self.stats['last_other_arm_collision']=True
            cache[key]=result;return result
        def edge(a,b):
            return all(clear(a*(1-u)+b*u)for u in np.linspace(0,1,max(2,int(np.ceil(np.max(abs(a-b))/.025))+1)))
        start=np.asarray(start,float);goal=np.asarray(goal,float)
        if not clear(start,True) or not clear(goal,True):self.stats['seconds']+=time.perf_counter()-started;self.stats['blocked_endpoints']=self.stats.get('blocked_endpoints',0)+1;return None
        deadline=time.perf_counter()+max(.5,seconds)
        # Calibrated free-space bridges retract out of the bin before turning
        # a part. They are candidate robot poses, not workpiece coordinates.
        from scipy.interpolate import CubicSpline
        current_H=c.arm.fk(start);goal_H=c.arm.fk(goal)
        retract=current_H.copy();retract[:3,3]=[.25,-.60,max(1.25,float(current_H[2,3]))]
        retract_q=c.arm.solve(retract[:3,3],retract[:3,:3],start)
        for bridge_position in [[.05,-.50,1.55],[.15,-.55,1.45]]:
            if time.perf_counter()>deadline:break
            bridge_q=c.arm.solve(np.asarray(bridge_position),goal_H[:3,:3],goal)
            if bridge_q is None:continue
            candidate=[start]+([retract_q] if retract_q is not None else [])+[bridge_q,goal]
            if not all(edge(a,b)for a,b in zip(candidate[:-1],candidate[1:])):continue
            if getattr(c,'execute_checked_edges',False):
                self.stats['proposals']+=1;self.stats['seconds']+=time.perf_counter()-started;return candidate
            curve=CubicSpline(np.linspace(0,1,len(candidate)),candidate)(np.linspace(0,1,65))
            if np.any(curve<c.arm.bounds[:,0]) or np.any(curve>c.arm.bounds[:,1]):continue
            if all(edge(a,b)for a,b in zip(curve[:-1],curve[1:])):
                self.stats['proposals']+=1;self.stats['seconds']+=time.perf_counter()-started;return candidate
        rng=np.random.default_rng(20261021);trees=[]
        for q in [start,goal]:
            values=np.empty((4096,7));values[0]=q;parents=np.full(4096,-1,int);trees.append(dict(q=values,p=parents,n=1))
        def extend(tree,target):
            n=tree['n']
            if n>=4096:return None,False
            near=int(np.argmin(np.sum((tree['q'][:n]-target)**2,axis=1)));a=tree['q'][near];delta=target-a;length=np.linalg.norm(delta);q=a+delta*min(1.,.35/max(length,1e-12))
            if not edge(a,q):return None,False
            tree['q'][n]=q;tree['p'][n]=near;tree['n']+=1;return n,length<=.35
        def chain(tree,index):
            path=[]
            while index>=0:path.append(tree['q'][index].copy());index=int(tree['p'][index])
            return path[::-1]
        turn=0;path=None
        while time.perf_counter()<deadline:
            a=turn%2;b=1-a;sample=rng.random()
            if sample<.25:target=trees[b]['q'][trees[b]['n']-1]
            elif sample<.80:
                u=rng.random();target=np.clip(start*(1-u)+goal*u+rng.normal(0,.65,7),c.arm.bounds[:,0],c.arm.bounds[:,1])
            else:target=rng.uniform(c.arm.bounds[:,0],c.arm.bounds[:,1])
            i,_=extend(trees[a],target)
            if i is not None:
                for _ in range(40):
                    if time.perf_counter()>deadline:break
                    j,reached=extend(trees[b],trees[a]['q'][i])
                    if j is None:break
                    if reached:
                        path=chain(trees[a],i)+chain(trees[b],j)[::-1][1:];path=path if a==0 else path[::-1];break
            if path is not None:break
            turn+=1
        if path is not None:
            for _ in range(24):
                if len(path)<3:break
                i,j=sorted(rng.choice(len(path),2,replace=False))
                if j>i+1 and edge(path[i],path[j]):path=path[:i+1]+path[j:]
            # Cubic interpolation may cut a collision-free RRT corner. Densify
            # the same polyline until its EXECUTED curve is also free; reject
            # it if none of these finite refinements is safe.
            safe=path if getattr(c,'execute_checked_edges',False) else None
            for subdivisions in ([] if safe is not None else [1,2,4,8]):
                candidate=[path[0]]
                for a,b in zip(path[:-1],path[1:]):candidate.extend(a*(1-u)+b*u for u in np.linspace(0,1,subdivisions+1)[1:])
                curve=CubicSpline(np.linspace(0,1,len(candidate)),candidate)(np.linspace(0,1,max(65,16*len(candidate))))
                if np.any(curve<c.arm.bounds[:,0]) or np.any(curve>c.arm.bounds[:,1]):continue
                if all(edge(a,b)for a,b in zip(curve[:-1],curve[1:])):safe=candidate;break
            path=safe
            if path is not None:self.stats['proposals']+=1
        self.stats['seconds']+=time.perf_counter()-started
        return path


def prepare(stage,checkers,key,payload_paths=()):
    if fcl is None:return dict(enabled=False,reason='python-fcl is not installed')
    started=time.perf_counter();cache=getattr(builtins,'_m80_fcl_static_cache',{})
    cache_key=hashlib.sha256(b'fixed-collision-cad-v1'+str(key).encode()).hexdigest()
    if cache_key not in cache:cache[cache_key]=StaticWorld(stage);builtins._m80_fcl_static_cache=cache
    static=cache[cache_key]
    for checker in checkers.values():
        checker.fast_planner=FastPlanner(checker,static)
        for _,_,box in checker.pair_boxes:
            checker.fast_planner.geometry(box[3]);checker.fast_planner.geometry(box[3],.0005)
    if checkers:
        checker=next(iter(checkers.values()))
        for path in payload_paths:
            checker.attach_payload(path,np.eye(4))
            try:
                for _,_,box in checker.payload_boxes:
                    checker.fast_planner.geometry(box[3]);checker.fast_planner.geometry(box[3],.0005)
            finally:checker.detach_payload()
    return dict(enabled=True,version=fcl.__version__,static_objects=len(static.objects),skipped_static_colliders=static.skipped,preparation_s=time.perf_counter()-started,uses_free_body_poses=False,proposal_only=True,live_PhysX_path_verification_required=True)
