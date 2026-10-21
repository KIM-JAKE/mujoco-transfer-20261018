"""Plan against the live Isaac PhysX scene, without moving physical actors.

Invisible, non-colliding USD meshes are only query shapes. PhysX cooks the
same convex approximation used by the actual robot collision shapes.
"""
import numpy as np
import omni.physx
from pxr import UsdGeom,UsdPhysics,PhysicsSchemaTools,Gf,Sdf
from isaac_kinematics import world, frame
from predictive_pair_clearance import proxy, pair_clear, other_at

class Checker:
    def __init__(self,stage,arm,side,allow_loose_fingertips=False):
        self.allow_loose_fingertips=allow_loose_fingertips
        self.pair_boxes=[];self.preview_other_boxes=None;self.preview_other_actors=set()
        self.payload_path=None;self.payload_rows=[];self.payload_boxes=[];self.payload_relative=None;self.side=side
        self.stage=stage;self.arm=arm;self.query=omni.physx.get_physx_scene_query_interface();self.rows=[];self.own_prefix=arm.hand_path.split('/'+side+'_arm_base')[0]+'/'+side+'_arm_base';allowed=set(arm.link_paths+[arm.hand_path]+[arm.hand_path+'/'+side+'_'+label+'_finger'for label in ['left','right']])
        for prim in list(stage.Traverse()):
            if not prim.HasAPI(UsdPhysics.CollisionAPI)or UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()==False:continue
            body=prim
            while body and not body.HasAPI(UsdPhysics.RigidBodyAPI):body=body.GetParent()
            if not body or str(body.GetPath())not in allowed:continue
            bp=str(body.GetPath());local=np.linalg.inv(world(stage,bp))@world(stage,str(prim.GetPath()))
            if prim.IsA(UsdGeom.Mesh):
                mesh=UsdGeom.Mesh(prim);v=np.asarray(mesh.GetPointsAttr().Get());f=np.asarray(mesh.GetFaceVertexIndicesAttr().Get());counts=np.asarray(mesh.GetFaceVertexCountsAttr().Get())
            elif prim.IsA(UsdGeom.Cube):
                half=UsdGeom.Cube(prim).GetSizeAttr().Get()/2;v=np.array([[x,y,z]for x in [-half,half]for y in [-half,half]for z in [-half,half]]);f=np.array([0,1,3,0,3,2,4,6,7,4,7,5,0,4,5,0,5,1,2,3,7,2,7,6,0,2,6,0,6,4,1,5,7,1,7,3]);counts=np.array([3]*12)
            else:continue
            vertices=v@local[:3,:3].T+local[:3,3];self.pair_boxes.append((bp,str(prim.GetPath()),proxy(vertices)));path='/World/PlanningQueries/'+side+'_shape'+str(len(self.rows));m=UsdGeom.Mesh.Define(stage,path);m.CreatePointsAttr(vertices.astype(np.float32));m.CreateFaceVertexCountsAttr(counts);m.CreateFaceVertexIndicesAttr(f);m.CreateSubdivisionSchemeAttr('none');m.CreateVisibilityAttr('invisible');xf=UsdGeom.Xformable(m);xf.ClearXformOpOrder();op=xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble);encoded=PhysicsSchemaTools.encodeSdfPath(Sdf.Path(path));self.rows.append((bp,str(prim.GetPath()),op,encoded))
    def other_arm_at(self,other,q,opening):return other_at(self,other,q,opening)
    def attach_payload(self,path,relative):
        """Collision planning only: never attach a physical workpiece."""
        self.payload_path=path;self.payload_relative=relative;self.payload_rows=[];self.payload_boxes=[];base=np.linalg.inv(world(self.stage,path));root=self.stage.GetPrimAtPath(path+'/CPUConvexContact')
        for prim in root.GetChildren():
            if not prim.IsA(UsdGeom.Mesh):continue
            mesh=UsdGeom.Mesh(prim);v=np.asarray(mesh.GetPointsAttr().Get());local=base@world(self.stage,str(prim.GetPath()));vertices=v@local[:3,:3].T+local[:3,3];self.payload_boxes.append((path,str(prim.GetPath()),proxy(vertices)));qpath='/World/PlanningQueries/'+self.side+'_payload'+str(len(self.payload_rows));m=UsdGeom.Mesh.Define(self.stage,qpath);m.CreatePointsAttr(vertices.astype(np.float32));m.CreateFaceVertexCountsAttr(mesh.GetFaceVertexCountsAttr().Get());m.CreateFaceVertexIndicesAttr(mesh.GetFaceVertexIndicesAttr().Get());m.CreateSubdivisionSchemeAttr('none');m.CreateVisibilityAttr('invisible');xf=UsdGeom.Xformable(m);xf.ClearXformOpOrder();op=xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble);encoded=PhysicsSchemaTools.encodeSdfPath(Sdf.Path(qpath));self.payload_rows.append((path,str(prim.GetPath()),op,encoded))
    def detach_payload(self):self.payload_path=None;self.payload_rows=[];self.payload_boxes=[];self.payload_relative=None
    def clear(self,q,opening,selected=None):
        if self.preview_other_boxes is not None:
            ok,why=pair_clear(self,q,opening)
            if not ok:return ok,why
        transforms=self.arm.bodies(q,opening)
        if self.payload_path:transforms[self.payload_path]=transforms[self.arm.hand_path]@self.payload_relative
        return self._clear_transforms(transforms,selected)
    def hand_clear(self,tcp,opening,selected=None):
        """Reject a fixed end-effector collision before searching elbow IK."""
        root=tcp@frame([0,0,-.1034],np.eye(3));transforms={self.arm.hand_path:root}
        for sign,label in [(1,'left'),(-1,'right')]:
            transforms[self.arm.hand_path+'/'+self.side+'_'+label+'_finger']=root@frame([0,sign*opening/2,.0584],np.eye(3)if sign==1 else np.diag([-1.,-1.,1.]))
        return self._clear_transforms(transforms,selected)
    def _clear_transforms(self,transforms,selected):
        for body,source,op,encoded in self.rows+self.payload_rows:
            if body not in transforms:continue
            T=transforms[body];op.Set(Gf.Matrix4d(*T.T.ravel().tolist()));hits=[]
            def callback(hit):
                actor=str(hit.rigid_body);collision=str(hit.collision)
                if __import__('os').environ.get('M80_ROAS_RGB_ONLY')=='1' and ('/BinStock/'in actor or '/workpiece_'in actor):return True
                if actor in getattr(self,'ignored_future_actors',set()):return True
                if actor in self.preview_other_actors:return True
                if actor.startswith(self.own_prefix):return True
                # A carried object's current actor is a stale obstacle for
                # future robot poses; its query geometry moves with the hand.
                if self.payload_path and actor==self.payload_path:return True
                if body==self.payload_path and actor==self.payload_path:return True
                if selected and actor==selected and source.endswith('/tip'):return True
                # Loose stock may be nudged by fingertips during bin picking.
                # The palm, arm, basket and tray still require free clearance.
                if self.allow_loose_fingertips and source.endswith('/tip')and(actor.startswith('/World/BinStock/')or actor in ['/World/Cell/Geometry/workpiece_ring1','/World/Cell/Geometry/workpiece_ring2']):return True
                hits.append(dict(query=source,actor=actor,collider=collision));return False
            self.query.overlap_mesh(*encoded,callback,False)
            if hits:return False,hits[0]
        return True,None
    def path(self,qs,opening,selected=None):
        for j,(a,b)in enumerate(zip(qs[:-1],qs[1:])):
            for sample,u in enumerate(np.linspace(0,1,max(2,int(np.ceil(np.max(abs(np.array(a)-b))/getattr(self,'resolution_rad',.035)))+1))):
                if j and sample==0:continue  # same immutable-scene endpoint was just checked
                ok,why=self.clear(np.array(a)*(1-u)+np.array(b)*u,opening,selected)
                if not ok:return False,dict(segment=j,fraction=float(u),collision=why)
        return True,None

    def route(self,start,goal,opening,selected=None,seconds=12,skip_direct=False,use_fast=True):
        """Short bounded RRT-Connect fallback using this live PhysX world.

        This CPU fallback is deliberately not labelled as cuRobo. Every edge
        uses the same structural obstacle checks as the direct path.
        """
        import time
        start=np.asarray(start,float);goal=np.asarray(goal,float)
        if not skip_direct and self.path([start,goal],opening,selected)[0]:return[start,goal]
        if not self.clear(goal,opening,selected)[0]:return None
        fast=getattr(self,'fast_planner',None)
        if fast is not None and use_fast:
            try:
                proposed=fast.route(start,goal,opening,max(.35,seconds))
                if proposed is not None:
                    from scipy.interpolate import CubicSpline
                    # Validate the curve the executor actually follows, using
                    # the original live PhysX queries at a finer resolution.
                    # The scattered controller stops at these checked joint
                    # corners. Verify that exact execution geometry rather
                    # than rejecting it for an unexecuted global cubic.
                    curve=np.asarray(proposed) if getattr(self,'execute_checked_edges',False) else CubicSpline(np.linspace(0,1,len(proposed)),proposed)(np.linspace(0,1,max(33,16*len(proposed))))
                    old_resolution=getattr(self,'resolution_rad',.035);self.resolution_rad=.010
                    try:accepted,rejection=self.path(curve,opening,selected)
                    finally:self.resolution_rad=old_resolution
                    if accepted:fast.stats['PhysX_verified_proposals']=fast.stats.get('PhysX_verified_proposals',0)+1;return proposed
                    fast.stats['PhysX_rejected_proposals']=fast.stats.get('PhysX_rejected_proposals',0)+1
                    fast.stats['last_PhysX_rejection']=rejection
            except Exception as error:
                fast.stats['last_error']=str(error)
        rng=np.random.default_rng(20261020);trees=[([start],[-1]),([goal],[-1])];deadline=time.monotonic()+seconds
        def extend(tree,target):
            nodes,parents=tree;near=int(np.argmin([np.linalg.norm(v-target)for v in nodes]));delta=target-nodes[near];distance=np.linalg.norm(delta);new=nodes[near]+delta*min(1.,.20/max(distance,1e-12))
            if not self.path([nodes[near],new],opening,selected)[0]:return None,False
            nodes.append(new);parents.append(near);return len(nodes)-1,distance<=.20
        def chain(tree,i):
            result=[]
            while i>=0:result.append(tree[0][i]);i=tree[1][i]
            return result[::-1]
        turn=0
        while time.monotonic()<deadline:
            a=turn%2;b=1-a;target=trees[b][0][-1]if rng.random()<.3 else rng.uniform(self.arm.bounds[:,0],self.arm.bounds[:,1]);i,_=extend(trees[a],target)
            if i is not None:
                for _ in range(50):
                    if time.monotonic()>deadline:break
                    j,reached=extend(trees[b],trees[a][0][i])
                    if j is None:break
                    if reached:
                        left=chain(trees[a],i);right=chain(trees[b],j);path=left+right[::-1][1:]
                        path=path if a==0 else path[::-1]
                        # Remove detours before returning a stop-at-corners path.
                        for _ in range(24):
                            if len(path)<3:break
                            i,j=sorted(rng.choice(len(path),2,replace=False))
                            if j>i+1 and self.path([path[i],path[j]],opening,selected)[0]:path=path[:i+1]+path[j:]
                        return path
            turn+=1
        return None
