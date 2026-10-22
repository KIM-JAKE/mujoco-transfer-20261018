"""CPU batch image formation from the live Isaac PhysX scene.

Static USD collider triangles are prepared once. Dynamic collider triangles
use live PhysX body poses strictly inside this camera. The perception boundary
exports only RGB/depth/K/extrinsics, never IDs, poses or hit normals.
Embree is an image ray tracer; all simulation/contact dynamics remain PhysX.
"""
from pathlib import Path
import sys,time,numpy as np
from pxr import UsdGeom,UsdPhysics,Usd
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE/'bin-picking/realtime-deps-python312'))
from embreex import rtcore_scene,rtcore
_DEVICE=rtcore.EmbreeDevice()
from embreex.mesh_construction import TriangleMesh
from rgbd_material_sensor import look_at

def rotation(q):
    q=np.asarray(q);v=q[:3];w=q[3];K=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]]);return np.eye(3)+2*w*K+2*K@K

def triangles(prim):
    if prim.IsA(UsdGeom.Mesh):
        mesh=UsdGeom.Mesh(prim);v=np.asarray(mesh.GetPointsAttr().Get(),float)
        if prim.HasAPI(UsdPhysics.MeshCollisionAPI)and UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()=='convexHull':
            from scipy.spatial import ConvexHull,QhullError
            try:
                hull=ConvexHull(v);used=np.unique(hull.simplices.ravel());inverse=np.full(len(v),-1,np.int32);inverse[used]=np.arange(len(used));return v[used],inverse[hull.simplices]
            except QhullError:pass
        counts=np.asarray(mesh.GetFaceVertexCountsAttr().Get());idx=np.asarray(mesh.GetFaceVertexIndicesAttr().Get(),np.int32)
        if np.all(counts==3):return v,idx.reshape(-1,3)
        if np.all(counts==4):
            quad=idx.reshape(-1,4);return v,np.concatenate([quad[:,[0,1,2]],quad[:,[0,2,3]]])
        f=[];offset=0
        for size in counts:
            poly=idx[offset:offset+size];f.extend((poly[0],poly[j],poly[j+1])for j in range(1,size-1));offset+=size
        return v,np.array(f,np.int32).reshape(-1,3)
    if prim.IsA(UsdGeom.Cube):
        h=float(UsdGeom.Cube(prim).GetSizeAttr().Get())/2;v=np.array([[x,y,z]for x in [-h,h]for y in [-h,h]for z in [-h,h]]);return v,np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],[2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]],np.int32)
    if prim.IsA(UsdGeom.Cylinder)or prim.IsA(UsdGeom.Cone):
        shape=UsdGeom.Cylinder(prim)if prim.IsA(UsdGeom.Cylinder)else UsdGeom.Cone(prim);r=float(shape.GetRadiusAttr().Get());h=float(shape.GetHeightAttr().Get())/2;angle=np.arange(64)*2*np.pi/64;low=np.c_[r*np.cos(angle),r*np.sin(angle),np.full(64,-h)];top=np.c_[(r if prim.IsA(UsdGeom.Cylinder)else 0)*np.cos(angle),(r if prim.IsA(UsdGeom.Cylinder)else 0)*np.sin(angle),np.full(64,h)];v=np.vstack([low,top,[0,0,-h],[0,0,h]]);f=[]
        for j in range(64):
            b=(j+1)%64;f.extend([(j,b,64+b),(j,64+b,64+j),(128,b,j),(129,64+j,64+b)])
        axis=shape.GetAxisAttr().Get()
        if axis=='X':v=v[:,[2,0,1]]
        elif axis=='Y':v=v[:,[0,2,1]]
        return v,np.array(f,np.int32)
    if prim.IsA(UsdGeom.Sphere):
        radius=float(UsdGeom.Sphere(prim).GetRadiusAttr().Get());a=np.linspace(0,np.pi,25);b=np.arange(48)*2*np.pi/48;v=np.array([[radius*np.sin(t)*np.cos(u),radius*np.sin(t)*np.sin(u),radius*np.cos(t)]for t in a for u in b]);f=[]
        for i in range(24):
            for j in range(48):
                k=i*48+j;n=i*48+(j+1)%48;f.extend([(k,n,n+48),(k,n+48,k+48)])
        return v,np.array(f,np.int32)
    raise RuntimeError('Unsupported enabled camera collider '+str(prim.GetPath())+' '+prim.GetTypeName())

def assemble(rows):
    vertices=[];faces=[];colors=[];offset=0
    for v,f,c in rows:
        vertices.append(v);faces.append(f+offset);colors.append(np.repeat(c[None,:],len(f),axis=0));offset+=len(v)
    return np.concatenate(vertices).astype(np.float32),np.concatenate(faces).astype(np.int32),np.concatenate(colors).astype(np.float32)

def scene(rows):
    if not rows:return None,None
    v,f,c=assemble(rows);s=rtcore_scene.EmbreeScene(_DEVICE);TriangleMesh(s,v,f);s.run(np.array([[10,10,10]],np.float32),np.array([[1,0,0]],np.float32));return s,c

class BatchRGBDCamera:
    def __init__(self,stage,view,moving_paths=None,build_embree=True):
        from quality_physx_camera import material
        start=time.perf_counter();mesh_counter=0;anchored=set()
        for pr in stage.Traverse():
            if pr.IsA(UsdPhysics.FixedJoint):
                joint=UsdPhysics.FixedJoint(pr);a=joint.GetBody0Rel().GetTargets();b=joint.GetBody1Rel().GetTargets()
                if not a and b:anchored.add(str(b[0]))
                elif a and not b:anchored.add(str(a[0]))
        cache=UsdGeom.XformCache(Usd.TimeCode.Default());static=[];groups={};self.body_views={};self.geometry={}
        for p in stage.Traverse():
            if not p.HasAPI(UsdPhysics.CollisionAPI)or not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get():continue
            if '/PlanningQueries/'in str(p.GetPath()):continue
            v,f=triangles(p);mesh_counter+=1
            if mesh_counter%100==0:print('BATCH_GEOMETRY',mesh_counter,flush=True)
            if not len(f):continue
            M=np.array(cache.GetLocalToWorldTransform(p)).T;body=p
            while body and not body.HasAPI(UsdPhysics.RigidBodyAPI):body=body.GetParent()
            color=material(stage,str(p.GetPath()))[0]
            if body and UsdPhysics.RigidBodyAPI(body).GetRigidBodyEnabledAttr().Get()and ((str(body.GetPath())in moving_paths)if moving_paths is not None else(not UsdPhysics.RigidBodyAPI(body).GetKinematicEnabledAttr().Get()and str(body.GetPath())not in anchored)):
                path=str(body.GetPath());B=np.array(cache.GetLocalToWorldTransform(body)).T;local=np.linalg.inv(B)@M;groups.setdefault(path,[]).append((v@local[:3,:3].T+local[:3,3],f,color))
            else:static.append((v@M[:3,:3].T+M[:3,3],f,color))
        print('BATCH_GEOMETRY_SUMMARY',dict(static_meshes=len(static),dynamic_bodies=len(groups),static_triangles=sum(len(r[1])for r in static),dynamic_triangles=sum(len(r[1])for rows in groups.values()for r in rows)),flush=True)
        self.static_vertices,self.static_faces,self.static_colors=assemble(static);self.static=None
        if build_embree:
            self.static=rtcore_scene.EmbreeScene(_DEVICE);TriangleMesh(self.static,self.static_vertices,self.static_faces);self.static.run(np.array([[10,10,10]],np.float32),np.array([[1,0,0]],np.float32))
        self.local_scenes={};self.local_bounds={}
        for path,rows in groups.items():
            vv,ff,cc=assemble(rows);self.geometry[path]=(vv,ff,cc);self.body_views[path]=None;local=None
            if build_embree:
                local=rtcore_scene.EmbreeScene(_DEVICE);TriangleMesh(local,vv,ff);local.run(np.array([[10,10,10]],np.float32),np.array([[1,0,0]],np.float32))
            self.local_scenes[path]=local;lo=vv.min(0);hi=vv.max(0);self.local_bounds[path]=np.array([[x,y,z]for x in [lo[0],hi[0]]for y in [lo[1],hi[1]]for z in [lo[2],hi[2]]])
        self.all_bodies=view.create_rigid_body_view(list(self.geometry));self.pose_indices={str(p):i for i,p in enumerate(self.all_bodies.prim_paths)}
        for path in self.geometry:
            if path not in self.pose_indices:raise RuntimeError('Camera body tensor not bound: '+path)
        self.preparation_s=time.perf_counter()-start;print('BATCH_CAMERA_PREPARED',len(static),len(groups),self.preparation_s,flush=True)
    def export(self,path):
        payload=dict(static_vertices=self.static_vertices,static_faces=self.static_faces,static_colors=self.static_colors,paths=np.array(list(self.geometry)))
        for i,name in enumerate(self.geometry):
            v,f,c=self.geometry[name];payload[f'v{i}']=v;payload[f'f{i}']=f;payload[f'c{i}']=c;payload[f'b{i}']=self.local_bounds[name]
        np.savez(path,**payload)
    def snapshot(self):
        poses=self.all_bodies.get_transforms().copy();return {path:poses[self.pose_indices[path]]for path in self.geometry}
    def capture(self,eye,target,span=.20,width=512,height=512,out=None,body_poses=None,far_clip=None):
        started=time.perf_counter();body_poses=self.snapshot()if body_poses is None else body_poses;T=look_at(eye,target);f=width*np.linalg.norm(np.array(eye)-target)/span;u,v=np.meshgrid(np.arange(width)-(width-1)/2,np.arange(height)-(height-1)/2);local=np.stack([u/f,v/f,np.ones_like(u)],-1).reshape(-1,3);scale=np.linalg.norm(local,axis=1);rays=(local/scale[:,None]@T[:3,:3].T).astype(np.float32);origin=np.asarray(eye);far=float(np.linalg.norm(np.array(eye)-target)+.25)if far_clip is None else float(far_clip);best=np.full(len(rays),np.inf,np.float32);normal=np.zeros_like(rays);base=np.zeros_like(rays)
        for path,ss in self.local_scenes.items():
            actor_started=time.perf_counter();tr=body_poses[path];R=rotation(tr[3:]);position=tr[:3];corners=(self.local_bounds[path]@R.T+position-T[:3,3])@T[:3,:3]
            if corners[:,2].max()<=0:continue
            if corners[:,2].min()<=.001:x0,y0,x1,y1=0,0,width,height
            else:
                pixel=corners[:,:2]/corners[:,2,None]*f+[(width-1)/2,(height-1)/2];low=np.floor(pixel.min(0)-2).astype(int);high=np.ceil(pixel.max(0)+2).astype(int);x0=max(0,low[0]);y0=max(0,low[1]);x1=min(width,high[0]+1);y1=min(height,high[1]+1)
            if x1<=x0 or y1<=y0:continue
            indices=(np.arange(y0,y1)[:,None]*width+np.arange(x0,x1)).ravel();local_origin=(origin-position)@R;local_rays=(rays[indices]@R).astype(np.float32);hit=ss.run(np.tile(local_origin,(len(indices),1)).astype(np.float32),local_rays,dists=np.minimum(best[indices],far).astype(np.float32),output=True);keep=(hit['primID']>=0)&(hit['tfar']<best[indices]);selected=indices[keep];best[selected]=hit['tfar'][keep];normal[selected]=hit['Ng'][keep]@R.T;base[selected]=self.geometry[path][2][hit['primID'][keep]]
            actor_elapsed=time.perf_counter()-actor_started
            if actor_elapsed>.02:print('BATCH_SLOW_ACTOR',path,actor_elapsed,len(indices),len(self.geometry[path][1]),flush=True)
        dynamic_done=time.perf_counter()
        if self.static is not None:
            hit=self.static.run(np.tile(origin,(len(rays),1)).astype(np.float32),rays,dists=np.minimum(best,far).astype(np.float32),output=True);valid=(hit['primID']>=0)&(hit['tfar']<best);best[valid]=hit['tfar'][valid];normal[valid]=hit['Ng'][valid];base[valid]=self.static_colors[hit['primID'][valid]]
        static_done=time.perf_counter()
        valid=np.isfinite(best);normal/=np.maximum(np.linalg.norm(normal,axis=1)[:,None],1e-12);dot=np.einsum('ij,ij->i',normal,rays);normal*=np.where(dot>0,-1.,1.)[:,None];light=np.array([-.4,-.5,.9]);light/=np.linalg.norm(light);shade=.35+.55*np.maximum(0,normal@light)+.10*np.maximum(0,np.einsum('ij,ij->i',normal,-rays));rgb=np.full((len(rays),3),32,np.uint8);rgb[valid]=np.clip(base[valid]*shade[valid,None]*255,0,255);depth=best/scale;depth[~valid]=np.nan
        packet=dict(rgb=rgb.reshape(height,width,3),depth_m=depth.astype(np.float32).reshape(height,width),K=np.array([[f,0,(width-1)/2],[0,f,(height-1)/2],[0,0,1.]]),T_cell_camera=T)
        if out:
            with Path(out).open('wb')as fp:np.savez(fp,**packet)
        self.last_capture_s=time.perf_counter()-started;self.last_profile=dict(dynamic_s=dynamic_done-started,static_s=static_done-dynamic_done,shading_s=time.perf_counter()-static_done);return packet
