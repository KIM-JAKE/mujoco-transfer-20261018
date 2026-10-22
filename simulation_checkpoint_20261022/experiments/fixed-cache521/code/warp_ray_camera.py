"""Optional GPU ray-image backend, API-compatible with NumericRayCamera.

Warp triangle rays form RGB-D pixels only. Private sensor snapshots never
leave capture(), and packet outputs contain no actor identifiers/body poses.
This is a GPU software ray camera, not RTX rendering or GPU PhysX.
"""
from pathlib import Path
import time
import numpy as np
import warp as wp


def rotation(q):
    q=np.asarray(q);v=q[:3];w=q[3];K=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    return np.eye(3)+2*w*K+2*K@K


def look_at(eye,target):
    eye=np.asarray(eye,float);forward=np.asarray(target,float)-eye;forward/=np.linalg.norm(forward);right=np.cross(forward,[0,0,1.])
    if np.linalg.norm(right)<.01:right=np.array([1.,0,0])
    right/=np.linalg.norm(right);down=np.cross(forward,right);T=np.eye(4);T[:3,:3]=np.column_stack([right,down,forward]);T[:3,3]=eye
    return T


@wp.func
def _aabb_hit(origin:wp.vec3,direction:wp.vec3,low:wp.vec3,high:wp.vec3,far:float):
    enter=float(0.0);leave=far
    for axis in range(3):
        if wp.abs(direction[axis])<1.0e-10:
            if origin[axis]<low[axis]-1.0e-6 or origin[axis]>high[axis]+1.0e-6:
                return False
        else:
            a=(low[axis]-1.0e-6-origin[axis])/direction[axis]
            b=(high[axis]+1.0e-6-origin[axis])/direction[axis]
            enter=wp.max(enter,wp.min(a,b));leave=wp.min(leave,wp.max(a,b))
    return enter<=leave


@wp.kernel
def _trace(rays:wp.array(dtype=wp.vec3),origin:wp.vec3,width:int,far:float,
           mesh_ids:wp.array(dtype=wp.uint64),rotations:wp.array(dtype=wp.mat33),positions:wp.array(dtype=wp.vec3),
           bounds_low:wp.array(dtype=wp.vec3),bounds_high:wp.array(dtype=wp.vec3),rectangles:wp.array(dtype=wp.vec4i),
           color_offsets:wp.array(dtype=wp.int32),colors:wp.array(dtype=wp.vec3),dynamic_count:int,static_id:wp.uint64,
           static_offset:int,static_only:int,best_t:wp.array(dtype=float),normals:wp.array(dtype=wp.vec3),bases:wp.array(dtype=wp.vec3)):
    i=wp.tid();x=i%width;y=i//width;ray=rays[i]
    best=float(wp.inf);normal=wp.vec3(0.0);color=wp.vec3(0.0)
    if static_only==0:
        for actor in range(dynamic_count):
            rect=rectangles[actor]
            if x>=rect[0] and x<rect[2] and y>=rect[1] and y<rect[3]:
                R=rotations[actor];inverse=wp.transpose(R)
                local_origin=inverse*(origin-positions[actor]);local_ray=inverse*ray
                limit=wp.min(best,far)
                if _aabb_hit(local_origin,local_ray,bounds_low[actor],bounds_high[actor],limit):
                    query=wp.mesh_query_ray(mesh_ids[actor],local_origin,local_ray,limit)
                    if query.result and query.t<best:
                        best=query.t;normal=R*query.normal;color=colors[color_offsets[actor]+query.face]
    query=wp.mesh_query_ray(static_id,origin,ray,wp.min(best,far))
    if query.result and query.t<best:
        best=query.t;normal=query.normal;color=colors[static_offset+query.face]
    best_t[i]=best;normals[i]=normal;bases[i]=color


@wp.kernel
def _shade(rays:wp.array(dtype=wp.vec3),scales:wp.array(dtype=float),best_t:wp.array(dtype=float),
           normals:wp.array(dtype=wp.vec3),bases:wp.array(dtype=wp.vec3),rgb:wp.array(dtype=wp.uint8,ndim=2),depth:wp.array(dtype=float)):
    i=wp.tid();distance=best_t[i]
    rgb[i,0]=wp.uint8(32);rgb[i,1]=wp.uint8(32);rgb[i,2]=wp.uint8(32);depth[i]=float(wp.nan)
    if wp.isfinite(distance):
        normal=normals[i];length=wp.length(normal);normal=normal/wp.max(length,1.0e-12);ray=rays[i]
        if wp.dot(normal,ray)>0.0:normal=-normal
        light=wp.normalize(wp.vec3(-.4,-.5,.9))
        shade=.35+.55*wp.max(0.0,wp.dot(normal,light))+.10*wp.max(0.0,wp.dot(normal,-ray))
        color=bases[i]*shade*255.0
        rgb[i,0]=wp.uint8(wp.clamp(color[0],0.0,255.0));rgb[i,1]=wp.uint8(wp.clamp(color[1],0.0,255.0));rgb[i,2]=wp.uint8(wp.clamp(color[2],0.0,255.0))
        depth[i]=distance/scales[i]


class WarpRayCamera:
    def __init__(self,model,device='cuda:0',bvh_constructor='lbvh',gpu_shading=True):
        began=time.perf_counter();wp.init();self.device=wp.get_device(device)
        if not self.device.is_cuda:raise ValueError('WarpRayCamera requires an explicitly reserved CUDA device')
        self.gpu_shading=bool(gpu_shading);self.paths=[];self.local_bounds={};self.meshes=[];self._buffers=None
        colors=[];offsets=[];count=0
        print('WARP_CAMERA_BVH_START',str(self.device),flush=True)
        with np.load(model) as z:
            for index,path in enumerate(z['paths'].tolist()):
                vertices=np.ascontiguousarray(z[f'v{index}'],np.float32);faces=np.ascontiguousarray(z[f'f{index}'],np.int32)
                self.meshes.append(wp.Mesh(points=wp.array(vertices,dtype=wp.vec3,device=self.device),indices=wp.array(faces.ravel(),dtype=wp.int32,device=self.device),bvh_constructor=bvh_constructor))
                self.paths.append(path);self.local_bounds[path]=z[f'b{index}'].copy();c=np.ascontiguousarray(z[f'c{index}'],np.float32)
                offsets.append(count);colors.append(c);count+=len(c)
            self.static_offset=count
            self.static=wp.Mesh(points=wp.array(np.ascontiguousarray(z['static_vertices'],np.float32),dtype=wp.vec3,device=self.device),
                                indices=wp.array(np.ascontiguousarray(z['static_faces'],np.int32).ravel(),dtype=wp.int32,device=self.device),bvh_constructor=bvh_constructor)
            colors.append(np.ascontiguousarray(z['static_colors'],np.float32));self.triangle_count=count+len(colors[-1])
        self.colors=wp.array(np.concatenate(colors),dtype=wp.vec3,device=self.device)
        self.mesh_ids=wp.array(np.asarray([mesh.id for mesh in self.meshes],np.uint64),dtype=wp.uint64,device=self.device)
        self.color_offsets=wp.array(np.asarray(offsets,np.int32),dtype=wp.int32,device=self.device)
        self.bounds_low=wp.array(np.asarray([self.local_bounds[p].min(0) for p in self.paths],np.float32),dtype=wp.vec3,device=self.device)
        self.bounds_high=wp.array(np.asarray([self.local_bounds[p].max(0) for p in self.paths],np.float32),dtype=wp.vec3,device=self.device)
        wp.synchronize_device(self.device);self.preparation_s=time.perf_counter()-began
        print('WARP_CAMERA_BVH_READY',self.triangle_count,self.preparation_s,flush=True)

    def snapshot(self):
        raise RuntimeError('GPU renderer requires a sensor-internal snapshot')

    def capture(self,eye,target,span=.20,width=512,height=512,out=None,body_poses=None,far_clip=None,static_only=False):
        began=time.perf_counter();body_poses=self.snapshot()if body_poses is None else body_poses
        eye=np.asarray(eye,float);target=np.asarray(target,float);width=int(width);height=int(height)
        if eye.shape!=(3,)or target.shape!=(3,)or not np.isfinite(np.r_[eye,target]).all()or not 0<span<=5 or not 1<=min(width,height) or max(width,height)>4096 or np.linalg.norm(eye-target)<.001:
            raise ValueError('Finite metric camera pose/FOV and bounded image size required')
        T=look_at(eye,target);f=width*np.linalg.norm(eye-target)/span
        u,v=np.meshgrid(np.arange(width)-(width-1)/2,np.arange(height)-(height-1)/2)
        local=np.stack([u/f,v/f,np.ones_like(u)],-1).reshape(-1,3);scales=np.linalg.norm(local,axis=1)
        rays=(local/scales[:,None]@T[:3,:3].T).astype(np.float32)
        far=float(np.linalg.norm(eye-target)+.25)if far_clip is None else float(far_clip)
        if not np.isfinite(far)or far<=0:raise ValueError('Positive finite ray far clip required')
        rotations=[];positions=[];rectangles=[]
        for path in self.paths:
            tr=np.r_[np.zeros(3),[0.,0.,0.,1.]]if static_only else np.asarray(body_poses[path],float)
            if tr.shape!=(7,)or not np.isfinite(tr).all()or abs(np.linalg.norm(tr[3:])-1)>.001:raise ValueError('Invalid private sensor rigid transform')
            R=rotation(tr[3:]);position=tr[:3];rotations.append(R);positions.append(position)
            corners=(self.local_bounds[path]@R.T+position-T[:3,3])@T[:3,:3]
            if corners[:,2].max()<=0:rectangles.append([0,0,0,0]);continue
            if corners[:,2].min()<=.001:x0,y0,x1,y1=0,0,width,height
            else:
                pixel=corners[:,:2]/corners[:,2,None]*f+[(width-1)/2,(height-1)/2]
                low=np.floor(pixel.min(0)-2).astype(int);high=np.ceil(pixel.max(0)+2).astype(int)
                x0=max(0,int(low[0]));y0=max(0,int(low[1]));x1=min(width,int(high[0])+1);y1=min(height,int(high[1])+1)
            rectangles.append([x0,y0,x1,y1])
        n=width*height
        if self._buffers is None or self._buffers[0]!=n:
            self._buffers=(n,wp.empty(n,dtype=float,device=self.device),wp.empty(n,dtype=wp.vec3,device=self.device),
                           wp.empty(n,dtype=wp.vec3,device=self.device),wp.empty((n,3),dtype=wp.uint8,device=self.device),wp.empty(n,dtype=float,device=self.device))
        _,best,normal,base,rgb_gpu,depth_gpu=self._buffers
        rays_gpu=wp.array(rays,dtype=wp.vec3,device=self.device);scales_gpu=wp.array(scales.astype(np.float32),dtype=float,device=self.device)
        R_gpu=wp.array(np.asarray(rotations,np.float32),dtype=wp.mat33,device=self.device);P_gpu=wp.array(np.asarray(positions,np.float32),dtype=wp.vec3,device=self.device)
        rect_gpu=wp.array(np.asarray(rectangles,np.int32),dtype=wp.vec4i,device=self.device)
        uploaded=time.perf_counter()
        wp.launch(_trace,n,inputs=[rays_gpu,wp.vec3(*eye),width,far,self.mesh_ids,R_gpu,P_gpu,self.bounds_low,self.bounds_high,rect_gpu,
                                   self.color_offsets,self.colors,len(self.paths),self.static.id,self.static_offset,int(static_only),best,normal,base],device=self.device)
        wp.synchronize_device(self.device);traced=time.perf_counter()
        if self.gpu_shading:
            wp.launch(_shade,n,inputs=[rays_gpu,scales_gpu,best,normal,base,rgb_gpu,depth_gpu],device=self.device)
            rgb=rgb_gpu.numpy();depth=depth_gpu.numpy()
        else:
            distances=best.numpy();normals=normal.numpy();bases=base.numpy();valid=np.isfinite(distances)
            normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-12);dot=np.einsum('ij,ij->i',normals,rays);normals*=np.where(dot>0,-1.,1.)[:,None]
            light=np.array([-.4,-.5,.9]);light/=np.linalg.norm(light);shade=.35+.55*np.maximum(0,normals@light)+.10*np.maximum(0,np.einsum('ij,ij->i',normals,-rays))
            rgb=np.full((n,3),32,np.uint8);rgb[valid]=np.clip(bases[valid]*shade[valid,None]*255,0,255);depth=(distances/scales).astype(np.float32);depth[~valid]=np.nan
        packet=dict(rgb=rgb.reshape(height,width,3),depth_m=depth.reshape(height,width),K=np.array([[f,0,(width-1)/2],[0,f,(height-1)/2],[0,0,1.]]),T_cell_camera=T)
        if out:
            with Path(out).open('wb')as fp:np.savez(fp,**packet)
        self.last_capture_s=time.perf_counter()-began
        self.last_profile=dict(upload_and_setup_s=uploaded-began,gpu_ray_s=traced-uploaded,gpu_shading_and_download_s=time.perf_counter()-traced,
                               dynamic_s=traced-began,static_s=0.,shading_s=time.perf_counter()-traced,backend='warp_gpu_triangle_rays',device=str(self.device))
        return packet


# Drop-in constructor name when explicitly selected by the worker adapter.
NumericRayCamera=WarpRayCamera
