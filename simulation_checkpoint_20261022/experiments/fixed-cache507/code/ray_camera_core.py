"""Pure CPU ray-image backend. No simulator imports; image formation only."""
from pathlib import Path
import sys,time,numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE/'bin-picking/realtime-deps-python312'))
from embreex import rtcore_scene,rtcore
from embreex.mesh_construction import TriangleMesh
_DEVICE=rtcore.EmbreeDevice()
def rotation(q):
    q=np.asarray(q);v=q[:3];w=q[3];K=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]]);return np.eye(3)+2*w*K+2*K@K
def look_at(eye,target):
    eye=np.asarray(eye,float);forward=np.asarray(target,float)-eye;forward/=np.linalg.norm(forward);right=np.cross(forward,[0,0,1.])
    if np.linalg.norm(right)<.01:right=np.array([1.,0,0])
    right/=np.linalg.norm(right);down=np.cross(forward,right);T=np.eye(4);T[:3,:3]=np.column_stack([right,down,forward]);T[:3,3]=eye;return T
class NumericRayCamera:
    def __init__(self,model):
        started=time.perf_counter();print('CPU_CAMERA_BVH_START',flush=True)
        with np.load(model)as z:
            v=z['static_vertices'];f=z['static_faces'];self.static_colors=z['static_colors'];self.static=rtcore_scene.EmbreeScene(_DEVICE);TriangleMesh(self.static,v,f);self.static.run(np.array([[10,10,10]],np.float32),np.array([[1,0,0]],np.float32));self.geometry={};self.local_bounds={};self.local_scenes={};print('CPU_CAMERA_STATIC_READY',time.perf_counter()-started,flush=True)
            for i,path in enumerate(z['paths'].tolist()):
                v=z[f'v{i}'];f=z[f'f{i}'];c=z[f'c{i}'];self.geometry[path]=(v,f,c);self.local_bounds[path]=z[f'b{i}'];scene=rtcore_scene.EmbreeScene(_DEVICE);TriangleMesh(scene,v,f);scene.run(np.array([[10,10,10]],np.float32),np.array([[1,0,0]],np.float32));self.local_scenes[path]=scene
                if i%10==0:print('CPU_CAMERA_DYNAMIC_READY',i,time.perf_counter()-started,flush=True)
    def snapshot(self):raise RuntimeError('Numeric renderer requires a sensor-internal snapshot')
    def capture(self,eye,target,span=.20,width=512,height=512,out=None,body_poses=None,far_clip=None,static_only=False):
        started=time.perf_counter();body_poses=self.snapshot()if body_poses is None else body_poses;T=look_at(eye,target);f=width*np.linalg.norm(np.array(eye)-target)/span;u,v=np.meshgrid(np.arange(width)-(width-1)/2,np.arange(height)-(height-1)/2);local=np.stack([u/f,v/f,np.ones_like(u)],-1).reshape(-1,3);scale=np.linalg.norm(local,axis=1);rays=(local/scale[:,None]@T[:3,:3].T).astype(np.float32);origin=np.asarray(eye);far=float(np.linalg.norm(np.array(eye)-target)+.25)if far_clip is None else float(far_clip);best=np.full(len(rays),np.inf,np.float32);normal=np.zeros_like(rays);base=np.zeros_like(rays)
        for path,ss in ({} if static_only else self.local_scenes).items():
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
