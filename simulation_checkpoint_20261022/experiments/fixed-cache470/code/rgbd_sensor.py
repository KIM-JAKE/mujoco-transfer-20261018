"""Ideal pinhole RGB-D camera of live Isaac CPU PhysX surfaces.

Only RGB, metric depth, intrinsics and calibrated camera extrinsics leave this
module. No collider identities, body transforms, hit normals or semantic masks
are exposed to perception. Physics collision geometry is the rendered surface;
this is an ideal synthetic depth sensor, not RTX or a hardware camera model.
"""
from pathlib import Path
import numpy as np
import omni.physx

def look_at(eye,target):
    eye=np.asarray(eye,float);forward=np.asarray(target,float)-eye;forward/=np.linalg.norm(forward)
    right=np.cross(forward,[0,0,1.])
    if np.linalg.norm(right)<.01:right=np.array([1.,0,0])
    right/=np.linalg.norm(right);down=np.cross(forward,right)
    T=np.eye(4);T[:3,:3]=np.column_stack([right,down,forward]);T[:3,3]=eye
    return T

def capture(eye,target,span=.20,width=384,height=384,out=None):
    T=look_at(eye,target);distance=np.linalg.norm(np.array(eye)-target);f=width*distance/span
    u,v=np.meshgrid(np.arange(width)-(width-1)/2,np.arange(height)-(height-1)/2)
    local=np.stack([u/f,v/f,np.ones_like(u)],-1).reshape(-1,3);scale=np.linalg.norm(local,axis=1);rays=local/scale[:,None]@T[:3,:3].T
    depth=np.full(len(rays),np.nan,np.float32);rgb=np.full((len(rays),3),32,np.uint8)
    query=omni.physx.get_physx_scene_query_interface();light=np.array([-.4,-.5,.9]);light/=np.linalg.norm(light)
    for i,d in enumerate(rays):
        h=query.raycast_closest(tuple(T[:3,3]),tuple(d),4.)
        if not h['hit']:continue
        depth[i]=float(h['distance'])/scale[i]
        n=np.array(h['normal']);n=n if n@d<0 else -n
        # Deliberately identical neutral material for all surfaces: no ID colors.
        shade=.22+.65*max(0.,float(n@light))+.13*max(0.,float(n@(-d)))
        rgb[i]=np.clip(np.array([.60,.62,.65])*shade*255,0,255)
    packet=dict(rgb=rgb.reshape(height,width,3),depth_m=depth.reshape(height,width),K=np.array([[f,0,(width-1)/2],[0,f,(height-1)/2],[0,0,1.]]),T_cell_camera=T)
    if out:
        out=Path(out);out.parent.mkdir(parents=True,exist_ok=True);np.savez_compressed(out,**packet)
        with out.with_suffix('.ppm').open('wb')as fp:fp.write(f'P6\n{width} {height}\n255\n'.encode()+packet['rgb'].tobytes())
    return packet
