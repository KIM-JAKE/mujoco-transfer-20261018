"""Vectorized shading around Isaac PhysX CPU raycasts, for trace videos."""
import numpy as np
import omni.physx
from pxr import UsdGeom,Usd

def render(stage,camera_path,width=384,height=288):
    camera=UsdGeom.Camera(stage.GetPrimAtPath(camera_path));T=np.asarray(UsdGeom.Xformable(camera).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T;f=width*camera.GetFocalLengthAttr().Get()/camera.GetHorizontalApertureAttr().Get();u,v=np.meshgrid(np.arange(width)-(width-1)/2,np.arange(height)-(height-1)/2);rays=np.stack([u/f,-v/f,-np.ones_like(u)],axis=-1).reshape(-1,3);rays/=np.linalg.norm(rays,axis=1)[:,None];rays=rays@T[:3,:3].T;origin=tuple(T[:3,3]);query=omni.physx.get_physx_scene_query_interface();normal=np.zeros_like(rays);indices=np.full(len(rays),-1,int);names={};colors=[]
    for i,d in enumerate(rays):
        hit=query.raycast_closest(origin,tuple(d),5.)
        if not hit['hit']:continue
        name=str(hit.get('collision',''));key=names.get(name)
        if key is None:
            key=len(colors);names[name]=key;color=[.57,.59,.62]
            if any(f'instance_{k:04d}'in name for k in [659,664,665,670,671,672,673,674,675,686]):color=[.12,.13,.15]
            if 'NativeHand'in name:color=[.83,.84,.86]if 'finger'not in name else[.08,.08,.10]
            if '/workpiece_ring1/'in name:color=[.58,.60,.63]
            elif '/workpiece_ring2/'in name:color=[.58,.60,.63]
            elif '/workpiece_pin/'in name:color=[.58,.60,.63]
            elif '/BinStock/'in name:color=[.58,.60,.63]
            if '/ROAS/'in name:color=[.7,.73,.76]
            if 'ProposedThreePinHolder'in name:color=[.12,.65,.2]
            colors.append(color)
        normal[i]=hit['normal'];indices[i]=key
    mask=indices>=0;rgb=np.full((len(rays),3),36,np.uint8)
    if np.any(mask):
        light=np.array([-.3,-.4,.866]);light/=np.linalg.norm(light);n=normal[mask];half=-rays[mask]+light;half/=np.maximum(np.linalg.norm(half,axis=1)[:,None],1e-9);diffuse=np.maximum(0,n@light);spec=np.maximum(0,np.einsum('ij,ij->i',n,half))**35;base=np.asarray(colors)[indices[mask]];rgb[mask]=np.clip(255*(base*(.35+.65*diffuse[:,None])+.18*spec[:,None]),0,255)
    return rgb.reshape(height,width,3)
