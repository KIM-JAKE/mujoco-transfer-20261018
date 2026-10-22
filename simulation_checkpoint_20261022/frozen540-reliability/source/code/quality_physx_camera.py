"""Isaac CPU ray-query renderer of visible USD meshes, with shadow lighting.

For inspection when all GPU reservations are occupied. This is not RTX output.
Use a render-only stage: its triangle queries must never change test physics.
"""
import numpy as np
import re
import omni.physx
from pxr import Usd, UsdGeom


def material(stage, path):
    if any(x in path for x in ['workpiece_ring','BinStock']):
        return np.array([.55,.59,.64]), .8
    if 'workpiece_pin' in path:return np.array([.55,.59,.64]),.8
    if 'ProposedThreePinHolder' in path:return np.array([.05,.48,.18]),.1
    if 'NativeHand' in path:
        return (np.array([.035,.042,.050]),.2)if 'finger'in path else(np.array([.83,.85,.87]),.5)
    if '_arm_fr3' in path:
        links=re.findall(r'_arm_fr3v2_link(\d+)',path)
        link=links[-1]if links else ''
        return (np.array([.045,.055,.07]),.3)if link in ['0','6','8']else(np.array([.86,.88,.91]),.4)
    if any('instance_%04d'%k in path for k in [659,664,665,670,671,672,673,674,675,686]):return np.array([.09,.105,.125]),.05
    if 'Ground' in path:return np.array([.20,.23,.27]),.05
    p=stage.GetPrimAtPath(path)
    color=UsdGeom.Gprim(p).GetDisplayColorAttr().Get()if p.IsA(UsdGeom.Gprim)else None
    if color and len(color):return np.clip(np.array(color[0]),.08,.85),.25
    return np.array([.50,.54,.59]),.35


def render(stage, camera_path, width=960, height=720, shadows=True):
    cam=UsdGeom.Camera(stage.GetPrimAtPath(camera_path));T=np.asarray(UsdGeom.Xformable(cam).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
    f=width*cam.GetFocalLengthAttr().Get()/cam.GetHorizontalApertureAttr().Get();u,v=np.meshgrid(np.arange(width)-(width-1)/2,np.arange(height)-(height-1)/2)
    rays=np.stack([u/f,-v/f,-np.ones_like(u)],-1).reshape(-1,3);rays/=np.linalg.norm(rays,axis=1)[:,None];rays=rays@T[:3,:3].T;origin=tuple(T[:3,3]);query=omni.physx.get_physx_scene_query_interface()
    sky=np.array([.20,.26,.34])[None,:]*(.8+.2*np.maximum(0,rays[:,2,None]));linear=sky.copy()
    light=np.array([-.45,-.55,.9]);light/=np.linalg.norm(light);fill=np.array([.7,.1,.6]);fill/=np.linalg.norm(fill)
    palette={}
    for i,d in enumerate(rays):
        hit=query.raycast_closest(origin,tuple(d),8.)
        if not hit['hit']:continue
        path=str(hit.get('collision',''))
        if path not in palette:palette[path]=material(stage,path)
        base,metal=palette[path];n=np.array(hit['normal'],float)
        if n@d>0:n=-n
        diffuse=max(0.,n@light);shadow=1.
        if shadows and diffuse>.01:
            point=np.array(hit['position'])+n*.0004
            shadow=.22 if query.raycast_closest(tuple(point),tuple(light),4.)['hit']else 1.
        half=light-d;half/=np.linalg.norm(half);spec=max(0.,n@half)**(55 if metal>.5 else 28)
        rim=max(0.,1-abs(n@d))**4
        linear[i]=base*(.18+.7*diffuse*shadow+.22*max(0.,n@fill))+(metal*.5+.06)*spec*shadow+base*.10*rim
    # Soft filmic mapping and sRGB output; no diagnostic object-color coding.
    linear=linear/(.75+linear);rgb=np.clip(linear,0,1)**(1/2.2)
    return np.uint8(np.clip(rgb*255,0,255)).reshape(height,width,3)
