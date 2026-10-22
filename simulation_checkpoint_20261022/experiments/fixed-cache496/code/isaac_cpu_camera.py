"""RGB/depth diagnostic from live Isaac PhysX scene queries, with no GPU.

This is a CPU collision-surface camera, not RTX/photoreal rendering. It shares
the actual simulated scene, camera pose and contact geometry. Object IDs are
saved only as validation labels and are not an input to segmentation.
"""
from pathlib import Path
import json,math,time
import numpy as np
import omni.physx
from pxr import UsdGeom,Usd

def capture(stage,out,width=512,height=384):
    out=Path(out);out.mkdir(exist_ok=True);camera=UsdGeom.Camera(stage.GetPrimAtPath('/World/Cameras/BasketCamera'));mat=np.array(UsdGeom.Xformable(camera).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T;R=mat[:3,:3];origin=mat[:3,3];f=float(camera.GetFocalLengthAttr().Get());ap=float(camera.GetHorizontalApertureAttr().Get());fx=width*f/ap;fy=fx;cx=(width-1)/2;cy=(height-1)/2
    rgb=np.full((height,width,3),38,np.uint8);depth=np.full((height,width),np.nan,np.float32);ids=np.full((height,width),-1,np.int32);names={};query=omni.physx.get_physx_scene_query_interface();light=np.array([-.3,-.4,.866]);light/=np.linalg.norm(light);start=time.monotonic()
    for v in range(height):
        for u in range(width):
            local=np.array([(u-cx)/fx,-(v-cy)/fy,-1.]);local/=np.linalg.norm(local);direction=R@local;hit=query.raycast_closest(tuple(origin),tuple(direction),4.)
            if not hit['hit']:continue
            name=str(hit.get('collision',hit.get('rigidBody','')));normal=np.array(hit['normal']);distance=float(hit['distance']);depth[v,u]=distance*(-local[2]);ids[v,u]=names.setdefault(name,len(names))
            dark=any(f'instance_{k:04d}'in name for k in [659,664,665,670,671,672,673,674,675,686]);base=np.array([.12,.13,.15])if dark else np.array([.57,.59,.62])
            if 'NativeHand'in name:base=np.array([.83,.84,.86])if 'finger'not in name else np.array([.08,.08,.10])
            if '/CPUConvexContact/'in name:base=np.array([.66,.68,.70])
            diffuse=max(0.,float(normal@light));view=-direction;half=(view+light);half/=max(np.linalg.norm(half),1e-9);spec=max(0.,float(normal@half))**35
            rgb[v,u]=np.clip((base*(.35+.65*diffuse)+.18*spec)*255,0,255)
    # Portable image output needs no external imaging dependency inside Kit.
    with (out/'basket-camera.ppm').open('wb')as fp:fp.write(f'P6\n{width} {height}\n255\n'.encode()+rgb.tobytes())
    K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
    # OpenCV convention (+X right,+Y down,+Z forward) for RGB-D consumers.
    T=mat.copy();T[:3,:3]=R@np.diag([1.,-1.,-1.])
    np.savez_compressed(out/'basket-rgbd.npz',rgb=rgb,depth_m=depth,K=K,T_cell_camera=T)
    np.savez_compressed(out/'basket-validation-only.npz',collider_ids=ids)
    report=dict(renderer='Isaac PhysX CPU collision-surface raycast camera',RTX=False,width=width,height=height,intrinsics=K.tolist(),T_cell_camera=T.tolist(),depth_scale_m=1,rectified_registered_rgbd=True,seconds=time.monotonic()-start,collider_id_validation_labels=names,pose_inference_performed=False)
    (out/'camera-report.json').write_text(json.dumps(report,indent=2));print('ISAAC_CPU_CAMERA_CAPTURED',width,height,report['seconds'],flush=True)
    return report
