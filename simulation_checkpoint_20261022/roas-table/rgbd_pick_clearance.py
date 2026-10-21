"""Observed-cloud checks for an open native hand approaching a ring.

Uses RGB-D pixels and class CAD only. It never queries scene actors or object
identities. Static structures are checked separately by the PhysX planner.
"""
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
from rgbd_geometry import cloud,axis_frame

class ObservedGraspClearance:
    def __init__(self,here):
        from isaac_port.fieldkit.perception import CadPoseEstimator
        self.estimator=CadPoseEstimator();mesh=np.load(Path(here)/'native-official-finger.npz');self.planes=ConvexHull(mesh['vertices']).equations
        self.palm_planes=ConvexHull(np.load(Path(here)/'native-hand-assets.npz')['vertices']).equations
    def points(self,packet,calibration):
        P=cloud(packet);rgb=packet['rgb'];gray=rgb.mean(2);bounds=np.asarray(calibration['outer_bounds_world_m']);mask=np.isfinite(P).all(2)&(np.ptp(rgb.astype(float),axis=2)<45)&(gray>35)&(gray<180)&np.all((P>=bounds[0]-.003)&(P<=bounds[1]+.015),axis=2);v=P[mask];return v[::max(1,len(v)//12000)]
    def clear(self,points,pose,pre,grasp,opening=.08):
        center=np.array(pose['center_m']);near=points[np.linalg.norm(points-center,axis=1)<.13]
        if not len(near):return True
        distance,_=self.estimator.trees[pose['part']].query((near-center)@axis_frame(pose['axis']));obstacles=near[distance>.0025]
        if not len(obstacles):return True
        R=pre[:3,:3];root=pre[:3,3]-R@np.array([0,0,.1034]);delta=grasp[:3,3]-pre[:3,3];A=self.planes[:,:3];b=self.planes[:,3];margin=.0006
        for sign in [1,-1]:
            F=R@(np.eye(3) if sign==1 else np.diag([-1.,-1.,1.]));origin=root+R@np.array([0,sign*opening/2,.0584]);local=(obstacles-origin)@F;direction=delta@F;low=np.zeros(len(local));high=np.ones(len(local));valid=np.ones(len(local),bool)
            for normal,offset in zip(A,b):
                c=local@normal+offset-margin;d=float(direction@normal)
                if d>1e-10:low=np.maximum(low,c/d)
                elif d< -1e-10:high=np.minimum(high,c/d)
                else:valid&=c<=0
            if np.any(valid&(low<=high)):return False
        return True
    def palm_clear(self,points,pose,pre,grasp):
        """Permit movable-stock fingertip contact, keep the palm clear.

        Physical collisions, finite actuator effort and measured speed guards
        still apply to every loose body. No collision filter is changed here.
        """
        center=np.asarray(pose['center_m']);near=points[np.linalg.norm(points-center,axis=1)<.23]
        if not len(near):return True
        R=pre[:3,:3];root=pre[:3,3]-R@np.array([0,0,.1034]);local=(near-root)@R;direction=(grasp[:3,3]-pre[:3,3])@R
        low=np.zeros(len(local));high=np.ones(len(local));valid=np.ones(len(local),bool)
        for plane in self.palm_planes:
            normal,offset=plane[:3],plane[3]
            value=local@normal+offset-.0006;speed=float(direction@normal)
            if speed>1e-10:low=np.maximum(low,value/speed)
            elif speed< -1e-10:high=np.minimum(high,value/speed)
            else:valid&=value<=0
        return not np.any(valid&(low<=high))
