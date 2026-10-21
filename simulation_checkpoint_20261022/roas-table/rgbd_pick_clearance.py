"""Observed-cloud checks for an open native hand approaching a ring.

Uses RGB-D pixels and class CAD only. It never queries scene actors or object
identities. Static structures are checked separately by the PhysX planner.
"""
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull,cKDTree
from rgbd_geometry import cloud,axis_frame

class ObservedGraspClearance:
    def __init__(self,here):
        from isaac_port.fieldkit.perception import CadPoseEstimator
        self.estimator=CadPoseEstimator();self._asset_dir=Path(here);mesh=np.load(Path(here)/'native-official-finger.npz');self.planes=ConvexHull(mesh['vertices']).equations
        self.palm_planes=ConvexHull(np.load(Path(here)/'native-hand-assets.npz')['vertices']).equations
    def remove_known_hand_surface(self,points,tcp,opening):
        """Remove only pixel points matching measured robot surface geometry.

        This is a 1.2mm surface band, never the Hand enclosing volume, the
        selected ring, a free-body pose, or an assertion about occluded space.
        The actual static-world/native collisions and force abort remain.
        """
        if not hasattr(self,'_hand_surface_trees'):
            trees=[]
            for name in ['native-hand-assets.npz','native-official-finger.npz']:
                mesh=np.load(self._asset_dir/name);V=mesh['vertices'];samples=[]
                for face in mesh['faces']:
                    triangle=V[face];count=max(1,int(np.ceil(max(np.linalg.norm(triangle[(i+1)%3]-triangle[i])for i in range(3))/.00075)))
                    u,v=np.triu_indices(count+1);a=u/count;b=(v-u)/count
                    samples.append(triangle[0]+a[:,None]*(triangle[1]-triangle[0])+b[:,None]*(triangle[2]-triangle[0]))
                trees.append(cKDTree(np.concatenate(samples)))
            self._hand_surface_trees=trees
        H=np.asarray(tcp,float);opening=float(opening)
        if H.shape!=(4,4)or not np.isfinite(H).all()or not 0<=opening<=.081:raise ValueError('Actual encoder Hand pose/aperture required for surface exclusion')
        local=(np.asarray(points)-H[:3,3])@H[:3,:3];local[:,2]+=.1034
        distance=self._hand_surface_trees[0].query(local,workers=1)[0]
        for sign in [1,-1]:
            finger=local-[0,sign*opening/2,.0584];finger[:,:2]*=sign
            distance=np.minimum(distance,self._hand_surface_trees[1].query(finger,workers=1)[0])
        keep=distance>.0012
        self.last_hand_surface_exclusion=dict(input_points=len(points),surface_matches=int((~keep).sum()),maximum_surface_distance_m=.0012,uses_actual_encoders=True,uses_free_body_poses=False)
        return np.asarray(points)[keep]

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
