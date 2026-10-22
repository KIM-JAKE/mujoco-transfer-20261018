"""Camera-neutral metric RGB-D/point-cloud CAD registration for pickup poses.

No robot connection. Calibration, rectification and sensor precision are site
inputs. An unobservable ring axial angle stays at its calibrated prior.
"""
from dataclasses import dataclass
from datetime import datetime,timezone
from pathlib import Path
import json,time
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

PARTS=('ring1','ring2','pin')
ASSETS=Path(__file__).resolve().parent/'assets'


def transform(value):
    t=np.asarray(value,dtype=float)
    if t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1],atol=1e-8):raise ValueError('Invalid metric SE3 matrix')
    r=t[:3,:3]
    if not np.allclose(r.T@r,np.eye(3),atol=1e-6) or abs(np.linalg.det(r)-1)>1e-6:raise ValueError('Rotation must be proper and orthonormal')
    if np.linalg.norm(t[:3,3])>10:raise ValueError('Transform translation exceeds 10m; check millimetres/metres')
    return t


def pose(position,quaternion_wxyz):
    q=np.asarray(quaternion_wxyz,float)
    if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-5:raise ValueError('Invalid unit quaternion')
    t=np.eye(4);t[:3,:3]=Rotation.from_quat(q[[1,2,3,0]]).as_matrix();t[:3,3]=position
    return transform(t)


def deproject(depth,intrinsics,depth_scale_m,*,rectified,labels=None):
    if not rectified:raise ValueError('Use rectified, registered RGB-D frames')
    d=np.asarray(depth);k=np.asarray(intrinsics,float)
    if d.ndim!=2 or k.shape!=(3,3) or not np.isfinite(k).all() or min(k[0,0],k[1,1])<=0:raise ValueError('Invalid depth/intrinsics')
    if not np.isfinite(depth_scale_m) or not 0<depth_scale_m<=1:raise ValueError('Explicit depth-to-metre scale required')
    v,u=np.indices(d.shape);z=d.astype(float)*depth_scale_m;valid=np.isfinite(z)&(z>.05)&(z<5.)
    points=np.stack([(u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z],axis=-1)[valid]
    if labels is not None:
        if np.asarray(labels).shape!=d.shape:raise ValueError('Masks and depth are not registered')
        labels=np.asarray(labels)[valid]
    return points,labels


def weighted_fit(source,target,weights):
    weights=weights/weights.sum();a=np.sum(source*weights[:,None],axis=0);b=np.sum(target*weights[:,None],axis=0)
    u,_,vt=np.linalg.svd(((source-a)*weights[:,None]).T@(target-b));r=vt.T@u.T
    if np.linalg.det(r)<0:vt[-1]*=-1;r=vt.T@u.T
    return r,b-r@a


@dataclass(frozen=True)
class PoseEstimate:
    part:str
    T_cell_part:np.ndarray
    rms_m:float
    inlier_fraction:float
    inliers:int
    position_uncertainty_floor_m:float
    orientation_scope:str
    surface_extent_fraction:tuple

    def as_dict(self):
        return dict(part=self.part,T_cell_part=self.T_cell_part.tolist(),rms_m=self.rms_m,inlier_fraction=self.inlier_fraction,inliers=self.inliers,position_uncertainty_floor_m=self.position_uncertainty_floor_m,orientation_scope=self.orientation_scope,surface_extent_fraction=list(self.surface_extent_fraction))


class CadPoseEstimator:
    def __init__(self,model_path=ASSETS/'part-models.npz'):
        z=np.load(model_path);self.models={n:z[n+'_points'].copy()for n in PARTS};self.normals={n:z[n+'_normals'].copy()for n in PARTS};z.close();self.trees={n:cKDTree(p)for n,p in self.models.items()}

    def refine_surface(self,name,observed,r,p):
        # Independent samples are not point correspondences. Minimise distance
        # to the CAD tangent planes after coarse ICP, avoiding sample-density
        # bias on partially visible cylindrical surfaces.
        model,tree=self.models[name],self.trees[name]
        for _ in range(35):
            distance,index=tree.query((observed-p)@r)
            keep=distance<min(.006,max(.0015,float(np.quantile(distance,.85))))
            if keep.sum()<100:break
            vectors=model[index[keep]]@r.T
            normals=self.normals[name][index[keep]]@r.T
            residual=np.sum(normals*(observed[keep]-p-vectors),axis=1)
            weights=np.sqrt(np.minimum(1.,.0005/np.maximum(abs(residual),1e-12)))
            jac=np.c_[normals,np.cross(vectors,normals)]
            step=np.linalg.lstsq(jac*weights[:,None],residual*weights,rcond=1e-5)[0]
            step[:3]*=min(1.,.003/max(np.linalg.norm(step[:3]),1e-15))
            step[3:]*=min(1.,.025/max(np.linalg.norm(step[3:]),1e-15))
            p=p+step[:3];r=Rotation.from_rotvec(step[3:]).as_matrix()@r
            if np.linalg.norm(step[:3])+.03*np.linalg.norm(step[3:])<1e-8:break
        return r,p

    def fit(self,name,observed,prior,*,max_shift_m=(.035,.035,.005),max_tilt_rad=.2,rms_limit_m=.0015,min_inlier_fraction=.6):
        prior=transform(prior);observed=np.asarray(observed,float)
        if observed.ndim!=2 or observed.shape[1]!=3:raise ValueError('Points must be Nx3 metres')
        observed=observed[np.isfinite(observed).all(axis=1)]
        if len(observed)<150:raise RuntimeError('Insufficient visible CAD surface: '+name)
        if len(observed)>2200:observed=observed[np.linspace(0,len(observed)-1,2200).astype(int)]
        model=self.models[name];tree=self.trees[name];limit=np.asarray(max_shift_m,float)
        if limit.shape!=(3,) or np.any(limit<=0):raise ValueError('Invalid pickup displacement bounds')
        center=np.median(observed,axis=0)-prior[:3,:3]@np.median(model,axis=0)
        center=np.clip(center,prior[:3,3]-limit,prior[:3,3]+limit)
        starts=[prior[:3,3],center,.5*(center+prior[:3,3])]
        if name.startswith('ring'):
            # Partial annular faces can yield a low-residual local minimum
            # with the centre sliding several millimetres. Cover the local
            # translation basin instead of accepting the first ICP solution.
            starts.extend(np.clip(prior[:3,3]+[x,y,0.],prior[:3,3]-limit,prior[:3,3]+limit)
                          for x in [-.008,0.,.008] for y in [-.008,0.,.008] if x or y)
        results=[]
        for start in starts:
            r=prior[:3,:3].copy();p=np.array(start,copy=True)
            for iteration in range(70):
                local=(observed-p)@r;distance,index=tree.query(local)
                cutoff=min(.025 if iteration<10 else .008,max(.0015,float(np.quantile(distance,.8))))
                keep=distance<cutoff
                if keep.sum()<100:break
                weights=np.minimum(1.,.0015/np.maximum(distance[keep],1e-12))
                nr,np_=weighted_fit(model[index[keep]],observed[keep],weights)
                displacement=nr@r.T
                angle=Rotation.from_matrix(displacement).as_rotvec()
                if np.linalg.norm(angle)>.06:nr=Rotation.from_rotvec(angle*.06/np.linalg.norm(angle)).as_matrix()@r
                np_=np.clip(np_,p-.005,p+.005)
                change=np.linalg.norm(np_-p)+.03*np.linalg.norm(Rotation.from_matrix(nr@r.T).as_rotvec());r,p=nr,np_
                if change<1e-7:break
            r,p=self.refine_surface(name,observed,r,p)
            if name.startswith('ring'):
                # Axial clocking of a round ring is not observable/relevant to
                # the current grasp. Preserve it instead of inventing certainty.
                a=prior[:3,2];b=r[:,2]
                if a@b<0:b=-b
                cross=np.cross(a,b);s=np.linalg.norm(cross)
                delta=Rotation.from_rotvec(cross/max(s,1e-12)*np.arctan2(s,a@b)).as_matrix()
                r=delta@prior[:3,:3]
                # Preserve the centre found from surface normals; matching
                # individual tessellation samples would reintroduce bias.
            distance,_=tree.query((observed-p)@r);keep=distance<rms_limit_m*2
            fraction=float(np.mean(keep));rms=float(np.sqrt(np.mean(distance[keep]**2)))if keep.any()else float('inf')
            local=(observed[keep]-p)@r
            extent=np.ptp(local,axis=0)/np.maximum(np.ptp(model,axis=0),1e-9) if keep.any() else np.zeros(3)
            minimum_extent=np.array([.6,.6,.2]if name.startswith('ring')else[.3,.3,.45])
            tilt=Rotation.from_matrix(r@prior[:3,:3].T).magnitude()
            if np.any(abs(p-prior[:3,3])>limit+1e-8) or tilt>max_tilt_rad:continue
            if rms<=rms_limit_m and fraction>=min_inlier_fraction and keep.sum()>=150 and np.all(extent>=minimum_extent):
                t=np.eye(4);t[:3,:3]=r;t[:3,3]=p;results.append((rms/max(fraction,.01),PoseEstimate(name,t,rms,fraction,int(keep.sum()),.0005,'axis normal detected; axial angle uses prior'if name.startswith('ring')else'6D CAD registration near calibrated orientation prior',tuple(float(x)for x in extent))))
        if not results:raise RuntimeError('Pose fit failed quality/displacement/orientation gate: '+name)
        results.sort(key=lambda row:row[0]);return results[0][1]

    def observe(self,points_camera,T_cell_camera,priors,*,labels=None,roi_margin_m=.035):
        calibration=transform(T_cell_camera);points=np.asarray(points_camera,float)
        if points.ndim!=2 or points.shape[1]!=3:raise ValueError('Invalid camera point cloud')
        points=points@calibration[:3,:3].T+calibration[:3,3]
        if labels is not None and (np.asarray(labels).ndim!=1 or len(labels)!=len(points)):raise ValueError('Point labels must be a vector matching the cloud')
        estimates={}
        for label,name in enumerate(PARTS,1):
            prior=transform(priors[name]);expected=self.models[name]@prior[:3,:3].T+prior[:3,3]
            low=expected.min(axis=0)-[roi_margin_m,roi_margin_m,.006];high=expected.max(axis=0)+[roi_margin_m,roi_margin_m,.006]
            keep=np.all((points>=low)&(points<=high),axis=1)
            if labels is not None:keep&=np.asarray(labels)==label
            estimates[name]=self.fit(name,points[keep],prior)
        return estimates


def load_snapshot(path,config,*,replay=False,max_age_s=2.):
    """NPZ is a camera adapter boundary; depth and RGB must already align."""
    metadata=json.loads(Path(path).with_suffix('.json').read_text())
    if metadata['camera_frame']!=config['camera_frame'] or metadata['calibration_id']!=config['calibration_id']:raise RuntimeError('Camera/calibration frame mismatch')
    stamp=datetime.fromisoformat(metadata['captured_at_utc'].replace('Z','+00:00'))
    if stamp.tzinfo is None:raise ValueError('Capture timestamp needs timezone')
    age=(datetime.now(timezone.utc)-stamp).total_seconds()
    if not replay and not 0<=age<=max_age_s:raise RuntimeError('Camera frame stale or from the future')
    with np.load(path,allow_pickle=False)as z:
        labels=z['labels']if 'labels'in z else None
        if 'points_camera_m'in z:points=z['points_camera_m']
        else:points,labels=deproject(z['depth'],config['intrinsics'],config['depth_scale_m'],rectified=config['rectified_registered_depth'],labels=labels)
        if labels is not None:labels=np.asarray(labels).copy()
        points=np.asarray(points).copy()
    return points,labels,metadata
