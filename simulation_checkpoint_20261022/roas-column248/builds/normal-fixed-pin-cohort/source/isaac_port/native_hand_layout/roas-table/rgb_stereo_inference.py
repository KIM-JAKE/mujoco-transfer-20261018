"""SAM RGB segmentation + RGB stereo + CAD fitting; no simulator imports."""
import numpy as np,cv2
from rgb_stereo import reconstruct
from rgb_pose import axis_frame,project
from rgb_initial import seeds
from isaac_port.fieldkit.perception import CadPoseEstimator

class StereoInference:
    def __init__(self,detector):self.detector=detector;self.estimator=CadPoseEstimator()
    def infer(self,packets,request,work):
        import torch,json
        from PIL import Image
        from pathlib import Path
        work=Path(work);left,right=packets;points,disparity,stereo=reconstruct(left,right);rgb=left['rgb'];kind=request.get('part');mode=request['mode'];self.detector.prompts=('ring','metal ring')if kind!='pin'else('metal pin','bolt','metal cylinder')
        with torch.autocast('cpu',dtype=torch.bfloat16):masks=self.detector.segment(rgb)
        segment=dict(self.detector.last_report);segment.update(backend='SAM3.1 RGB only',crop_source='full RGB frame',depth_input=False,class_and_pose_source='SAM masks, RGB stereo triangulation, known CAD')
        np.savez_compressed(work/'stereo-reconstruction.npz',disparity_pixels=disparity,points_cell=points)
        np.savez_compressed(work/'sam-masks.npz',masks=np.array([m['mask']for m in masks])if masks else np.empty((0,*rgb.shape[:2]),bool),scores=[m['score']for m in masks])
        overlay=rgb.copy()
        for i,m in enumerate(masks):overlay[m['mask']]=(overlay[m['mask']]*.55+np.array([250,90,35])*.45).astype(np.uint8)
        Image.fromarray(overlay).save(work/'sam-overlay.png')
        if mode=='initial':
            cal=request['calibration'];normal=np.array(cal['basket_to_world_R'])[:,2];ss=seeds(left,masks,cal['floor_origin_world_m'],normal);ss.sort(key=lambda s:s['center'][0]);chosen=[]
            for s in ss:
                if not any(np.linalg.norm(s['center']-c['center'])<.04 for c in chosen):chosen.append(s)
                if len(chosen)==2:break
            if len(chosen)<2:raise RuntimeError('SAM did not detect two accessible RGB rings')
            poses=[]
            for s in chosen:
                D=s['measured_diameter_m'];part='ring1'if abs(D-.075)<abs(D-.062)else'ring2';pose=self.fit(part,points,s['mask'],s['center'],normal)
                pose.update(measured_classification_diameter_m=D,size_classification='RGB ellipse scale using calibrated basket plane; color unused');poses.append(pose)
            if {p['part']for p in poses}!={'ring1','ring2'}:raise RuntimeError('RGB size classifier did not find one 75 mm and one 62 mm ring')
            result=dict(poses=poses)
        else:
            seed=np.array(request['seed'],float);axis=np.array(request['axis'],float)
            if kind=='pin':
                # Pin may be too small for text-only SAM. Neutral-metal RGB
                # foreground and the encoder-predicted grasp region supply a
                # classical RGB fallback; triangulation still supplies metric pose.
                gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);mask=((rgb.max(2)-rgb.min(2))<45)&(gray>35)&(gray<180)
                # Region selection is in image-derived 3D, never simulator depth.
                mask&=np.linalg.norm(points-seed,axis=2)<.035
            else:
                uv=project(seed[None,:],left)[0];choices=[]
                for m in masks:
                    v,u=np.nonzero(m['mask'])
                    if len(u)<60:continue
                    choices.append((np.linalg.norm(np.array([u.mean(),v.mean()])-uv),m))
                if not choices:raise RuntimeError('No SAM RGB mask for '+kind)
                mask=min(choices,key=lambda a:a[0])[1]['mask']
            result=dict(pose=self.fit(kind,points,mask,seed,axis,pin_initial=mode=='pin_initial'))
        result.update(stereo=stereo,segmentation=segment,depth_sensor_input=False,simulator_body_pose_input=False)
        return result
    def fit(self,part,points,mask,seed,axis,pin_initial=False):
        seed=np.array(seed,float);axis=np.array(axis,float);axis/=np.linalg.norm(axis);valid=mask&np.isfinite(points).all(2)
        valid&=np.linalg.norm(points-seed,axis=2)<(.065 if part!='pin'else .035)
        pts=points[valid]
        if len(pts)<150:raise RuntimeError('Insufficient RGB stereo points on '+part+': '+str(len(pts)))
        prior=np.eye(4);prior[:3,:3]=axis_frame(axis);prior[:3,3]=seed
        # Principal direction gives a second RGB-derived orientation seed for
        # an elongated pin. Axial ring yaw remains unobservable.
        priors=[prior]
        if part=='pin':
            c=np.median(pts,axis=0);_,_,V=np.linalg.svd(pts-c,full_matrices=False);n=V[0];n*=1 if n@axis>=0 else -1;other=prior.copy();other[:3,:3]=axis_frame(n);priors.append(other)
        candidates=[]
        for p in priors:
            try:
                fit=self.estimator.fit(part,pts,p,max_shift_m=(.025,.025,.025),max_tilt_rad=1.2 if part=='pin'else .35,rms_limit_m=.002,min_inlier_fraction=.5)
                candidates.append(fit)
            except RuntimeError:continue
        if not candidates:raise RuntimeError('RGB stereo CAD pose fit rejected '+part)
        fit=min(candidates,key=lambda f:f.rms_m/max(f.inlier_fraction,.01));T=fit.T_cell_part
        return dict(part=part,center_m=T[:3,3].tolist(),axis=T[:3,2].tolist(),stereo_points=len(pts),rms_m=fit.rms_m,inlier_fraction=fit.inlier_fraction,uses_depth_sensor=False,uses_simulator_body_poses=False,orientation_scope=fit.orientation_scope)
