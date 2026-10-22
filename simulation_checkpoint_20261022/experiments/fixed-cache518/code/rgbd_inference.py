"""One-time SAM detection followed by low-latency RGB-D tracking; no simulator API."""
import numpy as np,cv2
from rgbd_geometry import cloud
from rgb_pose import axis_frame,project
from rgb_initial import seeds
from isaac_port.fieldkit.perception import CadPoseEstimator

class RGBDInference:
    def __init__(self,detector):
        from rgbd_fast_tracker import FastRGBDTracker
        self.detector=detector;self.estimator=CadPoseEstimator();self.tracker=FastRGBDTracker(self.estimator)
    def infer(self,packets,request,work):
        import torch,json
        from PIL import Image
        from pathlib import Path
        work=Path(work);left=packets[0]
        if request['mode']!='initial':
            pose,mask=self.tracker.estimate(left,request['part'],request['seed'],request['axis'],initial_pin=request['mode']=='pin_initial')
            overlay=left['rgb'].copy();overlay[mask]=(overlay[mask]*.5+np.array([50,220,120])*.5).astype(np.uint8);Image.fromarray(overlay).save(work/'tracking-overlay.png')
            return dict(pose=pose,segmentation=dict(backend='RGB-D ROI tracking after one-time SAM initialization',neural_mask_used=False),depth_sensor_input=True,simulator_body_pose_input=False)
        points=cloud(left);stereo=dict(algorithm='Aligned RGB-D backprojection',sensor_depth_input=True,simulator_body_pose_input=False,ideal_simulated_depth=True);rgb=left['rgb'];kind=request.get('part');mode=request['mode'];self.detector.prompts=('ring','metal ring')if kind!='pin'else('metal pin','bolt','metal cylinder')
        if kind=='pin':
            masks=[];segment=dict(backend='RGB neutral-metal foreground + depth axial face',depth_input=True,neural_mask_used=False,simulator_body_pose_input=False)
        else:
            import hashlib
            key=hashlib.sha256(rgb.tobytes()+str(rgb.shape).encode()).hexdigest();cache=Path(__file__).resolve().parent/'bin-picking/sam-mask-cache';cache.mkdir(exist_ok=True);cachefile=cache/(key+'.npz')
            if cachefile.exists():
                with np.load(cachefile)as z:masks=[dict(mask=m,score=float(score))for m,score in zip(z['masks'],z['scores'])]
                segment=dict(backend='SAM3.1 identical-RGB mask cache',identical_RGB_sha256=key,reused_identical_RGB_masks=True,cache_uses_depth=False)
            else:
                with torch.autocast('cpu',dtype=torch.bfloat16):masks=self.detector.segment(rgb)
                np.savez_compressed(cachefile,masks=np.array([m['mask']for m in masks]),scores=[m['score']for m in masks]);segment=dict(self.detector.last_report);segment.update(identical_RGB_sha256=key,reused_identical_RGB_masks=False)
            segment.update(backend='SAM3.1 RGB segmentation + aligned depth',crop_source='full RGB frame',depth_input=True,class_and_pose_source='SAM masks, aligned depth projection, known CAD')
        np.savez_compressed(work/'image-pointcloud.npz',points_cell=points)
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
                valid=s['mask']&np.isfinite(points).all(2);pts=points[valid];local=(pts-s['center'])@axis_frame(normal);D=float(max(np.percentile(local[:,:2],99.5,axis=0)-np.percentile(local[:,:2],.5,axis=0)));part='ring1'if abs(D-.075)<abs(D-.062)else'ring2';pose=self.fit(part,points,s['mask'],s['center'],normal)
                pose.update(measured_classification_diameter_m=D,size_classification='Metric diameter from SAM-masked depth points; color unused');poses.append(pose)
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
            if kind=='pin':
                overlay[mask]=(overlay[mask]*.5+np.array([50,220,120])*.5).astype(np.uint8);Image.fromarray(overlay).save(work/'sam-overlay.png')
            result=dict(pose=self.fit(kind,points,mask,seed,axis,pin_initial=mode=='pin_initial'))
        result.update(stereo=stereo,segmentation=segment,depth_sensor_input=True,simulator_body_pose_input=False)
        return result
    def fit(self,part,points,mask,seed,axis,pin_initial=False):
        seed=np.array(seed,float);axis=np.array(axis,float);axis/=np.linalg.norm(axis);valid=mask&np.isfinite(points).all(2)
        valid&=np.linalg.norm(points-seed,axis=2)<(.065 if part!='pin'else .035)
        pts=points[valid]
        if len(pts)<150:raise RuntimeError('Insufficient RGB stereo points on '+part+': '+str(len(pts)))
        if part=='pin':
            from rgbd_pin_axes import fit_pin
            return fit_pin(pts,seed,axis,pin_initial)
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
        return dict(part=part,center_m=T[:3,3].tolist(),axis=T[:3,2].tolist(),stereo_points=len(pts),rms_m=fit.rms_m,inlier_fraction=fit.inlier_fraction,uses_depth_sensor=True,uses_simulator_body_poses=False,orientation_scope=fit.orientation_scope)
