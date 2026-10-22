"""Metric reconstruction exclusively from two calibrated RGB images."""
import numpy as np,cv2

def reconstruct(left,right):
    assert set(left)==set(right)=={'rgb','K','T_cell_camera'}, 'RGB-only packet required'
    K=left['K'];L=left['T_cell_camera'];R=right['T_cell_camera'];assert np.max(abs(L[:3,:3]-R[:3,:3]))<1e-6 and np.max(abs(K-right['K']))<1e-6
    baseline=(R[:3,3]-L[:3,3])@L[:3,0];assert baseline>0
    gray=[cv2.cvtColor(p['rgb'],cv2.COLOR_RGB2GRAY)for p in [left,right]];width=gray[0].shape[1]
    matcher=cv2.StereoSGBM_create(minDisparity=0,numDisparities=min(192,width//2//16*16),blockSize=5,P1=8*25,P2=32*25,disp12MaxDiff=2,preFilterCap=31,uniquenessRatio=1,speckleWindowSize=0,speckleRange=2,mode=cv2.STEREO_SGBM_MODE_HH)
    disparity=matcher.compute(*gray).astype(float)/16;valid=disparity>2;z=K[0,0]*baseline/np.maximum(disparity,.01);valid&=(z>.05)&(z<2.)
    v,u=np.indices(z.shape);cam=np.stack([(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z],-1);world=cam@L[:3,:3].T+L[:3,3]
    world[~valid]=np.nan
    return world,disparity,dict(algorithm='OpenCV calibrated RGB stereo SGBM',baseline_m=float(baseline),sensor_depth_input=False,ground_truth_disparity_input=False,valid_pixels=int(valid.sum()),total_pixels=valid.size,metric_depth_is_estimated_from_RGB=True)
