"""Asynchronous fresh camera perception; image formation + detection are timed."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import time,numpy as np,json
from rgbd_geometry import axis_frame
from rgbd_initial_fast import detect
from rgbd_fast_tracker import FastRGBDTracker
from isaac_port.fieldkit.perception import CadPoseEstimator
from isaac_kinematics import frame

class Vision:
    def __init__(self,camera,calibration):
        self.camera=camera;self.calibration=calibration;self.estimator=CadPoseEstimator();self.tracker=FastRGBDTracker(self.estimator);self.pool=ThreadPoolExecutor(max_workers=1);self.jobs=[];self.start=None
    def request(self,event,H):
        mode=event['mode'];part=event['part'];H=H.copy();snapshot_started=time.perf_counter();poses=self.camera.snapshot()
        if mode in ['initial','floor']:target=np.array([.51,-.70,1.1]);eye=[.75,-1.15,1.6];span=.28;width=256
        elif mode=='pin_initial':target=np.array([.19,-.79,.916]);eye=[.19,-.79,1.35];span=.08;width=256
        else:target=H[:3,3];eye=target+(np.array([-.25,.12,-.2])if part!='pin'else np.array([0,0,.30]));span=.16 if part!='pin'else .08;width=256
        if mode=='floor':seed=np.array(event['reference']['pose']['center_m']);axis=np.array(self.calibration['basket_to_world_R'])[:,2]
        elif mode=='pin_initial':seed=np.array([.19,-.79,.916]);axis=np.array([0,0,-1.])
        elif mode=='held':axis=-H[:3,2]if part!='pin'else H[:3,2];axis*=1 if part=='pin'or axis[2]>=0 else -1;seed=target-axis*.003 if part!='pin'else target
        else:seed=axis=None
        def job():
            render_start=time.perf_counter();packet=self.camera.capture(eye,target,span,width,width,body_poses=poses);capture_done=time.perf_counter()
            try:
                if mode=='initial':result,masks=detect(packet,self.calibration,self.estimator);mask=np.logical_or.reduce(masks)
                else:pose,mask=self.tracker.estimate(packet,part,seed,axis,initial_pin=mode=='pin_initial');result=dict(pose=pose,depth_sensor_input=True,simulator_body_pose_input=False)
            except Exception:
                import traceback;result=dict(error=traceback.format_exc());mask=np.zeros(packet['rgb'].shape[:2],bool)
            done=time.perf_counter();return dict(result=result,packet=packet,mask=mask,H=H,captured_wall_s=snapshot_started-self.start,ready_wall_s=done-self.start,capture_s=capture_done-render_start,processing_s=done-capture_done,queue_wait_s=render_start-snapshot_started,camera_profile=dict(self.camera.last_profile))
        print('REALTIME_SNAPSHOT',mode,part,time.perf_counter()-snapshot_started,flush=True);future=self.pool.submit(job);self.jobs.append((event,future));return future
    def save(self,out):
        self.pool.shutdown(wait=True)
        root=Path(out)/'rgb-perception';root.mkdir(exist_ok=True);records=[]
        for i,(event,future)in enumerate(self.jobs):
            if not future.done():continue
            if future.exception():
                records.append(dict(mode=event['mode'],part=event['part'],error=repr(future.exception())));continue
            value=future.result();work=root/f'observation-{i+1:03d}-{event["mode"]}';work.mkdir(exist_ok=True);np.savez_compressed(work/'rgbd.npz',**value['packet']);overlay=value['packet']['rgb'].copy();overlay[value['mask']]=(overlay[value['mask']]*.5+[25,120,60]).clip(0,255).astype(np.uint8);imagefile=work/('detection-overlay.ppm'if event['mode']=='initial'else'tracking-overlay.ppm');imagefile.write_bytes(f'P6\n{overlay.shape[1]} {overlay.shape[0]}\n255\n'.encode()+overlay.tobytes());small={k:v for k,v in value.items()if k not in ['packet','mask','H']};small.update(mode=event['mode'],part=event['part'],work=str(work),scene_sim_time_s=value['captured_wall_s']);records.append(small)
        (Path(out)/'rgb-observations.json').write_text(json.dumps(records,indent=2));self.pool.shutdown(wait=True)
