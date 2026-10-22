"""Persistent RealSense RGB-D acquisition with bounded frame age.

Call start once, then read repeatedly; initialization/exposure warmup is not
repeated per observation. No robot actions. Measured T_cell_camera is required.
SDK reference: librealsense wrappers/python/examples/align-depth2color.py
"""
from pathlib import Path
import argparse,json,time,numpy as np

class RealSenseStream:
    def __init__(self,calibration,serial=None,width=640,height=480,fps=30):
        self.T=np.array(json.load(open(calibration))['T_cell_camera'],float)
        if self.T.shape!=(4,4)or not np.isfinite(self.T).all()or not np.allclose(self.T[3],[0,0,0,1]):raise ValueError('Measured metric T_cell_camera required')
        if not np.allclose(self.T[:3,:3].T@self.T[:3,:3],np.eye(3),atol=1e-5)or np.linalg.det(self.T[:3,:3])<.999:raise ValueError('Camera calibration rotation is invalid')
        self.serial=serial;self.width=width;self.height=height;self.fps=fps;self.pipe=None
    def start(self):
        if self.pipe is not None:return self
        import pyrealsense2 as rs
        self.rs=rs;pipe=rs.pipeline();cfg=rs.config()
        if self.serial:cfg.enable_device(self.serial)
        cfg.enable_stream(rs.stream.depth,self.width,self.height,rs.format.z16,self.fps);cfg.enable_stream(rs.stream.color,self.width,self.height,rs.format.rgb8,self.fps)
        profile=pipe.start(cfg);self.pipe=pipe;self.profile=profile;self.align=rs.align(rs.stream.color);self.pc=rs.pointcloud()
        try:
            # Keep a single newest frame in the SDK queue when supported.
            for sensor in profile.get_device().query_sensors():
                if sensor.supports(rs.option.frames_queue_size):sensor.set_option(rs.option.frames_queue_size,1)
            self.scale=profile.get_device().first_depth_sensor().get_depth_scale()
            for _ in range(30):pipe.wait_for_frames(10000)
        except Exception:self.close();raise
        return self
    def read(self,max_acquisition_s=.25):
        if self.pipe is None:raise RuntimeError('Start the camera once before reading')
        started=time.monotonic();frames=self.pipe.wait_for_frames(1000)
        # Drop queued old frames rather than allowing a growing perception queue.
        for _ in range(8):
            newer=self.pipe.poll_for_frames()
            if not newer:break
            frames=newer
        frames=self.align.process(frames);depth=frames.get_depth_frame();color=frames.get_color_frame()
        if not depth or not color:raise RuntimeError('Aligned RGB/depth frame unavailable')
        rgb=np.asanyarray(color.get_data()).copy();d=np.asanyarray(depth.get_data()).astype(np.float32)*self.scale
        intr=color.profile.as_video_stream_profile().intrinsics;K=np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1.]])
        vertices=self.pc.calculate(depth);xyz=np.asanyarray(vertices.get_vertices()).view(np.float32).reshape(*d.shape,3).copy();invalid=d<=0;d[invalid]=np.nan;xyz[invalid]=np.nan
        elapsed=time.monotonic()-started
        if elapsed>max_acquisition_s:raise RuntimeError('RealSense acquisition exceeded latency budget; reject this frame')
        device=self.profile.get_device();report=dict(source='RealSense librealsense',camera_model=device.get_info(self.rs.camera_info.name),serial=device.get_info(self.rs.camera_info.serial_number),depth_scale_m=float(self.scale),aligned_to='RGB optical frame',point_cloud_source='SDK deprojection of aligned depth',RGB_distortion_model=str(intr.model),RGB_distortion_coefficients=list(intr.coeffs),rectified=False,SDK_intrinsics=K.tolist(),T_cell_camera=self.T.tolist(),hardware_timestamp_ms=float(color.get_timestamp()),frame_number=int(color.get_frame_number()),received_monotonic_s=time.monotonic(),acquisition_ms=elapsed*1000,valid_depth_fraction=float(np.isfinite(d).mean()),robot_connected=False)
        return dict(rgb=rgb,depth_m=d,points_camera_m=xyz,K=K,T_cell_camera=self.T.copy()),report
    def close(self):
        if self.pipe is not None:self.pipe.stop();self.pipe=None
    def __enter__(self):return self.start()
    def __exit__(self,*args):self.close()

def capture(output,calibration,serial=None,width=640,height=480,fps=30):
    with RealSenseStream(calibration,serial,width,height,fps)as stream:packet,report=stream.read()
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True);np.savez_compressed(output,**packet);output.with_suffix('.json').write_text(json.dumps(report,indent=2));return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--calibration',type=Path,required=True);p.add_argument('--serial');p.add_argument('--width',type=int,default=640);p.add_argument('--height',type=int,default=480);p.add_argument('--fps',type=int,default=30);a=p.parse_args();print(json.dumps(capture(a.output,a.calibration,a.serial,a.width,a.height,a.fps),indent=2))
