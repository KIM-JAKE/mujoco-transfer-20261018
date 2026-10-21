"""Isolated CPU camera + RGB-D perception process; avoids the control GIL.

Body poses are private camera image-formation inputs. Detection/tracking sees
only RGB/depth/calibration, never this render snapshot or geometry IDs.
"""
from pathlib import Path
import os
# Respect a small private CPU budget before Embree/TBB initializes.
os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[-4:])
import sys,time,numpy as np,traceback
from multiprocessing.connection import Connection
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE/'bin-picking/realtime-deps-python312'),str(HERE),str(HERE.parents[2])]
from ray_camera_core import NumericRayCamera
from rgbd_initial_fast import detect
from rgbd_fast_tracker import FastRGBDTracker
from isaac_port.fieldkit.perception import CadPoseEstimator
conn=Connection(int(sys.argv[1]));started=time.perf_counter();camera=NumericRayCamera(sys.argv[2]);estimator=CadPoseEstimator();tracker=FastRGBDTracker(estimator);conn.send(dict(ready=True,preparation_s=time.perf_counter()-started))
while True:
    request=conn.recv()
    if request is None:break
    packet=None
    try:
        render_start=time.perf_counter();packet=camera.capture(request['eye'],request['target'],request['span'],request['width'],request.get('height',request['width']),body_poses=request['render_snapshot']);captured=time.perf_counter()
        if request['mode']=='held_left':
            # Empty-fixture calibration render. This depends solely on the
            # fixed CAD BVH and camera calibration, never free-body transforms.
            baseline=camera.capture(request['eye'],request['target'],request['span'],request['width'],request.get('height',request['width']),body_poses={},static_only=True)
            packet['fixture_depth_m']=baseline['depth_m'];captured=time.perf_counter()
        if request['mode']=='initial':
            if request['calibration'].get('loose_bin_mode'):
                from rgbd_scattered_rings import detect as scatter_detect
                result,masks=scatter_detect(packet,dict(request['calibration'],global_plane_faces=True),estimator)
            else:result,masks=detect(packet,request['calibration'],estimator)
            mask=np.logical_or.reduce(masks)
        elif request['mode']=='floor':
            try:
                found,masks=detect(packet,request['calibration'],estimator,desired_part=request['part'],seed=request['seed']);pose=found['poses'][0];mask=masks[0]
            except RuntimeError:pose,mask=tracker.estimate(packet,request['part'],request['seed'],request['axis'])
            result=dict(pose=pose,depth_sensor_input=True,simulator_body_pose_input=False)
        elif request['part'] in ['ring1','ring2'] and request['mode'] in ['held','held_left','pregrasp_ring']:
            from rgbd_scattered_rings import detect as relocalize
            seed=np.asarray(request['seed']);local_cal=dict(floor_origin_world_m=seed-[0,0,.06],basket_to_world_R=np.eye(3),outer_bounds_world_m=[seed-.065,seed+.065],allow_sideways_faces=True)
            found,masks=relocalize(packet,local_cal,estimator,desired_part=request['part'],seed=seed)
            if found['poses']:pose=found['poses'][0];mask=masks[0];pose['method']='Fresh RGB-D annular face relocalization after lift/transfer, arbitrary axis'
            else:
                if request.get('require_annular_face'):raise RuntimeError('No measurable annular face: change camera view before transfer')
                pose,mask=tracker.estimate(packet,request['part'],request['seed'],request['axis'])
            from rgbd_ring_translation import refine
            pose=refine(packet,pose,estimator)
            result=dict(pose=pose,depth_sensor_input=True,simulator_body_pose_input=False)
        else:pose,mask=tracker.estimate(packet,request['part'],request['seed'],request['axis'],initial_pin=request['mode']=='pin_initial');result=dict(pose=pose,depth_sensor_input=True,simulator_body_pose_input=False)
        done=time.perf_counter();conn.send(dict(id=request['id'],result=result,packet=packet,mask=mask,H=request['H'],captured_wall_s=request['snapshot_monotonic']-request['cycle_started'],ready_wall_s=done-request['cycle_started'],capture_s=captured-render_start,processing_s=done-captured,queue_wait_s=render_start-request['snapshot_monotonic'],camera_profile=dict(camera.last_profile)))
    except Exception:
        value=dict(id=request['id'],result=dict(error=traceback.format_exc()))
        if packet is not None:value.update(packet=packet,mask=np.zeros(packet['rgb'].shape[:2],bool),H=request['H'],captured_wall_s=request['snapshot_monotonic']-request['cycle_started'],ready_wall_s=time.perf_counter()-request['cycle_started'],capture_s=captured-render_start,processing_s=time.perf_counter()-captured)
        conn.send(value)
conn.close()
