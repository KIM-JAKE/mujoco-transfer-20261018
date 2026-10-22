"""Persistent CPU RGB-D tracking demo; publishes poses, never robot commands.

An initial class and metric seed are supplied by initialization/detection.
For a moving held object, --encoder-seed-json may provide updated camera ROI
priors from robot FK. That file is robot state, never simulator object state.
"""
from pathlib import Path
import argparse,json,sys,time,numpy as np
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE.parents[2])]
from capture_realsense import RealSenseStream
from rgbd_fast_tracker import FastRGBDTracker
from isaac_port.fieldkit.perception import CadPoseEstimator

def run(a):
    tracker=FastRGBDTracker(CadPoseEstimator());seed=np.array(a.seed);axis=np.array(a.axis)
    with RealSenseStream(a.calibration,a.serial,a.width,a.height,a.fps)as stream:
        for _ in range(a.frames):
            try:
                cycle_started=time.monotonic();packet,metadata=stream.read(max_acquisition_s=.15);received=metadata['received_monotonic_s']
                if a.encoder_seed_json:
                    prior=json.load(open(a.encoder_seed_json));seed=np.array(prior['seed_m']);axis=np.array(prior['axis'])
                    if time.time()-float(prior['timestamp_unix_s'])>.2:raise RuntimeError('Encoder seed older than 200 ms')
                pose,_=tracker.estimate(packet,a.part,seed,axis,initial_pin=a.pin_in_holder)
                age=time.monotonic()-received
                total_age=time.monotonic()-cycle_started
                if total_age>.2:raise RuntimeError('RGB-D acquisition + tracking exceeded 200 ms; discard')
                seed=np.array(pose['center_m']);axis=np.array(pose['axis']);print(json.dumps(dict(state='tracked',pose=pose,acquisition_ms=metadata['acquisition_ms'],frame_number=metadata['frame_number'],processing_age_ms=age*1000,acquisition_and_tracking_ms=total_age*1000,robot_command_sent=False)),flush=True)
            except RuntimeError as error:print(json.dumps(dict(state='observation_rejected',reason=str(error),robot_command_sent=False)),flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--calibration',type=Path,required=True);p.add_argument('--part',choices=['ring1','ring2','pin'],required=True);p.add_argument('--seed',type=float,nargs=3,required=True);p.add_argument('--axis',type=float,nargs=3,default=[0,0,1]);p.add_argument('--encoder-seed-json',type=Path);p.add_argument('--pin-in-holder',action='store_true');p.add_argument('--serial');p.add_argument('--width',type=int,default=640);p.add_argument('--height',type=int,default=480);p.add_argument('--fps',type=int,default=30);p.add_argument('--frames',type=int,default=1000);run(p.parse_args())
