"""Measure CPU image-only tracking latency on captured Isaac RGB-D frames.

No simulator modules or ground-truth workpiece transforms are loaded. This is
an algorithm benchmark, not a claim about CPU rendering/physical camera speed.
"""
import argparse,json,time,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE),str(HERE/'bin-picking/perception-deps-python311'),str(HERE.parents[2])]
from isaac_port.fieldkit.perception import CadPoseEstimator
from rgbd_fast_tracker import FastRGBDTracker
CASES=[('initial-basket1','ring1',[.4912,-.7255,1.0924],[-.087,0,.996],False),('initial-basket1','ring2',[.4913,-.6455,1.0916],[-.087,0,.996],False),('ring1-held0','ring1',[-.05,-.1498,1.3998],[.086,-.065,.994],False),('ring2-held0','ring2',[-.15,-.3001,1.4],[.0777,-.0459,.9959],False),('initial-pin','pin',[.19,-.79,.916],[0,0,-1],True),('pin-held0','pin',[-.1,-.2496,1.3499],[0,.086,.996],False)]
def benchmark(root,repeats):
    tracker=FastRGBDTracker(CadPoseEstimator());records=[]
    for frame,part,seed,axis,initial in CASES:
        with np.load(root/(frame+'.npz'))as z:packet={k:z[k].copy()for k in z.files}
        elapsed=[]
        for _ in range(repeats):
            start=time.perf_counter();result,_=tracker.estimate(packet,part,seed,axis,initial);elapsed.append((time.perf_counter()-start)*1000)
        records.append(dict(frame=frame,part=part,repeats=repeats,median_ms=float(np.median(elapsed)),p95_ms=float(np.percentile(elapsed,95)),max_ms=max(elapsed),last_pose=result))
    timings=np.array([r['median_ms']for r in records]);return dict(input=str(root),device='CPU',real_camera_tested=False,simulator_poses_used=False,SAM_per_frame=False,includes_camera_acquisition=False,includes_simulator_render=False,includes_one_time_SAM_initialization=False,repeated_saved_frames=True,frame_shapes=[512,512],median_case_latency_ms=float(np.median(timings)),worst_case_median_ms=float(timings.max()),algorithm_only_equivalent_hz=float(1000/timings.max()),cases=records)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input',type=Path,default=HERE.parents[1]/'runs/native-roas-table-rgbd-sam-probe01');p.add_argument('--output',type=Path,default=HERE/'bin-picking/rgbd-tracking-latency.json');p.add_argument('--repeats',type=int,default=30);a=p.parse_args();result=benchmark(a.input,a.repeats);a.output.write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items()if k!='cases'},indent=2))
