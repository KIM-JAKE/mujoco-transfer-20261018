"""CPU-only replay of ALL recorded Isaac body poses, at wall-clock speed.

Renderer snapshots are not controller inputs or new physics trials. Loose
stock is replayed from the same trace as the robot, never from an arranged
replacement scene. Source pass/fail is retained in the title and manifest.
"""
from pathlib import Path
import argparse
import json
import subprocess
import sys

HERE=Path(__file__).resolve().parent
sys.path[:0]=[str(HERE/'bin-picking/realtime-deps-python312'),str(HERE)]
import numpy as np
from ray_camera_core import NumericRayCamera

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source',required=True)
parser.add_argument('--fps',type=int,default=20)
parser.add_argument('--part',choices=['ring1','ring2','pin','auto'],default='ring1')
args=parser.parse_args()
import re
assert re.fullmatch(r'native-roas-table-[a-zA-Z0-9_-]+',args.source)
source=HERE.parents[1]/'runs'/args.source
report=json.loads((source/'pipeline-results.json').read_text())
trace=np.load(source/'physx-sequence-trace.npz')
out=source/'cpu-replay';out.mkdir(exist_ok=True)
camera=NumericRayCamera(source/'camera-local-models.npz')
paths=trace['camera_body_paths'].tolist()
assert set(camera.geometry)<=set(paths)
times=trace['time_s']
bodies=trace['camera_bodies'];parts=trace['parts'];part_names=trace['part_names'].tolist();phases=trace['pin_phase']
end=float(report['wall_verified_elapsed_s']);fps=args.fps
switches=[dict(wall_s=0.,part='ring1')]
if args.part=='auto':
    observations=json.loads((source/'rgb-observations.json').read_text())
    # Follow the executed pickup order, including image-selected small-first
    # layouts and physical re-picks, rather than assume large then small.
    pickups=sorted((float(o['captured_wall_s']),o['part']) for o in observations
        if o.get('part') in ['ring1','ring2'] and o['mode']=='pregrasp_ring')
    if pickups:switches=[dict(wall_s=0.,part=pickups[0][1])]
    for captured,part in pickups:
        if switches[-1]['part']!=part:switches.append(dict(wall_s=captured,part=part))
    pin_steps=[i for i,p in enumerate(phases) if str(p).startswith(('pin_empty','pin_tail','pin_donor'))]
    if pin_steps:switches.append(dict(wall_s=float(times[pin_steps[0]]),part='pin'))
ffmpeg=HERE.parents[3]/'.isaac/lab3.0.0-rc1/packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2'
video=out/'recorded-physx-1x.mp4'
filters='pad=1024:520:0:40:color=0x111827'
command=[str(ffmpeg),'-y','-f','rawvideo','-pix_fmt','rgb24','-s','1024x480',
         '-r',str(fps),'-i','-','-an','-vf',filters,'-c:v','libx264','-preset',
         'fast','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(video)]
count=0;last_render_key=None;last_canvas_bytes=None
with (out/'encoding.log').open('wb') as log:
    writer=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=log,stderr=log)
    try:
        for wall in np.arange(max(1,int(np.ceil(end*fps))))/fps:
            k=max(0,min(len(times)-1,int(np.searchsorted(times,wall,side='right')-1)))
            active_part=next(s['part'] for s in reversed(switches) if wall>=s['wall_s']) if args.part=='auto' else args.part
            render_key=(k,active_part)
            if render_key==last_render_key:
                writer.stdin.write(last_canvas_bytes);count+=1
                continue
            poses={path:pose for path,pose in zip(paths,bodies[k])}
            global_rgb=camera.capture([-1.50,-1.85,1.95],[0,-.18,1.18],span=2.1,
                width=640,height=480,body_poses=poses,far_clip=4.5)['rgb']
            active_part=next(s['part'] for s in reversed(switches) if wall>=s['wall_s']) if args.part=='auto' else args.part
            target=parts[k,part_names.index(active_part),:3].astype(float)
            offset=np.array([-.32,-.35,-.08] if target[1]>.05 else
                            [-.20,-.18,.5] if target[1]<-.5 else [.22,-.30,.28])
            close_rgb=camera.capture(target+offset,target,span=.28,width=384,height=384,
                body_poses=poses,far_clip=2.)['rgb']
            canvas=np.full((480,1024,3),24,np.uint8);canvas[:,:640]=global_rgb
            canvas[48:432,640:]=close_rgb
            last_canvas_bytes=canvas.tobytes();last_render_key=render_key
            writer.stdin.write(last_canvas_bytes);count+=1
            if count%40==0:print('RECORDED_CPU_FRAME',count,round(wall,2),flush=True)
        writer.stdin.close()
        assert writer.wait()==0,'Video encoding failed; see encoding.log'
    except BaseException:
        writer.kill();writer.wait();raise
manifest=dict(source_run=args.source,source_full_task_pass=bool(report.get('full_task_pass')),
    source_under20=bool(report.get('end_to_end_under20')),physics='Recorded Isaac Sim CPU PhysX',
    renderer='CPU Embree ray camera of original Isaac CAD geometry',RTX=False,
    replay_only=True,new_physics_steps=0,speed_multiplier=1.,clock='Recorded monotonic wall clock including camera/planning waits',
    source_wall_end_s=end,video_duration_s=count/fps,frame_count=count,fps=fps,
    all_loose_stock_poses_replayed=True,all_recorded_dynamic_bodies=len(paths),
    evaluator_poses_used_for_rendering_only=True,closeup_part=args.part,closeup_switches=switches if args.part=='auto' else [])
(out/'replay-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('RECORDED_CPU_VIDEO',video,flush=True)
