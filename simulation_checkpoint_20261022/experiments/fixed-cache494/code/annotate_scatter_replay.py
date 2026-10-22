"""Annotate an existing scientific replay; never edit its object/robot poses."""
from pathlib import Path
import argparse,json,subprocess
import numpy as np
from PIL import Image,ImageDraw,ImageFont

HERE=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source',required=True)
parser.add_argument('--backend',choices=['embree_cpu','warp_cuda'],default='embree_cpu')
args=parser.parse_args()
import re
assert re.fullmatch(r'native-roas-table-[a-zA-Z0-9_-]+',args.source)
root=HERE.parents[1]/'runs'/args.source;out=root/('gpu-replay'if args.backend=='warp_cuda'else'cpu-replay')
report=json.loads((root/'pipeline-results.json').read_text());manifest=json.loads((out/'replay-manifest.json').read_text())
ffmpeg=HERE.parents[3]/'.isaac/lab3.0.0-rc1/packages/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2'
fontpath=HERE.parents[1]/'assets/NotoSansKR.ttf';font=ImageFont.truetype(str(fontpath),18);small=ImageFont.truetype(str(fontpath),14)
for f in [font,small]:
    try:f.set_variation_by_name('Regular')
    except (ValueError,OSError):pass
title=('ALL 3 PARTS PASSED | UNDER 20 s' if report.get('end_to_end_under20') else 'ALL 3 PARTS PASSED | OVER 20 s') if report.get('full_task_pass') else 'ONE RING ONLY passed | 3-part task NOT completed' if report.get('one_ring_component_accepted') else 'PARTIAL | Full task FAILED'
width,height,fps=1024,520,manifest['fps'];video=out/'recorded-physx-1x-captioned.mp4'
trace=np.load(root/'physx-sequence-trace.npz')
trace_times=trace['time_s'];trace_phases=trace['pin_phase']
decoder=subprocess.Popen([str(ffmpeg),'-v','error','-i',str(out/'recorded-physx-1x.mp4'),'-f','rawvideo','-pix_fmt','rgb24','-'],stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
with (out/'caption-encoding.log').open('wb') as log:
    encoder=subprocess.Popen([str(ffmpeg),'-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{width}x{height}','-r',str(fps),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(video)],stdin=subprocess.PIPE,stdout=log,stderr=log)
    count=0
    while True:
        raw=decoder.stdout.read(width*height*3)
        if not raw:break
        if len(raw)!=width*height*3:raise RuntimeError('Truncated replay frame')
        frame=Image.fromarray(np.frombuffer(raw,np.uint8).reshape(height,width,3));draw=ImageDraw.Draw(frame)
        draw.rectangle((0,0,width,39),fill=(17,24,39));draw.text((12,7),title,font=font,fill='white');draw.text((790,9),f'{count/fps:.2f} s | '+('GPU ray'if args.backend=='warp_cuda'else'CPU')+' 1x',font=small,fill='white')
        # The close-up always looks at the selected recorded ring center. This
        # projected-location marker is rendering-only, never a sensor cue.
        x,y=832,280
        for a,b in [((x-14,y),(x-5,y)),((x+5,y),(x+14,y)),((x,y-14),(x,y-5)),((x,y+5),(x,y+14))]:draw.line((a,b),fill=(255,211,75),width=2)
        active_part=manifest.get('closeup_part','ring1')
        if active_part=='auto':active_part=next(s['part'] for s in reversed(manifest['closeup_switches']) if count/fps>=s['wall_s'])
        part_label={'ring1':'Large ring','ring2':'Small ring','pin':'Pin'}[active_part]
        draw.text((650,486),part_label+' position (replay overlay)',font=small,fill=(255,211,75))
        at=max(0,min(len(trace_times)-1,int(np.searchsorted(trace_times,count/fps,side='right')-1)))
        stage=str(trace_phases[at]).replace('_',' ')
        draw.rectangle((0,495,638,519),fill=(17,24,39));draw.text((10,497),stage,font=small,fill='white')
        if count in [0,manifest['frame_count']//2,manifest['frame_count']-1]:frame.save(out/('first.png' if count==0 else 'last.png' if count==manifest['frame_count']-1 else 'middle.png'))
        encoder.stdin.write(np.asarray(frame).tobytes());count+=1
    encoder.stdin.close();assert encoder.wait()==0;assert decoder.wait()==0
assert count==manifest['frame_count']
manifest.update(display_video=video.name,display_scope=title,annotation='Text and projected-location reticle overlays only; no object geometry, poses, materials or playback timing changed',controller_RGBD_uses_annotation=False)
(out/'replay-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(video)
