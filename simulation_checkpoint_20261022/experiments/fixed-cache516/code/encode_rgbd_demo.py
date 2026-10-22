"""Encode measured physical states at 1x with the actual perception snapshots.

CPU frame-generation delays are not reproduced in the motion replay, and are
reported explicitly. The vision panel shows the last observation, not a live
continuous stream. Completion requires the fresh physical report to pass.
"""
from pathlib import Path
import json,sys,subprocess,numpy as np
import imageio_ffmpeg
from PIL import Image,ImageDraw,ImageFont
HERE=Path(__file__).resolve().parent;RUNS=HERE.parents[1]/'runs';out=RUNS/sys.argv[1];manifest=json.load(open(out/'video-frames.json'));source=RUNS/manifest['source_run'];report=json.load(open(source/'pipeline-results.json'))
assert report.get('vision_end_to_end_pass')and report.get('full_task_pass'),'Require measured image-driven physical success'
assert manifest['frames'][0]['sim_time_s']<.1 and manifest['frames'][-1]['sim_time_s']>=report['verified_elapsed_s']-.05, 'Require the complete physical timeline, including retention'
observations=json.load(open(source/'rgb-observations.json'));fontroot=Path('/usr/share/fonts/truetype/dejavu');font=lambda size:ImageFont.truetype(str(fontroot/'DejaVuSans.ttf'),size);rendered=[];records=manifest['frames']
for i,row in enumerate(records):
    raw=Image.open(out/row['file']).convert('RGB');w,h=raw.size;scale=3;W=w*scale+330;H=h*scale+146;canvas=Image.new('RGB',(W,H),(13,19,29));canvas.paste(raw.resize((w*scale,h*scale),Image.Resampling.LANCZOS),(0,82));d=ImageDraw.Draw(canvas);t=row['sim_time_s'];done=t>=report['cycle_time_s']
    d.text((16,8),'ISAAC SIM | FR3 DUO + NATIVE HAND | RGB-D CONTROL',font=font(22),fill=(230,236,244));d.text((16,42),f'{t:05.2f} s  '+(f'COMPLETE at {report["cycle_time_s"]:.2f} s'if done else'Picking / handover / insertion')+' | 1x physics motion',font=font(21),fill=(94,231,170)if done else(194,206,224))
    valid=[v for v in observations if v['scene_sim_time_s']<=t];obs=valid[-1]if valid else observations[0]
    if t<1.5:obs=observations[0]
    elif t<3 and len(observations)>1:obs=observations[1]
    work=source/'rgb-perception'/Path(obs['work']).name;picture=work/('sam-overlay.png'if obs['mode']=='initial'else'tracking-overlay.png');x=w*scale+12
    d.text((x,92),'LAST RGB-D OBSERVATION',font=font(17),fill=(230,236,244))
    if picture.exists():canvas.paste(Image.open(picture).convert('RGB').resize((306,306)),(x,122))
    result=obs['result'];ms=result.get('pose',{}).get('processing_ms');d.text((x,440),f'{obs["mode"]} / {obs["part"]or"size detection"}',font=font(16),fill=(170,194,220));d.text((x,465),f'Captured at {obs["scene_sim_time_s"]:.2f} s',font=font(16),fill=(170,194,220))
    if obs['mode']=='initial':d.text((x,490),'Size classes: 75 mm / 62 mm',font=font(16),fill=(94,231,170))
    if ms is not None:d.text((x,490),f'CPU tracking: {ms:.1f} ms',font=font(16),fill=(94,231,170))
    d.text((16,H-48),'Actual PhysX trace | camera pixels + depth for control | no live workpiece poses',font=font(17),fill=(170,194,220));d.text((16,H-24),'Offline CPU rendering pauses omitted from motion replay. Ideal simulated depth; hardware untested.',font=font(15),fill=(170,194,220));file=out/f'rgbd_video_{i:04d}.png';canvas.save(file);rendered.append(file)
concat=out/'rgbd-frames-1x.txt'
with concat.open('w')as f:
    for i,file in enumerate(rendered):
        f.write("file '"+file.name+"'\n")
        if i+1<len(rendered):f.write('duration %.9f\n'%(records[i+1]['sim_time_s']-records[i]['sim_time_s']))
    f.write("file '"+rendered[-1].name+"'\n")
video=out/'rgbd-three-parts-1x.mp4';subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-y','-hide_banner','-loglevel','error','-f','concat','-safe','0','-i',str(concat),'-vf','fps=30,pad=ceil(iw/2)*2:ceil(ih/2)*2','-c:v','libx264','-preset','fast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(video)],check=True)
meta=dict(source_run=manifest['source_run'],physical_full_task_pass=True,vision_end_to_end_pass=True,completion_s=report['cycle_time_s'],verification_end_s=report['verified_elapsed_s'],playback_speed=1,offline_CPU_capture_pauses_omitted=True,perception_observations=report['perception_observations'],perception_wall_s=report['perception_wall_s'],frames_are_actual_physics_states=True,renderer=manifest['renderer'],RTX=False,ideal_pose_input=False,ideal_synthetic_depth=True,hardware_tested=False,video=str(video));(out/'rgbd-video-evidence.json').write_text(json.dumps(meta,indent=2));print(json.dumps(meta,indent=2))
