"""Encode actual wall timestamps, including every camera/vision/control wait.

Offline generation of the replay images is separate from the timed operation.
The observation panel changes only after a live estimate reached the controller.
"""
from pathlib import Path
import json,sys,subprocess,numpy as np
import imageio_ffmpeg
from PIL import Image,ImageDraw,ImageFont
HERE=Path(__file__).resolve().parent;RUNS=HERE.parents[1]/'runs';out=RUNS/sys.argv[1];manifest=json.load(open(out/'video-frames.json'));source=RUNS/manifest['source_run'];report=json.load(open(source/'pipeline-results.json'))
partial_probe=report.get('scattered_pick',{}).get('state')=='physical_handover_camera_confirmed'
passed=bool(report.get('vision_end_to_end_pass')and report.get('full_task_pass'));assert passed or '--allow-failed'in sys.argv,'Failed trials require explicit --allow-failed labeling'
under_budget=bool(report.get('end_to_end_under30')and report.get('wall_verified_elapsed_s',1e9)<30)
assert under_budget or '--allow-overbudget'in sys.argv or(not passed and '--allow-failed'in sys.argv),'Over-budget previews require an explicit flag'
assert manifest['frames'][0]['sim_time_s']<.1 and manifest['frames'][-1]['sim_time_s']>=report.get('verified_elapsed_s',report['wall_verified_elapsed_s'])-.05, 'Require the complete physical timeline, including retention'
observations=json.load(open(source/'rgb-observations.json'));fontroot=Path('/usr/share/fonts/truetype/dejavu');font=lambda size:ImageFont.truetype(str(fontroot/'DejaVuSans.ttf'),size);rendered=[];records=manifest['frames']
for i,row in enumerate(records):
    cached=out/f'rgbd_video_{i:04d}.png'
    if '--reuse-images'in sys.argv and cached.exists() and cached.stat().st_mtime>=(out/row['file']).stat().st_mtime:rendered.append(cached);continue
    raw=Image.open(out/row['file']).convert('RGB');w,h=raw.size;scale=2;W=w*scale+330;H=h*scale+146;canvas=Image.new('RGB',(W,H),(13,19,29));canvas.paste(raw.resize((w*scale,h*scale),Image.Resampling.LANCZOS),(0,82));d=ImageDraw.Draw(canvas);t=row['sim_time_s'];done=bool(passed and t>=report['cycle_time_s']);failed_end=bool(not passed and t>=report['wall_verified_elapsed_s']-.05)
    d.text((16,8),('ISAAC SIM | FR3 DUO + NATIVE HAND | FULL WALL CLOCK'if passed else'ISAAC SIM | PICKUP + HANDOVER ONLY | FULL TASK INCOMPLETE' if partial_probe else'ISAAC SIM | RANDOMIZED TRIAL | FAILED / INCOMPLETE'),font=font(22),fill=(230,236,244));d.text((16,42),f'{t:05.2f} s  '+(f'COMPLETE at {report["cycle_time_s"]:.2f} s'if done else(('PROBE END - insertion not completed' if partial_probe else 'FAILED - no verified completion')if failed_end else'Picking / handover / insertion'))+' | 1x wall-clock replay',font=font(21),fill=(94,231,170)if done else(255,125,125)if not passed else(194,206,224))
    valid=[v for v in observations if (v.get('delivered_wall_s')or v['ready_wall_s'])<=t];obs=valid[-1]if valid else None
    if obs is None:
        x=w*scale+12;d.text((x,110),'Acquiring first RGB-D frame...',font=font(16),fill=(170,194,220));file=out/f'rgbd_video_{i:04d}.png';canvas.save(file,compress_level=1);rendered.append(file);continue
    work=source/'rgb-perception'/Path(obs['work']).name;picture=next((work/name for name in (['detection-overlay.ppm','detection-overlay.png','sam-overlay.png']if obs['mode']=='initial'else['tracking-overlay.ppm','tracking-overlay.png'])if(work/name).exists()),work/'missing.png');x=w*scale+12
    d.text((x,92),'LAST RGB-D OBSERVATION',font=font(17),fill=(230,236,244))
    if picture.exists():canvas.paste(Image.open(picture).convert('RGB').resize((306,306)),(x,122))
    result=obs['result'];ms=result.get('pose',{}).get('processing_ms');d.text((x,440),f'{obs["mode"]} / {obs["part"]or"size detection"}',font=font(16),fill=(170,194,220));d.text((x,465),f'Captured at {obs["scene_sim_time_s"]:.2f} s',font=font(16),fill=(170,194,220))
    if obs['mode']=='initial':d.text((x,490),'Size classes: 75 mm / 62 mm',font=font(16),fill=(94,231,170))
    if ms is not None:d.text((x,490),f'Capture+vision: {((obs.get("delivered_wall_s")or obs["ready_wall_s"])-obs["captured_wall_s"])*1000:.0f} ms',font=font(16),fill=(94,231,170))
    d.text((16,H-48),'Actual PhysX trace | camera pixels + depth for control | no live workpiece poses',font=font(17),fill=(170,194,220));d.text((16,H-24),'Capture + fresh RGB-D detection + corrections + motion + retention included. Ideal depth; hardware untested.',font=font(15),fill=(170,194,220));file=out/f'rgbd_video_{i:04d}.png';canvas.save(file,compress_level=1);rendered.append(file)
concat=out/'rgbd-frames-1x.txt'
with concat.open('w')as f:
    for i,file in enumerate(rendered):
        f.write("file '"+file.name+"'\n")
        if i+1<len(rendered):f.write('duration %.9f\n'%(records[i+1]['sim_time_s']-records[i]['sim_time_s']))
    f.write("file '"+rendered[-1].name+"'\n")
video=out/('rgbd-pick-handover-only-1x.mp4' if partial_probe else 'rgbd-full-wall-FAILED-1x.mp4'if not passed else'rgbd-full-wall-under30-1x.mp4'if under_budget else'rgbd-full-wall-1x.mp4');subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-y','-hide_banner','-loglevel','error','-f','concat','-safe','0','-i',str(concat),'-vf','fps=30,pad=ceil(iw/2)*2:ceil(ih/2)*2','-c:v','libx264','-preset','fast','-threads','2','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(video)],check=True)
meta=dict(source_run=manifest['source_run'],physical_full_task_pass=passed,vision_end_to_end_pass=passed,failed_attempt_visualization=not passed,completion_s=report.get('cycle_time_s'),verification_end_s=report.get('verified_elapsed_s',report['wall_verified_elapsed_s']),playback_speed=1,offline_CPU_capture_pauses_omitted=False,end_to_end_under30=under_budget,timing_basis=report['timing_basis'],perception_observations=report.get('perception_observations',len(observations)),perception_wall_s_included=True,wall_verified_elapsed_s=report['wall_verified_elapsed_s'],frames_are_actual_physics_states=True,renderer=manifest['renderer'],RTX=False,ideal_pose_input=False,ideal_synthetic_depth=True,hardware_tested=False,video=str(video));(out/'full-wall-video-evidence.json').write_text(json.dumps(meta,indent=2));print(json.dumps(meta,indent=2))
