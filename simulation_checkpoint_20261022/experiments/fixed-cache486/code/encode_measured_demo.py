"""Encode CPU views at their recorded physical timestamps, without speedup."""
from pathlib import Path
import json,sys,subprocess,numpy as np
import imageio_ffmpeg
from PIL import Image,ImageDraw,ImageFont
HERE=Path(__file__).resolve().parent;RUNS=HERE.parents[1]/'runs';out=RUNS/sys.argv[1]
manifest=json.load(open(out/'video-frames.json'));source=RUNS/manifest['source_run'];report=json.load(open(source/'pipeline-results.json'))
assert report.get('full_task_pass') and report['cycle_time_s']<30, 'Require measured full-task success below 30 seconds'
trace=np.load(source/'physx-sequence-trace.npz');goals=json.load(open(source/'arm-plan.json'))['goals'];fontroot=Path('/usr/share/fonts/truetype/dejavu');font=lambda size:ImageFont.truetype(str(fontroot/'DejaVuSans.ttf'),size)
records=manifest['frames'];rendered=[]
for i,row in enumerate(records):
 raw=Image.open(out/row['file']).convert('RGB');w,h=raw.size;scale=3;canvas=Image.new('RGB',(w*scale,h*scale+140),(13,19,29));canvas.paste(raw.resize((w*scale,h*scale),Image.Resampling.LANCZOS),(0,82));d=ImageDraw.Draw(canvas)
 d.text((20,9),'ISAAC SIM  |  FR3 DUO + NATIVE HAND  |  1x',font=font(25),fill=(230,236,244));t=row['sim_time_s'];done=t>=report['cycle_time_s'];d.text((20,44),f'{t:05.2f} s   '+(f'COMPLETE at {report["cycle_time_s"]:.2f} s'if done else'Picking / handover / insertion'),font=font(22),fill=(94,231,170)if done else(194,206,224))
 k=int(np.argmin(abs(trace['time_s']-t)));x=20;y=h*scale+94
 for j,p in enumerate(trace['part_names']):
  tr=trace['parts'][k,j];err=float(np.linalg.norm(tr[:3]-goals[p]['P']));ax=float(np.arccos(np.clip(1-2*(tr[3]**2+tr[4]**2),-1,1)));good=err<.0015 and ax<.05;color=(94,231,170)if good else(156,169,187);label={'ring1':'LARGE RING','ring2':'SMALL RING','pin':'PIN'}[p]
  d.ellipse((x,y+5,x+12,y+17),fill=color);d.text((x+20,y),label+(' AT TARGET'if good else' IN PROGRESS'),font=font(18),fill=color);x+=w*scale//3
 file=out/f'video_{i:04d}.png';canvas.save(file);rendered.append(file)
concat=out/'frames-1x.txt'
with concat.open('w')as f:
 for i,file in enumerate(rendered):
  f.write("file '"+file.name+"'\n")
  if i+1<len(rendered):f.write('duration %.9f\n'%(records[i+1]['sim_time_s']-records[i]['sim_time_s']))
 f.write("file '"+rendered[-1].name+"'\n")
video=out/'three-parts-under30-1x.mp4';subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-y','-hide_banner','-loglevel','error','-f','concat','-safe','0','-i',str(concat),'-vf','fps=30','-c:v','libx264','-preset','fast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(video)],check=True)
meta=dict(source_run=manifest['source_run'],physical_full_task_pass=True,completion_s=report['cycle_time_s'],verification_end_s=report['verified_elapsed_s'],playback_speed=1,frames_are_actual_physics_states=True,renderer=manifest['renderer'],RTX=False,ideal_pose_input=True,hardware_tested=False,background_unselected_rings='settled starting poses; only selected parts recorded continuously',retention_fixture_model='finite-force surrogate clamp; not calibrated hardware',video=str(video));(out/'video-evidence.json').write_text(json.dumps(meta,indent=2));print(json.dumps(meta,indent=2))
