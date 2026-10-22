"""Combine opaque overview and X-ray details from identical measured frames."""
from pathlib import Path
import argparse,json,hashlib,math
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();run=a.run.resolve();out=run/'render';out.mkdir(exist_ok=True)
import imageio.v2 as imageio
import numpy as np
from PIL import Image,ImageDraw,ImageFont
normal=run/'render-normal';xray=run/'render-xray';reports=[json.loads((d/'status.json').read_text())for d in [normal,xray]]
for key in ['physics_tape_sha256','acceptance_sha256','decoded_frames','fps','first_physics_time_s','last_physics_time_s','cycle_only']:assert reports[0][key]==reports[1][key],key
assert all(r['state']=='passed'and r['max_pose_error_m']<1e-8 for r in reports)
readers=[imageio.get_reader(str(d/'m80-physx-contact.mp4'))for d in [normal,xray]];partial=out/'video.partial.mp4';fps=reports[0]['fps'];writer=imageio.get_writer(str(partial),fps=fps,codec='libx264',quality=8,macro_block_size=2,ffmpeg_params=['-movflags','+faststart','-threads','4'])
font=ImageFont.truetype(str(Path(__file__).resolve().parent/'assets/NotoSansKR.ttf'),16);samples=[];checkpoints=set(int(v)for v in np.linspace(0,reports[0]['decoded_frames']-1,9));count=0
try:
 for first,second in zip(*readers):
  assert first.shape==second.shape==(720,1344,3)
  frame=first.copy();frame[48:688,896:]=second[48:688,896:];canvas=Image.fromarray(frame);draw=ImageDraw.Draw(canvas);draw.text((910,13),'X-RAY DETAILS',font=font,fill='white');writer.append_data(np.asarray(canvas))
  if count in checkpoints:
   canvas.save(out/f'frame-{count:04d}.png');samples.append(canvas.resize((448,240)))
  count+=1
finally:
 writer.close()
 for reader in readers:reader.close()
assert count==reports[0]['decoded_frames'];partial.replace(out/'m80-physx-contact.mp4');reader=imageio.get_reader(str(out/'m80-physx-contact.mp4'));decoded=sum(1 for frame in reader);reader.close();assert decoded==count
sheet=Image.new('RGB',(1344,math.ceil(len(samples)/3)*240))
for i,frame in enumerate(samples):sheet.paste(frame,((i%3)*448,(i//3)*240))
sheet.save(out/'contact-sheet.png');report=reports[0]|dict(mode='measured_physx_state_render_with_xray_details',decoded_frames=decoded,video_sha256=hashlib.sha256((out/'m80-physx-contact.mp4').read_bytes()).hexdigest(),input_render_reports=[str(d/'status.json')for d in [normal,xray]])
(out/'status.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
