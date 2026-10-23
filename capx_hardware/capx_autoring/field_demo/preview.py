"""Standalone RGB/depth sampling preview. Pixel XYZ is measured camera data only."""
import argparse
import base64
import html
import json
from pathlib import Path
import struct
import zlib


def png_bytes(pixels):
    """Encode RGB/RGBA uint8 using the PNG specification; no imaging dependency."""
    import numpy as np
    a = np.asarray(pixels)
    if a.dtype != np.uint8 or a.ndim != 3 or a.shape[2] not in (3,4) or min(a.shape[:2]) < 1:
        raise ValueError('PNG requires nonempty HxWx3/4 uint8 image')
    h,w,channels = a.shape
    def chunk(kind,data):
        return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data)&0xffffffff)
    scanlines = b''.join(b'\x00'+a[y].tobytes() for y in range(h))
    return (b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',w,h,8,2 if channels==3 else 6,0,0,0))+
            chunk(b'IDAT',zlib.compress(scanlines))+chunk(b'IEND',b''))


def pixel_xyz(points, x, y):
    """Return only the actual finite SDK XYZ. Zero/invalid depth is not a point."""
    import numpy as np
    a = np.asarray(points)
    if a.ndim != 3 or a.shape[2] != 3:
        raise ValueError('Per-pixel SDK points must be HxWx3')
    if type(x) is not int or type(y) is not int or not 0 <= x < a.shape[1] or not 0 <= y < a.shape[0]:
        raise ValueError('Pixel outside image')
    p = a[y,x]
    if not np.isfinite(p).all() or not .05 < float(p[2]) < 5:
        raise ValueError('No valid measured depth at this pixel; choose another visible surface')
    return [float(v) for v in p]


def render_preview(snapshot, output):
    import numpy as np
    snapshot, output = Path(snapshot),Path(output)
    if output.exists():
        raise FileExistsError('Preview output exists; use a new path')
    metadata_path = snapshot.with_suffix('.json')
    metadata = json.loads(metadata_path.read_text()) if metadata_path.is_file() else {}
    with np.load(snapshot,allow_pickle=False) as z:
        if 'rgb' not in z:
            raise ValueError('Snapshot has no RGB image; capture a new camera sample')
        rgb = np.asarray(z['rgb'])
        encoded = base64.b64encode(png_bytes(rgb)).decode('ascii')
        points = np.asarray(z['points_camera_image_m']) if 'points_camera_image_m' in z else None
    h,w = rgb.shape[:2]
    if points is not None and points.shape != (h,w,3):
        raise ValueError('Per-pixel SDK points and RGB image shapes do not match')
    limitation = ('Click the image to read the SDK camera-frame XYZ in metres. '
                  'Independently measure the matching cell-frame point. A click does not calibrate the camera or detect a part.')
    payload = dict(width=w,height=h,points_base64=None,camera_frame=metadata.get('camera_frame','unrecorded'),
                   capture_id=metadata.get('capture_id','unrecorded'),camera_serial=metadata.get('camera_serial','unrecorded'))
    overlay_encoded, depth_range = '', 'Unavailable'
    if points is not None:
        points = np.asarray(points,dtype='<f4')
        payload['points_base64'] = base64.b64encode(points.tobytes()).decode('ascii')
        valid = np.isfinite(points).all(2)&(points[:,:,2]>.05)&(points[:,:,2]<5)
        overlay = np.zeros((h,w,4),dtype=np.uint8)
        if valid.any():
            lo,hi = float(points[:,:,2][valid].min()),float(points[:,:,2][valid].max())
            t = np.clip((np.where(valid,points[:,:,2],lo)-lo)/max(hi-lo,1e-6),0,1)
            overlay[:,:,0] = (255*(1-t)).astype(np.uint8)
            overlay[:,:,2] = (255*t).astype(np.uint8)
            overlay[:,:,1] = (128*(1-abs(2*t-1))).astype(np.uint8)
            overlay[:,:,3] = np.where(valid,140,0).astype(np.uint8)
            depth_range = f'{lo:.4f} to {hi:.4f} m camera optical Z; red near, blue far'
        else:
            depth_range = 'No valid measured depth in this frame'
        overlay_encoded = base64.b64encode(png_bytes(overlay)).decode('ascii')
    else:
        limitation = ('This snapshot contains only a flat point cloud. RGB is shown, but pixel XYZ cannot be recovered. '
                      'Capture a new camera sample with per-pixel SDK points. No coordinate will be invented.')
    safe_payload = json.dumps(payload,ensure_ascii=True).replace('<','\\u003c').replace('>','\\u003e').replace('&','\\u0026')
    title = html.escape(snapshot.name)
    document = '''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Camera calibration sampling</title>
<style>body{font:16px system-ui;max-width:1100px;margin:24px auto;padding:0 16px;background:#111827;color:#eef2ff}
button,textarea{font:inherit}button{padding:8px 12px}textarea{width:100%;box-sizing:border-box;min-height:160px;background:#fff;color:#111}
#stage{position:relative;max-width:100%;display:inline-block;border:1px solid #64748b;line-height:0;cursor:crosshair}
#rgb{display:block;max-width:100%;height:auto}#depth{position:absolute;inset:0;width:100%;height:100%;pointer-events:none}
#result{padding:12px;background:#1e293b;white-space:pre-wrap}label{display:block;margin:12px 0}.muted{color:#cbd5e1}</style>
<h1>Camera points for calibration</h1><p>SNAPSHOT</p><p>LIMITATION</p>
<p class="muted" id="identity"></p><label><input id="toggle" type="checkbox" checked>Measured depth overlay</label>
<p class="muted">DEPTH_RANGE</p><div id="stage"><img id="rgb" alt="Captured RGB" src="data:image/png;base64,RGB_DATA"><img id="depth" alt="Measured depth overlay" src="data:image/png;base64,DEPTH_DATA"></div>
<p id="result">Select a visible, stationary calibration point. Invalid depth is rejected.</p>
<button id="add" disabled>Add camera point</button> <button id="copy">Copy points_camera_m JSON</button>
<p>Only camera-frame points appear below. Measure the corresponding cell coordinates independently in the same point order; use those measurements with the calibration fit command. No robot commands are available here.</p>
<textarea id="points" readonly>{"units":"m","points_camera_m":[]}</textarea>
<script id="payload" type="application/json">PAYLOAD</script><script>
const cfg=JSON.parse(document.getElementById('payload').textContent), img=document.getElementById('rgb');
const result=document.getElementById('result'), add=document.getElementById('add'), textarea=document.getElementById('points');
const chosen=[];let selected=null, cloud=null;
if(cfg.points_base64){const bytes=Uint8Array.from(atob(cfg.points_base64),c=>c.charCodeAt(0));cloud=new DataView(bytes.buffer);}
document.getElementById('identity').textContent='Camera: '+cfg.camera_serial+' | frame: '+cfg.camera_frame+' | capture: '+cfg.capture_id;
document.getElementById('toggle').disabled=!cloud;
if(!cloud)document.getElementById('depth').style.display='none';
document.getElementById('toggle').addEventListener('change',e=>document.getElementById('depth').style.display=e.target.checked?'block':'none');
img.addEventListener('click',e=>{
 selected=null;add.disabled=true;
 if(!cloud){result.textContent='Pixel XYZ unavailable: flat cloud has no pixel correspondence. Capture a new sample.';return;}
 const rect=img.getBoundingClientRect(),x=Math.floor((e.clientX-rect.left)*cfg.width/rect.width),y=Math.floor((e.clientY-rect.top)*cfg.height/rect.height);
 if(x<0||y<0||x>=cfg.width||y>=cfg.height){result.textContent='Pixel outside image.';return;}
 const offset=(y*cfg.width+x)*12,p=[0,1,2].map(i=>cloud.getFloat32(offset+4*i,true));
 if(!p.every(Number.isFinite)||p[2]<=0.05||p[2]>=5){result.textContent='No valid measured depth at pixel '+x+', '+y+'. Choose another surface.';return;}
 selected=p;add.disabled=false;result.textContent='Pixel '+x+', '+y+' | camera XYZ m: '+JSON.stringify(p)+'\\nThis is a measured surface point, not the object center or robot TCP.';
});
add.addEventListener('click',()=>{if(!selected)return;chosen.push(selected.slice());textarea.value=JSON.stringify({units:'m',points_camera_m:chosen},null,2);selected=null;add.disabled=true;});
document.getElementById('copy').addEventListener('click',async()=>{textarea.focus();textarea.select();try{await navigator.clipboard.writeText(textarea.value);}catch(e){document.execCommand('copy');}});
</script></html>'''
    # Replace tokens once, keeping user strings inside escaped text/JSON.
    tokens = {'SNAPSHOT':title,'LIMITATION':html.escape(limitation),'DEPTH_RANGE':html.escape(depth_range),
              'RGB_DATA':encoded,'DEPTH_DATA':overlay_encoded,'PAYLOAD':safe_payload}
    import re
    document = re.sub('|'.join(tokens),lambda match:tokens[match.group(0)],document)
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(document)
    return dict(state='camera_preview_created',output=str(output.resolve()),pixel_xyz_available=points is not None,
                hardware_ready=False,robot_command_sent=False,limitation=limitation)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--snapshot',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(argv)
    try:
        print(json.dumps(render_preview(a.snapshot,a.output)))
        return 0
    except (ValueError,KeyError,OSError,ImportError) as e:
        print(json.dumps(dict(state='preview_rejected',reason=str(e),robot_command_sent=False)))
        return 2


if __name__=='__main__':
    raise SystemExit(main())
