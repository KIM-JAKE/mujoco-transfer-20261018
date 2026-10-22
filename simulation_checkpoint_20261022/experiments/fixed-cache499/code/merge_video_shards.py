"""Merge independent CPU render batches, keeping physical timestamp order."""
from pathlib import Path
import json,sys,shutil
HERE=Path(__file__).resolve().parent;RUNS=HERE.parents[1]/'runs'
name=sys.argv[1];count=int(sys.argv[2])if len(sys.argv)>2 else 4
out=RUNS/name;out.mkdir(exist_ok=True);rows=[];sources=[]
for i in range(count):
    root=RUNS/f'{name}-s{i}of{count}';meta=json.load(open(root/'video-frames.json'));sources.append(meta)
    rows.extend((row['sim_time_s'],root/row['file'],row)for row in meta['frames'])
assert len({m['source_run']for m in sources})==1
frames=[]
for j,(_,path,row)in enumerate(sorted(rows,key=lambda item:item[0])):
    file=f'frame_{j:04d}.ppm';shutil.copyfile(path,out/file);frames.append(dict(row,file=file))
assert all(b['sim_time_s']>a['sim_time_s']for a,b in zip(frames,frames[1:]))
meta=dict(sources[0],frames=frames,CPU_render_workers=count,total_worker_CPU_quota=count*2)
(out/'video-frames.json').write_text(json.dumps(meta,indent=2));print('MERGED',len(frames),'frames')
