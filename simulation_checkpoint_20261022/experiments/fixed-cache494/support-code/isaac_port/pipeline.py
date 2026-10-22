"""Durable container entry point: cached conversion -> PhysX validation -> status."""
from __future__ import annotations
import argparse
import datetime
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--scene',choices=['exchange','m80'],default='exchange')
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--steps',type=int,default=480)
parser.add_argument('--video',action='store_true')
parser.add_argument('--fresh',action='store_true')
args=parser.parse_args()
here=Path(__file__).resolve().parent
root=here/'generated'/args.scene
out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
status=out/'pipeline-status.json'
started=time.monotonic()
manifest={'scene':args.scene,'worker_pid':os.getpid(),'started_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),
          'supervisor':'isaac-run Docker container; pipeline is the container main process',
          'visible_gpu_uuids':os.environ.get('NVIDIA_VISIBLE_DEVICES'),
          'steps':args.steps,'video':args.video,'force_conversion':args.fresh,
          'output':str(out),'restart':'After a failure, inspect pipeline status and child log. Rerun run.sh for a new output directory; matching completed conversion is reused. No physical state is resumed from a failed run.'}

def update(**values):
    manifest.update(values,elapsed_seconds=time.monotonic()-started)
    temporary=status.with_suffix('.tmp');temporary.write_text(json.dumps(manifest,indent=2)+'\n');temporary.replace(status)


def child(script,arguments,log):
    with (out/log).open('w') as stream:
        worker=subprocess.Popen([sys.executable,'-u',str(here/script),*arguments],stdout=stream,stderr=subprocess.STDOUT)
        update(state='running',phase=script,child_pid=worker.pid,child_log=str(out/log))
        print(f'{script}: worker PID {worker.pid}; log {out/log}',flush=True)
        result=worker.wait()
    if result:
        raise RuntimeError(f'{script} exited with status {result}; inspect {log}')


try:
    with (root/'pipeline.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        source=json.loads((root/'source.json').read_text())
        update(state='running',phase='cache_check',source_sha256=source['source_sha256'],asset_digest=source['asset_digest'])
        if args.video and importlib.util.find_spec('imageio_ffmpeg') is None:
            # PIP_TARGET belongs to this workspace; keep the common image intact.
            update(state='running',phase='video_encoder_setup')
            subprocess.run(['/usr/local/bin/pip','install','--no-deps','imageio-ffmpeg==0.6.0'],check=True)
        cached=None
        try:
            cached=json.loads((root/'conversion-status.json').read_text())
        except (OSError,ValueError):pass
        valid=(cached and cached.get('state')=='passed'
               and cached.get('prepared_sha256')==source['prepared_sha256']
               and cached.get('asset_digest')==source['asset_digest']
               and (here/cached.get('usd_path','missing')).is_file())
        if args.fresh or not valid:
            child('convert.py',['--scene',args.scene],'conversion.log')
            cached=json.loads((root/'conversion-status.json').read_text())
            assert cached['state']=='passed',cached
            reused=False
        else:
            reused=True
        update(state='running',phase='physx_validation',conversion_reused=reused,usd_path=cached['usd_path'])
        options=['--scene',args.scene,'--steps',str(args.steps),'--output',str(out)]
        if args.scene=='m80':options.append('--probe-only')
        elif args.video:options.append('--video')
        child('simulate.py',options,'physics.log')
        result=json.loads((out/'status.json').read_text())
        assert result['state']=='passed',result
        update(state='passed',phase='complete',child_pid=None,physics_result=str(out/'status.json'),
               physical_assembly_pass=False,scope='Scene conversion, pose parity and PhysX control smoke test; no full manipulation task success claim')
        print('ISAAC_PORT_PIPELINE_PASS',status,flush=True)
except Exception as exc:
    traceback.print_exc();update(state='failed',error=repr(exc));sys.exit(1)
