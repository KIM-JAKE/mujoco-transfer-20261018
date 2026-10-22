"""Supervise cached MJCF conversion and the immutable button-cycle replay."""
import argparse,fcntl,json,os,subprocess,sys,time,traceback
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
here=Path(__file__).resolve().parent;root=here/'generated/cap_button';out=a.output.resolve();out.mkdir(parents=True,exist_ok=True)
started=time.monotonic();state=dict(worker_pid=os.getpid(),visible_gpu_uuids=os.environ.get('NVIDIA_VISIBLE_DEVICES'),mode='kinematic_recording_replay')
def save(**fields):
    state.update(fields,seconds=time.monotonic()-started)
    tmp=out/'pipeline-status.tmp';tmp.write_text(json.dumps(state,indent=2)+'\n');tmp.replace(out/'pipeline-status.json')
def child(script,options,log):
    with (out/log).open('w') as stream:
        proc=subprocess.Popen([sys.executable,'-u',str(here/script),*options],stdout=stream,stderr=subprocess.STDOUT)
        save(state='running',phase=script,child_pid=proc.pid,child_log=log)
        if proc.wait():raise RuntimeError(f'{script} failed; inspect {log}')
try:
    with (root/'pipeline.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        source=json.loads((root/'source.json').read_text())
        try:cache=json.loads((root/'conversion-status.json').read_text())
        except (OSError,ValueError):cache={}
        cached=(cache.get('state')=='passed' and cache.get('prepared_sha256')==source['prepared_sha256']
                and cache.get('asset_digest')==source['asset_digest'] and (here/cache.get('usd_path','missing')).is_file())
        if not cached:child('convert.py',['--scene','cap_button'],'conversion.log')
        save(conversion_reused=bool(cached))
        child('replay_button.py',['--output',str(out)],'replay.log')
        assert json.loads((out/'status.json').read_text())['state']=='passed'
        save(state='passed',phase='complete',child_pid=None,physical_assembly_pass=False)
except Exception as exc:
    traceback.print_exc();save(state='failed',error=repr(exc));sys.exit(1)
