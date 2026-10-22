"""Keep the CPU Isaac runtime loaded between fresh isolated physical trials."""
from pathlib import Path
import os,sys,json,time,runpy,traceback,importlib,gc,hashlib,re
from pxr import UsdUtils
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE/'bin-picking/realtime-deps-python312'),str(HERE),str(HERE.parents[1]),str(HERE.parents[2])];ROOT=HERE/'bin-picking/realtime-jobs';ROOT.mkdir(exist_ok=True);os.environ['M80_REALTIME_KEEPALIVE']='1';print('REALTIME_SERVICE_READY',flush=True)
while not(ROOT/'STOP').exists():
    pending=[p for p in sorted(ROOT.glob('*.json'))if not p.with_suffix('.done').exists()and not p.with_suffix('.running').exists()]
    if not pending:time.sleep(.05);continue
    file=pending[0];request=json.loads(file.read_text());label=request['run'];assert re.fullmatch(r'native-roas-table-[a-zA-Z0-9_-]+',label);out=HERE.parents[1]/'runs'/label;out.mkdir(exist_ok=True);file.with_suffix('.running').write_text(label);os.environ['M80_ROAS_RUN']=label
    try:
        for name in ['rgbd_geometry','rgbd_pin_axes','rgbd_fast_tracker','rgbd_initial_fast','rgbd_batch_sensor','realtime_recipe','realtime_vision']:
            if name in sys.modules:importlib.reload(sys.modules[name])
        snapshot=out/'code_snapshot';snapshot.mkdir(exist_ok=True);manifest={}
        for source in HERE.glob('*.py'):
            data=source.read_bytes();(snapshot/source.name).write_bytes(data);manifest[source.name]=hashlib.sha256(data).hexdigest()
        (out/'code-manifest.json').write_text(json.dumps(manifest,indent=2));print('REALTIME_JOB_START',label,flush=True);script=request.get('script','run_realtime_recipe.py');assert script in ['run_realtime_recipe.py','run_pin_tail_contact.py'];namespace=runpy.run_path(str(HERE/script))
        if 'stage'in namespace:UsdUtils.StageCache.Get().Erase(namespace['stage'])
        del namespace;gc.collect();file.with_suffix('.done').write_text('finished');print('REALTIME_JOB_DONE',label,flush=True)
    except Exception:
        (out/'daemon-error.txt').write_text(traceback.format_exc());file.with_suffix('.done').write_text('error');print(traceback.format_exc(),flush=True)
import omni.kit.app;omni.kit.app.get_app().post_quit(0)
