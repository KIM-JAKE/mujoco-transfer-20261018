"""Automatic two-ring policy. Default commands never connect or move devices."""
import argparse,json
from pathlib import Path
from .config import initialize,pin,preflight

def main(argv=None):
 p=argparse.ArgumentParser(description=__doc__);s=p.add_subparsers(dest='command',required=True)
 q=s.add_parser('init');q.add_argument('directory');q.add_argument('--hardware',default='hardware.json')
 q=s.add_parser('pin-files');q.add_argument('--config',required=True)
 for name in ['preflight','run']:
  q=s.add_parser(name);q.add_argument('--config',required=True)
  if name=='run':q.add_argument('--output');q.add_argument('--execute',action='store_true')
 q=s.add_parser('self-test');q.add_argument('--output',required=True)
 q=s.add_parser('replay');q.add_argument('--recipe',required=True);q.add_argument('--manifest',required=True);q.add_argument('--output',required=True)
 a=p.parse_args(argv)
 try:
  if a.command=='init':result={'created':str(initialize(a.directory,a.hardware)),'device_connected':False,'motion_issued':False}
  elif a.command=='pin-files':result={'pinned':pin(a.config),'authorization_changed':False,'device_connected':False,'motion_issued':False}
  elif a.command=='preflight'or(a.command=='run'and not a.execute):
   result=preflight(a.config)[0];print(json.dumps(result,indent=2));return 0 if result['local_config_complete']else 2
  elif a.command=='self-test':
   from .offline import integration_check
   result=integration_check(a.output)
  elif a.command=='replay':
   from .runner import PacketReplayRuntime,RingSession,run_runtime
   from .field_demo.random_cycle import validate_recipe
   from hardware_v1.config import sha
   recipe_path=Path(a.recipe).resolve();raw=json.loads(recipe_path.read_text());site=json.loads((recipe_path.parent/raw['site_config']).read_text());recipe=validate_recipe(recipe_path,site)
   if raw.get('replay_only')is not True:raise ValueError('Explicit replay_only recipe required; replay cannot enable hardware')
   output=Path(a.output).resolve();output.mkdir(parents=True,exist_ok=False);session=RingSession(domain='recorded_pixel_replay',source_sha256=__import__('capx_autoring.config',fromlist=['engine_identity']).engine_identity(),config_sha256=sha(recipe_path))
   runtime=PacketReplayRuntime(recipe,a.manifest,output,session=session)
   try:result=run_runtime(runtime,recipe,session,output)
   finally:runtime.close()
  else:
   if not a.output:p.error('run --execute requires a NEW --output directory')
   from .runner import execute
   result=execute(a.config,a.output,execute=True)
  print(json.dumps(result,indent=2));return 0
 except Exception as e:
  result={'schema':'capx-rings-command-failure/v1','error':{'type':type(e).__name__,'message':str(e)},'hardware_success_claimed':False}
  # Persist blocked launch without touching previous results or site files.
  if getattr(a,'output',None):
   d=Path(a.output).resolve()
   if not d.exists():d.mkdir(parents=True);(d/'blocked-launch.json').write_text(json.dumps(result,indent=2)+'\n')
  print(json.dumps(result,indent=2));return 2
if __name__=='__main__':raise SystemExit(main())
