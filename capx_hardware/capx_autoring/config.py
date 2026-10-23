"""Offline setup and immutable source/site binding for the concrete ring runner."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib,json
from hardware_v1.config import canonical,resolve,sha
from hardware_v1.backend import HardwareBackend

ROOT=Path(__file__).resolve().parent

def load(path):
 p=Path(path).resolve();d=json.loads(p.read_text(),parse_constant=lambda x:(_ for _ in ()).throw(ValueError('Nonfinite JSON '+x)))
 if d.get('schema')!='capx-autonomous-rings/v1':raise ValueError('Expected capx-autonomous-rings/v1')
 return d,p

def engine_identity():
 record=json.loads((ROOT/'engine-manifest.json').read_text())
 expected=record['files'];actual={str(p.relative_to(ROOT)):sha(p)for p in ROOT.rglob('*')if p.is_file()and '__pycache__'not in p.parts and p.name!='engine-manifest.json'}
 if actual!=expected:raise ValueError('Installed automatic ring engine/assets changed')
 return hashlib.sha256(canonical(actual).encode()).hexdigest()

def pin(path):
 """Pin current local bytes; NEVER mark commissioning/calibration/admission valid."""
 c,p=load(path);c['engine_source_sha256']=engine_identity()
 for name in ['hardware','recipe']:
  q=resolve(p.parent,c[name]['path']);c[name]['sha256']=sha(q)
 recipe=json.loads(resolve(p.parent,c['recipe']['path']).read_text());base=resolve(p.parent,c['recipe']['path']).parent
 paths=[recipe.get(k)for k in ['site_config','camera_config','planner_config','payload_profile']]
 paths+=list(recipe.get('camera_configs',{}).values())
 c['dependencies']={str(resolve(base,n)):sha(resolve(base,n))for n in paths if n}
 # Relative manifest entries survive relocation when kept beside the recipe.
 import os
 c['dependencies']={os.path.relpath(k,p.parent):v for k,v in c['dependencies'].items()}
 p.write_text(json.dumps(c,indent=2)+'\n');return c

def preflight(path):
 c,p=load(path);issues=[];deps={p:sha(p)};hardware=None;recipe=None
 def fail(field,error):issues.append({'field':field,'reason':str(error)})
 try:
  identity=engine_identity()
  if c.get('engine_source_sha256')!=identity:raise ValueError('Pin current installed engine identity')
 except Exception as e:fail('engine_source_sha256',e)
 for name in ['hardware','recipe']:
  try:
   v=c[name];q=resolve(p.parent,v['path'])
   if sha(q)!=v['sha256']:raise ValueError('Exact local bytes must be pinned')
   deps[q]=v['sha256']
   if name=='hardware':hardware=HardwareBackend(q);issues.extend(hardware.preflight()['blockers'])
  except Exception as e:fail(name,e)
 try:
  if not isinstance(c.get('dependencies'),dict)or not c['dependencies']:raise ValueError('Pin recipe, site, camera, planner and payload files')
  for name,digest in c['dependencies'].items():
   q=resolve(p.parent,name)
   if sha(q)!=digest:raise ValueError('Dependency changed: '+name)
   deps[q]=digest
 except Exception as e:fail('dependencies',e)
 try:
  from .field_demo.random_cycle import validate_recipe,validate_live_inputs
  from .field_demo.random_perception import load_random_config
  from .field_demo.random_planner import validate_config,load as planner_load
  rp=resolve(p.parent,c['recipe']['path']);raw=json.loads(rp.read_text());root=rp.parent
  site_path=resolve(root,raw['site_config']);site=json.loads(site_path.read_text())
  if hardware is None or site_path!=hardware._site_config()[1]:raise ValueError('Runner and backend must use the same pinned site file')
  for name in ['motion','state']:
   actual=resolve(site_path.parent,site[name+'_binary']);expected=hardware.config['binaries'][name]
   if actual!=resolve(hardware.base,expected['path'])or sha(actual)!=expected['sha256']:raise ValueError('Actual site '+name+' binary differs from pinned backend')
  for side in ['left','right']:
   actual=resolve(site_path.parent,site['grippers'][side]['binary']);expected=hardware.config['binaries']['hand']
   if actual!=resolve(hardware.base,expected['path'])or sha(actual)!=expected['sha256']:raise ValueError('Actual Hand binary differs from pinned backend')
  recipe=validate_recipe(rp,site)
  if raw.get('replay_only')is True:raise ValueError('Replay recipe cannot enable live execution')
  if raw.get('clamp_control',{}).get('mode')!='actual_PLC':raise ValueError('Automatic runner requires mapped actual_PLC (no manual confirmation shortcut)')
  if raw.get('stabilization_profile')is not None:raise ValueError('Old source-tree stabilization manifests need separate port/requalification; no silent profile substitution')
  validate_live_inputs(recipe,site)
  for n in set([raw['camera_config'],*raw.get('camera_configs',{}).values()]):
   cp=resolve(root,n);camera=load_random_config(cp)
   if cp not in deps:raise ValueError('Camera not in exact dependency set')
   if camera.get('_replay_only'):raise ValueError('Replay camera cannot enable live hardware')
   hc=hardware.config['camera'];hardware_cal=json.loads(resolve(hardware.base,hc['calibration']).read_text())
   if camera['camera_serial']!=hardware_cal['camera_serial']or camera['T_cell_camera']!=hardware_cal['T_cell_camera']:raise ValueError('Automatic camera differs from the measured backend camera')
   if camera.get('camera_profile')!={k:hc[k]for k in ['K','width','height','fps']}:raise ValueError('Exact measured camera K/size/fps profile required')
   if type(hc.get('max_age_s'))not in (int,float)or not 0<hc['max_age_s']<=.5:raise ValueError('Original RealSense age limit (0,.5] required')
   for side in ['left','right']:
    if camera['robots'][side]['T_cell_base']!=site['robots'][side]['T_cell_base']:raise ValueError('Camera/site base frame mismatch')
  planner=resolve(root,raw['planner_config']);pd={};validate_config(planner_load(planner),planner.parent,pd,datetime.now(timezone.utc))
  optional=planner_load(planner).get('cuRobo_proposal')
  if optional is not None:
   for key in ['executable','model_record']:
    q=resolve(planner.parent,optional[key]['path'])
    if sha(q)!=optional[key]['sha256']:raise ValueError('cuRobo '+key+' identity mismatch')
    deps[q]=optional[key]['sha256']
  required=[site_path,planner,resolve(root,raw['payload_profile'])]
  if any(q not in deps for q in required):raise ValueError('Missing exact recipe dependency')
  profile=json.loads(resolve(root,raw['payload_profile']).read_text())
  # Native binaries and measurement evidence transitively validated by original modules.
  for q in [resolve(planner.parent,planner_load(planner)['planner_binary']),resolve(root,profile['driver_binary'])]:deps[q]=sha(q)
  deps.update({Path(r['path']):r['sha256']for r in pd.values()})
  def collect_evidence(value,base):
   if isinstance(value,dict):
    if value.get('evidence_file'):
     q=resolve(base,value['evidence_file']);deps[q]=sha(q)
    for child in value.values():collect_evidence(child,base)
   elif isinstance(value,list):
    for child in value:collect_evidence(child,base)
  collect_evidence(raw,root);collect_evidence(profile,root)
  collect_evidence(planner_load(planner),planner.parent)
  for n in set([raw['camera_config'],*raw.get('camera_configs',{}).values()]):
   cp=resolve(root,n);collect_evidence(json.loads(cp.read_text()),cp.parent)
 except Exception as e:fail('measured_ring_policy',e)
 return {'schema':'capx-autonomous-rings-preflight/v1','local_config_complete':not issues,'blockers':issues,'device_connected':False,'motion_issued':False,'hardware_ready':False,'automatic_ring_runner_included':True,'required_order':['ring2','ring1']},c,p,deps,recipe

def initialize(directory,hardware='hardware.json'):
 d=Path(directory).resolve();d.mkdir(parents=True,exist_ok=True)
 names=['autoring.json','rings.recipe.json','rings.camera.json','rings.planner.json','rings.payload.json']
 if any((d/n).exists()for n in names):raise FileExistsError('New files required; existing site/calibration never overwritten')
 templates=ROOT/'templates'
 for n in names[1:]:
  (d/n).write_bytes((templates/n).read_bytes())
 cfg={'schema':'capx-autonomous-rings/v1','engine_source_sha256':engine_identity(),'hardware':{'path':hardware,'sha256':None},'recipe':{'path':'rings.recipe.json','sha256':sha(d/'rings.recipe.json')},'dependencies':{},'note':'Actual measured values remain null/unverified. pin-files pins bytes only; it does not authorize motion.'}
 (d/'autoring.json').write_text(json.dumps(cfg,indent=2)+'\n');return d/'autoring.json'
