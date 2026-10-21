from pathlib import Path
import os,subprocess,json
T=Path('/tmp/field-analytic-yaw596-correct-widths');PY='/home/gpuadmin/workspace/jake/mujoco/keti_m80_mujoco_20261013/.venv/bin/python';tracking=os.environ['DIAG_TRACKING_RAD'];progress=[]
for mode in ['original','swap']:
 label=mode+'-'+tracking
 with(T/(label+'.log')).open('w')as f:subprocess.run([PY,str(T/(mode+'.py'))],env=os.environ,stdout=f,stderr=subprocess.STDOUT,check=True,timeout=180)
 d=json.loads((T/(label+'.json')).read_text());rows=d['campaign'];result=dict(mode=mode,tracking_rad=float(tracking),plans=len(rows),geometry_rejections=sum(x['stage']=='hypothetical_endpoint_full_model_guard'for x in rows),IK_rejections=sum(x['stage']=='current_native_DLS_IK'for x in rows),endpoint_survivors=sum(x.get('accepted',False)for x in rows),selected=d['screen_selected']);progress.append(result);(T/('progress-'+tracking+'.json')).write_text(json.dumps(progress,indent=2)+'\n');print(json.dumps(result),flush=True)
 if result['endpoint_survivors']:raise SystemExit(0)
