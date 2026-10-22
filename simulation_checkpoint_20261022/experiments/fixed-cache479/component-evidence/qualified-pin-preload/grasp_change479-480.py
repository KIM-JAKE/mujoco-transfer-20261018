from pathlib import Path
import numpy as np,json
results=[]
for v in ['479','480']:
 rows=json.load(open('/tmp/pin465-analysis/sequence'+v+'.json'))['rows'];g=np.array([r['G_translation']for r in rows]);axes=np.array([np.array(r['A'])[:3,:3].T@np.array(r['axis'])for r in rows]);axes/=np.linalg.norm(axes,axis=1)[:,None]
 dt=np.linalg.norm(np.diff(g,axis=0),axis=1);da=np.arccos(np.clip(np.sum(axes[1:]*axes[:-1],axis=1),-1,1));fromstart=np.arccos(np.clip(axes@axes[0],-1,1));step=[]
 for i in range(1,len(rows)):
  A0,A1=np.array(rows[i-1]['A']),np.array(rows[i]['A']);p0,p1=np.array(rows[i-1]['P']),np.array(rows[i]['P']);rigid_prediction=A1@np.r_[g[i-1],1.];pose_shift=(p1-rigid_prediction[:3]);axis_prediction=A1[:3,:3]@axes[i-1];axis_prediction/=np.linalg.norm(axis_prediction);measured_axis=np.array(rows[i]['axis']);measured_axis/=np.linalg.norm(measured_axis);slipaxis=float(np.arctan2(np.linalg.norm(np.cross(axis_prediction,measured_axis)),axis_prediction@measured_axis))
  step.append(dict(iterations=[i-1,i],relative_center_change_mm=float(dt[i-1]*1e3),relative_axis_change_mrad=float(da[i-1]*1e3),observed_center_change_mm=float(np.linalg.norm(p1-p0)*1e3),world_center_residual_vs_fixed_grasp_mm=(pose_shift*1e3).tolist(),axis_residual_vs_fixed_grasp_mrad=slipaxis*1e3))
 d=dict(run=v,scope='Only fresh RGB-D centers/signed axes and same-frame public actual encoder Hand FK; no actor poses',frames=len(rows),start_G_translation_m=g[0].tolist(),end_G_translation_m=g[-1].tolist(),start_to_end_translation_mm=float(np.linalg.norm(g[-1]-g[0])*1e3),maximum_translation_from_start_mm=float(np.max(np.linalg.norm(g-g[0],axis=1))*1e3),start_to_end_axis_mrad=float(fromstart[-1]*1e3),maximum_axis_from_start_mrad=float(np.max(fromstart)*1e3),maximum_single_step_translation_mm=float(max(dt)*1e3),mean_single_step_translation_mm=float(np.mean(dt)*1e3),maximum_single_step_axis_mrad=float(max(da)*1e3),mean_single_step_axis_mrad=float(np.mean(da)*1e3),steps=step);results.append(d)
 print(v,{k:round(val,5)for k,val in d.items()if isinstance(val,float)})
 for x in sorted(step,key=lambda x:x['relative_axis_change_mrad'],reverse=True)[:3]:print('AXIS_TOP',x)
Path('/tmp/fixed478-pin-qualified-preload/public-relative-grasp-change479-480.json').write_text(json.dumps(results,indent=2)+'\n')
