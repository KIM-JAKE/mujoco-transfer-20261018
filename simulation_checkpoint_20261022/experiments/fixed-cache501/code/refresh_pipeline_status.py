"""Summarize fresh physical reports; planned clocks never count as success."""
from pathlib import Path
import json,datetime
HERE=Path(__file__).resolve().parent
RUNS=HERE.parents[1]/'runs'
rows=[]
for folder in sorted(RUNS.iterdir()):
    if not(folder.name.startswith('native-roas-table-bin-retimed')or folder.name.startswith('native-roas-table-bin-OD')):continue
    file=folder/'pipeline-results.json'
    if not file.exists():continue
    r=json.load(open(file))
    rows.append(dict(run=folder.name,state=r.get('state'),full_task_pass=r.get('full_task_pass',False),cycle_time_s=r.get('cycle_time_s'),verified_elapsed_s=r.get('verified_elapsed_s'),planned_s=r.get('timing',{}).get('planned_s'),final_checks=r.get('final_checks'),command_audit=r.get('FR3_command_limit_audit'),error=(r.get('error')or'').strip().splitlines()[-1:] or None))
valid=[row for row in rows if row['full_task_pass']and row['cycle_time_s']is not None]
legacy=RUNS/'native-roas-table-bin-under30-capture01/pipeline-results.json'
r=json.load(open(legacy))
if r.get('full_task_pass')and r.get('cycle_time_s')is not None:valid.append(dict(run=legacy.parent.name,cycle_time_s=r['cycle_time_s'],full_task_pass=True))
best=min(valid,key=lambda row:row['cycle_time_s'])if valid else None
now=datetime.datetime.now(datetime.timezone.utc).isoformat()
summary=dict(updated_at_utc=now,target_s=30.,target_achieved=bool(best and best['cycle_time_s']<30),best_validated_run=best['run']if best else None,best_validated_cycle_s=best['cycle_time_s']if best else None,physical_trials=rows,vision_end_to_end_pass=False,hardware_tested=False,perception='USD geometry and ideal simulated body poses',completion_rule='All three parts: center error <1.5 mm, axis error <0.05 rad; both hand roots >160 mm; retained for >=0.5 s',time_basis='Simulation motion time, excluding offline planning/render wall time; start at prepared pregrasp, no return-home requirement',model_limits='Finite-force uncalibrated fixture clamps; simplified four-cylinder pin with 0.2 mm radial allowance; original finger meshes; one mixed-bin arrangement')
(HERE/'bin-picking/RETIMING_STATUS.json').write_text(json.dumps(summary,indent=2))
if best:
    report=json.load(open(RUNS/best['run']/'pipeline-results.json'))
    current={key:report.get(key)for key in ['full_task_pass','continuous_cycle_pass','cycle_time_s','verified_elapsed_s','final_checks','FR3_command_limit_audit','part_attachments','workpiece_teleports_after_initialization','cross_arm_peak_contact_n','pin_collision_approximation']}
    current.update(updated_at_utc=now,run=best['run'],source_report=str(RUNS/best['run']/'pipeline-results.json'),target_under30_achieved=summary['target_achieved'],vision_end_to_end_pass=False,hardware_tested=False,state='full_task_under30_pass'if summary['target_achieved']else'full_task_pass_optimization_pending')
    (HERE/'bin-picking/PIPELINE_STATUS.json').write_text(json.dumps(current,indent=2))
    active=json.load(open(HERE/'ACTIVE_STATUS.json'));active.update(current);(HERE/'ACTIVE_STATUS.json').write_text(json.dumps(active,indent=2))
print(json.dumps({k:v for k,v in summary.items()if k!='physical_trials'},indent=2))
