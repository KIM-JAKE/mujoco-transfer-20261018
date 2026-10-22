"""Start an experimental CPU Isaac Sim trial; full random-bin assembly is unvalidated."""
from pathlib import Path
import argparse,datetime,json,subprocess,sys
HERE=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seed',type=int,default=103,help='Test scene initialization only; never supplied to the controller')
parser.add_argument('--policy',type=Path,default=HERE/'bin-picking/direct-pin-policy.json',help='Experimental simulation policy JSON; frozen with the trial')
parser.add_argument('--run',default='native-roas-table-direct-'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
args=parser.parse_args()
import re
if not re.fullmatch(r'native-roas-table-[a-zA-Z0-9_-]+',args.run):parser.error('Invalid run label')
out=HERE.parents[1]/'runs'/args.run
if out.exists():parser.error('Choose a new run label; existing evidence is never overwritten')
config=json.loads(args.policy.expanduser().resolve().read_text());config['seed']=args.seed
out.mkdir();(out/'trial-config.json').write_text(json.dumps(config,indent=2)+'\n')
result=subprocess.run([sys.executable,str(HERE/'run_cpu_physx.py'),'run_realtime_recipe.py','candidate',args.run])
report=out/'pipeline-results.json'
if not report.exists():raise SystemExit('Isaac did not produce a completed report')
value=json.loads(report.read_text());print(json.dumps({key:value.get(key) for key in ['state','full_task_pass','end_to_end_under20','wall_verified_elapsed_s','final_checks']},indent=2))
if not value.get('end_to_end_under20'):raise SystemExit(2)
verification=subprocess.run([sys.executable,str(HERE/'verify_wall_trial.py'),args.run,'--require-under20'])
raise SystemExit(verification.returncode)
