"""Stop only explicitly named goal-owned Isaac sessions at the user cutoff."""
from pathlib import Path
import argparse
import json
import subprocess
import time
from optimization_clock import DEADLINE,remaining_seconds

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--session',action='append',required=True)
p.add_argument('--finished-marker',type=Path,required=True)
p.add_argument('--status',type=Path,required=True)
a=p.parse_args()
if any(not value.startswith('isaac-jake-gpu-') or any(c not in 'abcdefghijklmnopqrstuvwxyz0123456789-'for c in value)for value in a.session):
    raise SystemExit('Unexpected session identity')
def save(**values):
    a.status.write_text(json.dumps(values,indent=2)+'\n')
save(state='armed',deadline_utc=DEADLINE.isoformat(),owned_sessions=a.session)
while remaining_seconds()>0:
    if a.finished_marker.exists():
        save(state='cancelled_after_goal_completion',owned_sessions=a.session)
        raise SystemExit(0)
    time.sleep(min(30.,max(.01,remaining_seconds())))
results=[]
for session in a.session:
    result=subprocess.run(['sg','docker','-c',f'isaac stop --session {session}'],capture_output=True,text=True,timeout=45)
    results.append(dict(session=session,returncode=result.returncode,output=result.stdout[-2000:],error=result.stderr[-1000:]))
save(state='cutoff_enforced',deadline_utc=DEADLINE.isoformat(),sessions=results,
     next_action='No further optimization. Finish field documentation, evidence and GitHub push.')
