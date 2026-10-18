"""Publish the prepared, reviewable release ref at the user-requested deadline.

Never commits the working tree. Only the explicitly maintained release ref is
pushed, without force. Credentials stay in the external askpass file.
"""
from pathlib import Path
import argparse,json,os,subprocess,time
from datetime import datetime,timezone
from optimization_clock import DEADLINE,remaining_seconds

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--repo',type=Path,required=True)
p.add_argument('--askpass',type=Path,required=True)
p.add_argument('--status',type=Path,required=True)
a=p.parse_args();ref='refs/heads/m80-noon-release'
def save(**values):
    a.status.parent.mkdir(parents=True,exist_ok=True)
    tmp=a.status.with_suffix('.tmp');tmp.write_text(json.dumps(values,indent=2)+'\n');tmp.replace(a.status)
def git(*args,**kwargs):return subprocess.run(['git',*args],cwd=a.repo,capture_output=True,text=True,**kwargs)
head=git('rev-parse','--verify',ref)
if head.returncode:raise SystemExit('Prepare release ref before arming publisher')
save(state='armed',deadline_utc=DEADLINE.isoformat(),release_ref=ref,initial_commit=head.stdout.strip())
while remaining_seconds()>0:time.sleep(min(30.,max(.001,remaining_seconds())))
commit=git('rev-parse','--verify',ref).stdout.strip()
env=os.environ.copy();env['GIT_ASKPASS']=str(a.askpass.resolve());env['GIT_TERMINAL_PROMPT']='0'
attempts=[]
for attempt in range(4):
    began=datetime.now(timezone.utc).isoformat()
    try:
        result=git('-c','credential.helper=','push','origin',ref+':refs/heads/main',env=env,timeout=60)
        attempts.append(dict(started_at_utc=began,returncode=result.returncode,output=result.stdout[-2000:],error=result.stderr[-2000:]))
        if result.returncode==0:
            save(state='published',deadline_utc=DEADLINE.isoformat(),commit=commit,release_ref=ref,completed_at_utc=datetime.now(timezone.utc).isoformat(),attempts=attempts)
            raise SystemExit(0)
    except subprocess.TimeoutExpired:
        attempts.append(dict(started_at_utc=began,error='push timed out'))
    time.sleep(2**attempt)
save(state='publish_failed',deadline_utc=DEADLINE.isoformat(),commit=commit,attempts=attempts)
raise SystemExit(1)
