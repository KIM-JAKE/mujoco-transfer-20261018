"""User-requested hard cutoff, independent of the server's timezone."""
from datetime import datetime,timezone
from pathlib import Path
import argparse,json
import signal
import subprocess
import os
DEADLINE=datetime(2026,10,19,5,0,tzinfo=timezone.utc)
def remaining_seconds(now=None):return (DEADLINE-(now or datetime.now(timezone.utc))).total_seconds()
def require_optimization_window(expected_seconds=0):
    left=remaining_seconds()
    if left<=0 or expected_seconds>left:raise RuntimeError('Optimization cutoff reached or run would cross 2026-10-19 14:00 Asia/Seoul. Package/push field preparation; no new optimization.')
    return left
def install_cutoff_alarm():
    """Interrupt a running Python optimizer at the user deadline (Linux)."""
    left=require_optimization_window()
    def cutoff(signum,frame):
        raise TimeoutError('Optimization deadline reached; package/push field kit.')
    signal.signal(signal.SIGALRM,cutoff)
    signal.setitimer(signal.ITIMER_REAL,left)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--expected-seconds',type=float,default=0);a=p.parse_args();left=require_optimization_window(a.expected_seconds);print(json.dumps(dict(deadline_utc=DEADLINE.isoformat(),deadline_kst='2026-10-19T14:00:00+09:00',remaining_seconds=left)))
