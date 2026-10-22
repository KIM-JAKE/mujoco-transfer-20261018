"""Plan, run dynamics, independently accept, then render measured states."""
from pathlib import Path
import argparse,json,subprocess,sys,time

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--speed',type=float,default=3);p.add_argument('--device',default='cpu');p.add_argument('--stride',type=int,default=1);mode=p.add_mutually_exclusive_group();mode.add_argument('--conservative-timing',action='store_true',help='Use the archived 70%% velocity / 4 rad/s2 profile');mode.add_argument('--baseline-timing',action='store_true',help='Use the archived 22.88s 95%% profile instead of the under-15 target');a=p.parse_args();target_15=not(a.conservative_timing or a.baseline_timing)
root=Path(__file__).resolve().parent;out=a.output.resolve();out.mkdir(parents=True,exist_ok=True);start=time.monotonic()
def save(**v):(out/'pipeline.json').write_text(json.dumps(v,indent=2)+'\n')
def run(name,args):
    save(state='running',phase=name)
    with(out/(name+'.log')).open('w')as log:
        return subprocess.run([sys.executable,*map(str,args)],stdout=log,stderr=subprocess.STDOUT).returncode

planning=0
if not a.conservative_timing:
    planning=run('planning',[root/'prepare_fr3_timing.py']+(['--target-15']if target_15 else[]))
    if planning:
        save(state='failed',phase='planning',returncode=planning);sys.exit(1)
physics_args=[root/'test_contact_task.py','--output',out,'--feedback','--precision','--physical-sensors','--clamps','--posture','--fast','--fr3-limits','--pin-preload','.01','--cycle-deadline','60','--speed',a.speed,'--device',a.device]
if not a.conservative_timing:physics_args.append('--efficient-timing')
if target_15:physics_args.append('--target-15')
physics=run('physics',physics_args)
if not(out/'trace.json').exists():
    save(state='failed',phase='physics',returncode=physics);sys.exit(1)
limits=run('fr3-limits',[root/'evaluate_fr3_limits.py',out])if(out/'joint-motion.npz').exists()else 1
acceptance=run('acceptance',[root/'evaluate_contact.py',out])
if physics or limits or acceptance:
    save(state='failed',phase='validation',physics_exit=physics,limits_exit=limits,acceptance_exit=acceptance,seconds=time.monotonic()-start,physical_simulation_pass=False);sys.exit(1)
recording=run('recording-audit',[root/'verify_contact_recording.py',out])
if recording:
    save(state='failed',phase='recording-audit',returncode=recording);sys.exit(1)
run('plot',[root/'plot_fr3_limits.py',out])
render_normal=run('render-normal',[root/'render_contact.py','--run',out,'--stride',a.stride,'--cycle-only','--output',out/'render-normal'])
render_xray=run('render-xray',[root/'render_contact.py','--run',out,'--stride',a.stride,'--cycle-only','--xray','--output',out/'render-xray'])
render=run('compose',[root/'compose_contact_video.py','--run',out])if render_normal==render_xray==0 else 1
passed=render==0 and json.loads((out/'acceptance.json').read_text())['physical_simulation_pass']
save(state='passed'if passed else'failed',planning_exit=planning,cycle_target_s=15. if target_15 else None,recording_audit_exit=recording,planning_fraction=.70 if a.conservative_timing else .95,physics_exit=physics,limits_exit=limits,acceptance_exit=acceptance,render_exit=render,seconds=time.monotonic()-start,physical_simulation_pass=bool(passed));sys.exit(0 if passed else 1)
