"""Launch Isaac Sim's installed CPU PhysX with no exposed GPU devices."""
from pathlib import Path
import sys,subprocess,shlex,re,time,json,shutil,hashlib
HERE=Path(__file__).resolve().parent
script=sys.argv[1]if len(sys.argv)>1 else'native_roas_physx.py'
case=sys.argv[2]if len(sys.argv)>2 else'baseline'
assert case in ['baseline','candidate']
assert Path(script).name==script and script.endswith('.py')
runlabel=sys.argv[3]if len(sys.argv)>3 else''
assert not runlabel or re.fullmatch(r'native-roas-table-[a-zA-Z0-9_-]+',runlabel)
if script=='run_bin_arm.py':
    assert runlabel,'Bin tests require an explicit unique run label'
    dest=HERE.parents[1]/'runs'/runlabel;dest.mkdir(exist_ok=True)
    case_input=HERE/'bin-picking'
    if len(sys.argv)>4:
        assert re.fullmatch(r'[a-zA-Z0-9_-]+',sys.argv[4]);case_input=case_input/'cases'/sys.argv[4]
    for name in ['arm-plan.json','initial-robot-pose.json']:shutil.copyfile(case_input/name,dest/name)
    frozen=dest/'code_snapshot';frozen.mkdir(exist_ok=True);manifest={}
    for root in [HERE.parents[1],HERE]:
        for source in root.glob('*.py'):
            contents=source.read_bytes();(frozen/source.name).write_bytes(contents);manifest[source.stem]=dict(original_file=str(source),sha256=hashlib.sha256(contents).hexdigest())
    (dest/'code-manifest.json').write_text(json.dumps(manifest,indent=2))
    holder=HERE/'bin-picking/three-pin-holder-mesh.npz'
    if holder.exists():shutil.copyfile(holder,dest/holder.name)
if script in ['run_basket_pipeline.py','run_retimed_cached.py','run_rgb_cached.py','run_realtime_recipe.py']:
    assert runlabel
    dest=HERE.parents[1]/'runs'/runlabel;dest.mkdir(exist_ok=True)
    shutil.copyfile(HERE/'bin-picking/pipeline-options.json',dest/'pipeline-options.json')
    saved_options=json.load(open(dest/'pipeline-options.json'))
    if saved_options.get('motion_file'):shutil.copyfile(HERE/saved_options['motion_file'],dest/'motion-commands.npz')
    frozen=dest/'code_snapshot';frozen.mkdir(exist_ok=True)
    for source in HERE.glob('*.py'):shutil.copyfile(source,frozen/source.name)
prefix='/workspace/project/mujoco/isaac_port/native_hand_layout/roas-table/'
args=['docker','run','--rm','--name','isaac-jake-m80-cpu-'+str(__import__('os').getpid()),'--runtime','runc','--network','none','--cpus',('2'if script=='run_trace_cpu_video.py'and re.search(r'-s\d+of\d+$',runlabel)else'8'),'--memory','48g','--shm-size','1g','--user','1000:1000','--env','ACCEPT_EULA=Y','--env','NVIDIA_VISIBLE_DEVICES=void','--env','OPENBLAS_NUM_THREADS=1','--env','OMP_NUM_THREADS=1','--env','M80_ROAS_CASE='+case,'--env','HOME=/tmp/m80-isaac-cpu-home','--mount','type=bind,src=/home/gpuadmin/workspace/jake,dst=/workspace/project','--entrypoint','/bin/bash','sha256:24731957113a5c8b61ad29ad1da15bfa7a9ac3768408087306eeba4871a54256','-c','mkdir -p /tmp/m80-isaac-cpu-home; exec /isaac-sim/kit/kit '+prefix+'physx-cpu.kit --no-window --allow-root --exec '+prefix+script]
if runlabel:args[2:2]=['--env','M80_ROAS_RUN='+runlabel]
started=time.time()
with open(HERE/(Path(script).stem+('-'+runlabel if runlabel else'-candidate'if case=='candidate'else'')+'.log'),'w')as out:
    result=subprocess.run(['sg','docker','-c',shlex.join(args)],stdout=out,stderr=subprocess.STDOUT)
print('ISAAC_CPU_PHYSX_EXIT',result.returncode,flush=True)
if script=='run_native_pickup_probe.py':
    report=HERE.parents[1]/'runs'/(runlabel or'native-roas-table-03')/'physx-pickup-results.json'
    if not report.exists()or report.stat().st_mtime<started:
        print('ISAAC_CPU_PHYSX_NO_COMPLETED_REPORT',flush=True);raise SystemExit(3)
    state=json.load(open(report));print('ISAAC_CPU_PICKUP_PASS',state.get('pickup_pass',False),flush=True)
    if not state.get('pickup_pass'):raise SystemExit(2)
if script=='run_bin_arm.py':
    report=HERE.parents[1]/'runs'/runlabel/'physx-pickup-results.json'
    if not report.exists()or report.stat().st_mtime<started:raise SystemExit('No fresh completed full-arm bin diagnostic report')
    state=json.load(open(report));print('ISAAC_BIN_ARM_PICKUP_PASS',state.get('pickup_pass',False),flush=True)
    requested=json.load(open(dest/'arm-plan.json'));status=dict(updated_at_utc=__import__('datetime').datetime.now(__import__('datetime').timezone.utc).isoformat(),run=runlabel,report=str(report),physics='Isaac Sim 6.1 Omni PhysX CPU',mixed_ring_counts=requested.get('clutter_counts'),pickup_pass=state.get('pickup_pass',False),three_part_assembly_pass=state.get('full_task_pass',False),vision_end_to_end_pass=False,hardware_tested=False,cycle_time_s=state.get('sequence',{}).get('cycle_time_s'),state=state.get('state'),error=state.get('error')or state.get('sequence',{}).get('error'))
    (HERE/'bin-picking/STATUS.json').write_text(json.dumps(status,indent=2))
    if requested.get('continue_sequence'):
        print('ISAAC_THREE_PART_ASSEMBLY_PASS',state.get('full_task_pass',False),flush=True)
        if not state.get('full_task_pass'):raise SystemExit(2)
    elif not state.get('pickup_pass'):raise SystemExit(2)
raise SystemExit(result.returncode)
