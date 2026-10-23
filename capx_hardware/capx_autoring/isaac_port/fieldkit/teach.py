"""Record measured stationary FR3 poses and register conservative taught CaP stages.

Capture is read-only. Compilation checks joint envelopes, not collisions or fit.
"""
import argparse
import csv
import hashlib
import ipaddress
import json
import math
from pathlib import Path
import subprocess
from .hand import bounded


def capture(binary, ip):
    ip=str(ipaddress.ip_address(ip))
    r=subprocess.run([str(binary),ip],check=True,capture_output=True,text=True,timeout=10)
    s=json.loads(r.stdout)
    if s.get('read_only') is not True or s.get('server_protocol_version')!=10:
        raise ValueError('Protocol 10 measured state required')
    for key in ('q','q_d','dq','dq_d'):
        if len(s[key])!=7 or any(type(x) not in (int,float) or not math.isfinite(x) for x in s[key]):
            raise ValueError('Invalid measured joint state')
    if max(abs(x) for x in s['dq']+s['dq_d'])>.01 or max(abs(a-b)for a,b in zip(s['q'],s['q_d']))>.005:
        raise RuntimeError('Stop the robot before capturing; no movement command is sent')
    return dict(robot_ip=ip,source='m80_read_state',state=s)


def compile_stage(site_path, stage, poses, seconds, reviewed):
    if not reviewed:raise ValueError('--path-reviewed is required: review the entire interpolated path in the real cell')
    bounded(seconds,2,30,'seconds_per_segment')
    if len(poses)<2 or seconds*(len(poses)-1)>60:raise ValueError('Need >=2 stationary poses, total stage <=60s')
    site_path=site_path.resolve();config=json.loads(site_path.read_text())
    side,action,part=stage.split('.')
    if side not in ('left','right') or action not in ('grasp','insert','release','approach'):
        raise ValueError('Invalid stage key')
    if config['grippers'][side].get('type')!='franka_hand':raise ValueError('Native Hand configuration required')
    rows=[json.loads(p.read_text()) for p in poses];robot=config['robots'][side]
    first=rows[0]['state'];frame=first['F_T_EE_column_major']
    for row in rows:
        s=row['state']
        if row['source']!='m80_read_state' or row['robot_ip']!=robot['ip'] or s['server_protocol_version']!=10:
            raise ValueError('Taught poses must belong to the configured robot')
        for key,expected in [('m_ee_kg',robot['tool_mass_kg']),('m_load_kg',robot['load_mass_kg'])]:
            if not math.isfinite(s[key]) or abs(s[key]-expected)>.001:raise ValueError('Measured payload differs from site config')
        if len(s['F_T_EE_column_major'])!=16 or any(abs(a-b)>1e-8 for a,b in zip(s['F_T_EE_column_major'],frame)):
            raise ValueError('TCP changed between taught poses')
        for key in ('q_d','dq','dq_d'):
            if len(s[key])!=7 or any(not math.isfinite(x) for x in s[key]):raise ValueError('Invalid joint state')
        if max(abs(x)for x in s['dq']+s['dq_d'])>.01:raise ValueError('Taught pose was moving')
    directory=site_path.parent/'stages';directory.mkdir(exist_ok=True)
    path=directory/(stage+'.csv');temporary=path.with_suffix('.candidate.csv')
    try:
        with temporary.open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['time_s']+[k+str(j) for k in ('q','dq','ddq') for j in range(1,8)])
            for i,row in enumerate(rows):w.writerow([seconds*i,*row['state']['q_d'],*([0]*14)])
        binary=Path(config['motion_binary']);binary=binary if binary.is_absolute() else site_path.parent/binary
        subprocess.run([str(binary),'--validate-csv',str(temporary)],check=True,capture_output=True,text=True,timeout=15)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
    config['stages'][stage]=dict(csv=str(path.relative_to(site_path.parent)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        tool_type='franka_hand',field_path_reviewed=True,robot_ip=robot['ip'],F_T_EE_column_major=frame)
    # Editing a path invalidates prior approval of a whole assembly.
    config['assembly_commissioned']=False
    site_path.write_text(json.dumps(config,indent=2)+'\n')
    return dict(stage=stage,csv=str(path),stationary_waypoints=len(rows),duration_s=seconds*(len(rows)-1),
                collision_checked_by_software=False,assembly_commissioned=False)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    c=sub.add_parser('init');c.add_argument('--left-ip',required=True);c.add_argument('--right-ip',required=True);c.add_argument('--output',type=Path,required=True)
    c=sub.add_parser('capture');c.add_argument('--ip',required=True);c.add_argument('--binary',type=Path,default=Path('/tmp/m80-field-build/m80_read_state'));c.add_argument('--output',type=Path,required=True)
    c=sub.add_parser('compile');c.add_argument('--config',type=Path,required=True);c.add_argument('--stage',required=True);c.add_argument('--poses',nargs='+',type=Path,required=True);c.add_argument('--seconds-per-segment',type=float,default=10);c.add_argument('--path-reviewed',action='store_true')
    a=p.parse_args(argv)
    if a.command=='init':
        if a.output.exists():raise FileExistsError(a.output)
        config=json.loads(Path(__file__).with_name('site.franka.example.json').read_text())
        for side in ('left','right'):config['robots'][side]['ip']=str(ipaddress.ip_address(getattr(a,side+'_ip')))
        if config['robots']['left']['ip']==config['robots']['right']['ip']:raise ValueError('Distinct arm IPs required')
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(config,indent=2)+'\n');result=dict(configuration=str(a.output.resolve()))
    elif a.command=='capture':
        if a.output.exists():raise FileExistsError(a.output)
        result=capture(a.binary,a.ip);a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    else:result=compile_stage(a.config,a.stage,a.poses,a.seconds_per_segment,a.path_reviewed)
    print(json.dumps(result,indent=2));return 0


if __name__=='__main__':raise SystemExit(main())
