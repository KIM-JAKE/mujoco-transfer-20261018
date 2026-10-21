"""Sequential taught three-part cycle, optional reviewed handover, measured withdrawal.

This controller uses commissioned stage paths and actual PLC/native Hand
feedback. It does not generate collision-free paths or contact-search motions.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import time
from datetime import datetime,timezone
from isaac_port.fieldkit.hand import FrankaHand,bounded
from isaac_port.fieldkit.site import Site,rotation
from .calibration import utc_stamp
from .execution import StopListener
from . import handover

PARTS=('ring1','ring2','pin')


def measured_transform(value):
    rotation({'robots':{'pose':{'T_cell_base':value}}},'pose')
    if math.sqrt(sum(value[i][3]**2 for i in range(3)))>5:
        raise ValueError('Measured transform outside 5m; check metre units')
    return value


def validate(site,recipe_path):
    path=Path(recipe_path).resolve();recipe=json.loads(path.read_text())
    if (recipe.get('site_config_sha256')!=handover.digest(site.filename) or
        json.loads(Path(site.filename).read_text())!=site.config):
        raise ValueError('Cycle reviewed site configuration changed')
    entries=recipe['parts']
    if recipe.get('schema_version')!=1 or len(entries)!=3 or {p.get('part') for p in entries}!=set(PARTS):
        raise ValueError('Cycle must contain ring1, ring2 and pin exactly once')
    bounded(recipe['feedback_timeout_s'],.1,5,'feedback_timeout_s')
    bounded(recipe['feedback_dwell_s'],.1,1,'feedback_dwell_s')
    bounded(recipe['release_width_tolerance_m'],0,.002,'release_width_tolerance_m',positive=True)
    if recipe['feedback_timeout_s']<recipe['feedback_dwell_s']:raise ValueError('Feedback timeout must cover dwell')
    stages={}
    for part in entries:
        name=part['part'];pickup,insert=part['pickup_side'],part['insert_side']
        if pickup not in ('left','right') or insert not in ('left','right'):raise ValueError('Measured arm assignments required')
        measured_transform(part['T_cell_part'])
        bounded(part['min_tcp_clearance_m'],.04,1,'min_tcp_clearance_m')
        for side in set((pickup,insert)):
            site.validate_motion_config(side)
            if not isinstance(site.grippers[side],FrankaHand):raise ValueError('Native Franka Hands required')
        actions=site.config['gripper_actions'][name]
        for action in ('open','release'):FrankaHand.validate_action(actions[action])
        FrankaHand.validate_action(actions['grasp'],grasp=True)
        if actions['open']['width_m']<=actions['grasp']['width_m']+actions['grasp']['epsilon_outer_m']:
            raise ValueError('Pickup opening must clear measured grasp width')
        for suffix in ('grasped','seated','clamped','retained','clamp_command'):
            signal=site.plc.signals[name+'_'+suffix]
            kinds=('coil',) if suffix=='clamp_command' else ('discrete','input','holding')
            if signal.get('kind') not in kinds or type(signal.get('address'))is not int:
                raise ValueError('Mapped physical part feedback and clamp output required')
        for key,side,action in [('pickup_stage',pickup,'grasp'),('insert_stage',insert,'insert'),
                                 ('release_stage',insert,'release'),('withdraw_stage',insert,'approach')]:
            stage=part[key]
            if (not isinstance(stage,str) or not stage.startswith(side+'.'+action+'.') or
                (key!='withdraw_stage' and stage!=side+'.'+action+'.'+name)):
                raise ValueError('Cycle stage is on the wrong arm or action: '+key)
            row=site.config['stages'][stage];csv_path=site.path(row['csv'])
            if (row.get('tool_type')!='franka_hand' or row.get('field_path_reviewed') is not True or
                row.get('robot_ip')!=site.config['robots'][side]['ip'] or handover.digest(csv_path)!=row.get('sha256')):
                raise ValueError('Cycle stage hash/tool/IP/review mismatch')
            frame=row.get('F_T_EE_column_major',[])
            if len(frame)!=16 or any(type(v) not in (int,float) or not math.isfinite(v) for v in frame):
                raise ValueError('Measured native stage TCP required')
            with csv_path.open() as stream:points=list(csv.DictReader(stream))
            if len(points)<2:raise ValueError('Cycle stage needs at least two stationary waypoints')
            if key=='release_stage' and any(abs(float(p['q'+str(j)])-float(points[0]['q'+str(j)]))>1e-6 for p in points for j in range(1,8)):
                raise ValueError('Release stage must hold the seated position')
            subprocess.run([str(site.path(site.config['motion_binary'])),'--validate-csv',str(csv_path)],
                check=True,capture_output=True,text=True,timeout=15)
            stages[stage]=row['sha256']
        transfer_path=part.get('handover_recipe')
        if pickup!=insert:
            if not transfer_path:raise ValueError('Different pickup/insert arms require reviewed handover')
            transfer_path=path.parent/transfer_path
            if handover.digest(transfer_path)!=part.get('handover_sha256'):raise ValueError('Cycle handover recipe changed')
            transfer=handover.validate(site,transfer_path)
            if (transfer['part'],transfer['donor_side'],transfer['receiver_side'])!=(name,pickup,insert):
                raise ValueError('Handover part/arm assignments differ from cycle')
            stages.update(transfer['stage_sha256'])
        elif transfer_path:raise ValueError('Same-arm part must not request a two-arm handover')
    if recipe.get('stage_sha256')!=stages:raise ValueError('Cycle review must bind every exact stage hash')
    review_path=path.parent/recipe['review_file']
    if handover.digest(review_path)!=recipe['review_sha256']:raise ValueError('Cycle review changed')
    review=json.loads(review_path.read_text())
    if (review.get('recipe_content_sha256')!=handover.recipe_content_sha256(recipe) or not review.get('reviewer') or
        review.get('full_path_reviewed') is not True or review.get('checked_actual_cell') is not True or
        utc_stamp(review['at_utc'])>datetime.now(timezone.utc)):
        raise ValueError('Exact actual-cell cycle path/poses review required')
    return recipe


def read_tcp_cell(site,side):
    robot=site.config['robots'][side]
    p=subprocess.run([str(site.path(site.config['state_binary'])),robot['ip']],check=True,
        capture_output=True,text=True,timeout=10)
    state=json.loads(p.stdout)
    if state.get('read_only') is not True or state.get('server_protocol_version')!=10:
        raise ValueError('Measured protocol-10 state required after withdrawal')
    raw=state['O_T_EE_column_major']
    if len(raw)!=16:raise ValueError('Measured TCP transform required')
    tcp=measured_transform([[raw[4*c+r] for c in range(4)] for r in range(4)])
    base=measured_transform(robot['T_cell_base'])
    point=[sum(base[i][j]*tcp[j][3] for j in range(3))+base[i][3] for i in range(3)]
    return point


def clearance(site,entry,side=None,*,tcp_cell=None):
    point=read_tcp_cell(site,side or entry['insert_side']) if tcp_cell is None else tcp_cell
    distance=math.sqrt(sum((point[i]-entry['T_cell_part'][i][3])**2 for i in range(3)))
    if distance<entry['min_tcp_clearance_m']:
        raise RuntimeError('Measured hand TCP has not withdrawn beyond the required clearance')
    return distance


def execute(site,recipe_path):
    original_ready=site.require_ready;latched=set();completed=[]
    def ready():
        original_ready()
        for signal in tuple(latched):
            if not site.plc.read(signal):raise RuntimeError('Installed part feedback lost: '+signal)
    try:
        recipe=validate(site,recipe_path);site.require_ready=ready
        ready()
        for part in PARTS:
            for suffix in ('grasped','seated','clamped','retained'):
                if site.plc.read(part+'_'+suffix):raise RuntimeError('New cycle requires cleared feedback: '+part+'_'+suffix)
        def wait(signal):
            site.guarded(lambda cancel:site.plc.wait(signal,timeout_s=recipe['feedback_timeout_s'],
                dwell_s=recipe['feedback_dwell_s'],cancelled=cancel))
        def motion(entry,key,side):
            row=site.config['stages'][entry[key]];site.motion(side,'csv',[site.path(row['csv'])]);ready()
        for entry in recipe['parts']:
            part=entry['part'];pickup,insert=entry['pickup_side'],entry['insert_side']
            donor=site.grippers[pickup];actions=site.config['gripper_actions'][part]
            if donor.status()['is_grasped']:raise RuntimeError('Pickup Hand must start empty')
            site.guarded(lambda cancel:donor.move(**actions['open'],cancelled=cancel))
            motion(entry,'pickup_stage',pickup)
            if site.plc.read(part+'_grasped'):raise RuntimeError('Require a fresh grasp feedback transition')
            site.guarded(lambda cancel:donor.move(**actions['grasp'],grasp=True,cancelled=cancel))
            wait(part+'_grasped')
            if not donor.status()['is_grasped']:raise RuntimeError('Native Hand did not hold the picked part')
            site.expected_held[pickup]=part
            if pickup!=insert:
                handover.transfer(site,Path(recipe_path).resolve().parent/entry['handover_recipe'])
                site.expected_held.pop(pickup,None);site.expected_held[insert]=part
            hand=site.grippers[insert]
            if not hand.status()['is_grasped']:raise RuntimeError('Insertion Hand lost the part')
            if site.plc.read(part+'_seated'):raise RuntimeError('Require fresh seating feedback')
            motion(entry,'insert_stage',insert);wait(part+'_seated');latched.add(part+'_seated')
            if site.plc.read(part+'_clamped'):raise RuntimeError('Require fresh clamp feedback')
            site.guarded(lambda cancel:site.plc.write(part+'_clamp_command',True))
            wait(part+'_clamped');latched.add(part+'_clamped');ready()
            if not hand.status()['is_grasped']:raise RuntimeError('Insertion Hand lost part before controlled release')
            if site.plc.read(part+'_retained'):raise RuntimeError('Require fresh post-release retention feedback')
            motion(entry,'release_stage',insert)
            held=hand.status()
            if actions['release']['width_m']<=held['width_m']+recipe['release_width_tolerance_m']:
                raise ValueError('Release width must exceed the measured held width')
            site.expected_held.pop(insert,None)
            opened=site.guarded(lambda cancel:hand.move(**actions['release'],cancelled=cancel))
            if opened.get('is_grasped') is not False or opened.get('width_m',-1)<actions['release']['width_m']-recipe['release_width_tolerance_m']:
                raise RuntimeError('Measured hand opening required before withdrawal')
            wait(part+'_retained');latched.add(part+'_retained');ready()
            motion(entry,'withdraw_stage',insert)
            distance=clearance(site,entry);ready()
            completed.append(part)
            site.event('part_withdrawn_and_retained',part=part,tcp_clearance_m=distance)
        began=time.monotonic()
        while time.monotonic()-began<.5:
            ready();time.sleep(.02)
        ready()
        # Later motions can revisit earlier fixtures. Check both CURRENT TCPs
        # against every installed part, after the final retention dwell. This
        # is a TCP-distance check; whole-hand/path clearance remains reviewed.
        current_tcp={}
        for side in site.grippers:
            current_tcp[side]=read_tcp_cell(site,side);ready()
        for side,point in current_tcp.items():
            for entry in recipe['parts']:clearance(site,entry,side,tcp_cell=point)
        if any(g.status()['is_grasped'] for g in site.grippers.values()):raise RuntimeError('Cycle ended with a held part')
        ready()
        site.event('three_part_cycle_completed',completed_parts=completed,final_retention_dwell_s=.5)
        return dict(state='three_part_cycle_completed',completed_parts=completed,
                    all_parts_retained=True,all_insert_hands_withdrawn=True,hardware_ready=False)
    except BaseException:
        site.stop()
        raise
    finally:site.require_ready=original_ready


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path);p.add_argument('--recipe',type=Path)
    p.add_argument('--log-dir',type=Path,default=Path('field-logs'))
    group=p.add_mutually_exclusive_group(required=True)
    group.add_argument('--validate',action='store_true');group.add_argument('--execute',action='store_true');group.add_argument('--dry-run',action='store_true')
    a=p.parse_args(argv)
    if a.dry_run:
        print(json.dumps(dict(state='cycle_contract_only',parts=list(PARTS),robot_connections=0,
            phases=['pickup','optional_taught_handover','insert','clamp','release','withdraw','final_retention_dwell'],hardware_ready=False)));return 0
    if not a.config or not a.recipe:p.error('--config and --recipe required')
    site=Site(a.config,a.log_dir)
    if a.validate:
        validate(site,a.recipe);result=dict(state='cycle_configuration_checked',robot_connections=0,hardware_ready=False)
    else:
        with site.control_lock(),StopListener(site):result=execute(site,a.recipe)
    print(json.dumps(result,indent=2));return 0


if __name__=='__main__':raise SystemExit(main())
