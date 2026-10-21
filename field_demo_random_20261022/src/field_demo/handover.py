"""One operator-selected, taught two-hand transfer; no automatic path planning.

The donor must already hold the selected part. This unit does not execute its
pickup or insertion and does not establish a collision-free handover by itself.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess

from isaac_port.fieldkit.hand import FrankaHand, bounded
from isaac_port.fieldkit.site import Site
from .calibration import utc_stamp
from .execution import StopListener


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def recipe_content_sha256(recipe):
    body={key:value for key,value in recipe.items() if key not in ('review_file','review_sha256')}
    return hashlib.sha256(json.dumps(body,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def validate(site, recipe_path):
    path=Path(recipe_path).resolve();recipe=json.loads(path.read_text())
    if (recipe.get('site_config_sha256')!=digest(site.filename) or
        json.loads(Path(site.filename).read_text())!=site.config):
        raise ValueError('Handover reviewed site configuration changed')
    if recipe.get('schema_version')!=1 or recipe.get('part') not in ('ring1','ring2','pin'):
        raise ValueError('Handover recipe requires schema 1 and a known part')
    donor,receiver=recipe['donor_side'],recipe['receiver_side']
    if donor not in ('left','right') or receiver not in ('left','right') or donor==receiver:
        raise ValueError('Handover requires distinct donor and receiver arms')
    for side in (donor,receiver):
        if not isinstance(site.grippers[side],FrankaHand):raise ValueError('Native Franka Hands required')
        site.validate_motion_config(side)
    stages=[]
    for key,side in [('donor_transport_stage',donor),('receiver_approach_stage',receiver),('donor_retreat_stage',donor)]:
        name=recipe[key]
        if not isinstance(name,str) or not name.startswith(side+'.approach.'):
            raise ValueError('Handover stages must be explicitly taught approach stages on the correct arm')
        row=site.config['stages'][name];stage_path=site.path(row['csv'])
        if (row.get('tool_type')!='franka_hand' or row.get('field_path_reviewed') is not True or
            row.get('robot_ip')!=site.config['robots'][side]['ip'] or digest(stage_path)!=row.get('sha256')):
            raise ValueError('Handover stage hash/tool/IP/review mismatch')
        frame=row.get('F_T_EE_column_major',[])
        if len(frame)!=16 or any(type(v) not in (int,float) or not math.isfinite(v) for v in frame):
            raise ValueError('Measured stage TCP required')
        with stage_path.open() as stream:points=list(csv.DictReader(stream))
        if len(points)<2:raise ValueError('At least two taught stage records required')
        subprocess.run([str(site.path(site.config['motion_binary'])),'--validate-csv',str(stage_path)],
                       check=True,capture_output=True,text=True,timeout=15)
        stages.append((row,points))
    if stages[0][0]['F_T_EE_column_major']!=stages[2][0]['F_T_EE_column_major']:
        raise ValueError('Donor TCP changed between transport and retreat')
    for j in range(1,8):
        if abs(float(stages[0][1][-1]['q'+str(j)])-float(stages[2][1][0]['q'+str(j)]))>1e-6:
            raise ValueError('Donor transport and retreat must share the handover pose')
    for key in ('receiver_open','donor_release'):FrankaHand.validate_action(recipe[key])
    FrankaHand.validate_action(recipe['receiver_grasp'],grasp=True)
    if recipe['receiver_open']['width_m']<=recipe['receiver_grasp']['width_m']+recipe['receiver_grasp']['epsilon_outer_m']:
        raise ValueError('Receiver opening must clear the measured grasp')
    bounded(recipe['feedback_timeout_s'],.1,5,'feedback_timeout_s')
    bounded(recipe['feedback_dwell_s'],.1,1,'feedback_dwell_s')
    bounded(recipe['release_width_tolerance_m'],0,.002,'release_width_tolerance_m',positive=True)
    if recipe['feedback_timeout_s']<recipe['feedback_dwell_s']:
        raise ValueError('Feedback timeout must cover its dwell')
    signals=[recipe['donor_grasped_signal'],recipe['receiver_grasped_signal']]
    if any(not isinstance(s,str) or not s for s in signals) or signals[0]==signals[1]:
        raise ValueError('Independent donor and receiver grasped signals required')
    for name in signals:
        row=site.plc.signals[name]
        if row.get('kind') not in ('discrete','input','holding') or type(row.get('address'))is not int:
            raise ValueError('Mapped measured grasp inputs required')
    left,right=[site.plc.signals[name] for name in signals]
    # Distinct labels for the same physical bit are not independent feedback.
    if (left.get('kind'),left.get('address'),left.get('mask'))==(right.get('kind'),right.get('address'),right.get('mask')):
        raise ValueError('Donor and receiver inputs alias the same PLC channel')
    review_path=path.parent/recipe['review_file']
    if digest(review_path)!=recipe['review_sha256']:raise ValueError('Handover review file changed')
    review=json.loads(review_path.read_text())
    if (review.get('recipe_content_sha256')!=recipe_content_sha256(recipe) or not review.get('reviewer') or
        review.get('full_path_reviewed') is not True or review.get('checked_actual_cell') is not True or
        utc_stamp(review['at_utc'])>datetime.now(timezone.utc)):
        raise ValueError('Exact actual-cell handover path/poses review required')
    # Bind the review to exact stage hashes as part of the recipe, not mutable names.
    if recipe.get('stage_sha256')!={recipe[key]:site.config['stages'][recipe[key]]['sha256']
                                  for key in ('donor_transport_stage','receiver_approach_stage','donor_retreat_stage')}:
        raise ValueError('Recipe must bind all three reviewed stage hashes')
    return recipe


def transfer(site, recipe_path):
    """Donor held -> receiver secured -> donor opens -> donor retreats; no retries."""
    original_ready=site.require_ready
    monitors=set()
    recipe=None
    def ready():
        original_ready()
        for name in tuple(monitors):
            if not site.plc.read(name):raise RuntimeError('Handover grasp feedback lost: '+name)
    try:
        recipe=validate(site,recipe_path)
        donor,receiver=recipe['donor_side'],recipe['receiver_side']
        dg,rg=site.grippers[donor],site.grippers[receiver]
        donor_signal,receiver_signal=recipe['donor_grasped_signal'],recipe['receiver_grasped_signal']
        site.require_ready=ready
        original_ready()
        donor_state=dg.status()
        if not donor_state['is_grasped'] or not site.plc.read(donor_signal):
            raise RuntimeError('Donor must already hold the selected part with measured feedback')
        if rg.status()['is_grasped'] or site.plc.read(receiver_signal):
            raise RuntimeError('Receiver must start empty with cleared independent grasp feedback')
        if recipe['donor_release']['width_m']<=donor_state['width_m']+recipe['release_width_tolerance_m']:
            raise ValueError('Donor release must open beyond its measured held width')
        monitors.add(donor_signal)
        opened=site.guarded(lambda cancel:rg.move(**recipe['receiver_open'],cancelled=cancel))
        if opened.get('is_grasped') is not False or opened.get('width_m',-1)<recipe['receiver_open']['width_m']-recipe['release_width_tolerance_m']:
            raise RuntimeError('Receiver opening was not measured')
        for key,side in [('donor_transport_stage',donor),('receiver_approach_stage',receiver)]:
            row=site.config['stages'][recipe[key]]
            site.motion(side,'csv',[site.path(row['csv'])])
            ready()
        if site.plc.read(receiver_signal):
            raise RuntimeError('Receiver feedback became high before the commanded grasp; require a fresh transition')
        site.guarded(lambda cancel:rg.move(**recipe['receiver_grasp'],grasp=True,cancelled=cancel))
        site.guarded(lambda cancel:site.plc.wait(receiver_signal,timeout_s=recipe['feedback_timeout_s'],
                    dwell_s=recipe['feedback_dwell_s'],cancelled=cancel))
        monitors.add(receiver_signal)
        ready()
        if not rg.status()['is_grasped'] or not dg.status()['is_grasped']:
            raise RuntimeError('Both native Hands must confirm the part before donor release')
        # Removing donor feedback is necessary only for this commanded release.
        # Receiver feedback remains continuously active through release/retreat.
        ready();monitors.remove(donor_signal)
        result=site.guarded(lambda cancel:dg.move(**recipe['donor_release'],cancelled=cancel))
        if result.get('is_grasped') is not False or result.get('width_m',-1)<recipe['donor_release']['width_m']-recipe['release_width_tolerance_m']:
            raise RuntimeError('Donor opening was not measured; do not retreat')
        ready()
        if not rg.status()['is_grasped']:raise RuntimeError('Receiver lost part after donor release')
        row=site.config['stages'][recipe['donor_retreat_stage']]
        site.motion(donor,'csv',[site.path(row['csv'])])
        ready()
        if not rg.status()['is_grasped']:raise RuntimeError('Receiver lost part during donor retreat')
        site.event('handover_unit_completed',part=recipe['part'],donor=donor,receiver=receiver)
        return dict(state='handover_unit_completed',part=recipe['part'],donor=donor,receiver=receiver,
                    pickup_and_insertion_executed=False,hardware_ready=False)
    except BaseException:
        site.stop()
        raise
    finally:
        site.require_ready=original_ready


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--recipe',type=Path,required=True)
    p.add_argument('--log-dir',type=Path,default=Path('field-logs'))
    group=p.add_mutually_exclusive_group(required=True)
    group.add_argument('--validate',action='store_true');group.add_argument('--execute',action='store_true')
    a=p.parse_args(argv)
    site=Site(a.config,a.log_dir)
    if a.validate:
        recipe=validate(site,a.recipe)
        result=dict(state='handover_configuration_checked',part=recipe['part'],robot_connections=0,hardware_ready=False)
    else:
        with site.control_lock(),StopListener(site):result=transfer(site,a.recipe)
    print(json.dumps(result,indent=2));return 0


if __name__=='__main__':raise SystemExit(main())
