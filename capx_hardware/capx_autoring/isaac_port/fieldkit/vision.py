"""Fit pickup poses from the camera-neutral NPZ snapshot contract."""
from datetime import datetime,timezone
from pathlib import Path
import argparse,hashlib,json,time
import numpy as np
from .perception import ASSETS,CadPoseEstimator,PARTS,load_snapshot,transform


def detect(snapshot,config_path,output,replay=False):
    started=time.monotonic();config=json.loads(Path(config_path).read_text())
    if not config.get('calibration_id') or not config.get('camera_frame'):raise ValueError('Named camera frame and calibration record required')
    priors={n:transform(config['nominal_stock_poses'][n])for n in PARTS}
    points,labels,metadata=load_snapshot(snapshot,config,replay=replay)
    estimator=CadPoseEstimator();estimates=estimator.observe(points,config['T_cell_camera'],priors,labels=labels,roi_margin_m=config.get('roi_margin_m',.035))
    result=dict(state='poses_estimated',hardware_ready=False,replay=bool(replay),capture_id=metadata['capture_id'],captured_at_utc=metadata['captured_at_utc'],calibration_id=config['calibration_id'],camera_frame=config['camera_frame'],output_frame='cell',units='m',parts={n:estimates[n].as_dict()for n in PARTS},template_sha256=hashlib.sha256((ASSETS/'part-models.npz').read_bytes()).hexdigest(),calibration_sha256=hashlib.sha256(Path(config_path).read_bytes()).hexdigest(),elapsed_s=time.monotonic()-started,limitations=['CAD registration near calibrated upright pickup priors','Ring axial orientation stays at prior','Statistical fit residual excludes sensor bias, reflective metal depth errors and hand-eye calibration error','Fresh observation must be checked again before executing a cached plan; this output is not a hardware motion authorization'])
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2)+'\n');return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('snapshot',type=Path);p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--replay',action='store_true');a=p.parse_args();r=detect(a.snapshot,a.config,a.output,a.replay);print(json.dumps(dict(state=r['state'],parts={n:dict(rms_m=x['rms_m'],inlier_fraction=x['inlier_fraction'])for n,x in r['parts'].items()},elapsed_s=r['elapsed_s'],hardware_ready=False),indent=2))
