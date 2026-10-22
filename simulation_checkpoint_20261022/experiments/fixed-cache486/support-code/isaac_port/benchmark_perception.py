"""Independent partial-CAD/noise benchmark; no physical camera certification."""
from pathlib import Path
import argparse,json,time
import numpy as np
from scipy.spatial.transform import Rotation
from fieldkit.perception import ASSETS,PARTS,CadPoseEstimator,pose
from optimization_clock import require_optimization_window,install_cutoff_alarm


def benchmark(cases,output,seed=7251,respect_cutoff=False):
    if respect_cutoff:require_optimization_window(120);install_cutoff_alarm()
    started=time.monotonic();rng=np.random.default_rng(seed);estimator=CadPoseEstimator()
    samples=np.load(ASSETS/'part-validation-surfaces.npz');meta=json.loads((ASSETS/'part-models.json').read_text())['parts'];results=[]
    for case in range(cases):
        if respect_cutoff:require_optimization_window(15)
        eye=np.array([rng.uniform(-.4,.15),-.8,rng.uniform(1.85,2.1)])
        for name in PARTS:
            prior=pose(meta[name]['nominal_position'],meta[name]['nominal_quaternion_wxyz']);truth=prior.copy()
            truth[:3,3]+=rng.uniform([-.007,-.007,-.0005],[.007,.007,.0005]);truth[:3,:3]=Rotation.from_euler('z',rng.uniform(-5,5),degrees=True).as_matrix()@truth[:3,:3]
            points=samples[name+'_points']@truth[:3,:3].T+truth[:3,3];normals=samples[name+'_normals']@truth[:3,:3].T
            visible=points[np.sum(normals*(eye-points),axis=1)>0];observed=visible[rng.choice(len(visible),1600,replace=False)]+rng.normal(0,.00025,(1600,3))
            observed=np.r_[observed,rng.uniform(points.min(axis=0)-.005,points.max(axis=0)+.005,(200,3))]
            row=dict(case=case,part=name,translation_m=(truth[:3,3]-prior[:3,3]).tolist())
            try:
                # Truth is deliberately not an input to the registration.
                fit=estimator.fit(name,observed,prior)
                position_error=float(np.linalg.norm(fit.T_cell_part[:3,3]-truth[:3,3]))
                angle=float(np.arccos(np.clip(fit.T_cell_part[:3,2]@truth[:3,2],-1,1)))if name.startswith('ring')else float(Rotation.from_matrix(fit.T_cell_part[:3,:3]@truth[:3,:3].T).magnitude())
                row.update(accepted=True,position_error_m=position_error,orientation_error_rad=angle,rms_m=fit.rms_m,inlier_fraction=fit.inlier_fraction,passed=bool(position_error<.001 and angle<np.deg2rad(2)))
            except RuntimeError as exc:row.update(accepted=False,passed=False,error=str(exc))
            results.append(row);print(json.dumps(row),flush=True)
    report=dict(state='passed'if all(r['passed']for r in results)else'failed',cases=cases,part_fits=len(results),seed=seed,passed=sum(r['passed']for r in results),results=results,seconds=time.monotonic()-started,hardware_ready=False,scope='Independent CAD surface draws, visible faces, 0.25mm Gaussian point noise, clutter, +-7mm XY / +-0.5mm Z / +-5deg yaw; exact synthetic camera calibration. Not a real sensor, occlusion or calibration-error proof.')
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(report,indent=2)+'\n');return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--cases',type=int,default=12);p.add_argument('--seed',type=int,default=7251);p.add_argument('--output',type=Path,required=True);p.add_argument('--respect-cutoff',action='store_true');a=p.parse_args()
    r=benchmark(a.cases,a.output,a.seed,a.respect_cutoff);raise SystemExit(0 if r['state']=='passed'else 1)
