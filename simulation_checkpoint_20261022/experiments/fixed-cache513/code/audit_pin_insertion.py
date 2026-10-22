"""Score insertion procedure separately from final position. Validation only."""
from pathlib import Path
import json,sys,numpy as np
HERE=Path(__file__).resolve().parent

def audit(root,write=True):
    r=json.loads((root/'pipeline-results.json').read_text());goal=np.array(json.loads((root/'arm-plan.json').read_text())['goals']['pin']['P']);z=np.load(root/'physx-sequence-trace.npz');c=np.load(root/'executed-actuator-commands.npz');pi=z['part_names'].tolist().index('pin');p=z['parts'][:,pi,:3];q=z['parts'][:,pi,3:];angle=np.arccos(np.clip(1-2*(q[:,0]**2+q[:,1]**2),-1,1));radial=np.linalg.norm(p[:,:2]-goal[:2],axis=1);axial=p[:,2]-goal[2];fid=[i for i,n in enumerate(r['dof_names'])if n.startswith('right_native')];command_gap=np.sum(c['command'][:,fid],axis=1);gap=np.interp(z['time_s'],c['time_s'],command_gap);aligned=(radial<.003)&(angle<.05)&(axial>-.050)&(axial<.004);entry=np.flatnonzero(aligned);violations=[];early=None;first_full=None
    if len(entry):
        first=int(entry[0]);inserted=(np.linalg.norm(p-goal,axis=1)<.0015)&(angle<.05);full=np.flatnonzero(inserted&(np.arange(len(p))>=first));first_full=int(full[0])if len(full)else None;progress=(axial-axial[first])>.008;open_before=aligned&progress&(gap>.014)&(~inserted)&(np.arange(len(p))>=first)
        if first_full is not None:open_before[first_full:]=False
        a=np.flatnonzero(open_before)
        if len(a):
            early=int(a[0]);violations.append('Fingers opened before full insertion')
            if axial[early:].max()-axial[early]>.005:violations.append('Pin advanced again after early release / second push')
        if first_full is None:violations.append('No full seating observed')
        end=first_full if first_full is not None else len(p)-1
        if np.min(np.diff(axial[first:end+1]),initial=0)<-.002:violations.append('Axial reverse motion >2 mm before seating')
    else:first=None;violations.append('No aligned axial approach observed')
    result=dict(source_run=root.name,procedure_pass=not violations,final_pose_pass=bool(np.linalg.norm(p[-1]-goal)<.0015 and angle[-1]<.05),violations=violations,first_aligned_approach_wall_s=float(z['time_s'][first])if first is not None else None,first_full_seating_wall_s=float(z['time_s'][first_full])if first_full is not None else None,early_release_wall_s=float(z['time_s'][early])if early is not None else None,remaining_axial_depth_at_early_release_mm=float(-axial[early]*1000)if early is not None else None,commanded_gap_at_early_release_mm=float(gap[early]*1000)if early is not None else None,axial_advance_after_early_release_mm=float((axial[early:].max()-axial[early])*1000)if early is not None else None,source_final_pose_is_not_proof_of_direct_insertion=True,validation_uses_recorded_body_poses=True,controller_uses_validation_poses=False,hardware_tested=False)
    if write:(root/'pin-insertion-procedure.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
if __name__=='__main__':print(json.dumps(audit(HERE.parents[1]/'runs'/sys.argv[1]),indent=2))
