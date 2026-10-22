"""Static CAD/robot diagnostic, not a pickup or completed assembly trial."""
import time,numpy as np
from isaac_kinematics import frame,exp
from grasp_transfer_feasibility import receiving_poses,pad_aperture,sequential_transfer_clear

def execute(c):
    started=time.perf_counter();part='ring2';incoming=np.asarray(c['handoff_reference'][part]['part_H']);rc=c['checkers']['right'];lc=c['checkers']['left'];receivers=list(receiving_poses(c,part,incoming));solutions=[];counts=dict(giver_grasps=0,opposed_pad_grasps=0,receiver_pairs=0);limit=30.
    for mode in ['outer','inner']:
        for tilt_deg in [0,180,90,-90,45,-45,135,-135,75,-75,105,-105]:
            for yaw_deg in range(0,360,30):
                rotation=incoming[:3,:3]@exp(np.array([0,0,np.radians(yaw_deg)]))@exp(np.array([0,np.radians(tilt_deg),0]))@np.diag([1.,-1.,-1.])
                for x in [0.,.006,-.006]:
                    for z in [.002,.0045,-.002]:
                        H=frame(incoming[:3,3]-rotation@np.array([x,0,z]),rotation);G=np.linalg.inv(H)@incoming;counts['giver_grasps']+=1
                        if mode=='inner':
                            if abs(G[2,2])<.98 or x!=0.:continue
                            width=.025
                        else:
                            width=pad_aperture(c,part,G)
                            if width is None:continue
                        counts['opposed_pad_grasps']+=1
                        for target,metadata in receivers:
                            receiver_width=pad_aperture(c,part,np.linalg.inv(target)@incoming)
                            counts['receiver_pairs']+=1
                            if sequential_transfer_clear(rc,H,width,lc,target,receiver_width,.068,donor_release_width=.001 if mode=='inner'else .08):
                                solutions.append(dict(mode=mode,giver_tilt_deg=tilt_deg,giver_yaw_deg=yaw_deg,giver_object_in_TCP=G.tolist(),giver_width_m=width,receiver_metadata=[float(v)for v in metadata],receiver_object_in_TCP=(np.linalg.inv(target)@incoming).tolist()));break
                        if len(solutions)>=12 or time.perf_counter()-started>limit:break
                    if len(solutions)>=12 or time.perf_counter()-started>limit:break
                if len(solutions)>=12 or time.perf_counter()-started>limit:break
            if len(solutions)>=12 or time.perf_counter()-started>limit:break
        if len(solutions)>=12 or time.perf_counter()-started>limit:break
    print('STABLE_NATIVE_PAIR_DIAGNOSTIC',counts,len(solutions),time.perf_counter()-started,flush=True)
    return dict(state='geometry_diagnostic_only',full_task_pass=False,uses_free_body_poses=False,scope='Fixed CAD gripper-pair clearance only; no scene part moved; no arm path guarantee',stable_receiver_count=len(receivers),counts=counts,solutions=solutions,elapsed_s=time.perf_counter()-started)
