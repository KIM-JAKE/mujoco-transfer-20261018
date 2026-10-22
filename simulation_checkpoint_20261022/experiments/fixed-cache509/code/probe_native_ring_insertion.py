"""Static insertion diagnosis from saved RGB-D + robot encoders only.

This never executes an assembly or edits a robot/workpiece physical pose.
"""
from pathlib import Path
import json,time,numpy as np
from isaac_kinematics import frame,exp
from rgbd_geometry import axis_frame
from grasp_transfer_feasibility import insertion_pose_options,pad_aperture

def execute(c):
    h=Path(__file__).resolve().parent;root=h.parents[1]/'runs'/c['config']['insertion_probe_source_run'];report=json.loads((root/'pipeline-results.json').read_text());trace=np.load(root/'physx-sequence-trace.npz');observations=json.loads((root/'rgb-observations.json').read_text());ob=next(o for o in reversed(observations) if o.get('part')=='ring2' and o['mode']=='held_left' and 'pose'in o['result']);k=int(np.argmin(abs(trace['time_s']-ob['captured_wall_s'])));li=[report['dof_names'].index('left_arm_fr3v2_joint'+str(i)) for i in range(1,8)];ri=[report['dof_names'].index('right_arm_fr3v2_joint'+str(i)) for i in range(1,8)];actual_left=trace['q'][k,li];actual_right=trace['q'][k,ri];arm=c['left_arm'];lc=c['checkers']['left'];rc=c['checkers']['right'];pose=ob['result']['pose'];G=np.linalg.inv(arm.fk(actual_left))@frame(pose['center_m'],axis_frame(pose['axis']));goal=np.asarray(c['goals']['ring2']['P']);ref=c['handoff_reference']['ring2'];seeds=[np.array(ref['left_insert_seed']),np.array(ref['left_receive_q']),actual_left]+list(np.random.default_rng(7812).uniform(arm.bounds[:,0],arm.bounds[:,1],(48,7)));started=time.perf_counter();counts={};examples={}
    def reject(label,why=None):
        counts[label]=counts.get(label,0)+1
        if why is not None and label not in examples:examples[label]=why
    def check_grasp(grasp,stop_after=1):
        good=[];width=pad_aperture(c,'ring2',grasp)
        if width is None:return good
        for final,azimuth in insertion_pose_options(c,'ring2',grasp):
            for clearance in [.080,.060,.045,.100,.030,.020]:
                pre=final.copy();pre[2,3]-=clearance
                clear,why=lc.hand_clear(pre,width)
                if not clear:reject('pre_hand_collision',why);continue
                clear,why=lc.hand_clear(final,width)
                if not clear:reject('final_hand_collision',why);continue
                fixed=True
                for u in np.linspace(0,1,9):
                    H=pre.copy();H[:3,3]=pre[:3,3]*(1-u)+final[:3,3]*u;clear,why=lc.hand_clear(H,width)
                    if not clear:reject('stroke_hand_collision',why);fixed=False;break
                if not fixed:continue
                found=[]
                for seed in seeds:
                    q=arm.solve(pre[:3,3],pre[:3,:3],seed)
                    if q is not None and all(np.linalg.norm(q-old)>.08 for old in found):found.append(q)
                    if len(found)>=12:break
                if not found:reject('pre_IK');continue
                for q in found:
                    clear,why=lc.clear(q,width)
                    if not clear:reject('pre_robot_collision',why);continue
                    stroke=[q];valid=True
                    for u in np.linspace(0,1,13)[1:]:
                        nq=arm.solve(pre[:3,3]*(1-u)+final[:3,3]*u,final[:3,:3],stroke[-1])
                        if nq is None:reject('stroke_IK');valid=False;break
                        clear,why=lc.path([stroke[-1],nq],width)
                        if not clear:reject('stroke_robot_collision',why);valid=False;break
                        stroke.append(nq)
                    if valid:
                        good.append(dict(azimuth_rad=azimuth,clearance_m=clearance,opening_m=width,pre_q=q.tolist(),stroke_q=np.asarray(stroke).tolist(),grasp=grasp.tolist()))
                        if len(good)>=stop_after:return good
        return good
    with lc.other_arm_at(rc,actual_right,.08):
        measured=check_grasp(G);synthetic=[]
        if c['config'].get('rim_grasp_probe'):
            for azimuth in np.radians([0,45,90,135,180,-135,-90,-45]):
                radial=np.array([np.cos(azimuth),np.sin(azimuth),0.]);normal=np.array([0.,0.,1.]);R=np.column_stack([np.cross(normal,-radial),normal,-radial])
                for radius in [.0345,.035,.033,.032,.031,.030,.029,.028]:
                    candidate=np.linalg.inv(frame(radial*radius,R));found=check_grasp(candidate)
                    if found:synthetic.extend(found);print('INSERTION_RIM_CANDIDATE',azimuth,radius,found[0]['azimuth_rad'],flush=True)
                    if len(synthetic)>=6:break
                if len(synthetic)>=6:break
        elif not measured or c['config'].get('canonical_grasp_probe'):
            for theta in np.radians([0,30,-30,45,-45,60,-60,90,-90,120,-120,150,180]):
                for z in [0.,-.003,.003]:
                    for x in [0.,-.004,.004]:
                        R=exp(np.array([0.,theta,0.]));candidate=frame([x,0.,z],R.T)
                        if pad_aperture(c,'ring2',candidate) is None:continue
                        found=check_grasp(candidate)
                        if found:
                            synthetic.extend(found);print('INSERTION_GEOMETRY_CANDIDATE',float(theta),x,z,found[0]['azimuth_rad'],flush=True)
                        if len(synthetic)>=6:break
                    if len(synthetic)>=6:break
                if len(synthetic)>=6:break
    result=dict(state='static_insertion_diagnostic_only',full_task_pass=False,source_run=root.name,inputs='Saved RGB-D pose and robot encoders; known CAD and jig calibration',uses_free_body_poses=False,recorded_measured_grasp=G.tolist(),recorded_grasp_strokes=measured,synthetic_grasp_strokes=synthetic,counts=counts,rejection_examples=examples,elapsed_s=time.perf_counter()-started)
    print('INSERTION_GEOMETRY_DIAGNOSIS',json.dumps(result),flush=True);return result
