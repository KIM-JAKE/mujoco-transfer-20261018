"""Original native fixed-CAD admission throughout closure, with pose reserves."""
from pathlib import Path
import time,json,numpy as np
from isaac_kinematics import frame
from rgbd_geometry import axis_frame
from grasp_transfer_feasibility import donor_aperture

def run(checker,observed,config):
    start=time.perf_counter();prior=json.loads((Path('/workspace/project/mujoco/isaac_port/runs/native-roas-table-model-v454-fixed-chord-scan/fixed-grasp-scan.json')).read_text());base=prior['pose'];n=np.asarray(base['axis']);c=dict(config=config,observed_grasp_clearance=observed);rows=[]
    candidates=[(r['chord_offset_m'],r['height_m'],r['yaw_deg'])for r in prior['admitted']]
    candidates+= [(0.,height,yaw)for height in[.005,.0055,.006]for yaw in[-90.,90.,-60.,60.,-120.,120.,-45.,45.,-135.,135.]]
    candidates=list(dict.fromkeys(candidates));shifts=[np.zeros(3)]+[np.eye(3)[i]*sign*.0003 for i in range(3)for sign in[-1.,1.]]+[np.asarray([x,y,z])*.0002 for x in[-1.,1.]for y in[-1.,1.]for z in[-1.,1.]]
    for scenario_dx in[0.,.010,.020,.030]:
      P=np.asarray(base['center_m'])+np.array([scenario_dx,0,0])
      for chord,height,yaw_deg in candidates:
        yaw=np.radians(yaw_deg);z=np.array([0.,0.,-1.]);y=np.array([np.cos(yaw),np.sin(yaw),0.]);R=np.column_stack([np.cross(y,z),y,z]);toward=np.array([.23,-.70825,.884313439])-P;tcp_offset=R[:,0]*np.sign(toward@R[:,0])*chord;H=frame(P+n*np.sign(n[2])*height+tcp_offset,R)
        width=donor_aperture(c,'ring1',np.linalg.inv(H)@frame(P,axis_frame(n)));queries=0;failed=None;widths=[]
        if width is not None:
            widths=np.linspace(max(.001,width-.003),.08,5)
            for shift in shifts:
                target=H.copy();target[:3,3]+=shift
                for w in widths:
                    queries+=1;ok,why=checker.hand_clear(target,float(w))
                    if not ok:failed=dict(TCP_translation_reserve_m=shift.tolist(),width_m=float(w),collision=why);break
                if failed is not None:break
        rows.append(dict(synthetic_RGBD_X_shift_m=scenario_dx,center_world_m=P.tolist(),chord_offset_m=float(chord),TCP_offset_world_m=tcp_offset.tolist(),height_m=float(height),yaw_deg=float(yaw_deg),original_donor_contact_width_m=width,checked_closure_widths_m=np.asarray(widths).tolist(),original_native_fixed_queries=queries,robust_admitted=bool(width is not None and failed is None),first_native_fixed_rejection=failed))
    accepted=[r for r in rows if r['robust_admitted']]
    return dict(MODEL_only=True,source_pose='Frozen449 direct precision RGB-D + explicitly synthetic X shifts0/10/20/30mm; no current/free-body state',pose=base,sampled_candidates=len(rows),admitted_count=len(accepted),admitted=accepted,rows=rows,wall_s=time.perf_counter()-start,physical_commands_issued=0,scan_physics_steps=0,TCP_translation_reserves_m=[x.tolist()for x in shifts],scope='Necessary full native Hand versus original fixed CAD at 5 closure apertures including commanded diameter-minus3mm, with 15 encoder-TCP translation reserves. Original nominal donor contact authority preserved. Current whole robot/stock/exact issued route and actual contact remain mandatory.')
