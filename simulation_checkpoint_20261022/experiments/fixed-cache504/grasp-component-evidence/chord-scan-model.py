"""Original native fixed-CAD and donor contact scan of one archived RGB-D pose."""
import time,numpy as np
from isaac_kinematics import frame
from rgbd_geometry import axis_frame
from grasp_transfer_feasibility import donor_aperture

def run(checker,observed,config):
    start=time.perf_counter()
    pose=dict(part='ring1',center_m=[.21998565501736658,-.5653309538538466,.8920606560308658],axis=[1.7326916093690286e-06,-4.779898040075016e-08,-.9999999999984978])
    P=np.asarray(pose['center_m']);n=np.asarray(pose['axis']);c=dict(config=config,observed_grasp_clearance=observed);rows=[]
    for chord in [0.,.003,.006,.009,.012,.018,.024]:
      for height in np.arange(.005,.01201,.0005):
        for yaw_deg in list(np.arange(-180,180,15)):
            yaw=np.radians(yaw_deg);z=np.array([0.,0.,-1.]);y=np.array([np.cos(yaw),np.sin(yaw),0.]);R=np.column_stack([np.cross(y,z),y,z]);toward=np.array([.23,-.70825,.884313439])-P;tcp_offset=R[:,0]*np.sign(toward@R[:,0])*chord;H=frame(P+n*np.sign(n[2])*height+tcp_offset,R)
            clear,why=checker.hand_clear(H,.08)
            width=donor_aperture(c,'ring1',np.linalg.inv(H)@frame(P,axis_frame(n)))if clear else None
            rows.append(dict(chord_offset_m=float(chord),TCP_offset_world_m=tcp_offset.tolist(),height_m=float(height),yaw_deg=float(yaw_deg),original_native_fixed_hand_clear=bool(clear),original_donor_contact_width_m=width,admitted=bool(clear and width is not None),fixed_rejection=why))
    passing=[r for r in rows if r['admitted']]
    return dict(MODEL_only=True,source_pose='Frozen449 directly observed precision RGB-D LARGE pose; no current/free-body state',pose=pose,sampled_candidates=len(rows),admitted_count=len(passing),admitted=passing,rows=rows,wall_s=time.perf_counter()-start,physical_commands_issued=0,scan_physics_steps=0,scope='Necessary native fixed/pad geometry only; all current robot IK, route, stock, issued, fresh RGB-D contact checks still mandatory before any physical trial')
