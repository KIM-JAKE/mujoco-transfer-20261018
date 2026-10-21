"""One archived SIM image through public PIN cap and preclose math; no devices."""
from pathlib import Path
import hashlib,json,time,numpy as np
from field_demo.pin_cap import estimate
from field_demo.pin_precision import AXIS_OFFSET
from field_demo.pin_pickup import preclose_target
ROOT=Path(__file__).resolve().parents[1]

def main():
 d=ROOT/'replay/pin-pickup418';f=json.loads((d/'fixture.json').read_text());path=d/'preclose.npz';packet=dict(np.load(path));started=time.perf_counter();fit=estimate(packet,f['previous_pixel_and_encoder_ROI_body']);elapsed=time.perf_counter()-started;P=np.asarray(fit['T_cell_part']);H=np.asarray(f['actual_SIM_TCP']);_,alignment=preclose_target(H,H,P,f['class_only_T_part_tcp']);axis=P@AXIS_OFFSET;reference=f['source_RGBD_reference'];difference=float(np.linalg.norm(axis[:3,3]-reference['center_m']))
 result=dict(scope='One authentic saved SIM418 pixel packet plus recorded SIM encoder FK as SDK-shaped input; no actual SDK, robot or full planner dispatch',source_run=f['source_run'],source_frame=f['source_frame'],source_packet_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),hardware_tested=False,full_field_pick_verified=False,measured_cap_accepted=True,pixel_processing_s=elapsed,measured_center_difference_from_source_RGBD_m=difference,cap_edge_coverage_deg=fit['cap_edge_coverage_deg'],cap_rms_m=fit['cap_circular_edge_rms_m'],alignment=alignment,controller_free_object_pose_inputs=False)
 print(json.dumps(result,indent=2));return 0 if alignment['aligned']and difference<1e-9 else 1

if __name__=='__main__':raise SystemExit(main())
