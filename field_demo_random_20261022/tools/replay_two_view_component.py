"""Portable offline replay: two archived SIM images, not physical camera proof.

Replays ray fusion only. No SDK connection, robot motion, live freshness or
scene-lease admission is attempted. Original image timestamps are represented
by an explicit frozen OFFLINE comparison clock for deterministic replay.
"""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json
import numpy as np
from field_demo.residual_occupancy import build_occupancy

ROOT=Path(__file__).resolve().parents[1]
def main():
    directory=ROOT/'replay/recorded-closure-v280'
    observation=json.loads((directory/'overhead-same-snapshot-observation.json').read_text())
    config=json.loads((directory/'overhead-same-snapshot-camera.json').read_text())
    # Config contains the original nominal replay detector basket ROI. For this
    # ray component use the exact authored calibrated stock grid from report.
    archived=json.loads((ROOT/'reports/recorded-two-view-first-blocker.json').read_text())
    with np.load(ROOT/'reports/recorded-two-view-unknown-voxels.npz')as z:bounds=z['bounds_cell_m'].copy()
    clock=datetime.now(timezone.utc);snapshots=[];sources=[]
    for name,capture in [('global-rgbd.npz','v280-same-snapshot-global'),('overhead-same-snapshot-rgbd.npz','v280-same-snapshot-overhead')]:
        path=directory/name
        with np.load(path,allow_pickle=False)as z:packet={k:z[k]for k in z.files}
        metadata=dict(observation['source_metadata']);metadata.update(capture_id=capture,captured_at_utc=clock.isoformat(),cell_frame='recorded_v280_SIM_cell',source_packet_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),calibration_sha256=hashlib.sha256(packet['T_cell_camera'].tobytes()+packet['K'].tobytes()).hexdigest())
        snapshot=build_occupancy(packet,metadata,bounds,voxel_size_m=.004,depth_margin_m=.003,max_age_s=2,now_utc=clock,max_voxels=1000000)
        snapshots.append(snapshot);sources.append((packet,metadata))
    overhead=snapshots[1];global_view=snapshots[0]
    fused=build_occupancy(*sources[1],bounds,voxel_size_m=.004,depth_margin_m=.003,prior=global_view,max_age_s=2,now_utc=clock,max_voxels=1000000)
    result=dict(scope='OFFLINE two virtual SIM camera ray-fusion component only; no live lease or hardware',hardware_tested=False,robot_command_sent=False,physical_camera_count_proof=False,overhead_unknown=int((overhead.states==0).sum()),fused_unknown=int((fused.states==0).sum()),actual_rays_clear_unknown=int(((overhead.states==0)&(fused.states==1)).sum()),all_original_occupied_preserved=bool(np.all(fused.states[overhead.states==2]==2)),all_global_occupied_preserved=bool(np.all(fused.states[global_view.states==2]==2)),source_count=len(fused.sources),oldest_source_utc=min(x['captured_at_utc']for x in fused.sources),full_approach_verified=False,field_ready=False)
    expected=archived['two_view_fusion']
    assert result['actual_rays_clear_unknown']==expected['genuinely_ray_cleared_unknown']
    assert result['fused_unknown']==expected['fused_unknown']
    assert result['all_original_occupied_preserved']and result['all_global_occupied_preserved']
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
