"""Offline RAW RGB-D through the public field detector; no actual SDK/devices.

Includes different archived VIRTUAL camera calibrations. Previous pixel pose
and recorded SIM robot TCP propagate the search ROI; actor poses are absent.
"""
import argparse,copy,hashlib,json,time
from pathlib import Path
from datetime import datetime,timezone
import numpy as np
from field_demo.random_perception import load_random_config,detect_packet
from field_demo.pin_precision import AXIS_OFFSET

ROOT=Path(__file__).resolve().parents[1]
def metadata(config,path,index):
    return dict(capture_id=f'OFFLINE_SIM308_image{index}',captured_at_utc=datetime.now(timezone.utc).isoformat(),units='m',source='archived_SIM308_RGBD_replay_only',replay=True,camera_frame=config['camera_frame'],point_cloud_frame=config['camera_frame'],camera_serial=config['camera_serial'],calibration_id=config['calibration_id'],calibration_sha256=config['_calibration_sha256'],rectified_registered_depth=True,source_packet_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),timestamp_scope='OFFLINE replay comparison UTC; original SIM exposure index retained; not actual live freshness')

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--compare-icp',action='store_true');args=parser.parse_args(argv)
    directory=ROOT/'replay/pin-cap308';fixture=json.loads((directory/'adapter-fixtures.json').read_text());initial=fixture['initial_previous_image'];first_config=load_random_config(directory/initial['camera_config']);first_meta=metadata(first_config,directory/initial['snapshot'],initial['index'])
    pose=dict(part='pin',instance_id=first_meta['capture_id']+':image:pin',T_cell_part=initial['T_cell_part'])
    previous=dict(state='random_ring_observation',replay=True,calibration_sha256=first_config['_calibration_sha256'],source_metadata=first_meta,instances=[pose],held_robot='right',measured_tcp_pose=initial['actual_SIM_TCP'],_camera_config_path=str(directory/initial['camera_config']))
    previous_tcp=np.asarray(initial['actual_SIM_TCP']);previous_body=np.asarray(initial['T_cell_part']);rows=[]
    for frame in fixture['frames']:
        path=directory/frame['snapshot'];packet=dict(np.load(path));tcp=np.asarray(frame['actual_SIM_TCP']);G=np.linalg.inv(previous_tcp)@previous_body;row=dict(index=frame['index'],virtual_camera=True)
        for backend in ['cap']+(['icp']if args.compare_icp else[]):
            camera_path=directory/frame[backend+'_camera_config'];cfg=load_random_config(camera_path);meta=metadata(cfg,path,frame['index']);started=time.perf_counter()
            try:
                observed=detect_packet(packet,cfg,meta,replay=True,scope='held',part='pin',held_tcp_pose=tcp,previous_observation=previous,held_robot='right',held_grasp_transform=G)
                result=observed['instances'][0];axis_pose=np.asarray(result['T_cell_part'])@AXIS_OFFSET;reference=frame['source_RGBD_reference']
                center_difference=float(np.linalg.norm(axis_pose[:3,3]-reference['center_m']));angle=float(np.arccos(np.clip(axis_pose[:3,2]@reference['axis'],-1,1)))
                row[backend]=dict(accepted=True,elapsed_s=time.perf_counter()-started,center_difference_from_source_RGBD_m=center_difference,axis_difference_from_source_RGBD_rad=angle,circle_edge_coverage_deg=result.get('cap_edge_coverage_deg'),circle_rms_m=result.get('cap_circular_edge_rms_m'),plane_rms_m=result.get('plane_rms_m'),fallback_used=result.get('fallback_used'),source='actual archived camera pixels, not actor pose')
                if backend=='cap':cap_observation=copy.deepcopy(observed);cap_observation['_camera_config_path']=str(camera_path)
            except Exception as error:row[backend]=dict(accepted=False,elapsed_s=time.perf_counter()-started,error=str(error))
        rows.append(row)
        if not row['cap']['accepted']:break
        previous=cap_observation;previous_tcp=tcp;previous_body=np.asarray(previous['instances'][0]['T_cell_part'])
    result=dict(scope='Public field detector RAW SIM308 RGBD backend replay, different VIRTUAL views; no hardware camera/SDK/robot',hardware_tested=False,field_ready=False,physical_fixed_camera_visibility_verified=False,full_motion_or_assembly_executed=False,initial_prior_source='Previous source308 pixel-fit + recorded SIM encoder TCP',all_cap_frames_passed=bool(len(rows)==4 and all(r['cap']['accepted']for r in rows)),rows=rows)
    print(json.dumps(result,indent=2))
    if not result['all_cap_frames_passed']:return 1
    assert max(r['cap']['center_difference_from_source_RGBD_m']for r in rows)<1e-9
    return 0

if __name__=='__main__':raise SystemExit(main())
