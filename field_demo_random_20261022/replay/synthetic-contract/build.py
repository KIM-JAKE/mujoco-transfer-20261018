"""Build synthetic-labelled RGB-D contract data. No hardware or real recording.

The detector gets only rendered RGB/depth/organized points, never object labels
or body poses. Synthetic body state belongs only to this explicit fake device
fixture, which does not exercise the real SDK planner or certify collisions.
"""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tests')]
from datetime import datetime,timezone
import copy,json
import numpy as np
from test_random_cycle import FakeRuntime,recipe,pose
from test_random_perception import render_part_packet
from field_demo.random_perception import load_random_config,detect_packet
from field_demo.stabilized.rgbd_geometry import axis_frame
from field_demo import random_cycle

HERE=Path(__file__).resolve().parent

def write(path,data):path.write_text(json.dumps(data,indent=2)+'\n')

def main():
    frames=HERE/'frames';frames.mkdir(exist_ok=True)
    evidence=HERE/'SYNTHETIC_ONLY.txt';evidence.write_text('Synthetic RGB-D pixels and FAKE robot/Hand/PLC states. No actual robot execution, force, collision or timing verification. Not v119 recorded coverage.\n')
    m=dict(verified=True,by='SYNTHETIC_CONTRACT_NOT_HARDWARE',at_utc='2020-01-01T00:00:00Z',evidence_file=evidence.name)
    c=recipe();c.update(schema_version=2,recognition='seed_free_rgbd',replay_only=True,
        site_config='site.json',camera_config='camera.json',planner_config='planner-contract-only.json',commissioning=m,
        _root=str(HERE),insert_pose_confirmation_tolerance_m=.003,held_roi_radius_m=.048,max_task_s=300)
    R=axis_frame([0,0,-1]);pinR=axis_frame([.6,.2,.8])
    for i,e in enumerate(c['parts']):
        family=e['grasp_families'][0];family['T_part_tcp']=pose(0,0,-.01);family['measurement']=dict(m)
        e['payload_dimensions']['measurement']=dict(m)
        e['payload_dimensions'].update(radius_m=.0375 if e['part']=='ring1' else .031 if e['part']=='ring2' else .004,length_m=.016)
        h=e['handover'];presentation=np.eye(4);presentation[:3,:3]=R;presentation[:3,3]=[0,0,.16]
        h['donor_T_cell_tcp']=presentation.tolist();h['receiver_T_part_tcp']=pose(0,0,-.01)
        e['pregrasp_offset_part_m']=[0,0,-.07 if e['part']!='pin' else .07]
        e['insert_entry_offset_part_m']=[0,0,-.08 if e['part']!='pin' else .08]
        e['lift_offsets_cell_m']=[[0,0,.05]];e['withdraw_offset_cell_m']=[0,0,.08]
        goal=np.eye(4);goal[:3,:3]=R if e['part']!='pin' else pinR
        goal[:3,3]=[-.07 if i==0 else .07 if i==1 else 0,0 if i<2 else -.07,.06]
        c['insertion_targets'][e['part']]=goal.tolist()
    camera=dict(schema_version=2,units='m',replay_only=True,calibration_kind='simulation_replay',
        replay_source='explicit synthetic RGB-D contract renderer; NOT hardware or v119 held frames',
        calibration_id='SYNTHETIC_CONTRACT_ONLY',camera_frame='synthetic_optical',camera_serial='NOT_A_CAMERA',cell_frame='synthetic_cell',
        T_cell_camera=[[1,0,0,0],[0,-1,0,0],[0,0,-1,1],[0,0,0,1]],
        robots={s:dict(base_frame='synthetic_'+s,T_cell_base=np.eye(4).tolist()) for s in ('left','right')},
        part_robot=dict(ring1='right',ring2='right',pin='right'),
        basket=dict(floor_origin_cell_m=[0,0,0],basket_to_cell_R=np.eye(3).tolist(),outer_bounds_cell_m=[[-.25,-.25,-.01],[.25,.25,.3]]),
        held_workspace_bounds_cell_m=[[-.25,-.25,-.01],[.25,.25,.4]],max_observation_age_s=2,
        pin_holder=dict(enabled=True,T_cell_pin_seed=pose(0,-.07,.08)))
    seed=np.array(camera['pin_holder']['T_cell_pin_seed']);seed[:3,:3]=pinR;camera['pin_holder']['T_cell_pin_seed']=seed.tolist()
    write(HERE/'camera.json',camera);cfg=load_random_config(HERE/'camera.json')
    write(HERE/'site.json',dict(gripper_actions={e['part']:e['grasp_families'][0]['actions'] for e in c['parts']}))
    write(HERE/'planner-contract-only.json',dict(contract_only=True,scene=dict(require_basket_observation=False),
        notes='No real robot model or certified collision scene. Dry replay fakes these devices. Cannot validate or execute live.'))
    manifest=dict(schema_version=1,replay=True,source='SYNTHETIC_CONTRACT_RGBD_NOT_HARDWARE',frames=[],
        initial_tcp={s:pose(0,0,.25) for s in ('left','right')},initial_hand_width_m={s:.07 for s in ('left','right')})
    class Recorder(FakeRuntime):
        def __init__(self):super().__init__(HERE,c);self.tcps=copy.deepcopy(manifest['initial_tcp'])
        def object_pose(self,part):
            for side,h in self.hands.items():
                if h['part']==part:return (np.array(self.tcps[side])@np.array(h['G'])).tolist()
            if self.signals[part+'_retained']:return self.recipe['insertion_targets'][part]
            value=np.eye(4);value[:3,:3]=R if part!='pin' else pinR
            value[:3,3]=[-.08 if part=='ring1' else .08 if part=='ring2' else 0,0 if part!='pin' else -.07,.035 if part!='pin' else .08]
            return value.tolist()
        def observe(self,scope,part,*,side=None,previous=None):
            self.ready();self.counter+=1
            if scope=='basket':
                names=[p for p in ('ring1','ring2') if not self.signals[p+'_retained'] and not any(h['part']==p for h in self.hands.values())]
            else:names=[part]
            print('synthetic actual',self.counter,[(name,np.asarray(self.object_pose(name))[:3,3].tolist()) for name in names],flush=True)
            packets=[render_part_packet(p,self.object_pose(p),cfg,width=640,height=640,focal=2000.) for p in names]
            if not packets:raise RuntimeError('No synthetic target image invented after basket became empty')
            packet=packets[0]
            for other in packets[1:]:
                mask=other['depth_m']<packet['depth_m']
                for key in ('rgb','depth_m','points_camera_image_m'):packet[key][mask]=other[key][mask]
            meta=dict(capture_id='synthetic-contract-'+str(self.counter),captured_at_utc=datetime.now(timezone.utc).isoformat(),
                units='m',camera_frame=cfg['camera_frame'],point_cloud_frame=cfg['camera_frame'],calibration_id=cfg['calibration_id'],
                calibration_sha256=cfg['_calibration_sha256'],source='synthetic_rgbd_replay',replay=True,rectified_registered_depth=True)
            file=frames/('%03d-%s-%s.npz'%(self.counter,scope,part));np.savez_compressed(file,**packet);write(file.with_suffix('.json'),meta)
            report=detect_packet(packet,cfg,meta,replay=True,scope=scope,part=part,
                held_tcp_pose=self.tcp(side) if scope=='held' else None,held_roi_radius_m=.048,
                previous_observation=previous,held_robot=side if scope=='held' else None,
                held_grasp_transform=(previous or {}).get('T_tcp_part') if scope=='held' else None)
            report.update(_scope=scope,_part=part,_side=side,_camera_config_path=str(HERE/'camera.json'))
            reportfile=frames/('%03d-observation.json'%self.counter);write(reportfile,report);report['_path']=str(reportfile)
            manifest['frames'].append(dict(snapshot=str(file.relative_to(HERE)),scope=scope,part=part))
            print('rendered',self.counter,scope,part,flush=True)
            return report
        def execute(self,plan,observation):
            self.ready();self.tcps[plan['side']]=plan['targets'][-1]
            if plan['phase']=='insert':self.signals[plan['part']+'_seated']=True
            self.events.append(('move',plan['phase'],plan['part'],plan['side']));self.ready()
    recorder=Recorder();result=random_cycle.run(recorder,c)
    for key in list(c):
        if key.startswith('_'):c.pop(key)
    write(HERE/'recipe.json',c);write(HERE/'manifest.json',manifest);write(HERE/'build-result.json',result)
    print(json.dumps(result),flush=True)

if __name__=='__main__':main()
