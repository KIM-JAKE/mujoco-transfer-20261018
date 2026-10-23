"""Complete OFFLINE site schema exercises original validators, never hardware."""
from datetime import datetime,timezone
from pathlib import Path
import copy,json,tempfile,unittest
from unittest.mock import patch
import numpy as np
from capx_autoring.offline import fixture_recipe,pose
from capx_autoring.config import initialize,pin,preflight
from capx_autoring.admission import CurrentAdmission
from hardware_v1.backend import MotionDisabled
from hardware_v1.config import sha
from capx_deploy.cli import initialize as initialize_hardware

def complete_fixture(root):
 initialize_hardware(root);root=Path(root);initialize(root)
 def write(n,d):p=root/n;p.write_text(json.dumps(d,indent=2));return p
 for n in ['evidence','motion','state','hand','planner','payload','checker']:(root/n).write_text('OFFLINE ONLY: never real evidence or executable')
 record={'path':'evidence','sha256':sha(root/'evidence')};measurement={'verified':True,'by':'OFFLINE UNIT FIXTURE','method':'synthetic test only','at_utc':'2020-01-01T00:00:00+00:00','evidence_file':'evidence'}
 recipe=fixture_recipe();action=recipe.pop('_actions');recipe.update(schema_version=2,recognition='seed_free_rgbd',site_config='site.json',camera_config='rings.camera.json',planner_config='rings.planner.json',commissioning=measurement,max_task_s=60,held_roi_radius_m=.03,payload_profile='rings.payload.json',clamp_control={'mode':'actual_PLC'})
 for part in recipe['parts']:
  for family in part['grasp_families']:family['measurement']=measurement
  part['payload_dimensions']['measurement']=measurement
 def mass(m,frame):return {'units':'kg,m,kg*m^2','inertia_about':'center_of_mass','frame':frame,'mass_kg':m,'center_m':[0.,0.,0.],'inertia_column_major_kg_m2':(np.eye(3)*(1e-4 if m else 0.)).ravel(order='F').tolist(),'measurement':measurement}
 profile={'schema_version':1,'driver_binary':'payload','robots':{s:{'tool':mass(.5,'flange'),'empty_payload':mass(0.,'flange')}for s in ['left','right']},'parts':{p:mass(.1,'part')for p in ['ring1','ring2']}}
 write('rings.payload.json',profile);recipe['payload_profile_sha256']=sha(root/'rings.payload.json');write('rings.recipe.json',recipe)
 site=json.loads((root/'site.json').read_text());site.update(schema_version=1,motion_binary='motion',state_binary='state',calibration_record='evidence',commissioning_record='evidence',part_evidence_source='RGBD_SDK',abort_hand_policy='preserve_completed_grasp',gripper_actions={p:action for p in ['ring1','ring2']})
 site['commissioning']={k:True for k in ['workspace_checked','stop_tested','tool_and_payload_checked','single_arm_test_area_clear']}
 for i,side in enumerate(['left','right']):site['robots'][side]={'ip':f'192.0.2.{i+1}','T_cell_base':pose(),'tool_mass_kg':.5,'load_mass_kg':0.};site['grippers'][side]={'type':'franka_hand','binary':'hand'}
 site['plc']={'endpoint':{'host':'192.0.2.3','port':502},'signals':{name:{'kind':'coil','address':i,'active':True}for i,name in enumerate(['cell_ready','cell_fault','cell_stop','ring1_clamped','ring2_clamped','ring1_clamp_command','ring2_clamp_command'])}}
 write('site.json',site)
 bases={s:{'base_frame':s+'_base','T_cell_base':pose()}for s in ['left','right']}
 camera={'schema_version':2,'units':'m','replay_only':False,'calibration_id':'OFFLINE','camera_frame':'optical','camera_serial':'OFFLINE','cell_frame':'cell','T_cell_camera':pose(),'measured':measurement,'robots':bases,'part_robot':{'ring1':'right','ring2':'left','pin':'right'},'basket':{'floor_origin_cell_m':[0,0,0],'outer_bounds_cell_m':[[-1,-1,-1],[1,1,1]],'basket_to_cell_R':np.eye(3).tolist(),'measured':measurement},'max_observation_age_s':1.}
 profilecam={'K':np.eye(3).tolist(),'width':32,'height':32,'fps':15};camera['camera_profile']=profilecam;write('rings.camera.json',camera)
 cal=copy.deepcopy(camera);cal['schema_version']=1;write('camera.json',cal)
 def box():return {'T_cell_box':pose(),'half_extents_m':[1.,1.,1.]}
 envelope={'measurement':measurement,'whole_robot_hand_tool_cables_enclosed':True,'link_radii_m':[.01]*8,'tool_in_link_envelopes':True,'tool_capsules_tcp_m':[]}
 planner={'schema_version':2,'units':'m','site_config':'site.json','planner_binary':'planner','camera_configs':{'basket':'rings.camera.json'},'measurement':measurement,'collision_margin_m':.001,'max_observation_age_s':1.,'max_state_read_s':10.,'other_arm_q_tolerance_rad':.001,'target_position_tolerance_m':.001,'target_angle_tolerance_rad':.01,'robots':{s:envelope for s in ['left','right']},'scene':{'measurement':measurement,'complete_within_workspace':True,'unobserved_geometry_enclosed':True,'require_residual_occupancy':True,'workspace':box(),'obstacles':[box()]},'phases':{'pickup':{}}}
 write('rings.planner.json',planner)
 hw=json.loads((root/'hardware.json').read_text());hw.update(motion_enabled=True,site_config_sha256=sha(root/'site.json'),T_cell_fixture=pose(),geometry_record=record)
 hw['tools']={s:{'T_flange_tcp':pose(),'measurement':record}for s in ['left','right']};hw['binaries']={n:{'path':n,'sha256':sha(root/n)}for n in ['motion','state','hand']};hw['camera']={'type':'realsense','calibration':'camera.json','calibration_sha256':sha(root/'camera.json'),**profilecam,'max_age_s':.2,'intrinsic_tolerance_px':1e-6};hw['admission']={'hardware_qualified':True,'executable':{'path':'checker','sha256':sha(root/'checker')}};write('hardware.json',hw)
 pin(root/'autoring.json');return root/'autoring.json'

class CompleteConfigTests(unittest.TestCase):
 def test_real_validators_allow_complete_schema_without_any_device_construction(self):
  with tempfile.TemporaryDirectory()as parent:
   path=complete_fixture(Path(parent)/'site')
   with patch('socket.socket',side_effect=AssertionError('No network')),patch('subprocess.run',side_effect=AssertionError('No subprocess')),patch('subprocess.Popen',side_effect=AssertionError('No native process')):
    report=preflight(path)[0];self.assertEqual(report['blockers'],[]);self.assertTrue(report['local_config_complete']);self.assertFalse(report['hardware_ready']);gate=CurrentAdmission(path);self.assertIsNone(gate.backend._site)
 def test_complete_config_still_requires_explicit_motion_enable_and_same_site(self):
  with tempfile.TemporaryDirectory()as parent:
   path=complete_fixture(Path(parent)/'site');hw=path.parent/'hardware.json';d=json.loads(hw.read_text());d['motion_enabled']=False;hw.write_text(json.dumps(d));pin(path)
   self.assertTrue(preflight(path)[0]['local_config_complete'])
   with self.assertRaises(MotionDisabled):CurrentAdmission(path)

if __name__=='__main__':unittest.main()
