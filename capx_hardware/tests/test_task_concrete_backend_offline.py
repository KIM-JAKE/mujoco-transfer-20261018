"""Actual two-package integration. No device SDK import, connection or actuation."""
import hashlib,json,os,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
# Relocatable release puts hardware_v1 beside this project. Development override
# is explicit, never a device endpoint or an import from a simulator.
root=Path(os.environ.get('CAPX_HARDWARE_BACKEND_ROOT',Path(__file__).resolve().parents[2]))
if not(root/'hardware_v1/backend.py').is_file():
 root=Path(__file__).resolve().parents[3]/'capx-fr3-adapter-20261023'
if (root/'hardware_v1/backend.py').is_file():sys.path.insert(0,str(root))
from capx_ring_tasks.bridge import StageRequest,RingTaskBridge,canonical,sha,HEADER
I=[[1.,0,0,0],[0,1.,0,0],[0,0,1.,0],[0,0,0,1.]]
@unittest.skipUnless((root/'hardware_v1/backend.py').is_file(),'Concrete hardware_v1 must be installed/copied beside release')
class Integration(unittest.TestCase):
 def test_real_registration_then_motion_disabled(self):
  import csv
  from hardware_v1.backend import HardwareBackend,MotionDisabled
  with tempfile.TemporaryDirectory()as d:
   p=Path(d);csvpath=p/'path.csv'
   with csvpath.open('w')as f:w=csv.writer(f);w.writerow(HEADER);w.writerows([[0.]*22,[1.]+[0.]*21])
   csvsha=sha(csvpath.read_bytes());site=p/'site.json';site.write_text(json.dumps(dict(stages=dict(carry=dict(side='right',csv='path.csv',sha256=csvsha,tool_type='franka_hand',field_path_reviewed=True)))))
   sitesha=sha(site.read_bytes());config=p/'hardware.json';config.write_text(json.dumps(dict(schema='capx-fr3-hardware/v1',task_source_sha256='a'*64,site_config='site.json',site_config_sha256=sitesha,motion_enabled=False)))
   backend=HardwareBackend(config)
   bridge=RingTaskBridge(backend,source_sha256='a'*64,site_config_sha256=sitesha,max_sensor_age_s=.2,max_state_age_s=.2,clock=lambda:10.)
   r=StageRequest.create(side='right',part='ring1',phase='carry',stage_key='carry',csv_path=csvpath,csv_sha256=csvsha,current_states={s:dict(q=[0.]*7,q_command=[0.]*7,width_m=.01,captured_monotonic_s=9.9)for s in ('left','right')},observation=dict(part='ring1',signed_class_valid=True,domain='hardware',replay=False,uses_oracle=False,calibration_measured=True,capture_id='test-offline-only',raw_sha256='c'*64,calibration_sha256='d'*64,captured_monotonic_s=9.9),world_part_goal=I,observed_tcp_part=I,canonical_end_q=[0.]*7,source_sha256='a'*64,site_config_sha256=sitesha)
   with self.assertRaises(MotionDisabled):bridge.execute(r)
   # No driver object was even loaded; exact taskcontext is in the real backend.
   self.assertIsNone(backend._loader);self.assertIsNone(backend._site);self.assertIsNone(backend._camera)
   raw=backend._stages[('right','carry')];mapped=json.loads(raw)
   self.assertEqual(mapped['task_context'],r.payload)
   self.assertEqual(backend._runtime_rows[('right','carry')]['capx_request_sha256'],sha(raw.encode()))
   with self.assertRaises(ValueError):bridge.execute(r)
if __name__=='__main__':unittest.main()
