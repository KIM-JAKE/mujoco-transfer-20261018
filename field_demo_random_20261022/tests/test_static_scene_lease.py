"""SDK protocol/test-double boundaries only; no robot calls or real evidence."""
import copy,csv,json,unittest
from unittest.mock import patch
from datetime import timedelta
from pathlib import Path
import test_random_planner as fixtures
from field_demo import static_scene_lease as lease,random_planner as planner,random_cycle

class LeaseTests(unittest.TestCase):
 setUp=fixtures.PlannerTests.setUp
 write=fixtures.PlannerTests.write
 write_observation=fixtures.PlannerTests.write_observation
 plan=fixtures.PlannerTests.plan
 def make(self):
  self.observation['selected_part']='ring1';self.write_observation(self.observation);self.config['scene']['static_scene_lease']=dict(maximum_scene_age_s=12);self.write('random-motion.json',self.config)
  p=self.plan(contact='strict_empty_approach');p['proof']['test_only']=False
  file=Path(p['stage_row']['dynamic_proof_file']);file.chmod(0o644);file.write_text(json.dumps(p['proof']));p['stage_row']['dynamic_proof_sha256']=planner.digest(file)
  t=self.root/'mock-native-telemetry.csv';keys=['index','t_s','robot_s','io_ready','abort_requested','current_errors',*['O_F_ext_hat_K'+str(i)for i in range(6)],*['q'+str(i)for i in range(1,8)],*['q_d'+str(i)for i in range(1,8)]]
  with t.open('w')as f:
   w=csv.DictWriter(f,keys);w.writeheader()
   for index in range(2):
    row={k:0 for k in keys};row.update(index=index,t_s=index*.001,robot_s=10+index*.001,io_ready=1);w.writerow(row)
  driver=self.root/'motion';r=dict(stage_completed=True,telemetry_dropped=0,samples=2,actual_telemetry_file=str(t),actual_telemetry_sha256=planner.digest(t),actual_driver_file=str(driver),actual_driver_sha256=planner.digest(driver),actual_completed_at_utc=(self.now+timedelta(seconds=.001)).isoformat())
  return p,r,lease.create(self.observation,p,r,self.config,now=self.now+timedelta(seconds=.001),validity_file=self.root/'lease-validity.json')
 def test_untouched_map_keeps_original_exposure_and_has_hard_expiry(self):
  p,r,l=self.make();self.assertEqual(l['captured_at_utc'],self.observation['captured_at_utc']);self.assertFalse(l['physical_no_contact_certified']);self.assertEqual(lease.validate(l,now=self.now+timedelta(seconds=5)),12)
  with self.assertRaises(RuntimeError):lease.validate(l,now=self.now+timedelta(seconds=12))
  changed=copy.deepcopy(l);changed['explicit_stock_interaction_occurred']=True
  with self.assertRaises(ValueError):lease.validate(changed,now=self.now)
 def test_force_event_telemetry_change_or_contact_approach_refuses_lease(self):
  p,r,l=self.make();Path(r['actual_telemetry_file']).write_text('changed')
  with self.assertRaises(ValueError):lease.validate(l,now=self.now)
  p['proof']['contact']='grasp'
  with self.assertRaises(ValueError):lease.create(self.observation,p,r,self.config,now=self.now)
 def test_fresh_target_changed_or_new_visible_stock_invalidates(self):
  _,_,l=self.make();fresh=copy.deepcopy(self.observation);fresh['selected_instance']['T_cell_part'][0][3]+=.005
  with self.assertRaisesRegex(RuntimeError,'target changed'):lease.target_agrees(l,fresh,self.config)
  fresh=copy.deepcopy(self.observation);extra=copy.deepcopy(self.instance);extra.update(instance_id='newvisible');extra['T_cell_part'][0][3]+=.01;fresh['instances'].append(extra)
  with self.assertRaisesRegex(RuntimeError,'visible stock'):lease.target_agrees(l,fresh,self.config)
 def test_real_scene_consumer_accepts_only_matching_fresh_target_then_expires(self):
  _,_,l=self.make();fresh=copy.deepcopy(self.observation);stamp=(self.now+timedelta(seconds=3)).isoformat();fresh.update(scope='contact_target',capture_id='fresh-contact',captured_at_utc=stamp,static_scene_lease=l,scene_observations=[copy.deepcopy(self.observation)])
  fresh['source_metadata'].update(capture_id='fresh-contact',captured_at_utc=stamp,frame_number=2);meta=self.write('contact-meta.json',fresh['source_metadata']);fresh.update(source_metadata_file=str(meta),source_metadata_sha256=planner.digest(meta));fresh.pop('_path');fresh.pop('_sha256');path=self.write('contact-observation.json',fresh);fresh.update(_path=str(path),_sha256=planner.digest(path))
  now=self.now+timedelta(seconds=3);deps={};planner.observation_valid(fresh,self.config,self.root,deps,now);packets=planner.scene_packets(fresh,self.config,self.root,deps,now);self.assertEqual(len(packets),1);self.assertEqual(packets[0]['capture_id'],self.observation['capture_id']);self.assertEqual(packets[0]['captured_at_utc'],self.observation['captured_at_utc'])
  with patch('field_demo.collision.check_envelopes',wraps=planner.collision.check_envelopes)as checked:
   planner.geometry_check(self.other,self.other,self.config,self.site,'pickup','left','ring1',fresh,[],self.root,{},now,'grasp',stopped_aperture=True)
   self.assertEqual(checked.call_args.args[3]['_target_instance_id'],l['selected_instance']['instance_id'])
  with self.assertRaisesRegex(ValueError,'only stopped'):
   planner.geometry_check(self.native,self.other,self.config,self.site,'pickup','left','ring1',fresh,[],self.root,{},now,'grasp')
  with self.assertRaises(RuntimeError):planner.scene_packets(fresh,self.config,self.root,{},self.now+timedelta(seconds=12))
 def test_runtime_invalidation_clears_lease_before_later_use(self):
  _,_,l=self.make();r=random_cycle.LiveRuntime.__new__(random_cycle.LiveRuntime);r._static_scene_lease=l;r.event=lambda *a,**kw:None;r.invalidate_scene_lease('actual Hand command');self.assertIsNone(r._static_scene_lease)

if __name__=='__main__':unittest.main()
