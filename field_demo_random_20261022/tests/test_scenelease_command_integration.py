"""ONE live-adapter wiring test with mocked camera/arm SDK and local fake Hand.

No network or hardware calls. Real C2 geometry, stopped aperture consumer,
monitors, Hand adapter validation, locking and subprocess handshake are used.
"""
import copy,csv,json,sys,tempfile,threading,time,types,unittest
from pathlib import Path
from datetime import datetime,timezone
from unittest.mock import patch
import test_random_planner as fixtures
from field_demo import random_cycle as policy,random_planner as planner,static_scene_lease as leases
from field_demo.hand_monitor import HandMonitor
from isaac_port.fieldkit.hand import FrankaHand
from isaac_port.fieldkit.site import Site

class SceneLeaseCommandIntegration(unittest.TestCase):
 def test_live_adapter_lease_closed_once_and_all_old_copies_revoked(self):
  case=fixtures.PlannerTests();case.setUp();self.addCleanup(case.doCleanups);root=case.root;events=[];counter=[0]
  case.config['scene']['static_scene_lease']=dict(maximum_scene_age_s=12);case.config['max_observation_age_s']=2
  case.write('random-motion.json',case.config)
  executable=root/'mock-hand-sdk';executable.write_text('#!'+sys.executable+'''\nimport json,os,sys,time
from pathlib import Path
r=Path(__file__).parent;mode=sys.argv[1];side='left'if sys.argv[2]=='192.0.2.1'else'right';p=r/('hand-'+side+'.json');s=json.loads(p.read_text());s['robot_time_s']=time.monotonic()
if mode=='status':print(json.dumps(s));sys.exit(0)
# MOCK native preflight, not hardware: assert revocation precedes dispatch.
v=json.loads(Path((r/'validity-pointer.txt').read_text()).read_text())
if v.get('active')is not False:raise RuntimeError('Map was still active at Hand preflight')
if mode!='--execute-grasp':raise RuntimeError('Unexpected command')
w,speed,force,ei,eo=map(float,sys.argv[3:8])
if not(0<=w<=.08 and 0<speed<=.02 and 5<=force<=20 and 0<ei<=.002 and 0<eo<=.002):raise RuntimeError('Invalid SDK-shaped Hand command')
with(r/'hand-dispatch.jsonl').open('a')as f:f.write(json.dumps({'event':'mock_native_preflight','lease_active':v['active'],'mode':mode})+'\\n')
print('HEARTBEAT_REQUIRED',flush=True)
for line in sys.stdin:
 if line.strip()=='STOP':sys.exit(3)
 if line.strip()=='READY':
  with(r/'hand-dispatch.jsonl').open('a')as f:f.write(json.dumps({'event':'mock_native_dispatch','mode':mode})+'\\n')
  s.update(width_m=w,is_grasped=True,robot_time_s=time.monotonic());tmp=p.with_suffix('.tmp');tmp.write_text(json.dumps(s));os.replace(tmp,p);print(json.dumps(s),flush=True);sys.exit(0)
''');executable.chmod(0o700)
  site=Site.__new__(Site);site.config=copy.deepcopy(case.site);site.config['stages']={};site.filename=root/'site.json';site._control_mutex=threading.RLock();site._control_depth=0;site.expected_held={};site.cancelled=threading.Event();site.process=None;site.plc=None;site.event=lambda name,**kw:events.append((name,kw));site.grippers={}
  for side in ['left','right']:
   (root/('hand-'+side+'.json')).write_text(json.dumps(dict(completed=True,width_m=.08,max_width_m=.08,is_grasped=False,robot_time_s=1.)))
   site.grippers[side]=FrankaHand(dict(ip=site.config['robots'][side]['ip'],binary=executable))
  r=policy.LiveRuntime.__new__(policy.LiveRuntime);r.site=site;r.root=root;r.output=root/'runtime';r.output.mkdir();r.recipe=dict(_root=str(root),planner_config='random-motion.json',max_observation_age_s=2,target_position_tolerance_m=.001,target_angle_tolerance_rad=.01,release_width_tolerance_m=.001);r.static_scene_options=dict(maximum_scene_age_s=12);r._static_scene_lease=None;r.scene_epoch='mock-unchanged-stock';r.installed_parts=[];r.width_aware=True;r._active_aperture_binding=None;r._stopped_arm_monitors={};r._latest_plan_context=None;r.payload=types.SimpleNamespace(valid=True);r._base_ready=lambda:None;r.monitor=lambda:None;r.hand_monitors={};r._raw_hand_status={};r.carried_state=lambda:[];r._selected_occupancy=lambda observation:observation
  self.addCleanup(lambda:[m.close()for m in r.hand_monitors.values()])
  for side,g in site.grippers.items():
   raw=g.status;monitor=HandMonitor(raw);monitor.publish(raw());r.hand_monitors[side]=monitor;g.status=monitor.status
  site.require_ready=r.ready
  def sdk(side):
   state=copy.deepcopy(case.native['state']);state.update(read_only=True,server_protocol_version=10,robot_mode_name='idle',robot_time_s=time.monotonic(),O_F_ext_hat_K=[0.]*6);return state
  r.sdk_state=sdk
  def observe(scope,part,*,side=None,previous=None):
   counter[0]+=1;stamp=datetime.now(timezone.utc).isoformat();ident='mock-camera-'+str(counter[0]);ob=copy.deepcopy(case.observation);ob.update(state='random_ring_observation',scope=scope,_scope=scope,_part=part,_side=side,selected_part=part,ring_pose_seed_used=False,capture_id=ident,captured_at_utc=stamp);ob['source_metadata'].update(capture_id=ident,captured_at_utc=stamp,frame_number=counter[0]);ob['selected_instance']['instance_id']=ident+':0';meta=case.write(ident+'.json',ob['source_metadata']);ob.update(source_metadata_file=str(meta),source_metadata_sha256=planner.digest(meta));ob.pop('_path',None);ob.pop('_sha256',None);return r._record(ob)
  r.observe=observe
  def native_plan(phase,part,side,targets,observation,**kw):
   request=dict(phase=phase,part=part,robot=side,T_cell_tcp_targets=targets,observation=observation,held_parts=kw['held_parts'],contact=kw['contact'],aperture_binding=kw['aperture_binding'])
   p=planner.plan_phase(request,root/'random-motion.json',root,native_plan=lambda *_:copy.deepcopy(case.native),other_state=copy.deepcopy(case.other));p['proof']['test_only']=False # Explicit protocol test double exercises the live-shaped path; no hardware evidence.
   f=Path(p['proof_path']);f.chmod(0o644);f.write_text(json.dumps(p['proof']));p['stage_row']['dynamic_proof_sha256']=planner.digest(f);return p
  def revalidate(p,observation=None):
   planner.geometry_check(case.native,case.other,case.config,case.site,'pickup','left','ring1',observation,[],root,{},datetime.now(timezone.utc),'strict_empty_approach',aperture_binding=p['proof']['aperture_binding']);p['stage_row'].update(dynamic_observation_file=observation['_path'],dynamic_observation_sha256=observation['_sha256'])
  r.planner=types.SimpleNamespace(plan=native_plan,revalidate=revalidate)
  def arm_motion(side,mode,target):
   events.append(('mock_arm_dispatch',{}));path=root/'mock-native-telemetry.csv';keys=['index','t_s','robot_s','io_ready','abort_requested','current_errors',*['O_F_ext_hat_K'+str(i)for i in range(6)],*['q'+str(i)for i in range(1,8)],*['q_d'+str(i)for i in range(1,8)]]
   with path.open('w')as f:
    writer=csv.DictWriter(f,keys);writer.writeheader()
    for index in range(2):
     row={k:0 for k in keys};row.update(index=index,t_s=index*.001,robot_s=100+index*.001,io_ready=1);writer.writerow(row)
   driver=root/'motion';return dict(stage_completed=True,telemetry_dropped=0,samples=2,actual_telemetry_file=str(path),actual_telemetry_sha256=planner.digest(path),actual_driver_file=str(driver),actual_driver_sha256=planner.digest(driver),actual_completed_at_utc=datetime.now(timezone.utc).isoformat())
  site.motion=arm_motion
  observation=r.observe('basket','ring1',side='left');plan=r.plan('pickup','ring1','left',[case.native['samples'][-1]['tcp_base']],observation,[],contact=r.pickup_contact('ring1'));r.execute(plan,observation);old_lease=copy.deepcopy(r._static_scene_lease);self.assertIsNotNone(old_lease);(root/'validity-pointer.txt').write_text(old_lease['validity_file'])
  tickets=[];actual_ticket=leases.ClosureAdmission
  def ticket(proof):value=actual_ticket(proof);tickets.append(value);return value
  with patch.object(planner,'run_native',side_effect=lambda *_a,**_kw:copy.deepcopy(case.other)),patch.object(leases,'ClosureAdmission',side_effect=ticket):
   result=r.hand_move('left','ring1','grasp',dict(width_m=.075,speed_m_s=.005,force_n=5,epsilon_inner_m=.0005,epsilon_outer_m=.0005))
  self.assertTrue(result['is_grasped']);self.assertIsNone(r._static_scene_lease);self.assertEqual(sum(name=='mock_arm_dispatch'for name,_ in events),1)
  commands=[json.loads(x)for x in(root/'hand-dispatch.jsonl').read_text().splitlines()];self.assertEqual([x['event']for x in commands],['mock_native_preflight','mock_native_dispatch']);self.assertFalse(commands[0]['lease_active']);self.assertEqual(len(tickets),1)
  with self.assertRaisesRegex(RuntimeError,'already consumed'):tickets[0].consume()
  with self.assertRaisesRegex(RuntimeError,'revoked'):leases.validate(old_lease)
  proof=tickets[0].proof;stale=planner.load(proof['observation_file']);stale.update(_path=proof['observation_file'],_sha256=proof['observation_sha256'])
  with self.assertRaisesRegex(RuntimeError,'revoked'):planner.observation_valid(stale,case.config,root,{},datetime.now(timezone.utc))
  with self.assertRaisesRegex(RuntimeError,'revoked'):actual_ticket(proof)
  with patch.object(planner,'run_native',side_effect=AssertionError('Revoked proof must fail before another SDK preflight')):
   with self.assertRaisesRegex(RuntimeError,'revoked'):planner.check_stopped_aperture(root/'random-motion.json',site,'pickup','ring1','left',stale,[],proof['aperture_binding'],contact='grasp')
  self.assertEqual(len((root/'hand-dispatch.jsonl').read_text().splitlines()),2)

if __name__=='__main__':unittest.main()
