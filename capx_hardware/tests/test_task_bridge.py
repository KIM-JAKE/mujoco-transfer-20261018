import copy,csv,io,json,sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from capx_ring_tasks.bridge import *
I=[[1.,0,0,0],[0,1.,0,0],[0,0,1.,0],[0,0,0,1.]]
class RecordingBackend:
 def __init__(self):self.calls=[];self.bad=False
 def register_stage(self,p):
  self.calls.append(('register',p));d=sha(canonical(p).encode())
  return dict(registered=True,collision_approved=False,motion_issued=False,request_sha256=d,capx_request_sha256=d if not self.bad else '0'*64,side=p['side'],stage_key=p['stage_key'],csv_sha256=p['csv_sha256'])
 def execute_stage(self,side,key):self.calls.append(('execute',side,key));return {'issued':True}
 def read_state(self,side):return dict(side=side,domain='hardware',state=dict(q=[0.]*7,q_d=[0.]*7),request_monotonic_s=9.8,received_monotonic_s=9.9)
 def hand_status(self,side):return {'width_m':.01}
 def hand_move(self,side,action):self.calls.append(('hand',side,action))
 def clamp(self,part,asserted):self.calls.append(('clamp',part,asserted))
 def capture(self):self.calls.append(('capture',));return 'actual driver packet'
class Tests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.path=Path(self.tmp.name)/'stage.csv';self.csv([[0.]*22,[1.]+[0.]*21]);self.backend=RecordingBackend();self.bridge=RingTaskBridge(self.backend,source_sha256='a'*64,site_config_sha256='b'*64,max_sensor_age_s=.2,max_state_age_s=.2,clock=lambda:10.)
  self.kw=dict(side='right',part='ring1',phase='carry',stage_key='ring1_carry',csv_path=self.path,csv_sha256=sha(self.path.read_bytes()),current_states={s:dict(q=[0.]*7,q_command=[0.]*7,width_m=.01,captured_monotonic_s=9.9)for s in ('left','right')},observation=dict(part='ring1',signed_class_valid=True,domain='hardware',replay=False,uses_oracle=False,calibration_measured=True,capture_id='fresh-7',raw_sha256='c'*64,calibration_sha256='d'*64,captured_monotonic_s=9.9),world_part_goal=I,observed_tcp_part=I,canonical_end_q=[0.]*7,source_sha256='a'*64,site_config_sha256='b'*64)
 def csv(self,rows):
  with self.path.open('w')as f:w=csv.writer(f);w.writerow(HEADER);w.writerows(rows)
 def request(self):return StageRequest.create(**self.kw)
 def test_register_execute_exact(self):
  r=self.request();self.kw['current_states']['left']['q'][0]=1;self.assertEqual(self.bridge.execute(r),{'issued':True});self.assertEqual([x[0]for x in self.backend.calls],['register','execute']);self.assertEqual(self.backend.calls[0][1]['task_context']['current_states']['left']['q'][0],0)
  with self.assertRaises(ValueError):self.bridge.execute(r)
 def test_wrong_ack_stops_execute(self):
  self.backend.bad=True
  with self.assertRaises(ValueError):self.bridge.execute(self.request())
  self.assertEqual([x[0]for x in self.backend.calls],['register'])
 def test_stale_future_source_and_mutation(self):
  for key,value in [('captured_monotonic_s',8.),('captured_monotonic_s',11.)]:
   with self.subTest(value=value):
    k=copy.deepcopy(self.kw);k['observation'][key]=value
    with self.assertRaises(ValueError):self.bridge.execute(StageRequest.create(**k))
  r=self.request();self.path.write_bytes(self.path.read_bytes()+b'\n')
  with self.assertRaises(ValueError):self.bridge.execute(r)
  self.assertEqual(self.backend.calls,[])
 def test_malformed_and_simulated_rejected(self):
  for mutation in ('replay','nans','start','end','wrongpart','no-calib'):
   with self.subTest(mutation=mutation):
    k=copy.deepcopy(self.kw)
    if mutation=='replay':k['observation']['replay']=True
    if mutation=='nans':k['observed_tcp_part'][0][0]=float('nan')
    if mutation=='start':k['current_states']['right']['q_command'][0]=.1
    if mutation=='end':k['canonical_end_q'][0]=.1
    if mutation=='wrongpart':k['observation']['part']='ring2'
    if mutation=='no-calib':k['observation']['calibration_measured']=False
    with self.assertRaises(ValueError):StageRequest.create(**k)
 def test_real_readshape_and_io_delegation(self):
  c=self.bridge.read_context();self.assertEqual(c['left']['captured_monotonic_s'],9.8);self.assertEqual(c['left']['q_command'],[0.]*7);self.assertEqual(self.bridge.capture(),'actual driver packet');self.bridge.clamp('ring1',True)
  with self.assertRaises(ValueError):self.bridge.hand_move('right',dict(grasp=True,force_n=30))
  self.bridge.hand_move('right',dict(grasp=True,force_n=10))
 def test_small_right_pickup_and_left_insert_explicit(self):
  for side in ('right','left'):
   k=copy.deepcopy(self.kw);k['part']='ring2';k['observation']['part']='ring2';k['side']=side;k['stage_key']='small_'+side
   request=StageRequest.create(**k);self.bridge.execute(request)
   self.assertEqual(self.backend.calls[-1],('execute',side,'small_'+side))
 def test_same_goal_grasp_inverse(self):
  W=copy.deepcopy(I);W[0][3]=1;G=copy.deepcopy(I);G[0][3]=.2;self.assertAlmostEqual(measured_tcp_target(W,G)[0][3],.8)
 def test_csv_finite_timing_and_stops(self):
  for rows in ([[0.]*22,[0.]*22],[[0.]*22,[1.]+[float('nan')]+[0.]*20],[[0.]*22,[1.]+[0.]*7+[1.]+[0.]*13]):
   self.csv(rows)
   with self.assertRaises(ValueError):validate_csv(self.path.read_bytes())
if __name__=='__main__':unittest.main()
