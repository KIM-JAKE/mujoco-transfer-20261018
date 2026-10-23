"""Hardware task boundary, with no simulator imports or device I/O at import.

CSV parsing here is identity/input checking. It is NOT collision, FR3 dynamic,
quintic-interpolant or hardware admission. The hardware backend must do those.
"""
from dataclasses import dataclass
from pathlib import Path
import csv,hashlib,io,json,math,time,uuid
PHASES=('perception','grasp','carry','preinsert','fixed_insert','release','withdraw','final_check')
SIDES={'ring1':'right','ring2':'left'}
HEADER=['time_s']+[f'{p}{j}'for p in ('q','dq','ddq')for j in range(1,8)]
def canonical(v):return json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False)
def sha(data):return hashlib.sha256(data).hexdigest()
def finite(v):return type(v)in(int,float)and math.isfinite(v)
def require(ok,message):
 if not ok:raise ValueError(message)
def vector(v,n):return isinstance(v,list)and len(v)==n and all(finite(x)for x in v)
def rigid(T):
 require(isinstance(T,list)and len(T)==4 and all(vector(r,4)for r in T),'Measured finite SE3 required')
 require(max(abs(a-b)for a,b in zip(T[3],[0,0,0,1]))<1e-9,'SE3 last row')
 R=[r[:3]for r in T[:3]]
 require(max(abs(sum(R[k][i]*R[k][j]for k in range(3))-(i==j))for i in range(3)for j in range(3))<1e-6,'SE3 rotation')
 det=sum(R[0][i]*(R[1][(i+1)%3]*R[2][(i+2)%3]-R[1][(i+2)%3]*R[2][(i+1)%3])for i in range(3))
 require(abs(det-1)<1e-6,'Right handed SE3 required')
 return T
def measured_tcp_target(world_part_goal,observed_tcp_part):
 """Same fixed world PART goal @ inverse(current measured TCP_PART), no sim bias."""
 W=rigid(world_part_goal);G=rigid(observed_tcp_part)
 inv=[[G[j][i]for j in range(3)]+[-sum(G[j][i]*G[j][3]for j in range(3))]for i in range(3)]+[[0,0,0,1]]
 return [[sum(W[i][k]*inv[k][j]for k in range(4))for j in range(4)]for i in range(4)]
def validate_csv(data):
 text=data.decode('utf-8');rows=list(csv.reader(io.StringIO(text)))
 require(rows and rows[0]==HEADER,'Exact native time/q/dq/ddq 22-column CSV header required')
 values=[]
 for row in rows[1:]:
  require(len(row)==22,'22 native CSV values required');v=[float(x)for x in row]
  require(all(math.isfinite(x)for x in v),'Nonfinite CSV');values.append(v)
 require(2<=len(values)<=100000 and values[0][0]==0,'CSV starts at zero, at least two rows')
 require(all(b[0]>a[0]for a,b in zip(values,values[1:])),'Strictly increasing native CSV time')
 require(all(abs(x)<1e-12 for row in (values[0],values[-1])for x in row[8:]),'Stopped stage boundaries required')
 return values
@dataclass(frozen=True)
class StageRequest:
 payload_json:str
 csv_bytes:bytes
 @property
 def payload(self):return json.loads(self.payload_json)
 @property
 def digest(self):return sha(self.payload_json.encode())
 @classmethod
 def create(cls,*,side,part,phase,stage_key,csv_path,csv_sha256,current_states,observation,world_part_goal,observed_tcp_part,canonical_end_q,source_sha256,site_config_sha256,request_id=None):
  require(part in SIDES and phase in PHASES and side in ('left','right'),'Explicit arm/ring/phase required')
  require(isinstance(stage_key,str)and stage_key,'Registered stage key required')
  data=Path(csv_path).read_bytes();require(sha(data)==csv_sha256,'CSV bytes changed');rows=validate_csv(data)
  require(set(current_states)=={'left','right'},'Both actual arm/Hand contexts required')
  for other_side,state in current_states.items():
   require(vector(state.get('q'),7)and vector(state.get('q_command'),7),'Actual and command joint states required')
   require(finite(state.get('width_m'))and 0<=state['width_m']<=.08,'Actual Hand width required')
   require(finite(state.get('captured_monotonic_s')),'Actual state capture clock required')
  require(vector(canonical_end_q,7),'Current canonical endpoint required')
  require(max(abs(a-b)for a,b in zip(rows[0][1:8],current_states[side]['q_command']))<=1e-9,'CSV command start mismatch; newly plan actual-to-command bridge')
  require(max(abs(a-b)for a,b in zip(rows[-1][1:8],canonical_end_q))<=1e-9,'CSV current endpoint mismatch')
  require(observation.get('part')==part and observation.get('signed_class_valid')is True,'Current signed ring class evidence required')
  require(observation.get('domain')=='hardware'and observation.get('replay')is False and observation.get('uses_oracle')is False,'Fresh hardware observation only')
  require(observation.get('calibration_measured')is True,'Measured actual camera calibration required')
  for key in ('capture_id','raw_sha256','calibration_sha256'):
   require(isinstance(observation.get(key),str)and bool(observation[key]),'Missing observation '+key)
  require(finite(observation.get('captured_monotonic_s')),'Capture clock required')
  for value in (source_sha256,site_config_sha256):require(isinstance(value,str)and len(value)==64 and all(c in '0123456789abcdef'for c in value),'Source/config SHA256 required')
  payload=dict(schema='capx-ring-hardware-stage/v1',domain='hardware',part=part,side=side,phase=phase,stage_key=stage_key,request_id=request_id or uuid.uuid4().hex,csv_path=str(Path(csv_path).resolve()),csv_sha256=sha(data),csv_interpolation='original_fieldkit_quintic_1kHz',source_sha256=source_sha256,site_config_sha256=site_config_sha256,current_states=current_states,observation=observation,world_part_goal=rigid(world_part_goal),observed_tcp_part=rigid(observed_tcp_part),canonical_end_q=canonical_end_q,T_cell_tcp_target=measured_tcp_target(world_part_goal,observed_tcp_part),simulation_bias_enabled=False)
  return cls(canonical(payload),data)
class RingTaskBridge:
 """Task calls the concrete hardware backend; no in-module device callbacks.

 Backend must register the complete exact request, not caller StageBindings. A
 read-only/config-complete report cannot stand in for admission. No automatic
 Hand opening, clamp release, home or error recovery is performed.
 """
 def __init__(self,backend,*,source_sha256,site_config_sha256,max_sensor_age_s,max_state_age_s,clock=time.monotonic):
  require(finite(max_sensor_age_s)and 0<max_sensor_age_s<=3,'Explicit sensor age <= original3s required')
  require(finite(max_state_age_s)and 0<max_state_age_s<=1,'Measured-state age limit required')
  self.backend=backend;self.source=source_sha256;self.site=site_config_sha256;self.sensor_age=max_sensor_age_s;self.state_age=max_state_age_s;self.clock=clock;self.consumed=set()
 def capture(self):return self.backend.capture()
 def read_context(self):
  result={}
  for side in ('left','right'):
   state=self.backend.read_state(side);hand_begin=self.clock();hand=self.backend.hand_status(side);hand_end=self.clock()
   require(state.get('domain')=='hardware'and state.get('side')==side,'Actual hardware state source required')
   result[side]=dict(q=state['state']['q'],q_command=state['state']['q_d'],width_m=hand['width_m'],captured_monotonic_s=state['request_monotonic_s'],received_monotonic_s=state['received_monotonic_s'],hand_status_request_monotonic_s=hand_begin,hand_status_received_monotonic_s=hand_end,raw_state=state,raw_hand=hand)
  return result
 def hand_status(self,side):
  require(side in ('left','right'),'Actual side required');return self.backend.hand_status(side)
 def hand_move(self,side,action):
  require(side in ('left','right'),'Actual side required')
  # The original hardware wrapper checks its own measured policy and limits.
  # Reject the known incompatible simulation force before delegating.
  if action.get('grasp')is True:require(finite(action.get('force_n'))and 5<=action['force_n']<=20,'Actual NativeHand5..20N; simulation30N/pad is incompatible')
  return self.backend.hand_move(side,action)
 def clamp(self,part,asserted):
  require(part in SIDES and type(asserted)is bool,'Explicit mapped ring clamp required');return self.backend.clamp(part,asserted)
 def execute(self,request):
  require(isinstance(request,StageRequest),'Immutable StageRequest required');p=request.payload;now=self.clock()
  require(p['source_sha256']==self.source and p['site_config_sha256']==self.site,'Task/site source changed')
  require(p['request_id']not in self.consumed,'Single-use stage request')
  require(0<=now-p['observation']['captured_monotonic_s']<=self.sensor_age,'Observation stale/future')
  require(all(0<=now-s['captured_monotonic_s']<=self.state_age for s in p['current_states'].values()),'SDK state stale/future')
  require(sha(Path(p['csv_path']).read_bytes())==p['csv_sha256']==sha(request.csv_bytes),'Immutable CSV changed')
  mapped={k:p[k]for k in ('request_id','side','part','phase','stage_key','csv_path','csv_sha256')}
  mapped.update(capture_id=p['observation']['capture_id'],calibration_id=p['observation']['calibration_sha256'],T_tcp_part=p['observed_tcp_part'],task_context=p)
  registered_digest=sha(canonical(mapped).encode())
  # Consume before registration/call; errors never permit replay of partial I/O.
  self.consumed.add(p['request_id'])
  bound=self.backend.register_stage(mapped)
  require(bound.get('registered')is True and bound.get('collision_approved')is False and bound.get('motion_issued')is False,'Registration cannot impersonate approval')
  require(bound.get('request_sha256')==registered_digest==bound.get('capx_request_sha256')and bound.get('csv_sha256')==p['csv_sha256']and bound.get('side')==p['side']and bound.get('stage_key')==p['stage_key'],'Backend exact registration acknowledgement mismatch')
  return self.backend.execute_stage(p['side'],p['stage_key'])
def validate_bundle(path):
 p=Path(path);data=json.loads(p.read_text(),parse_constant=lambda x:(_ for _ in ()).throw(ValueError('Nonfinite JSON')))
 require(data.get('schema')=='capx-ring-task-bundle/v1','Task bundle schema required')
 blockers=[]
 for key in ('hardware_config','source_sha256','site_config_sha256','measured_camera_record','measured_tool_record','measured_world_geometry','qualified_hardware_admission','stages'):
  if not data.get(key):blockers.append(dict(field=key,reason='Actual measured/qualified site value required'))
 if data.get('simulation_bias_enabled')is not False:blockers.append(dict(field='simulation_bias_enabled',reason='Must be false'))
 return dict(schema='capx-ring-task-offline-preflight/v1',blockers=blockers,local_bundle_complete=not blockers,device_connected=False,motion_issued=False,hardware_ready=False)
