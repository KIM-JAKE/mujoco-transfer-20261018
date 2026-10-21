"""Deterministic RGB-D + native SDK facts, with explicit evidence provenance.

No part PLC inputs are fabricated. Mechanical clamping is separately confirmed
by an actual mapped PLC or a recorded operator action. Optical seating/retention
is distinct from a contact-force or clamp-pressure measurement.
"""
from datetime import datetime,timezone
import copy,time
import numpy as np
from .calibration import rigid,utc_stamp

class SensorFacts:
 def __init__(self,recipe,*,clock=lambda:datetime.now(timezone.utc)):
  self.recipe=recipe;self.clock=clock;self.held={};self.seats={};self.clamps={};self.retained={};self.releases={};self.transfers={};self.records=[]
 def observation(self,observation,part):
  stamp=utc_stamp(observation['captured_at_utc']);age=(self.clock()-stamp).total_seconds()
  if not 0<=age<=self.recipe['max_observation_age_s']:raise RuntimeError('Fresh RGB-D evidence unavailable')
  if not observation.get('capture_id')or not observation.get('calibration_sha256'):raise RuntimeError('Bound camera capture/calibration required')
  selected=observation.get('selected_instance');values=[v for v in observation.get('instances',[])if v.get('part')==part]
  if selected is None:
   if len(values)!=1:raise RuntimeError('Part is not uniquely observable')
   selected=values[0]
  if selected.get('part')!=part or not any(v.get('instance_id')==selected.get('instance_id')and v.get('T_cell_part')==selected.get('T_cell_part')for v in values):raise RuntimeError('Selected part is not this image measurement')
  pose=rigid(selected['T_cell_part']);return pose,dict(capture_id=observation['capture_id'],captured_at_utc=observation['captured_at_utc'],calibration_sha256=observation['calibration_sha256'],part=part,T_cell_part=pose.tolist(),image_record=observation.get('_path'))
 @staticmethod
 def sdk_grasp(status):
  if status.get('is_grasped')is not True or not np.isfinite(status.get('width_m',np.nan))or not 0<status['width_m']<=.08:raise RuntimeError('Native SDK does not confirm grasp')
 def confirm_held(self,part,side,observation,G,status):
  self.sdk_grasp(status);_,record=self.observation(observation,part);rigid(G)
  record.update(source='fresh_RGBD_native_SDK',side=side,T_tcp_part=rigid(G).tolist(),SDK_width_m=status['width_m']);self.held[side]=record;self.records.append(copy.deepcopy(record));return record
 def grasped(self,part,side,status):
  return bool(side in self.held and self.held[side]['part']==part and status.get('is_grasped')is True)
 def confirm_release(self,part,side,status,minimum_width):
  if status.get('is_grasped')is not False or not np.isfinite(status.get('width_m',np.nan))or status['width_m']<minimum_width:raise RuntimeError('Native SDK does not confirm opening/release')
  record=dict(part=part,side=side,released_at_utc=self.clock().isoformat(),SDK_width_m=status['width_m'],source='native_SDK_measured_opening');self.releases[side]=record;self.held.pop(side,None);return record
 def confirm_transfer(self,part,donor,receiver,observation,G,donor_status,receiver_status,minimum_donor_width):
  self.confirm_held(part,receiver,observation,G,receiver_status);release=self.releases.get(donor)
  if release is None or release['part']!=part:raise RuntimeError('Measured donor release evidence missing')
  if donor_status.get('is_grasped')is not False or donor_status['width_m']<minimum_donor_width:raise RuntimeError('Donor has not actually released')
  _,image=self.observation(observation,part)
  if utc_stamp(image['captured_at_utc'])<utc_stamp(release['released_at_utc']):raise RuntimeError('Receiver image predates donor release')
  self.transfers[part]=dict(donor=donor,receiver=receiver,capture_id=image['capture_id'],source='postrelease_RGBD_receiver_SDK_donor_SDK');return self.transfers[part]
 def transfer_confirmed(self,part,donor,receiver):
  value=self.transfers.get(part,{});return value.get('donor')==donor and value.get('receiver')==receiver
 def target(self,part,observation):
  pose,record=self.observation(observation,part);goal=rigid(self.recipe['insertion_targets'][part]);position=float(np.linalg.norm(pose[:3,3]-goal[:3,3]));angle=float(np.arccos(np.clip(pose[:3,2]@goal[:3,2],-1,1)))
  if position>self.recipe['insert_pose_confirmation_tolerance_m']or angle>self.recipe['target_angle_tolerance_rad']:raise RuntimeError('RGB-D part center/axis does not confirm fixture seating')
  record.update(position_error_m=position,axis_error_rad=angle,source='RGBD_fixture_center_axis');return record
 def confirm_seated(self,part,observation):
  record=self.target(part,observation);self.seats[part]=record;self.records.append(copy.deepcopy(record));return record
 def confirm_clamp(self,part,*,source,evidence):
  if part not in self.seats or source not in ['actual_PLC','operator_manual_clamp']or not evidence:raise RuntimeError('Actual clamp action/confirmation required')
  self.clamps[part]=dict(source=source,evidence=copy.deepcopy(evidence),at_utc=self.clock().isoformat());return self.clamps[part]
 def confirm_retained(self,part,side,observations,status):
  if part not in self.clamps or part not in self.seats or status.get('is_grasped')is not False:raise RuntimeError('Released Hand, seating and explicit clamp confirmation required')
  release=self.releases.get(side)
  if release is None or release['part']!=part:raise RuntimeError('No measured release')
  if len(observations)<2:raise RuntimeError('Distinct postrelease images required')
  rows=[self.target(part,o)for o in observations];times=[utc_stamp(v['captured_at_utc'])for v in rows]
  if len({r['capture_id']for r in rows})!=len(rows)or min(times)<utc_stamp(release['released_at_utc'])or (max(times)-min(times)).total_seconds()<float(self.recipe.get('retention_observation_dwell_s',.3)):raise RuntimeError('Postrelease optical persistence not confirmed')
  self.retained[part]=dict(source='postrelease_RGBD_persistence',observations=rows,clamp=copy.deepcopy(self.clamps[part]));return self.retained[part]
 def fixture_fact(self,part,suffix):
  return part in {'seated':self.seats,'clamped':self.clamps,'retained':self.retained}[suffix]
