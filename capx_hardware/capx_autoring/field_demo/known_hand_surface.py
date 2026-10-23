"""Observed native-Hand surface association, never a free-space inference.

A matching occupied voxel may be ignored by its SAME SDK Hand body only.
Unknown/occluded voxels, mixed stock hits, other bodies/arms and payloads remain
blocking. The producer does not alter an OccupancySnapshot or its fingerprint.
"""
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib,json
import numpy as np
from scipy.spatial import cKDTree
from .native_hand_geometry import actual_hand_points,surfaces
from .calibration import rigid


def _stamp(value):
    t=value if isinstance(value,datetime)else datetime.fromisoformat(value.replace('Z','+00:00'))
    if t.tzinfo is None:raise ValueError('Timezone-aware capture/SDK timestamp required')
    return t.astimezone(timezone.utc)


def _json_value(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError('Only finite JSON/NumPy SDK values are accepted')


def _sha(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False,default=_json_value).encode()).hexdigest()


def _cloud(packet):
    d=np.asarray(packet['depth_m'],float);k=np.asarray(packet['K'],float);t=rigid(packet['T_cell_camera'])
    if d.ndim!=2 or k.shape!=(3,3)or not np.isfinite(k).all()or min(k[0,0],k[1,1])<=0:raise ValueError('Organized rectified camera depth/intrinsics required')
    valid=np.isfinite(d)&(d>.05)&(d<5);xyz=packet.get('points_camera_image_m',packet.get('points_camera_m'))
    if xyz is None:
        y,x=np.indices(d.shape);xyz=np.stack([(x-k[0,2])*d/k[0,0],(y-k[1,2])*d/k[1,1],d],-1)
    else:
        xyz=np.asarray(xyz,float)
        if xyz.shape!=(*d.shape,3):raise ValueError('SDK XYZ must retain actual pixel correspondence')
        valid &= np.isfinite(xyz).all(2)
        if np.any(abs(xyz[:,:,2][valid]-d[valid])>.001):raise ValueError('SDK XYZ/depth discrepancy')
    return xyz@t[:3,:3].T+t[:3,3],valid


@dataclass(frozen=True)
class HandSurfaceAssociation:
    pixel_body:np.ndarray
    owners:tuple
    proof:dict
    @property
    def fingerprint(self):return self.proof['association_sha256']


def associate_hand_pixels(packet,metadata,actual_states,*,surface_tolerance_m=.002,max_sdk_gap_s=.2):
    """states: side/capture_id/T_cell_tcp/F_T_EE/T_flange_hand/width_m,
    captured_at_utc, robot_time_s, source='actual_sdk_fk', pose_uncertainty_m.
    SDK/camera samples must be taken stopped under the same cell-control lock.
    """
    from .residual_occupancy import _source
    source=_source(metadata,packet);tol=float(surface_tolerance_m)
    if not .0002<=tol<=.003 or not 0<max_sdk_gap_s<=.2:raise ValueError('Surface/synchronization uncertainty must be explicitly bounded')
    if not 1<=len(actual_states)<=2:raise ValueError('One or two actual SDK Hand states required')
    points,valid=_cloud(packet);flat=points.reshape(-1,3);valid_ids=np.flatnonzero(valid.ravel());best=np.full(len(flat),np.inf);second=best.copy();labels=np.zeros(len(flat),np.uint8);owners=[];records=[];cloud=surfaces();seen=set();camera=rigid(packet['T_cell_camera']);k=np.asarray(packet['K']);depth=np.asarray(packet['depth_m']);h,w=depth.shape
    for state in actual_states:
        side=state.get('side');uncertainty=float(state.get('pose_uncertainty_m',float('nan')))
        if side not in ('left','right')or side in seen or state.get('source')!='actual_sdk_fk'or state.get('capture_id')!=source['capture_id']:raise ValueError('Unique actual SDK Hand side bound to this exact capture required')
        seen.add(side)
        gap=abs((_stamp(state['captured_at_utc'])-_stamp(source['captured_at_utc'])).total_seconds())
        if gap>max_sdk_gap_s or not np.isfinite(uncertainty)or uncertainty<0 or uncertainty+source['depth_uncertainty_m']>tol:raise ValueError('SDK/camera timing or reported surface uncertainty exceeds bound')
        if state.get('stopped_under_capture_lock')is not True or not np.isfinite(state.get('robot_time_s',float('nan')))or state['robot_time_s']<0:raise ValueError('Actual stopped SDK state/timestamp required')
        hand=actual_hand_points(state['T_cell_tcp'],state['F_T_EE'],state['T_flange_hand'],state['width_m']);counts=[len(cloud['palm_points']),len(cloud['finger_points']),len(cloud['finger_points'])];offset=0
        for body,count in zip(('palm','left_finger','right_finger'),counts):
            surface=hand[offset:offset+count];offset+=count;owners.append((side,body));code=len(owners)
            # Bounds are a KD-query optimization ONLY; no volume is excluded.
            ids=valid_ids[np.all((flat[valid_ids]>=surface.min(0)-tol)&(flat[valid_ids]<=surface.max(0)+tol),axis=1)]
            if not len(ids):continue
            distance=cKDTree(surface).query(flat[ids],workers=1)[0]
            # The observed return must agree with the FRONT model surface on
            # its pixel, not a nearby hidden backside. Dense official samples
            # are splatted to adjacent pixels; Euclidean residual still bounds
            # the association at silhouettes and between model samples.
            cam=(surface-camera[:3,3])@camera[:3,:3];visible=cam[:,2]>.05;cam=cam[visible];uv=cam[:,:2]/cam[:,2:]*[k[0,0],k[1,1]]+[k[0,2],k[1,2]];base=np.floor(uv).astype(int);front=np.full(h*w,np.inf)
            for dx,dy in [(0,0),(1,0),(0,1),(1,1)]:
                pixel=base+[dx,dy];inside=(pixel[:,0]>=0)&(pixel[:,0]<w)&(pixel[:,1]>=0)&(pixel[:,1]<h);np.minimum.at(front,pixel[inside,1]*w+pixel[inside,0],cam[inside,2])
            distance=np.where(abs(depth.ravel()[ids]-front[ids])<=tol,distance,np.inf);improved=distance<best[ids];second[ids]=np.minimum(second[ids],np.where(improved,best[ids],distance));best[ids]=np.minimum(best[ids],distance);labels[ids[improved]]=code
        records.append({key:state[key]for key in ['side','capture_id','captured_at_utc','robot_time_s','width_m','T_cell_tcp','F_T_EE','T_flange_hand','pose_uncertainty_m','source','stopped_under_capture_lock']})
    ambiguity=np.zeros(len(best),bool);finite=np.isfinite(best);ambiguity[finite]=(second[finite]-best[finite])<=.0001
    labels[(best>tol)|ambiguity]=0;labels=labels.reshape(depth.shape);labels.setflags(write=False)
    proof=dict(kind='observed_actual_SDK_native_Hand_surface',capture_id=source['capture_id'],source_packet_sha256=source['source_packet_sha256'],calibration_sha256=source['calibration_sha256'],camera_array_sha256=source['camera_array_sha256'],surface_tolerance_m=tol,max_sdk_capture_gap_s=max_sdk_gap_s,sdk_states_sha256=_sha(records),owners=[list(x)for x in owners],pixel_body_sha256=hashlib.sha256(labels.tobytes()).hexdigest(),matched_pixels=int(np.count_nonzero(labels)),unknown_voxels_changed=0,free_voxels_created=0,uses_hidden_object_poses=False)
    proof['association_sha256']=_sha(proof);return HandSurfaceAssociation(labels,tuple(owners),proof)


def planner_boxes_with_hand_association(snapshot,packet,metadata,association,*,max_boxes=5000,material=None):
    """Split occupied hits only; every unknown voxel remains byte-identical."""
    from .residual_occupancy import OCCUPIED,UNKNOWN,_source,_merged_ranges
    if not isinstance(association,HandSurfaceAssociation):raise TypeError('Typed SDK surface association required')
    source=_source(metadata,packet);proof=association.proof
    raw={key:value for key,value in proof.items()if key!='association_sha256'}
    if _sha(raw)!=association.fingerprint or hashlib.sha256(association.pixel_body.tobytes()).hexdigest()!=proof.get('pixel_body_sha256')or [list(x)for x in association.owners]!=proof.get('owners'):raise ValueError('Modified SDK Hand surface association')
    for key in ['capture_id','source_packet_sha256','calibration_sha256','camera_array_sha256']:
        if source[key]!=proof[key]:raise ValueError('Hand association belongs to another camera capture')
    if len(snapshot.sources)!=1 or snapshot.sources[0]['capture_id']!=source['capture_id']:raise ValueError('Hand association requires a single exact source snapshot; fused views must re-associate all hits')
    points,valid=_cloud(packet);bounds=np.asarray(snapshot.bounds_cell_m);spacing=(bounds[1]-bounds[0])/snapshot.shape;ids=np.flatnonzero(valid.ravel());grid=np.floor((points.reshape(-1,3)[ids]-bounds[0])/spacing).astype(int);inside=((grid>=0)&(grid<snapshot.shape)).all(1);ids=ids[inside];grid=grid[inside];flat=np.ravel_multi_index(grid.T,snapshot.shape);pixel=association.pixel_body.ravel()[ids];n=int(np.prod(snapshot.shape));owner=np.zeros(n,np.uint8);conflict=np.zeros(n,bool)
    for code in range(1,len(association.owners)+1):
        cells=np.unique(flat[pixel==code]);conflict[cells]|=(owner[cells]!=0)&(owner[cells]!=code);owner[cells]=code
    conflict[np.unique(flat[pixel==0])]=True;owner[conflict]=0;owner[snapshot.states.ravel()!=OCCUPIED]=0;owner=owner.reshape(snapshot.shape)
    original=snapshot.planner_boxes(max_boxes=max_boxes);selected=np.frombuffer(snapshot._selected,np.uint8).reshape(snapshot.shape).astype(bool);owner[selected]=0
    # Rebuild only occupied boxes so merged stock/self boxes cannot acquire a
    # blanket waiver. UNKNOWN original boxes are carried over without edits.
    result=[row for row in original if row['occupancy_kind']=='unknown'];source_ids=[source['capture_id']]
    material_proof=None;envelope_cells=np.zeros(snapshot.shape,bool);envelope_prior=False
    if material is not None:
        material_owner,material_owners,material_proof=material;material_owner=material_owner.copy()
        envelope_prior=material_proof.get('kind')=='eroded_pinned_official_collision_envelope_assumption'
        allowed_states=(snapshot.states==UNKNOWN)|(snapshot.states==OCCUPIED)if envelope_prior else snapshot.states==UNKNOWN
        if tuple(material_owners)!=association.owners or material_owner.shape!=snapshot.shape or np.any((material_owner!=0)&~allowed_states):raise ValueError('Known robot prior must bind this exact non-free grid and actual SDK body')
        material_owner[selected]=0  # never reclassify observed selected stock as robot
        if envelope_prior:
            envelope_cells=(material_owner!=0)&(snapshot.states==OCCUPIED)
            owner[envelope_cells]=material_owner[envelope_cells]
        result=[]
        for code in range(len(association.owners)+1):
            mask=(snapshot.states==UNKNOWN)&(material_owner==code)
            for x0,x1,y0,y1,z0,z1 in _merged_ranges(mask):
                low=bounds[0]+spacing*[x0,y0,z0];high=bounds[0]+spacing*[x1,y1,z1];t=np.eye(4);t[:3,3]=(low+high)/2
                kind='known_robot_envelope'if envelope_prior else'known_robot_material'
                row=dict(name='robot_prior_'+snapshot.fingerprint[:12]+'_'+str(len(result)),T_cell_box=t.tolist(),half_extents_m=((high-low)/2).tolist(),source='conservative_registered_RGBD_rays',occupancy_kind='unknown'if code==0 else kind,source_capture_ids=source_ids,voxel_resolution_m=spacing.tolist())
                if code:
                    side,body=association.owners[code-1];row['known_hand_surface']=dict(robot_side=side,hand_body=body,capture_id=source['capture_id'],association_sha256=association.fingerprint,material_source=material_proof['kind'])
                result.append(row)
                if len(result)>max_boxes:raise ValueError('Known robot prior box budget exceeded; external unknown geometry retained')
    for code in range(len(association.owners)+1):
        for eligible in [False,True]:
          for from_envelope in [False,True]:
            mask=(snapshot.states==OCCUPIED)&(owner==code)&(selected if eligible else ~selected)&(envelope_cells if from_envelope else ~envelope_cells)
            for x0,x1,y0,y1,z0,z1 in _merged_ranges(mask):
                low=bounds[0]+spacing*[x0,y0,z0];high=bounds[0]+spacing*[x1,y1,z1];t=np.eye(4);t[:3,3]=(low+high)/2;row=dict(name='associated_'+snapshot.fingerprint[:12]+'_'+str(len(result)),T_cell_box=t.tolist(),half_extents_m=((high-low)/2).tolist(),source='conservative_registered_RGBD_rays',occupancy_kind='known_robot_envelope'if from_envelope else'occupied',source_capture_ids=source_ids,voxel_resolution_m=spacing.tolist())
                if eligible:row.update(selected_contact_instance_id=snapshot.selected_contact_instance_id,selected_contact_scope='Observed selected surface; named fingers only')
                if code:
                    side,body=association.owners[code-1];row['known_hand_surface']=dict(robot_side=side,hand_body=body,capture_id=source['capture_id'],association_sha256=association.fingerprint)
                    if from_envelope:row['known_hand_surface']['material_source']=material_proof['kind']
                result.append(row)
                if len(result)>max_boxes:raise ValueError('Typed surface occupancy box budget exceeded; no geometry was erased')
    return result,dict(association.proof,occupancy_fingerprint_unchanged=snapshot.fingerprint,typed_known_hand_occupied_voxels=int(np.count_nonzero(owner)),unknown_voxels=int(np.count_nonzero(snapshot.states==UNKNOWN)),known_robot_material=material_proof)


def known_hand_contact_mask(obstacle,*,capsule,association_sha256,active_robot=False,is_payload=False):
    """A collision association for this same body, never an UNKNOWN waiver."""
    a=np.asarray(capsule['a']);result=np.zeros(len(a)if a.ndim==2 else 1,bool);tag=obstacle.get('known_hand_surface')
    if not active_robot or is_payload or obstacle.get('source')!='conservative_registered_RGBD_rays'or obstacle.get('occupancy_kind')not in ('occupied','known_robot_material','known_robot_envelope')or not isinstance(tag,dict):return result
    if obstacle.get('occupancy_kind')=='known_robot_envelope'and tag.get('material_source')!='eroded_pinned_official_collision_envelope_assumption':return result
    if obstacle.get('occupancy_kind')=='known_robot_material'and tag.get('material_source')!='strictly_eroded_closed_official_visual_material':return result
    if not isinstance(association_sha256,str)or len(association_sha256)!=64 or tag.get('association_sha256')!=association_sha256 or tag.get('capture_id')not in obstacle.get('source_capture_ids',[]):return result
    if capsule.get('robot_side')==tag.get('robot_side')and capsule.get('hand_body')==tag.get('hand_body')and capsule.get('geometry_role')in ('palm','finger')and tag.get('hand_body')in ('palm','left_finger','right_finger'):result[:]=True
    return result
