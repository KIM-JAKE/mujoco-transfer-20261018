"""Reuse one complete LEFT admission before ANY intervening physical step.

Only the task's explicitly sealed, immutable model/world contract is eligible.
No source/CAD/calibration edit is allowed within this planning epoch. Changed
public encoders/commands, payload/binding, installed geometry or policy miss.
"""
import hashlib,json
import numpy as np
CACHE='_last_loaded_left_issued_full_proof'


def _array(h,value):
    a=np.ascontiguousarray(value);h.update(a.dtype.str.encode());h.update(repr(a.shape).encode());h.update(a.tobytes())


def _json(h,value):
    h.update(json.dumps(value,sort_keys=True,separators=(',',':'),default=lambda x:np.asarray(x).tolist()).encode())


def _geometry(h,rows):
    for body,source,shape in rows:
        _json(h,[body,source])
        for value in shape:_array(h,value)


def _binding(h,binding):
    if binding is None:h.update(b'no-binding');return
    _json(h,dict(identity=id(binding),active=getattr(binding,'active',None),side=getattr(binding,'side',None),part=getattr(binding,'part',None),payload_path=getattr(binding,'payload_path',None),speed_cap=getattr(binding,'speed_cap_m_s',None),dt=getattr(binding,'dt',None),record=getattr(binding,'record',None)))
    for name in['lower','upper']:
        if hasattr(binding,name):_array(h,getattr(binding,name))


def key(c,part,G,issued,allow_fixture_contact):
    """Must run INSIDE current_query after its native binding.check()."""
    epoch=c.get('_collision_physics_epoch')
    if not isinstance(epoch,int)or epoch<0:return None,'no_explicit_physics_epoch'
    sealed=[]
    for name in['fixed_prevalidated_routes','fixed_prevalidated_small_left_routes']:
        state=c.get(name)
        if isinstance(state,dict)and state.get('certificate'):
            sealed.append((name,state['world_sha256'],state['model_sha256'],state['certificate'].metadata))
    if not sealed:return None,'no_sealed_immutable_world_model'
    h=hashlib.sha256(b'loaded-left-full-proof-one-physics-epoch-v1')
    _json(h,[id(c),epoch,part,bool(allow_fixture_contact),float(c['dt']),sealed,c.get('_collision_world_revision',0),c.get('_collision_calibration_revision',0),c['config'],c['goals'],c['part_collision_templates']])
    _array(h,issued);_array(h,G);_array(h,c['cmd']);_array(h,c['art'].get_dof_positions())
    for side in['left','right']:
        checker=c['checkers'][side];arm=checker.arm;guard=getattr(checker,'same_arm_clearance',None)
        _json(h,[side,id(checker),id(arm),getattr(checker,'resolution_rad',.035),getattr(checker,'payload_path',None),sorted(getattr(checker,'ignored_future_actors',set()))])
        for name in['base','tip','frames','axes','bounds']:
            if hasattr(arm,name):_array(h,getattr(arm,name))
        _geometry(h,checker.pair_boxes);_geometry(h,checker.payload_boxes)
        if checker.payload_relative is not None:_array(h,checker.payload_relative)
        _binding(h,getattr(checker,'loaded_contact_aperture',None))
        if guard is not None:
            _json(h,[id(guard),getattr(guard,'margin_m',None),getattr(guard,'pairs',None),getattr(guard,'geometries',None)is not None,getattr(guard,'broadphase_enabled',None)])
            _geometry(h,getattr(guard,'rows',[]))
        placed=getattr(checker,'placed_part_clearance',None)
        _json(h,[id(placed),getattr(placed,'generation',None)])
        for label,row in sorted(getattr(placed,'parts',{}).items()):
            _json(h,[label,row['uncertainty_m']])
            for shape in row['placed']:
                for value in shape:_array(h,value)
    return h.digest(),'eligible'
