"""Distinguish factory/CAD derivation from actual-cell measurement evidence.

A model-derived number is never rewritten as measured. One real installation
record binds the installed model, mount, cable allowance and part-CAD match;
all numeric subjects remain content-bound to the generated preset.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from .calibration import utc_stamp


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def subject_hash(value):
    def strip(v):
        if isinstance(v,dict):return {k:strip(x)for k,x in v.items()if k!='measurement'}
        if isinstance(v,list):return [strip(x)for x in v]
        return v
    value=strip(value)
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def installation(path, *, now=None):
    path=Path(path).resolve();value=json.loads(path.read_text());now=now or datetime.now(timezone.utc)
    if (value.get('schema_version')!=1 or value.get('kind')!='actual_installation_match' or
            value.get('verified')is not True or not str(value.get('by','')).strip() or utc_stamp(value['at_utc'])>now):
        raise ValueError('Complete the ONE actual installed-model/mount/cable/part-CAD match record')
    facts=value.get('facts',{})
    for name,expected in [('robot_model','fr3v2'),('hand_model','franka_hand'),('factory_mount','flange_Rz_minus_45deg'),('part_cad','m80-ring1-ring2-pin'),('part_load_basis','accepted_nominal_CAD_mass_COM_inertia')]:
        if facts.get(name)!=expected:raise ValueError('Unsupported/unconfirmed installation '+name)
    if facts.get('no_unmodeled_adapters')is not True:raise ValueError('Custom adapters require an explicit collision model; factory preset cannot hide them')
    allowance=facts.get('cable_radial_allowance_m')
    if type(allowance)not in(int,float)or not 0<=allowance<=.1:raise ValueError('One actual conservative cable radial allowance in metres is required')
    evidence=(path.parent/value['evidence_file']).resolve()
    if not evidence.is_file()or not evidence.read_bytes().strip():raise ValueError('Nonempty actual installation evidence required')
    return value,evidence


def validate(record,root,*,subject=None,dependencies=None,name='factory_model',now=None):
    if not isinstance(record,dict)or record.get('kind')!='model_derived' or record.get('verified')is not False:
        raise ValueError('Explicit model_derived / verified:false record required')
    root=Path(root);preset=(root/record['model_preset_file']).resolve()
    if digest(preset)!=record['model_preset_sha256']:raise ValueError('Derived model preset changed; regenerate its bound configuration')
    data=json.loads(preset.read_text());entry=data['subjects'][record['subject_key']]
    if record['subject_sha256']!=subject_hash(entry):raise ValueError('Derived subject binding changed')
    if subject is not None and subject_hash(subject)!=record['subject_sha256']:raise ValueError('Numeric model geometry differs from its bound derived preset')
    inst=(preset.parent/data['installation_file']).resolve()
    if digest(inst)!=data['installation_sha256']:raise ValueError('Installed model/mount/cables record changed; regenerate preset')
    _,evidence=installation(inst,now=now)
    for dep in data['source_dependencies']:
        path=(preset.parent/dep['path']).resolve()
        if not path.is_file()or digest(path)!=dep['sha256']:raise ValueError('Factory/CAD/actual-SDK source changed: '+str(path))
    if dependencies is not None:
        for suffix,path in [('preset',preset),('installation',inst),('evidence',evidence)]:dependencies[name+'_'+suffix]=dict(path=str(path),sha256=digest(path))
        for i,dep in enumerate(data['source_dependencies']):
            path=(preset.parent/dep['path']).resolve();dependencies[name+'_source_'+str(i)]=dict(path=str(path),sha256=dep['sha256'])
    return digest(preset)
