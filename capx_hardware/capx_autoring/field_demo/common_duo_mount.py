"""Opt-in fixed link0 pair semantics for the installed official rigid Duo mount.

Only two STATIC base bodies belonging to one verified factory assembly may
share their installed interface. Moving links, payloads and all scene checks
retain their complete original geometry and margins. No device calls.
"""
import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib,json,os
from pathlib import Path
import numpy as np
from .calibration import rigid,utc_stamp
from .model_records import validate as validate_model,subject_hash,installation

ASSETS=Path(__file__).resolve().parents[1]/'isaac_port/fieldkit/assets/common-duo-mount'
MODEL_SHA256='1ca58fabbba5fd4001e2ecc7ec46e67317fc2858dc068f333d808146a5d18009'
KEY='_validated_common_duo_mount'

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def canonical(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def model():
    path=ASSETS/'model.json'
    if digest(path)!=MODEL_SHA256:raise ValueError('Pinned official Duo mount model changed')
    data=json.loads(path.read_text())
    for name,expected in data['source_sha256'].items():
        if digest(ASSETS/name)!=expected:raise ValueError('Pinned official Duo source changed: '+name)
    return data

def _description(config,site,root,dependencies,now):
    official=model();robots={};installations={}
    for side in ['left','right']:
        env=config['robots'][side]
        if env.get('geometry_backend')!='sdk_link_columns':raise ValueError('Common Duo requires the pinned actual-SDK link model')
        validate_model(env.get('measurement'),root,subject=env,dependencies=dependencies,name='common_duo_'+side,now=now)
        record=env['measurement'];preset=(Path(root)/record['model_preset_file']).resolve();data=json.loads(preset.read_text())
        inst=(preset.parent/data['installation_file']).resolve();_,installation_evidence=installation(inst,now=now);installations[side]=dict(path=str(inst),sha256=digest(inst),evidence_sha256=digest(installation_evidence))
        robots[side]=dict(ip=site['robots'][side]['ip'],T_cell_base=rigid(site['robots'][side]['T_cell_base']).tolist(),model_subject_sha256=subject_hash(env),model_preset_sha256=digest(preset))
    if robots['left']['ip']==robots['right']['ip']:raise ValueError('Two distinct actual Duo robot IPs required')
    if installations['left']!=installations['right']:raise ValueError('Both robots must share the same bound installation record')
    relative=np.linalg.inv(np.asarray(robots['left']['T_cell_base']))@np.asarray(robots['right']['T_cell_base']);expected=rigid(official['T_left_base_right_base'])
    position=float(np.linalg.norm(relative[:3,3]-expected[:3,3]));angle=float(np.arccos(np.clip((np.trace(relative[:3,:3]@expected[:3,:3].T)-1)/2,-1,1)))
    if position>official['relative_position_match_tolerance_m']or angle>official['relative_rotation_match_tolerance_rad']:
        raise ValueError('Configured base placement does not match the official common Duo mount')
    return dict(model=official['model'],official_model_sha256=MODEL_SHA256,robots=robots,installation=installations['left'],T_left_base_right_base=relative.tolist(),factory_relative_position_difference_m=position,factory_relative_rotation_difference_rad=angle)

@dataclass(frozen=True)
class CommonDuoBinding:
    proof_sha256: str
    description_sha256: str
    sides: tuple=('left','right')

def binding(config,site,root,dependencies,now):
    """Revalidate all proof/model/base bytes at each original geometry admission."""
    spec=config.get('common_duo_mount')
    if spec is None:return None
    if not isinstance(spec,dict)or set(spec)!={'proof_file','proof_sha256','evidence_sha256'}:raise ValueError('Explicit common Duo proof-file binding required')
    path=(Path(root)/spec['proof_file']).resolve()
    if digest(path)!=spec['proof_sha256']:raise ValueError('Actual common Duo proof changed')
    proof=json.loads(path.read_text())
    if (proof.get('schema_version')!=1 or proof.get('kind')!='actual_official_common_duo_mount' or proof.get('verified')is not True or proof.get('actual_common_rigid_mount_installed')is not True or not isinstance(proof.get('by'),str)or not proof['by'].strip()or utc_stamp(proof['at_utc'])>now):
        raise ValueError('Actual verified installed official common rigid Duo mount evidence required')
    evidence=(path.parent/proof['evidence_file']).resolve()
    if not evidence.is_file()or not evidence.read_bytes().strip()or digest(evidence)!=spec['evidence_sha256']:raise ValueError('Actual common mount evidence is missing or changed')
    description=_description(config,site,root,dependencies,now)
    if proof.get('description')!=description:raise ValueError('Common Duo model, installation or base placement differs from its proof')
    for name,p in [('proof',path),('evidence',evidence),('official_model',ASSETS/'model.json')]:dependencies['common_duo_'+name]=dict(path=str(p),sha256=digest(p))
    for name,expected in model()['source_sha256'].items():dependencies['common_duo_source_'+name]=dict(path=str(ASSETS/name),sha256=expected)
    return CommonDuoBinding(digest(path),canonical(description))

def fixed_pair_allowed(envelope,first,second):
    proof=envelope.get(KEY)
    return bool(isinstance(proof,CommonDuoBinding)and first.get('geometry_role')=='arm'and second.get('geometry_role')=='arm'and type(first.get('link_index'))is int and type(second.get('link_index'))is int and first['link_index']==0 and second['link_index']==0 and first.get('robot_side')in proof.sides and second.get('robot_side')in proof.sides and first['robot_side']!=second['robot_side']and float(first.get('speed_bound_m_s',-1))==0. and float(second.get('speed_bound_m_s',-1))==0.)

def prepare(motion,site,output):
    motion,site,output=map(lambda p:Path(p).resolve(),[motion,site,output])
    if output.exists():raise FileExistsError('Existing actual mount proof is preserved')
    config=json.loads(motion.read_text());site_config=json.loads(site.read_text());now=datetime.now(timezone.utc)
    description=_description(config,site_config,motion.parent,{},now)
    # Paths in the description are bound provenance; operator verification is
    # intentionally absent until the actual common factory mount is inspected.
    proof=dict(schema_version=1,kind='actual_official_common_duo_mount',verified=False,actual_common_rigid_mount_installed=False,by=None,at_utc=None,evidence_file=None,description=description,
        scope='Only fixed leftlink0/rightlink0 pairs of this installed common rigid factory assembly; no moving link, obstacle, payload or robot-relative placement waiver')
    output.write_text(json.dumps(proof,indent=2)+'\n')
    return dict(state='unverified_common_duo_proof_prepared',path=str(output),enabled=False,hardware_tested=False)

def activate(motion,site,proof,output):
    motion,site,proof,output=map(lambda p:Path(p).resolve(),[motion,site,proof,output])
    if output.exists()or output.parent!=motion.parent:raise ValueError('Use a NEW motion profile beside the original')
    config=json.loads(motion.read_text());proof_data=json.loads(proof.read_text())
    if proof_data.get('verified')is not True or proof_data.get('actual_common_rigid_mount_installed')is not True or not isinstance(proof_data.get('evidence_file'),str)or not proof_data['evidence_file'].strip():raise ValueError('Complete the actual inspected common rigid mount record before activation')
    evidence=(proof.parent/proof_data['evidence_file']).resolve()
    config['common_duo_mount']=dict(proof_file=os.path.relpath(proof,motion.parent),proof_sha256=digest(proof),evidence_sha256=digest(evidence))
    record=binding(config,json.loads(site.read_text()),motion.parent,{},datetime.now(timezone.utc))
    output.write_text(json.dumps(config,indent=2)+'\n')
    return dict(state='fixed_common_duo_mount_binding_written',path=str(output),proof_sha256=record.proof_sha256,hardware_tested=False,full_task_verified=False)

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    for name in ['prepare','activate']:
        a=sub.add_parser(name);a.add_argument('--motion',type=Path,required=True);a.add_argument('--site',type=Path,required=True);a.add_argument('--output',type=Path,required=True)
        if name=='activate':a.add_argument('--proof',type=Path,required=True)
    a=p.parse_args(argv)
    result=prepare(a.motion,a.site,a.output)if a.command=='prepare'else activate(a.motion,a.site,a.proof,a.output)
    print(json.dumps(result,indent=2));return 0
if __name__=='__main__':raise SystemExit(main())
