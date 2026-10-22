"""Convert the prepared MJCF with the installed Isaac Sim MJCF importer."""
from __future__ import annotations
import argparse
import json
import sys
import time
import traceback
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--scene', choices=['m80','exchange','cap_button'], default='exchange')
parser.add_argument('--existing-usd', type=Path, help='Audit/select PhysX on an already completed local conversion')
args, _ = parser.parse_known_args()
root = Path(__file__).resolve().parent/'generated'/args.scene
status = root/'conversion-status.json'


def write_status(**items):
    temporary = status.with_suffix('.tmp')
    temporary.write_text(json.dumps(items, indent=2)+'\n')
    temporary.replace(status)


write_status(state='running', phase='app_start', scene=args.scene)
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'multi_gpu': False, 'anti_aliasing': 0})
exit_code = 0
started = time.monotonic()
try:
    import omni.kit.app
    manager = omni.kit.app.get_app().get_extension_manager()
    for name in ['omni.scene.optimizer.core','isaacsim.robot.schema','isaacsim.asset.importer.mjcf']:
        manager.set_extension_enabled_immediate(name, True)
    from isaacsim.asset.importer.mjcf import MJCFImporter, MJCFImporterConfig
    from pxr import Usd, UsdGeom, UsdPhysics
    write_status(state='running', phase='mjcf_import', scene=args.scene)
    print('PORT_IMPORT_START', args.scene, flush=True)
    config = MJCFImporterConfig(mjcf_path=str(root/(args.scene+'.xml')), usd_path=str(root/'usd'),
                               import_scene=True, merge_mesh=False, collision_from_visuals=False,
                               allow_self_collision=True, fix_base=None,
                               run_asset_transformer=True, run_multi_physics_conversion=True)
    if args.existing_usd:
        result = str(args.existing_usd.resolve())
        assert Path(result).is_relative_to(root/'usd')
    else:
        result = MJCFImporter(config).import_mjcf()
    stage = Usd.Stage.Open(result)
    assert stage is not None, result
    variant = stage.GetDefaultPrim().GetVariantSet('Physics')
    assert 'physx' in variant.GetVariantNames()
    variant.SetVariantSelection('physx')
    stage.GetRootLayer().Save()
    inventory = {'mesh_prims':0,'rigid_bodies':[],'articulation_roots':[],'joints':[], 'physics_scenes':[]}
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if prim.IsA(UsdGeom.Mesh): inventory['mesh_prims'] += 1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI): inventory['rigid_bodies'].append(str(prim.GetPath()))
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI): inventory['articulation_roots'].append(str(prim.GetPath()))
        if prim.IsA(UsdPhysics.Joint): inventory['joints'].append({'path':str(prim.GetPath()),'type':prim.GetTypeName()})
        if prim.IsA(UsdPhysics.Scene): inventory['physics_scenes'].append(str(prim.GetPath()))
    assert inventory['mesh_prims'] > 0
    record = dict(state='passed', physics_variant=variant.GetVariantSelection(),
                  prepared_sha256=json.loads((root/'source.json').read_text())['prepared_sha256'],
                  asset_digest=json.loads((root/'source.json').read_text())['asset_digest'],
                  usd_path=str(Path(result).relative_to(Path(__file__).resolve().parent)),
                  scene=args.scene, seconds=time.monotonic()-started, inventory=inventory)
    write_status(**record)
    print('PORT_IMPORT_PASS', json.dumps({k:v for k,v in record.items() if k!='inventory'}), flush=True)
except Exception as exc:
    traceback.print_exc()
    write_status(state='failed', error=repr(exc), seconds=time.monotonic()-started)
    exit_code = 1
finally:
    app.close(exit_code=exit_code)
sys.exit(exit_code)
