"""Prepare a container-addressable MJCF copy and record the source model contract."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
TRANSFER = HERE.parent
WORKSPACE = TRANSFER.parent
SOURCES = {
    'm80': TRANSFER/'keti_m80_mujoco_20261013/m80.xml',
    'exchange': TRANSFER/'keti_m80_mujoco_20261013/duo_fr3_2f85_20261013/g2_fr3_side_exchange_20261016/scene_outbound_candidate.xml',
    'cap_button': TRANSFER/'keti_m80_mujoco_20261013/duo_fr3_2f85_20261013/cap_pin_yaw_m90_20261014/scene.xml',
}


def prepare(scene: str) -> Path:
    source = SOURCES[scene]
    model = mujoco.MjModel.from_xml_path(str(source))
    destination = HERE/'generated'/scene
    destination.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(source)
    compiler = tree.getroot().find('compiler')
    assert compiler is not None
    inline_assets = []
    for mesh in tree.getroot().findall('./asset/mesh'):
        if mesh.get('file') is not None:
            continue
        vertices = np.fromstring(mesh.attrib['vertex'], sep=' ').reshape(-1, 3)
        if 'face' in mesh.attrib:
            faces = np.fromstring(mesh.attrib['face'], sep=' ', dtype=int).reshape(-1, 3)
        else:
            # Vertex-only MJCF meshes use the compiler's convex hull. Preserve
            # those exact triangles rather than choosing a new decomposition.
            index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MESH, mesh.attrib['name'])
            assert model.mesh_vertnum[index] == len(vertices)
            address, count = model.mesh_faceadr[index], model.mesh_facenum[index]
            faces = model.mesh_face[address:address+count]
        asset = destination/'assets'/(mesh.attrib['name']+'.obj')
        asset.parent.mkdir(exist_ok=True)
        asset.write_text(''.join('v '+' '.join(format(float(v), '.17g') for v in xyz)+'\n' for xyz in vertices)
                         + ''.join('f '+' '.join(str(int(v)+1) for v in tri)+'\n' for tri in faces))
        mesh.set('file', str(asset))
        for key in ['vertex', 'face', 'normal', 'texcoord']:
            mesh.attrib.pop(key, None)
        inline_assets.append({'name': mesh.attrib['name'], 'vertices': len(vertices), 'faces': len(faces)})
    directories = {'mesh': compiler.get('meshdir', ''), 'texture': compiler.get('texturedir', '')}
    assets = []
    for element in tree.getroot().iter():
        filename = element.get('file')
        if filename is None:
            continue
        path = Path(filename)
        if not path.is_absolute():
            path = source.parent/directories.get(element.tag, '')/path
        path = path.resolve(strict=True)
        relative = path.relative_to(WORKSPACE)
        container_path = Path('/workspace/project')/relative
        element.set('file', str(container_path))
        assets.append({'path': str(relative), 'bytes': path.stat().st_size,
                       'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
    compiler.attrib.pop('meshdir', None)
    compiler.attrib.pop('texturedir', None)
    # Only asset paths change. Original source XML/assets are never rewritten.
    target = destination/(scene+'.xml')
    ET.indent(tree)
    tree.write(target, encoding='utf-8', xml_declaration=True)
    # Prove that normalization preserves MuJoCo geometry and physical parameters
    # before handing the copy to a different physics engine.
    local_text = target.read_text().replace('/workspace/project', str(WORKSPACE))
    prepared_model = mujoco.MjModel.from_xml_string(local_text)
    compared = ['body_pos', 'body_quat', 'body_mass', 'body_inertia', 'geom_pos',
                'geom_quat', 'geom_size', 'geom_contype', 'geom_conaffinity',
                'jnt_pos', 'jnt_axis', 'jnt_range', 'mesh_vert', 'mesh_face']
    for name in compared:
        np.testing.assert_allclose(getattr(prepared_model, name), getattr(model, name), rtol=1e-6, atol=1e-7,
                                   err_msg=f'Prepared MJCF changed {name}')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    seed = mujoco.MjData(model)
    seed_source = None
    if scene == 'exchange':
        seed_source = source.parent/'g2_departure_physical/final_state.npz'
        with np.load(seed_source, allow_pickle=False) as state:
            seed.qpos[:] = state['qpos']
            seed.ctrl[:] = state['ctrl']
        mujoco.mj_forward(model, seed)
    joints = []
    for i in range(model.njnt):
        joints.append({'name': mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i),
                       'type': int(model.jnt_type[i]), 'qpos_address': int(model.jnt_qposadr[i]),
                       'body_name': mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.jnt_bodyid[i])),
                       'seed_position': float(seed.qpos[model.jnt_qposadr[i]]) if model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE else None,
                       'limited': bool(model.jnt_limited[i]), 'range': model.jnt_range[i].tolist()})
    base_axes = {}
    if scene == 'exchange':
        jacobian_position = np.zeros((3, model.nv))
        jacobian_rotation = np.zeros((3, model.nv))
        mujoco.mj_jacBody(model, seed, jacobian_position, jacobian_rotation, model.body('base_link').id)
        for name in ['base_x', 'base_y', 'base_yaw']:
            dof = int(model.jnt_dofadr[model.joint(name).id])
            base_axes[name] = {'world_linear_axis': jacobian_position[:,dof].tolist(),
                               'world_angular_axis': jacobian_rotation[:,dof].tolist()}
    contract = {'scene': scene, 'source': str(source.relative_to(WORKSPACE)),
                'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                'prepared_sha256': hashlib.sha256(target.read_bytes()).hexdigest(),
                'source_mujoco_version': mujoco.__version__,
                'nbody': model.nbody, 'njnt': model.njnt, 'nq': model.nq, 'nv': model.nv,
                'nu': model.nu, 'ngeom': model.ngeom, 'nmesh': model.nmesh,
                'collision_geoms': int(np.count_nonzero(model.geom_contype | model.geom_conaffinity)),
                'visual_only_geoms': int(np.count_nonzero(~(model.geom_contype | model.geom_conaffinity).astype(bool))),
                'inline_meshes_exported': inline_assets,
                'prepared_model_parity': {'passed': True, 'compared_arrays': compared, 'rtol': 1e-6, 'atol': 1e-7},
                'seed_state': str(seed_source.relative_to(WORKSPACE)) if seed_source else None,
                'base_axes': base_axes,
                'joints': joints, 'assets': assets,
                'asset_digest': hashlib.sha256(json.dumps(assets,sort_keys=True).encode()).hexdigest(),
                'body_names': [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)],
                'bodies': [{'name': model.body(i).name, 'mass': float(model.body_mass[i]),
                            'inertia': model.body_inertia[i].tolist(),
                            'center_of_mass': model.body_ipos[i].tolist(),
                            'inertial_quaternion': model.body_iquat[i].tolist(),
                            'gravity_compensation': float(model.body_gravcomp[i])} for i in range(1, model.nbody)],
                'equalities': [{'type': element.tag, **element.attrib} for element in tree.getroot().findall('./equality/*')],
                'source_geometry_issue': 'Known upstream workpiece/CAD intersections remain unresolved; conversion does not certify physical assembly.'}
    np.savez_compressed(destination/'reference.npz', body_positions=data.xpos.copy(), body_quaternions=data.xquat.copy(), qpos=data.qpos.copy(),
                        seed_body_positions=seed.xpos.copy(), seed_body_quaternions=seed.xquat.copy(), seed_qpos=seed.qpos.copy())
    (destination/'source.json').write_text(json.dumps(contract, indent=2)+'\n')
    print(json.dumps({k:contract[k] for k in ['scene','nbody','njnt','ngeom','nmesh','collision_geoms','visual_only_geoms']}), flush=True)
    return target


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', choices=list(SOURCES), default='exchange')
    prepare(parser.parse_args().scene)
