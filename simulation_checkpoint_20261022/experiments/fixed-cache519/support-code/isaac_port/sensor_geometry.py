"""Separate E32-T11N physical geometry from its CAD optical-axis solid.

CAD definition 0:1:1:155 has a head/tail solid and a 0.3 mm diameter,
754.2 mm cylinder extending from the optical face to the opposing sensor.
The latter represents light, not a mechanical rail. No mount is translated.
"""
import numpy as np

SENSOR_IDS = (369, 371, 373, 380, 382, 384)
SENSOR_NAMES = tuple(f'instance_{i:04d}' for i in SENSOR_IDS)


def physical_mesh(vertices, faces):
    vertices = np.asarray(vertices, dtype=float)
    faces = np.asarray(faces, dtype=np.int32)
    # MJCF and USD exporters duplicate vertices at face boundaries.
    _, inverse = np.unique(np.round(vertices, 10), axis=0, return_inverse=True)
    parent = np.arange(int(inverse.max()) + 1)

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a, b, c in inverse[faces]:
        parent[find(b)] = find(a)
        parent[find(c)] = find(a)
    groups = np.array([find(i) for i in inverse])
    components = []
    for group in np.unique(groups):
        f = faces[groups[faces[:, 0]] == group]
        used = np.unique(f)
        v = vertices[used]
        _, _, axes = np.linalg.svd(v - v.mean(axis=0), full_matrices=False)
        extents = np.ptp(v @ axes.T, axis=0)
        components.append((float(extents.max()), used, f, extents))
    if len(components) != 2:
        raise RuntimeError(f'Unexpected E32 CAD topology: {len(components)} components')
    head, beam = sorted(components, key=lambda item: item[0])
    # Check the dimensionless aspect ratios as well as the component count;
    # this also works for USD local coordinate units before xform scaling.
    if not (7 < beam[0] / head[0] < 10 and
            max(beam[3][1:]) / beam[0] < .0005):
        raise RuntimeError('E32 optical-axis identification did not match CAD dimensions')
    _, used, f, _ = head
    remap = np.full(len(vertices), -1, dtype=np.int32)
    remap[used] = np.arange(len(used))
    return vertices[used], remap[f], dict(
        components=2, physical_vertices=len(used), optical_axis_vertices=len(beam[1]),
        optical_axis_length_local=beam[0], physical_span_local=head[0],
        mounts_translated=False, physical_head_and_tail_preserved=True)


def apply(stage):
    from pxr import Usd, UsdGeom
    records = []
    for name in SENSOR_NAMES:
        for base in ['/World/Environment/m80/', '/World/Cell/Geometry/m80/']:
            parent = stage.GetPrimAtPath(base + name)
            if not parent:
                raise RuntimeError('Missing original sensor instance: ' + base + name)
            if parent.IsInstance():
                parent.SetInstanceable(False)
            meshes = [p for p in Usd.PrimRange(parent) if p.IsA(UsdGeom.Mesh)]
            if len(meshes) != 1:
                raise RuntimeError('Unexpected sensor mesh inventory: ' + str(meshes))
            mesh = UsdGeom.Mesh(meshes[0])
            counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
            if not np.all(counts == 3):
                raise RuntimeError('Expected triangulated sensor CAD')
            v, f, info = physical_mesh(mesh.GetPointsAttr().Get(),
                                      np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3))
            mesh.GetPointsAttr().Set(v.astype(np.float32))
            mesh.GetFaceVertexCountsAttr().Set([3] * len(f))
            mesh.GetFaceVertexIndicesAttr().Set(f.reshape(-1))
            # Imported normals belong to the old topology; let the renderer recompute.
            mesh.GetNormalsAttr().Clear()
            records.append(dict(path=str(mesh.GetPath()), **info))
    return dict(sensor_geometry='original E32-T11N head/tail; optical-axis solid excluded',
                original_sensor_mounts_preserved=True, optical_axis_nonphysical=True,
                sensor_meshes=records, source_definition='0:1:1:155',
                original_head_tail_assumed_rigid=True)
