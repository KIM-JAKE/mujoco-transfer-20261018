"""Color-independent CAD geometry classifier for the ideal-perception prototype.

This measures object-local mesh dimensions; it is NOT a tested RGB-D detector.
The detector can later supply the same outer-diameter measurement from depth.
"""
import numpy as np


def classify_ring(points, tolerance_m=.002):
    points = np.asarray(points, float)
    # The CAD's local Z is the ring axis, independent of its world pose.
    diameter = float(np.mean(np.ptp(points[:, :2], axis=0)))
    references = {'ring1': .075, 'ring2': .062}
    kind = min(references, key=lambda name: abs(diameter-references[name]))
    error = abs(diameter-references[kind])
    if error > tolerance_m:
        raise ValueError('Unknown ring diameter: %.3f mm' % (diameter*1000))
    return dict(part=kind, outer_diameter_m=diameter,
                nominal_diameter_m=references[kind], error_m=error,
                uses_color=False, input='Ideal object-local USD visual mesh')


def measure_stage(stage, paths):
    from pxr import UsdGeom
    rows = []
    for path in paths:
        root = stage.GetPrimAtPath(path)
        candidates = []
        from pxr import Usd
        for prim in Usd.PrimRange(root):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            if 'Contact' in str(prim.GetPath()):
                continue
            mesh = UsdGeom.Mesh(prim)
            p = np.asarray(mesh.GetPointsAttr().Get(), float)
            if len(p):
                candidates.append(p)
        if not candidates:
            raise ValueError('No visual ring mesh: '+path)
        points = max(candidates, key=len)
        row = classify_ring(points)
        row['path'] = path
        rows.append(row)
    return rows
