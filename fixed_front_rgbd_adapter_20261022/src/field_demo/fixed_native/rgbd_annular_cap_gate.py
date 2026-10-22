"""Conservative RGB-D veto for an annulus contradicted by observed free space.

Only the measured candidate pose, RGB-D camera packet and known part geometry
enter this function. No simulator actors, object IDs or stored scene poses.
Occluded samples do not contradict a hypothesis. This is a veto, not proof of
correct class or grasp accessibility; it does not invent hidden geometry.
"""
import numpy as np
from .rgbd_geometry import axis_frame

# Interior of actual CAD planar end faces, with edges deliberately excluded.
_CAP_RADII_M = {
    'ring1': {1: (.03580, .03710), -1: (.03210, .03625)},
    'ring2': {1: (.02960, .03075), -1: (.02555, .02985)},
}
_HALF_THICKNESS_M = {'ring1': .00775, 'ring2': .007}

def annular_cap_gate(packet, pose):
    center = np.asarray(pose['center_m'], dtype=float)
    axis = np.array(pose['axis'], dtype=float, copy=True)
    axis /= np.linalg.norm(axis)
    T = np.asarray(packet['T_cell_camera'])
    K = np.asarray(packet['K'])
    depth = packet['depth_m']
    side = 1 if (T[:3, 3] - center) @ axis >= 0 else -1
    low, high = _CAP_RADII_M[pose['part']][side]
    theta = np.linspace(0., 2*np.pi, 144, endpoint=False)
    basis = axis_frame(axis)[:, :2]
    planar = np.einsum('a,bi,ji->abj', np.linspace(low, high, 5),
                       np.c_[np.cos(theta), np.sin(theta)], basis)
    world = center + side * axis * _HALF_THICKNESS_M[pose['part']] + planar
    camera = (world - T[:3, 3]) @ T[:3, :3]
    uv = camera[..., :2] / camera[..., 2, None]
    uv = uv @ K[:2, :2].T + K[:2, 2]
    pixels = np.rint(uv).astype(int)
    u, v = pixels[..., 0], pixels[..., 1]
    valid = ((camera[..., 2] > .05) & (u >= 0) & (u < depth.shape[1]) &
             (v >= 0) & (v < depth.shape[0]))
    observed = np.full(valid.shape, np.nan)
    observed[valid] = depth[v[valid], u[valid]]
    error = observed - camera[..., 2]
    valid &= np.isfinite(observed) & (observed > .05)
    support = valid & (np.abs(error) <= .0012)
    occluded = valid & (error < -.0012)
    contradicted = valid & (error > .0012)
    seen = valid & ~occluded
    seen_count = int(seen.sum())
    contradiction_count = int(contradicted.sum())
    contradiction_bins = int(np.any(contradicted, axis=0).sum())
    free_fraction = contradiction_count / max(1, seen_count)
    reject = (seen_count >= 144 and contradiction_count >= 120 and
              free_fraction > .35 and contradiction_bins >= 48)
    return not reject, dict(
        backend='RGB-D annular CAD cap observed-free-space veto',
        uses_simulator_body_poses=False,
        support_count=int(support.sum()), seen_count=seen_count,
        occluded_count=int(occluded.sum()),
        contradiction_count=contradiction_count,
        contradiction_angle_bins=contradiction_bins,
        free_fraction=float(free_fraction), rejected=bool(reject))
