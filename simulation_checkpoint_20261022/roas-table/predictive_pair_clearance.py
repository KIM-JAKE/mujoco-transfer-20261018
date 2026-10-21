"""Conservative future arm-pair clearance, without moving simulation bodies.

Each collision mesh is enclosed by a PCA oriented box. A clear box test is
sufficient for mesh clearance; a rejected box pair may be a false positive.
Live PhysX structural queries and contact/encoder guards remain authoritative.
"""
from contextlib import contextmanager
import numpy as np
from scipy.spatial import ConvexHull, QhullError
from scipy.optimize import linprog, OptimizeWarning
import warnings


def enclose(vertices):
    vertices = np.asarray(vertices, dtype=float)
    _, axes = np.linalg.eigh(np.cov(vertices.T))
    local = vertices @ axes
    low, high = local.min(axis=0), local.max(axis=0)
    return axes @ ((low + high) / 2), axes, (high - low) / 2 + 1e-6


def overlaps(a, b):
    """15-axis separating axis theorem; touching boxes are obstacles."""
    ca, ra, ea = a[:3]
    cb, rb, eb = b[:3]
    rotation = ra.T @ rb
    absolute = np.abs(rotation) + 1e-10
    delta = ra.T @ (cb - ca)
    if np.any(np.abs(delta) > ea + absolute @ eb):
        return False
    if np.any(np.abs(delta @ rotation) > eb + absolute.T @ ea):
        return False
    for i in range(3):
        i1, i2 = (i + 1) % 3, (i + 2) % 3
        for j in range(3):
            j1, j2 = (j + 1) % 3, (j + 2) % 3
            distance = abs(delta[i2] * rotation[i1, j] - delta[i1] * rotation[i2, j])
            radius_a = ea[i1] * absolute[i2, j] + ea[i2] * absolute[i1, j]
            radius_b = eb[j1] * absolute[i, j2] + eb[j2] * absolute[i, j1]
            if distance > radius_a + radius_b:
                return False
    return True


def proxy(vertices):
    """Box broad phase plus convex supporting faces; still conservative."""
    vertices = np.asarray(vertices, dtype=float)
    box = enclose(vertices)
    try:
        hull = ConvexHull(vertices)
        normals = np.unique(np.round(hull.equations[:, :3], 7), axis=0)
        vertices = vertices[hull.vertices]
    except QhullError:
        normals = box[1].T
    return box + (vertices, normals)


def transform_proxy(box, transform):
    center, axes, extents, vertices, normals = box
    rotation, position = transform[:3, :3], transform[:3, 3]
    return (rotation @ center + position, rotation @ axes, extents,
            vertices @ rotation.T + position, normals @ rotation.T)


def transform_proxy_bounds(box, transform):
    """Identical transformed enclosing OBB; no unused vertex/normal copies.

    FCL callers retain the complete authored local hull in their cached
    CollisionObjects. SAT callers must still request transform_proxy().
    """
    center, axes, extents = box[:3]
    rotation, position = transform[:3, :3], transform[:3, 3]
    return (rotation @ center + position, rotation @ axes, extents)


def shape_overlap(a, b, margin=1e-6):
    if not overlaps((a[0],a[1],a[2]+margin),(b[0],b[1],b[2]+margin)):
        return False
    # A separating supporting-face direction proves clearance. Missing edge
    # cross-products can only reject extra candidates, never accept a collision.
    # This refines inflated boxes around curved palms and finger tips.
    inequalities=[];limits=[];vertex_witness=False
    for normals in [a[4], b[4]]:
        pa, pb = a[3] @ normals.T, b[3] @ normals.T
        if np.any(pa.max(axis=0) + 2*margin < pb.min(axis=0)) or np.any(pb.max(axis=0) + 2*margin < pa.min(axis=0)):
            return False
        upper=pa.max(axis=0) if normals is a[4] else pb.max(axis=0)
        other=pb if normals is a[4] else pa
        vertex_witness|=bool(np.any(np.all(other<=upper+margin,axis=1)))
        inequalities.append(normals);limits.append(upper+margin)
    if vertex_witness:return True
    # Supporting faces miss edge/edge separation. Resolve only ambiguous pairs
    # by bounded convex intersection feasibility, rather than rejecting a valid
    # transfer because two enclosing proxies overlap. Solver failure is blocked.
    with warnings.catch_warnings():
        warnings.simplefilter('ignore',OptimizeWarning)
        result=linprog(np.zeros(3),A_ub=np.vstack(inequalities),b_ub=np.concatenate(limits),
                       bounds=[(None,None)]*3,method='highs-ds',
                       options={'threads':1,'time_limit':.01})
    return result.status!=2


class LazyTransformedProxy:
    """Defer only vertex/normal projection until a broadphase survivor needs it.

    Every proxy retains the same authored convex CAD and transformed OBB.
    Nothing is approximated or cached across scene/pose contexts. Copying the
    transform prevents caller mutation from changing a deferred narrowphase.
    """
    def __init__(self, box, transform):
        self.local = box
        self.transform = np.asarray(transform).copy()
        self.bounds = transform_proxy_bounds(box, self.transform)
        self.full = None
    def __len__(self):return 5
    def __getitem__(self, index):
        if isinstance(index, slice):return tuple(self[i] for i in range(5)[index])
        if index < 0:index += 5
        if not 0 <= index < 5:raise IndexError(index)
        if index < 3:return self.bounds[index]
        if self.full is None:self.full = transform_proxy(self.local, self.transform)
        return self.full[index]

_LAZY_PAIR_PROJECTIONS = True

def at_pose(checker, q, opening, *, lazy=False):
    transforms = checker.arm.bodies(q, opening)
    result = []
    if checker.payload_path:
        transforms[checker.payload_path] = transforms[checker.arm.hand_path] @ checker.payload_relative
    for body, source, box in checker.pair_boxes + checker.payload_boxes:
        transform = transforms[body]
        result.append((body, source, LazyTransformedProxy(box, transform) if lazy else transform_proxy(box, transform)))
    return result


def pair_clear(checker, q, opening):
    other = checker.preview_other_boxes
    own=at_pose(checker,q,opening,lazy=_LAZY_PAIR_PROJECTIONS);margin=.0005
    if not own or not other:return True,None
    def bounds(rows):
        centers=np.array([row[2][0] for row in rows]);extents=np.array([abs(row[2][1])@(row[2][2]+margin) for row in rows]);return centers-extents,centers+extents
    amin,amax=bounds(own);bmin,bmax=bounds(other)
    candidates=np.all(amax[:,None,:]>=bmin[None,:,:],axis=2)&np.all(bmax[None,:,:]>=amin[:,None,:],axis=2)
    for i,j in np.argwhere(candidates):
        body,source,box=own[i];other_body,other_source,other_box=other[j]
        if shape_overlap(box,other_box,margin=margin):
            return False,dict(query=source,actor=other_body,collider=other_source,predictive_pair_box=True,minimum_pair_clearance_m=2*margin)
    return True, None


@contextmanager
def other_at(checker, other, q, opening):
    """Replace only the other moving arm's stale actors in planning queries."""
    previous = checker.preview_other_boxes, checker.preview_other_actors,getattr(checker,'preview_other_configuration',None)
    checker.preview_other_boxes = at_pose(other, q, opening, lazy=_LAZY_PAIR_PROJECTIONS)
    checker.preview_other_actors = {body for body, _, _ in checker.preview_other_boxes}
    checker.preview_other_configuration=(other,np.asarray(q).copy(),float(opening))
    try:
        yield
    finally:
        checker.preview_other_boxes, checker.preview_other_actors,checker.preview_other_configuration = previous
