"""Conservative capsule-envelope geometry, including bounded motion between SDK samples.

This checks explicitly field-verified enclosing geometry. It does not infer an
entire robot, cable, hand or clutter shape from a TCP or from a pass flag.
"""
import numpy as np


class CollisionError(ValueError):
    pass


def finite(value, shape=None):
    result = np.asarray(value, dtype=float)
    if not np.isfinite(result).all() or (shape is not None and result.shape != shape):
        raise ValueError('Finite geometry with shape ' + str(shape) + ' required')
    return result


def transform(value):
    matrix = finite(value, (4, 4))
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9) or
            not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-7) or
            abs(np.linalg.det(matrix[:3, :3]) - 1) > 1e-7):
        raise ValueError('Proper metre-based rigid transform required')
    return matrix


def points(matrix, value):
    matrix = np.asarray(matrix)
    return np.einsum('...ij,...j->...i', matrix[..., :3, :3], value) + matrix[..., :3, 3]


def box(value):
    pose = transform(value['T_cell_box'])
    half = finite(value['half_extents_m'], (3,))
    if np.any(half <= 0):
        raise ValueError('Positive measured box half extents required')
    return pose, half


def segment_box_distance(a, b, value):
    """Exact segment/OBB distance by convex piecewise quadratic minimization."""
    pose, half = box(value)
    a, b = finite(a), finite(b)
    a = (a - pose[:3, 3]) @ pose[:3, :3]
    b = (b - pose[:3, 3]) @ pose[:3, :3]
    direction = b - a
    # At most six crossing parameters delimit all quadratic pieces.
    with np.errstate(divide='ignore', invalid='ignore'):
        crossings = np.concatenate([(sign * half - a) / direction for sign in (-1, 1)], axis=-1)
    crossings = np.where(np.isfinite(crossings), crossings, 0.)
    boundaries = np.sort(np.concatenate([np.zeros((*a.shape[:-1], 1)),
                                         np.clip(crossings, 0, 1), np.ones((*a.shape[:-1], 1))], axis=-1), axis=-1)
    best = np.full(a.shape[:-1], np.inf)
    for index in range(7):
        low, high = boundaries[..., index], boundaries[..., index + 1]
        middle = (low + high) / 2
        probe = a + middle[..., None] * direction
        side = np.where(probe > half, 1., np.where(probe < -half, -1., 0.))
        active = side != 0
        delta = np.where(active, a - side * half, 0.)
        slope = np.where(active, direction, 0.)
        denominator = np.sum(slope * slope, axis=-1)
        stationary = np.divide(-np.sum(delta * slope, axis=-1), denominator,
                               out=low.copy(), where=denominator > 1e-30)
        parameter = np.clip(stationary, low, high)
        location = a + parameter[..., None] * direction
        distance2 = np.sum(np.maximum(np.abs(location) - half, 0.) ** 2, axis=-1)
        best = np.minimum(best, distance2)
    return np.sqrt(np.maximum(best, 0.))


def segment_distance(a, b, c, d):
    """Exact finite segment distance; vectorized, including parallel/zero segments."""
    a, b, c, d = np.broadcast_arrays(finite(a), finite(b), finite(c), finite(d))
    u, v, w = b - a, d - c, a - c
    aa, bb, cc = np.sum(u*u, -1), np.sum(u*v, -1), np.sum(v*v, -1)
    dd, ee = np.sum(u*w, -1), np.sum(v*w, -1)
    denominator = aa*cc - bb*bb
    safe_a, safe_c = np.maximum(aa, 1e-30), np.maximum(cc, 1e-30)
    candidates = [(np.zeros_like(aa), np.clip(ee/safe_c, 0, 1)),
                  (np.ones_like(aa), np.clip((ee+bb)/safe_c, 0, 1)),
                  (np.clip(-dd/safe_a, 0, 1), np.zeros_like(aa)),
                  (np.clip((bb-dd)/safe_a, 0, 1), np.ones_like(aa))]
    s = np.divide(bb*ee-cc*dd, denominator, out=np.zeros_like(aa), where=denominator > 1e-24)
    t = np.divide(aa*ee-bb*dd, denominator, out=np.zeros_like(aa), where=denominator > 1e-24)
    interior = (denominator > 1e-24) & (s >= 0) & (s <= 1) & (t >= 0) & (t <= 1)
    best = np.full(aa.shape, np.inf)
    for index, (sp, tp) in enumerate(candidates + [(s, t)]):
        distance2 = np.sum((w + sp[..., None]*u - tp[..., None]*v)**2, axis=-1)
        if index == 4:
            distance2 = np.where(interior, distance2, np.inf)
        best = np.minimum(best, distance2)
    return np.sqrt(np.maximum(best, 0.))


def inside_box(a, b, radius, value):
    pose, half = box(value)
    a, b = finite(a), finite(b)
    local_a = (a - pose[:3, 3]) @ pose[:3, :3]
    local_b = (b - pose[:3, 3]) @ pose[:3, :3]
    return np.all(np.maximum(np.abs(local_a), np.abs(local_b)) + np.asarray(radius)[..., None] <= half, axis=-1)


def local_capsules(value, poses, speed_bound, angular_speed_bound, gap, prefix):
    result = []
    for index, capsule in enumerate(value):
        a, b = finite(capsule['a'], (3,)), finite(capsule['b'], (3,))
        radius = float(capsule['radius_m'])
        if not np.isfinite(radius) or radius <= 0:
            raise ValueError('Positive measured capsule radius required')
        bound = speed_bound + max(np.linalg.norm(a), np.linalg.norm(b)) * angular_speed_bound
        result.append({'name': capsule.get('name', prefix + str(index)),
                       'a': points(poses, a), 'b': points(poses, b),
                       'r': radius + bound*gap/2, 'nominal_radius_m': radius,
                       'speed_bound_m_s': float(bound)})
    return result


def robot_capsules(link_points, radii, point_speed_bounds, gap, tcp_poses, envelope, prefix):
    links = finite(link_points)
    if links.ndim != 3 or links.shape[1:] != (9, 3):
        raise ValueError('Nine actual SDK base/joint/TCP points per sample required')
    radii, speeds = finite(radii, (8,)), finite(point_speed_bounds, (9,))
    if np.any(radii <= 0) or np.any(speeds < 0):
        raise ValueError('Measured enclosing radii and nonnegative speed bounds required')
    result = []
    for index in range(8):
        speed = max(speeds[index], speeds[index+1])
        result.append({'name': prefix + 'link' + str(index), 'a': links[:, index], 'b': links[:, index+1],
                       'r': radii[index] + speed*gap/2, 'nominal_radius_m': float(radii[index]),
                       'speed_bound_m_s': float(speed)})
    tools = envelope.get('tool_capsules_tcp_m', [])
    if not tools and envelope.get('tool_in_link_envelopes') is not True:
        raise ValueError('Actual hand/tool envelope must be explicitly enclosed')
    result.extend(local_capsules(tools, tcp_poses, speeds[-1], envelope['_angular_speed_bound_rad_s'], gap, prefix+'tool'))
    return result


def check_envelopes(active, other, scene, phase, envelope, *, payloads=(), dynamic_obstacles=(), margin):
    """Check conservative swept capsules, full robot workspace, self and other arm."""
    if not np.isfinite(margin) or margin <= 0:
        raise ValueError('Explicit positive collision margin required')
    obstacles = scene.get('obstacles')
    if not isinstance(obstacles, list) or not obstacles:
        raise ValueError('An empty/unknown measured scene cannot pass collision validation')
    names = [obstacle.get('name') for obstacle in obstacles]
    if any(not isinstance(name, str) or not name for name in names) or len(set(names)) != len(names):
        raise ValueError('Every measured obstacle needs a unique name')
    for obstacle in obstacles:
        box(obstacle)
    corridors = phase.get('corridors')
    if not isinstance(corridors, list) or not corridors:
        raise ValueError('Field-reviewed phase corridors required')
    workspace = scene['workspace']
    minimum = float('inf')
    count = 0
    def boxes(capsules, is_payload=False, active_robot=False):
        nonlocal minimum, count
        for capsule_index,capsule in enumerate(capsules):
            radius = capsule['r'] + margin
            if not np.all(inside_box(capsule['a'], capsule['b'], radius, workspace)):
                raise CollisionError('Swept envelope leaves the completely modeled workspace: ' + capsule['name'])
            for obstacle in obstacles:
                support=capsule.get('fixed_mount_support',{})
                if (not is_payload and capsule.get('link_index')==0 and
                        support.get('kind')=='fixed_base_known_mount_contact'and support.get('obstacle_name')==obstacle['name']):
                    continue  # exact static CAD/known support plane proved before this capsule-skin test
                contact = phase.get('allow_payload_contact_fixture')
                permitted = (is_payload and capsule.get('part') == phase.get('_active_part') and
                             obstacle['name'] == contact and phase.get('payload_contact_approved') is True and
                             obstacle.get('source') not in ('measured_unknown_scene_enclosure','conservative_registered_RGBD_rays'))
                selected_contact=(active_robot and not is_payload and capsule_index>=envelope.get('tool_capsule_offset',8) and
                    capsule_index in phase.get('target_contact_tool_capsules',[]) and phase.get('_contact')=='grasp' and
                    obstacle.get('source')=='conservative_registered_RGBD_rays' and obstacle.get('occupancy_kind')=='occupied' and
                    obstacle.get('selected_contact_instance_id')==phase.get('_target_instance_id') and phase.get('_target_instance_id')is not None)
                if selected_contact:continue  # Only observed selected surface; unknown/palm/other arm/payload never exempt.
                clearance = segment_box_distance(capsule['a'], capsule['b'], obstacle) - radius
                if phase.get('bounded_loose_stock_contact')is not None:
                    from .stock_contact import stock_contact_mask
                    allowed_stock=stock_contact_mask(obstacle,phase=phase,capsule_index=capsule_index,capsule=dict(capsule,r=radius),envelope=envelope,active_robot=active_robot,is_payload=is_payload)
                    clearance=np.where(allowed_stock,np.inf,clearance)
                if phase.get('bounded_loose_stock_departure')is not None:
                    from .stock_departure import departure_contact_mask,departure_support_floor_mask
                    allowed_departure=departure_contact_mask(obstacle,phase=phase,capsule_index=capsule_index,capsule=dict(capsule,r=radius),envelope=envelope,active_robot=active_robot,is_payload=is_payload)
                    allowed_floor=departure_support_floor_mask(obstacle,phase=phase,capsule=dict(capsule,r=radius),envelope=envelope,is_payload=is_payload)
                    clearance=np.where(allowed_departure|allowed_floor,np.inf,clearance)
                if permitted:
                    region = phase.get('payload_contact_region')
                    if region is None:
                        raise ValueError('Measured payload contact region required')
                    allowed = inside_box(capsule['a'], capsule['b'], radius, region)
                    clearance = np.where(allowed, np.inf, clearance)
                minimum = min(minimum, float(np.min(clearance))); count += len(np.atleast_1d(clearance))
                if np.any(clearance <= 0):
                    raise CollisionError(capsule['name'] + ' intersects measured obstacle ' + obstacle['name'])
    boxes(active,active_robot=True)
    boxes(other)
    boxes(payloads, True)
    # TCP corridor constrains the hand approach; full robot remains checked above.
    tcp = active[0].get('actual_tcp',active[7].get('actual_tcp', active[7]['b']))
    corridor_union = np.zeros(tcp.shape[:-1], dtype=bool)
    tool_radius = float(phase.get('tcp_corridor_radius_m', 0)) + float(phase.get('_tcp_sweep_m', 0))
    if not np.isfinite(tool_radius) or tool_radius <= 0:
        raise ValueError('Measured TCP corridor envelope radius required')
    for corridor in corridors:
        corridor_union |= inside_box(tcp, tcp, tool_radius, corridor)
    if not np.all(corridor_union):
        raise CollisionError('TCP trajectory leaves the reviewed phase corridor union')
    ignored = set()
    for pair in envelope.get('self_ignore_pairs', []):
        if (not isinstance(pair, list) or len(pair) != 2 or any(type(index) is not int for index in pair) or
                min(pair) < 0 or max(pair) >= len(active) or pair[0] == pair[1]):
            raise ValueError('Explicit reviewed self-exclusion capsule pairs required')
        if abs(pair[0]-pair[1]) != 1 and min(pair) < 6:
            raise ValueError('Self exclusions may cover adjacent joints or measured distal rigid hand/tool attachments only')
        ignored.add(tuple(sorted(pair)))
    # Exact distance is needed only when the complete swept AABBs overlap.
    # Cache once per capsule; never infer clearance from endpoints alone.
    sweep_bounds={}
    for row in [*active,*other,*payloads,*dynamic_obstacles]:
        coordinates=np.concatenate([np.asarray(row['a']).reshape(-1,3),np.asarray(row['b']).reshape(-1,3)])
        radius=float(np.max(row['r']));sweep_bounds[id(row)]=(coordinates.min(0)-radius,coordinates.max(0)+radius)
    def pair_check(a, b, description, allowed=None):
        nonlocal minimum, count
        ba,bb=sweep_bounds.get(id(a)),sweep_bounds.get(id(b))
        if ba is not None and bb is not None:
            delta=np.maximum(np.maximum(ba[0]-bb[1],bb[0]-ba[1]),0.)
            lower=float(np.linalg.norm(delta))-margin
            if lower>0:
                minimum=min(minimum,lower);count+=1;return
        clearance = segment_distance(a['a'], a['b'], b['a'], b['b']) - a['r'] - b['r'] - margin
        if allowed is not None:clearance=np.where(allowed,np.inf,clearance)
        minimum = min(minimum, float(np.min(clearance))); count += len(np.atleast_1d(clearance))
        if np.any(clearance <= 0):
            raise CollisionError(description + ': ' + a['name'] + '/' + b['name'])
    def broad_pairs(first,second):
        if not first or not second:return np.zeros((len(first),len(second)),dtype=bool)
        low_a=np.array([sweep_bounds[id(x)][0]for x in first]);high_a=np.array([sweep_bounds[id(x)][1]for x in first])
        low_b=np.array([sweep_bounds[id(x)][0]for x in second]);high_b=np.array([sweep_bounds[id(x)][1]for x in second])
        delta=np.maximum(np.maximum(low_a[:,None]-high_b[None],low_b[None]-high_a[:,None]),0.)
        return np.einsum('ijk,ijk->ij',delta,delta)<=margin*margin
    self_possible=broad_pairs(active,active);other_possible=broad_pairs(active,other)
    for index, first in enumerate(active):
        for second_index in np.flatnonzero(self_possible[index,index+1:])+index+1:
            from .model_geometry import self_connected
            model_connected=envelope.get('geometry_backend')=='sdk_link_columns'and self_connected(first,active[second_index])
            if (index, second_index) not in ignored and not model_connected:
                pair_check(first, active[second_index], 'Robot self collision')
        for second_index in np.flatnonzero(other_possible[index]):
            pair_check(first, other[second_index], 'Other-arm collision')
    payload_ignores = envelope.get('payload_robot_ignore_capsules', [])
    if any(type(index) is not int or index < 6 or index >= len(active) for index in payload_ignores):
        raise ValueError('Payload contact can exempt only explicitly measured distal hand/tool capsules')
    for payload in payloads:
        own = active if payload.get('owner') == 'active' else other
        opposite = other if payload.get('owner') == 'active' else active
        ignores = payload_ignores if payload.get('owner') == 'active' else payload.get('robot_ignore_capsules', [])
        for index, robot in enumerate(own):
            if index not in ignores:
                pair_check(payload, robot, 'Held payload/own robot collision')
        for index, robot in enumerate(opposite):
            contact = (payload.get('owner') == 'other' and payload.get('part') == phase.get('_active_part') and
                       phase.get('_contact') == 'handover' and index >= envelope.get('tool_capsule_offset',8) and
                       index in phase.get('handover_contact_tool_capsules', []))
            if not contact:
                pair_check(payload, robot, 'Held payload/other robot collision')
    for obstacle in dynamic_obstacles:
        for index, robot in enumerate(active):
            permitted = (obstacle.get('instance_id') == phase.get('_target_instance_id') and
                         index in phase.get('target_contact_tool_capsules', []) and index >= envelope.get('tool_capsule_offset',8) and
                         phase.get('_contact') == 'grasp')
            if not permitted:
                from .stock_departure import loose_stock_pair_mask
                allowed=loose_stock_pair_mask(dict(robot,r=robot['r']+margin),obstacle,phase=phase,envelope=envelope,capsule_index=index)
                pair_check(robot, obstacle, 'Observed loose-part collision',allowed)
        for payload in payloads:
            if payload.get('instance_id') != obstacle.get('instance_id'):
                from .stock_departure import loose_stock_pair_mask
                allowed=loose_stock_pair_mask(dict(payload,r=payload['r']+margin),obstacle,phase=phase,envelope=envelope,is_payload=True)
                pair_check(payload, obstacle, 'Held payload/observed loose-part collision',allowed)
    return {'scope': 'field-verified enclosing capsules against measured boxes, self, other arm and supplied observed geometry',
            'conservative_swept_envelopes_checked': True, 'minimum_clearance_m': minimum,
            'geometric_checks': count, 'robot_geometry_inferred_from_tcp': False,
            'physical_execution_certified': False}
