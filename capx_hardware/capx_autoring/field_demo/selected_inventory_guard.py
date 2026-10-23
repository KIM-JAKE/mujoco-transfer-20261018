"""Image/SDK evidence for one held ring, and for an actually empty open Hand.

A nearest-ring crop or Hand.is_grasped is never an item-count certificate.
All depth in a broad held ROI is checked against the selected class surface;
unknown depth/occlusion requires another actual view or a checked bin return.
Complementary views may come from ONE fixed camera and actual robot inspection
poses. No virtual camera, actor identity, object mass or pose write is used.
"""
from datetime import datetime, timezone
import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import label
from .collision import transform


def _stamp(value):
    result = value if isinstance(value, datetime) else datetime.fromisoformat(value.replace('Z', '+00:00'))
    if result.tzinfo is None: raise ValueError('Timezone-aware actual capture timestamp required')
    return result.astimezone(timezone.utc)


def _digest(value):
    return isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def _frames(observations, max_age_s, max_pair_span_s, now_utc):
    if not 0 < max_age_s <= 2 or not max_age_s <= max_pair_span_s <= 15:
        raise ValueError('Explicit bounded current-frame age/inspection-pair span required')
    if not 2 <= len(observations) <= 6: raise ValueError('Two to six actual inspection images required')
    now = _stamp(now_utc or datetime.now(timezone.utc)); ids = set(); stamps = []; epochs = set(); modes = set()
    for view in observations:
        meta = view['metadata']; capture = meta.get('capture_id')
        if not isinstance(capture, str) or not capture or capture in ids: raise ValueError('Distinct actual RGB-D captures required')
        ids.add(capture)
        for key in ('source_packet_sha256', 'calibration_sha256'):
            if not _digest(meta.get(key)): raise ValueError('Actual image/calibration SHA-256 required')
        if type(meta.get('replay')) is not bool or not isinstance(meta.get('source'), str): raise ValueError('Explicit camera source/replay provenance required')
        stamps.append(_stamp(meta['captured_at_utc'])); epochs.add(meta.get('holding_epoch')); modes.add(meta['replay'])
        transform(view['T_cell_tcp']); transform(view['packet']['T_cell_camera'])
    if len(modes) != 1 or any(stamp > now for stamp in stamps) or (now-max(stamps)).total_seconds() > max_age_s:
        raise ValueError('Current RGB-D is stale, future-dated or mixes live/replay')
    span = (max(stamps)-min(stamps)).total_seconds()
    if span > max_pair_span_s or (now-min(stamps)).total_seconds() > max_pair_span_s:
        raise ValueError('Inspection images exceed bounded holding interval')
    if span > max_age_s and (len(epochs) != 1 or None in epochs or not next(iter(epochs))):
        raise ValueError('Longer actual inspection motion requires one unchanged logical holding epoch')
    return [v['metadata']['capture_id'] for v in observations]


def _hand(state, expected_grasped, now_utc, max_age_s):
    if state.get('completed') is not True or state.get('is_grasped') is not expected_grasped:
        raise ValueError('Current actual native Hand acknowledgement/grasp state required')
    width = state.get('width_m'); maximum = state.get('max_width_m'); robot_time = state.get('robot_time_s')
    if (type(width) not in (int, float) or not np.isfinite(width) or not 0 <= width <= .0805 or
            type(maximum) not in (int, float) or not np.isfinite(maximum) or not .078 <= maximum <= .081 or
            type(robot_time) not in (int, float) or not np.isfinite(robot_time) or robot_time < 0):
        raise ValueError('Actual Hand width/max-width/SDK robot timestamp required')
    if state.get('captured_at_utc') is not None:
        age = (_stamp(now_utc or datetime.now(timezone.utc))-_stamp(state['captured_at_utc'])).total_seconds()
        if not 0 <= age <= max_age_s: raise ValueError('Actual Hand sample is stale')
    if not expected_grasped and width < .078: raise ValueError('Empty proof requires actual open native Hand>=78mm')


def _cloud(view, center, radius):
    packet = view['packet']; depth = np.asarray(packet['depth_m'], float); k = np.asarray(packet['K'], float); t = transform(packet['T_cell_camera'])
    if depth.ndim != 2 or min(depth.shape) < 16 or k.shape != (3, 3) or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError('Organized calibrated actual depth image required')
    sdk = packet.get('points_camera_image_m', packet.get('points_camera_m'))
    valid = np.isfinite(depth) & (depth > .05) & (depth < 5)
    if sdk is not None:
        camera = np.asarray(sdk, float)
        if camera.shape != (*depth.shape, 3): raise ValueError('SDK points must retain pixel correspondence')
        valid &= np.isfinite(camera).all(2)
        if np.any(abs(camera[:, :, 2][valid]-depth[valid]) > .001): raise ValueError('SDK XYZ/depth disagreement')
    else:
        if view['metadata'].get('rectified_registered_depth') is not True: raise ValueError('Use authentic SDK XYZ or explicitly rectified depth')
        yy, xx = np.indices(depth.shape); camera = np.stack([(xx-k[0, 2])*depth/k[0, 0], (yy-k[1, 2])*depth/k[1, 1], depth], -1)
    # An image-footprint validity gate covers the whole potential extra-item
    # ROI, including rays with no returned point. Missing metal depth is not
    # silently discarded as empty space.
    signs = np.array([[x, y, z] for x in [-1, 1] for y in [-1, 1] for z in [-1, 1]])
    corners = (center+signs*radius-t[:3, 3]) @ t[:3, :3]
    if np.any(corners[:, 2] <= .05): raise ValueError('Held inventory ROI is too close to camera')
    pixel = corners[:, :2]/corners[:, 2:] * [k[0, 0], k[1, 1]] + [k[0, 2], k[1, 2]]
    pad = view['metadata'].get('ray_footprint_padding_px', 0)
    low = np.floor(pixel.min(0)-pad).astype(int); high = np.ceil(pixel.max(0)+pad).astype(int)
    if np.any(low < 0) or np.any(high >= [depth.shape[1], depth.shape[0]]): raise ValueError('Broad held inventory ROI is partly outside actual camera view')
    footprint_valid = float(valid[low[1]:high[1]+1, low[0]:high[0]+1].mean())
    if footprint_valid < .95: raise ValueError('Held inventory ROI contains unobserved/missing depth; another actual view required')
    points = camera[valid] @ t[:3, :3].T+t[:3, 3]
    points = points[(abs(points-center) <= radius).all(1)]
    # Only actual-encoder model surfaces may explain known robot pixels.
    robot = view.get('known_robot_points_cell_m')
    if robot is not None:
        if view.get('robot_geometry_source') != 'actual_sdk_fk' or view.get('robot_geometry_capture_id') != view['metadata']['capture_id']:
            raise ValueError('Self-geometry exclusion must match this actual SDK/image capture')
        robot = np.asarray(robot, float)
        if robot.ndim != 2 or robot.shape[1] != 3 or not np.isfinite(robot).all() or len(robot) < 10:
            raise ValueError('Actual modeled robot surface samples required')
        self_tolerance = float(view.get('self_surface_tolerance_m', .001))
        if not .0001 <= self_tolerance <= .003: raise ValueError('Actual robot self-surface tolerance must be bounded at3mm')
        if len(points): points = points[cKDTree(robot).query(points)[0] > self_tolerance]
    return points, footprint_valid


def _coverage(points):
    if len(points) < 50: return 0.
    angles = np.sort(np.arctan2(points[:, 1], points[:, 0]))
    return float(np.rad2deg(2*np.pi-np.max(np.diff(np.r_[angles, angles[0]+2*np.pi]))))


def _components(points, radius):
    if len(points) == 0: return []
    resolution = .003; shape = int(np.ceil(2*radius/resolution))+1
    indices = np.clip(np.floor((points+radius)/resolution).astype(int), 0, shape-1)
    grid = np.zeros((shape,)*3, bool); grid[tuple(indices.T)] = True
    regions, _ = label(grid, np.ones((3, 3, 3))); counts = np.bincount(regions[tuple(indices.T)])
    return sorted((int(v) for v in counts[1:] if v >= 10), reverse=True)


def verify_single_inventory(part, observations, models, hand_state, *, roi_radius_m=.08, surface_tolerance_m=.0025,
                            max_age_s=2., max_pair_span_s=15., now_utc=None):
    result = dict(verified=False, part=part, reason='unverified', uses_simulator_body_poses=False,
                  scope='One observed class-consistent ring across actual complementary cap views; not a nearest-object crop')
    try:
        if part not in ('ring1', 'ring2'): raise ValueError('Loose-stock inventory guard supports the two known ring classes only')
        if not .06 <= roi_radius_m <= .12 or not .0005 <= surface_tolerance_m <= .0025: raise ValueError('Bounded broad ROI and at-most2.5mm CAD residual required')
        ids = _frames(observations, max_age_s, max_pair_span_s, now_utc); _hand(hand_state, True, now_utc, max_age_s)
        model = np.asarray(models[part], float)
        if model.ndim != 2 or model.shape[1] != 3 or len(model) < 100 or not np.isfinite(model).all(): raise ValueError('Known ring class CAD surface samples required')
        if np.max(abs(model)) > roi_radius_m-.01: raise ValueError('Broad held ROI must contain the entire selected class with margin')
        tree = cKDTree(model); lower, upper = np.quantile(model[:, 2], [.005, .995]); grasps = [np.linalg.inv(transform(v['T_cell_tcp']))@transform(v['T_cell_part']) for v in observations]; views = []; seen = set()
        origin = grasps[0]; translation = max(float(np.linalg.norm(g[:3, 3]-origin[:3, 3])) for g in grasps)
        axis = max(float(np.arccos(np.clip(g[:3, 2]@origin[:3, 2], -1, 1))) for g in grasps)
        result.update(source_capture_ids=ids, views=views, maximum_grasp_translation_change_m=translation, maximum_grasp_axis_change_rad=axis,observed_unexplained_point_count=0)
        for view in observations:
            tpart = transform(view['T_cell_part'])
            center = tpart[:3, 3]; points, validity = _cloud(view, center, roi_radius_m)
            local = (points-center) @ tpart[:3, :3]; distances = tree.query(local)[0] if len(local) else np.array([])
            matching = distances <= surface_tolerance_m; unexplained = int((~matching).sum())
            result['observed_unexplained_point_count'] += unexplained
            if unexplained > 8:
                views.append(dict(capture_id=view['metadata']['capture_id'],observed_points=len(points),unexplained_points=unexplained,class_matching_points=int(matching.sum()),ROI_valid_depth_fraction=validity))
                raise ValueError('Visible held geometry is not explained by ONE selected ring; extra item/contact or insufficient self-model')
            direction = (transform(view['packet']['T_cell_camera'])[:3, 3]-center) @ tpart[:3, :3]
            direction /= max(np.linalg.norm(direction), 1e-12); sign = 1 if direction[2] > 0 else -1
            level = upper if sign > 0 else lower; cap = local[matching & (abs(local[:, 2]-level) < .0008)]
            coverage = _coverage(cap)
            visible = bool(abs(direction[2]) >= .3 and coverage >= 240 and len(cap) >= 50)
            if visible: seen.add(sign)
            views.append(dict(capture_id=view['metadata']['capture_id'], observed_points=len(points), unexplained_points=unexplained,
                              class_matching_points=int(matching.sum()), visible_cap_sign=sign, visible_cap_points=len(cap), cap_coverage_deg=coverage,
                              direct_cap_visible=visible, ROI_valid_depth_fraction=validity, observed_components_point_counts=_components(local, roi_radius_m)))
        result.update(source_capture_ids=ids, views=views, maximum_grasp_translation_change_m=translation, maximum_grasp_axis_change_rad=axis,
                      actual_hand_width_m=hand_state['width_m'])
        if translation > .002 or axis > .05: raise ValueError('Actual image/encoder grasp is not stable through inspection')
        if seen != {-1, 1}: raise ValueError('Hidden stacked-item ambiguity: need actual complementary cap views, optionally reorient with one fixed camera')
        result.update(verified=True, reason='single_class_geometry_and_stable_grasp_observed', conditional_inventory_contract='Only the two known rigid ring classes; both caps and broad ROI directly observed')
    except (ValueError, KeyError, TypeError, IndexError) as error:
        result['reason'] = str(error)
    return result


def verify_empty_hand(observations, hand_state, *, roi_radius_m=.08, max_age_s=2., max_pair_span_s=15., now_utc=None):
    result = dict(verified=False, reason='unverified', uses_simulator_body_poses=False,
                  scope='Actual open Hand plus broad camera ROI after a real release; unexplained depth is not deleted')
    try:
        if not .06 <= roi_radius_m <= .12: raise ValueError('Broad actual Hand ROI required')
        ids = _frames(observations, max_age_s, max_pair_span_s, now_utc); _hand(hand_state, False, now_utc, max_age_s)
        views = []; directions = []; result.update(source_capture_ids=ids,views=views,observed_unexplained_point_count=0)
        for view in observations:
            tcp = transform(view['T_cell_tcp']); center = tcp[:3, 3]; points, validity = _cloud(view, center, roi_radius_m)
            result['observed_unexplained_point_count'] += len(points)
            if len(points) > 8:
                views.append(dict(capture_id=view['metadata']['capture_id'],unexplained_points=len(points),ROI_valid_depth_fraction=validity))
                raise ValueError('Empty Hand not observed: unexplained object or unmodeled robot surface in broad ROI')
            direction = (transform(view['packet']['T_cell_camera'])[:3, 3]-center) @ tcp[:3, :3]; direction /= max(np.linalg.norm(direction), 1e-12)
            directions.append(direction); views.append(dict(capture_id=view['metadata']['capture_id'], unexplained_points=len(points), ROI_valid_depth_fraction=validity))
        separation = max(float(np.arccos(np.clip(a@b, -1, 1))) for a in directions for b in directions)
        if separation < np.pi/3: raise ValueError('Hidden-item ambiguity: actual complementary empty-Hand inspection views required')
        result.update(verified=True, reason='open_hand_and_no_observed_carried_geometry', source_capture_ids=ids, views=views,
                      maximum_hand_relative_view_separation_rad=separation, actual_hand_width_m=hand_state['width_m'])
    except (ValueError, KeyError, TypeError, IndexError) as error:
        result['reason'] = str(error)
    return result
