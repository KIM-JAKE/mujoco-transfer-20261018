"""Conservative, label-free RGB-D residual stock in a calibrated cell ROI.

Only whole voxels certified by valid ray footprints become free. Surface hits
and everything behind them remain blocking; invalid depth never becomes free.
Selected-contact metadata labels observed surfaces only and does not alter the
occupancy grid. This is a finite-resolution sensor envelope, not hidden-object
recognition or a claim that a single overhead view proves a side grasp clear.
"""
from dataclasses import dataclass
from datetime import datetime, timezone
import base64
import hashlib
import json
import zlib
import numpy as np
from scipy.ndimage import minimum_filter
from scipy.spatial import cKDTree

UNKNOWN, FREE, OCCUPIED = 0, 1, 2


def _utc(value):
    if isinstance(value, datetime):
        result = value
    elif isinstance(value, str):
        result = datetime.fromisoformat(value.replace('Z', '+00:00'))
    else:
        raise ValueError('Explicit UTC capture timestamp required')
    if result.tzinfo is None:
        raise ValueError('Timezone-aware capture timestamp required')
    return result.astimezone(timezone.utc)


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def _sha(value):
    return hashlib.sha256(value).hexdigest()


def _hex(value, name):
    if not isinstance(value, str) or len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
        raise ValueError(name + ' must be an actual lowercase SHA-256')
    return value


def _fresh(sources, max_age_s, now_utc, static_scene_lease=None):
    ceiling=2
    if static_scene_lease is not None:
        from .static_scene_lease import validate
        if len(sources)!=1:raise ValueError('Static scene lease currently requires one exact initial camera source')
        max_age_s=validate(static_scene_lease,now=_utc(now_utc or datetime.now(timezone.utc)),expected_capture_id=sources[0]['capture_id'],expected_packet_sha256=sources[0]['source_packet_sha256']);ceiling=15
    if not np.isfinite(max_age_s) or not 0 < max_age_s <= ceiling:
        raise ValueError('Occupancy maximum observation age must be in (0,2] seconds')
    now = _utc(now_utc or datetime.now(timezone.utc))
    for source in sources:
        age = (now - _utc(source['captured_at_utc'])).total_seconds()
        if not 0 <= age <= max_age_s:
            raise ValueError('Occupancy contains a future or stale source capture')


def _geometry(bounds, voxel_size_m, max_voxels):
    bounds = np.asarray(bounds, float)
    if bounds.shape != (2, 3) or not np.isfinite(bounds).all() or np.any(bounds[0] >= bounds[1]) or np.max(abs(bounds)) > 10:
        raise ValueError('Finite metric cell-frame ROI bounds (2,3) required')
    if not .002 <= voxel_size_m <= .05 or np.any(bounds[1] - bounds[0] > 2):
        raise ValueError('Voxel resolution must be 2–50mm inside a bounded stock ROI')
    shape = tuple(map(int, np.ceil((bounds[1] - bounds[0]) / voxel_size_m)))
    if not 0 < np.prod(shape) <= min(int(max_voxels), 2000000):
        raise ValueError('Residual occupancy voxel budget exceeded')
    spacing = (bounds[1] - bounds[0]) / shape
    return bounds, shape, spacing


def _transform(value):
    t = np.asarray(value, float)
    if (t.shape != (4, 4) or not np.isfinite(t).all() or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-9)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-7)
            or abs(np.linalg.det(t[:3, :3]) - 1) > 1e-7):
        raise ValueError('Proper calibrated T_cell_camera required')
    return t


def _source(metadata, packet):
    for key in ('capture_id', 'scene_epoch', 'cell_frame', 'source'):
        if not isinstance(metadata.get(key), str) or not metadata[key].strip():
            raise ValueError('Occupancy source requires named ' + key)
    if type(metadata.get('replay')) is not bool:
        raise ValueError('Explicit replay/live source convention required')
    if metadata.get('rectified_registered_depth') is not True or metadata.get('projection_kind') != 'rectified_pinhole':
        raise ValueError('Residual free-ray proof requires authentically rectified registered pinhole depth')
    row = {key: metadata[key] for key in ('capture_id', 'scene_epoch', 'cell_frame', 'source', 'replay')}
    row['captured_at_utc'] = _utc(metadata.get('captured_at_utc')).isoformat()
    for key in ('calibration_sha256', 'source_packet_sha256'):
        row[key] = _hex(metadata.get(key), key)
    row['projection_kind'] = 'rectified_pinhole'
    padding = metadata.get('ray_footprint_padding_px', 0)
    if type(padding) is not int or not 0 <= padding <= 4:
        raise ValueError('Rectified ray footprint padding must be an integer0–4pixels')
    row['ray_footprint_padding_px'] = padding
    row['depth_uncertainty_m'] = float(metadata.get('depth_uncertainty_m', float('nan')))
    if not np.isfinite(row['depth_uncertainty_m']) or row['depth_uncertainty_m'] < 0:
        raise ValueError('Measured nonnegative depth uncertainty required')
    if metadata.get('rectification_provenance') is not None:
        row['rectification_provenance'] = metadata['rectification_provenance']
    digest = hashlib.sha256()
    for key in ('depth_m', 'K', 'T_cell_camera', 'points_camera_image_m', 'points_camera_m'):
        if key not in packet:
            continue
        array = np.ascontiguousarray(packet[key], dtype='<f8')
        digest.update(key.encode()); digest.update(str(array.shape).encode()); digest.update(array.tobytes())
    row['camera_array_sha256'] = digest.hexdigest()
    _canonical(row)
    return row


def _rectangle_depth_minimum(depth, low, high):
    """Exact inclusive pixel-rectangle minima, using four contained blocks.

    Each dyadic block lies entirely INSIDE the projected rectangle. Their
    union covers every pixel, including missing-depth zeros. Unlike a larger
    padded window, unrelated foreground outside the footprint cannot veto a
    genuinely observed free voxel. The caller has already bounded pixels.
    """
    low=np.asarray(low,int);high=np.asarray(high,int)
    if low.shape != high.shape or low.ndim != 2 or low.shape[1] != 2:
        raise ValueError('Bounded inclusive projected pixel rectangles required')
    if np.any(low<0) or np.any(high<low) or np.any(high>=[depth.shape[1],depth.shape[0]]):
        raise ValueError('Projected pixel rectangle leaves actual depth image')
    if not len(low):return np.empty(0,float)
    widths=high-low+1;blocks=np.left_shift(1,np.floor(np.log2(widths)).astype(int));result=np.empty(len(low),float)
    for bx,by in np.unique(blocks,axis=0):
        ids=np.flatnonzero((blocks==[bx,by]).all(1))
        # Negative origins make this an inclusive TOP-LEFT anchored block.
        minima=minimum_filter(depth,size=(int(by),int(bx)),origin=(-(int(by)//2),-(int(bx)//2)),mode='constant',cval=0.)
        x0,y0=low[ids].T;x1,y1=(high[ids]-[bx,by]+1).T
        result[ids]=np.minimum.reduce([minima[y0,x0],minima[y0,x1],minima[y1,x0],minima[y1,x1]])
    return result


def _merged_ranges(mask):
    """Exact disjoint axis-aligned box cover; never fills a free grid hole."""
    rectangles = []
    for x in range(mask.shape[0]):
        active = {}
        for y in range(mask.shape[1] + 1):
            intervals = set()
            if y < mask.shape[1]:
                transitions = np.diff(np.r_[False, mask[x, y], False].astype(np.int8))
                intervals = set(zip(np.flatnonzero(transitions == 1), np.flatnonzero(transitions == -1)))
            for interval in list(active):
                if interval not in intervals:
                    rectangles.append((x, x + 1, active.pop(interval), y, *interval))
            for interval in intervals:
                active.setdefault(interval, y)
    grouped = {}
    for rectangle in rectangles:
        grouped.setdefault(rectangle[2:], []).append(rectangle[0])
    merged = []
    for (y0, y1, z0, z1), values in sorted(grouped.items()):
        xs = sorted(values)
        if not xs:
            continue
        start = previous = xs[0]
        for x in xs[1:] + [xs[-1] + 2]:
            if x != previous + 1:
                merged.append((start, previous + 1, y0, y1, z0, z1)); start = x
            previous = x
    return merged


@dataclass(frozen=True)
class OccupancySnapshot:
    bounds_cell_m: tuple
    shape: tuple
    voxel_size_m: float
    depth_margin_m: float
    _states: bytes
    _selected: bytes
    _unselected_hits: bytes
    _sources_json: str
    selected_contact_instance_id: str | None = None

    @property
    def states(self):
        return np.frombuffer(self._states, np.uint8).reshape(self.shape)

    @property
    def sources(self):
        return json.loads(self._sources_json)

    def assert_fresh(self, *, max_age_s=2., now_utc=None):
        _fresh(self.sources, max_age_s, now_utc)

    def planner_boxes(self, *, max_boxes=5000):
        """Unknown boxes are NEVER contact-exempt. No voxel is deleted here.

        Selected metadata is eligible for a consumer's named finger-only
        contact rule, not for removal from palm, arm or payload collision tests.
        Unknown neighbouring/behind voxels always retain independent boxes.
        """
        bounds = np.asarray(self.bounds_cell_m); spacing = (bounds[1] - bounds[0]) / self.shape
        selected = np.frombuffer(self._selected, np.uint8).reshape(self.shape).astype(bool)
        result = []; sources = self.sources; source_ids = [s['capture_id'] for s in sources]; prefix = self.fingerprint[:12]
        for state, eligible in ((UNKNOWN, False), (OCCUPIED, False), (OCCUPIED, True)):
            mask = self.states == state
            if state == OCCUPIED:
                mask &= selected if eligible else ~selected
            for x0, x1, y0, y1, z0, z1 in _merged_ranges(mask):
                low = bounds[0] + spacing * [x0, y0, z0]; high = bounds[0] + spacing * [x1, y1, z1]
                t = np.eye(4); t[:3, 3] = (low + high) / 2
                row = dict(name='residual_' + prefix + '_' + str(len(result)),
                           T_cell_box=t.tolist(), half_extents_m=((high - low) / 2).tolist(),
                           source='conservative_registered_RGBD_rays', occupancy_kind='unknown' if state == UNKNOWN else 'occupied',
                           source_capture_ids=source_ids, voxel_resolution_m=spacing.tolist())
                if eligible:
                    row['selected_contact_instance_id'] = self.selected_contact_instance_id
                    row['selected_contact_scope'] = 'observed selected surface + CAD shell + bounded contact corridor; named fingers only'
                result.append(row)
                if len(result) > max_boxes:
                    raise ValueError('Residual occupancy box budget exceeded; acquire a clearer view or choose a declared coarser resolution')
        return result

    @property
    def fingerprint(self):
        return _sha(self._states + self._selected + self._unselected_hits + self._sources_json.encode())

    def to_record(self):
        row = dict(schema_version=1, kind='ray_residual_stock_occupancy', units='m', bounds_cell_m=self.bounds_cell_m,
                   shape=self.shape, voxel_size_m=self.voxel_size_m, depth_margin_m=self.depth_margin_m,
                   states_zlib_b64=base64.b64encode(zlib.compress(self._states)).decode(),
                   selected_zlib_b64=base64.b64encode(zlib.compress(self._selected)).decode(),
                   unselected_hits_zlib_b64=base64.b64encode(zlib.compress(self._unselected_hits)).decode(),
                   selected_contact_instance_id=self.selected_contact_instance_id, sources=self.sources,
                   state_codes=dict(unknown=UNKNOWN, free=FREE, occupied=OCCUPIED),
                   policy='Only complete valid depth-ray footprints certify free voxels; invalid/occluded space blocks motion',
                   uses_simulator_object_poses=False)
        row['record_sha256'] = _sha(_canonical(row).encode())
        return json.loads(_canonical(row))

    @classmethod
    def from_record(cls, record, *, expected_capture_id=None, expected_calibration_sha256=None, max_age_s=2., now_utc=None, static_scene_lease=None):
        row = json.loads(_canonical(record)); checksum = row.pop('record_sha256', None)
        if checksum != _sha(_canonical(row).encode()) or row.get('schema_version') != 1 or row.get('kind') != 'ray_residual_stock_occupancy' or row.get('uses_simulator_object_poses') is not False:
            raise ValueError('Invalid or modified residual occupancy record')
        bounds, shape, spacing = _geometry(row['bounds_cell_m'], row['voxel_size_m'], 2000000)
        if tuple(row['shape']) != shape or row.get('state_codes') != dict(unknown=0, free=1, occupied=2):
            raise ValueError('Occupancy grid layout mismatch')
        size = int(np.prod(shape)); arrays = []
        for key in ('states_zlib_b64', 'selected_zlib_b64', 'unselected_hits_zlib_b64'):
            inflater = zlib.decompressobj(); value = inflater.decompress(base64.b64decode(row[key], validate=True), size + 1)
            if not inflater.eof or len(value) != size:
                raise ValueError('Invalid occupancy grid encoding')
            arrays.append(value)
        states, selected, unselected = [np.frombuffer(x, np.uint8) for x in arrays]
        if np.any(states > 2) or np.any(selected > 1) or np.any(unselected > 1) or np.any((selected == 1) & ((states != OCCUPIED) | (unselected == 1))):
            raise ValueError('Invalid occupancy/contact state')
        sources = row['sources']
        if not sources or len({s['capture_id'] for s in sources}) != len(sources):
            raise ValueError('Unique camera source provenance required')
        for source in sources:
            for key in ('calibration_sha256', 'source_packet_sha256', 'camera_array_sha256'):
                _hex(source[key], key)
            if source.get('projection_kind') != 'rectified_pinhole' or type(source.get('replay')) is not bool:
                raise ValueError('Invalid projection/source provenance')
        if len({(s['scene_epoch'], s['cell_frame'], s['replay']) for s in sources}) != 1:
            raise ValueError('Occupancy cannot fuse different stock epochs, cell frames or replay/live sources')
        if expected_capture_id is not None and sources[-1]['capture_id'] != expected_capture_id:
            raise ValueError('Occupancy belongs to a different current image')
        if expected_calibration_sha256 is not None and sources[-1]['calibration_sha256'] != expected_calibration_sha256:
            raise ValueError('Occupancy calibration does not match current image')
        _fresh(sources, max_age_s, now_utc,static_scene_lease)
        return cls(tuple(map(tuple, bounds)), shape, row['voxel_size_m'], row['depth_margin_m'], *arrays, _canonical(sources), row.get('selected_contact_instance_id'))


def build_occupancy(packet, metadata, bounds_cell_m, *, voxel_size_m=.008, depth_margin_m=.003,
                    prior=None, selected_contact=None, max_age_s=2., now_utc=None, max_voxels=200000):
    """Fuse one fresh stopped-scene view; stock movement requires a NEW epoch.

    Rectification must preserve invalid holes and camera-Z depth semantics.
    selected_contact is optional metadata, not a class/scene identity oracle.
    It requires capture_id, instance_id, a boolean image surface_mask, measured
    class-pose CAD surface samples and bounded contact_bounds_cell_m.
    """
    bounds, shape, spacing = _geometry(bounds_cell_m, voxel_size_m, max_voxels)
    depth = np.asarray(packet['depth_m'], float); k = np.asarray(packet['K'], float); t = _transform(packet['T_cell_camera'])
    if depth.ndim != 2 or min(depth.shape) < 4 or k.shape != (3, 3) or not np.isfinite(k).all() or not np.allclose(k[2], [0, 0, 1]) or k[0, 0] <= 0 or k[1, 1] <= 0 or abs(k[0, 1]) > 1e-9 or abs(k[1, 0]) > 1e-9:
        raise ValueError('Rectified camera-Z depth image and zero-skew pinhole K required')
    source = _source(metadata, packet); sources = [source]; _fresh(sources, max_age_s, now_utc)
    if not .0001 <= depth_margin_m <= .02 or source['depth_uncertainty_m'] > depth_margin_m:
        raise ValueError('Depth margin must cover the measured uncertainty')
    grid_indices = np.indices(shape).reshape(3, -1).T
    centers = bounds[0] + (grid_indices + .5) * spacing
    signs = np.array([[x, y, z] for x in [-1, 1] for y in [-1, 1] for z in [-1, 1]])
    corners = centers[:, None, :] + signs * spacing / 2
    camera = (corners - t[:3, 3]) @ t[:3, :3]
    usable = np.all(camera[:, :, 2] > .05, axis=1)
    projected = camera[:, :, :2] / np.maximum(camera[:, :, 2:], .00001)
    projected = projected * [k[0, 0], k[1, 1]] + [k[0, 2], k[1, 2]]
    padding = source['ray_footprint_padding_px']
    low = np.floor(projected.min(1) - .5).astype(int) - padding; high = np.ceil(projected.max(1) + .5).astype(int) + padding
    usable &= (low >= 0).all(1) & (high < [depth.shape[1], depth.shape[0]]).all(1)
    middle = (low + high) // 2; radii = np.maximum(middle - low, high - middle)
    free = np.zeros(len(centers), bool); valid_depth = np.isfinite(depth) & (depth > .05) & (depth < 5)
    organized = packet.get('points_camera_image_m', packet.get('points_camera_m'))
    if organized is not None:
        organized = np.asarray(organized, float)
        if organized.shape != (*depth.shape, 3):
            raise ValueError('SDK residual depth points must retain HxWx3 pixel correspondence')
        valid_depth &= np.isfinite(organized).all(2)
        if np.any(abs(organized[:, :, 2][valid_depth] - depth[valid_depth]) > .001):
            raise ValueError('SDK point camera-Z disagrees with registered depth')
        yy, xx = np.nonzero(valid_depth)
        rays = organized[yy, xx, :2] / organized[yy, xx, 2:]
        projected_sdk = rays * [k[0, 0], k[1, 1]] + [k[0, 2], k[1, 2]]
        if len(rays) and np.max(abs(projected_sdk - np.c_[xx, yy])) > padding + .001:
            raise ValueError('Actual SDK rays exceed declared rectification footprint padding')
    conservative_depth = np.where(valid_depth, depth, 0.)
    # Group pixel footprints into larger power-of-two windows. Every selected
    # minimum-filter window encloses the ENTIRE voxel projection, including
    # invalid pixels and narrow foreground occluders between its corner rays.
    rounded = np.power(2, np.ceil(np.log2(np.maximum(radii, 1)))).astype(int)
    for rx, ry in np.unique(rounded[usable], axis=0):
        ids = np.flatnonzero(usable & (rounded == [rx, ry]).all(1))
        minimum = minimum_filter(conservative_depth, size=(2 * ry + 1, 2 * rx + 1), mode='constant', cval=0.)
        free[ids] = camera[ids, :, 2].max(1) + depth_margin_m < minimum[middle[ids, 1], middle[ids, 0]]
    # Refine only false negatives from the larger power-of-two windows.
    # This remains the SAME whole projected footprint (including SDK padding),
    # same all-valid rule and same worst corner depth/uncertainty margin.
    unresolved=np.flatnonzero(usable & ~free)
    if len(unresolved):
        exact=_rectangle_depth_minimum(conservative_depth,low[unresolved],high[unresolved])
        free[unresolved]=camera[unresolved,:,2].max(1)+depth_margin_m<exact
    states = np.where(free, FREE, UNKNOWN).astype(np.uint8)
    vv, uu = np.nonzero(valid_depth); zz = depth[vv, uu]
    points_camera = organized[vv, uu] if organized is not None else np.c_[(uu - k[0, 2]) * zz / k[0, 0], (vv - k[1, 2]) * zz / k[1, 1], zz]
    points = points_camera @ t[:3, :3].T + t[:3, 3]
    grid = np.floor((points - bounds[0]) / spacing).astype(int)
    inside = ((grid >= 0) & (grid < shape)).all(1); points, grid, vv, uu = points[inside], grid[inside], vv[inside], uu[inside]
    flat = np.ravel_multi_index(grid.T, shape) if len(grid) else np.array([], int)
    states[np.unique(flat)] = OCCUPIED
    selected = np.zeros(len(states), np.uint8); unselected = np.zeros(len(states), np.uint8); instance = None
    allowed_pixels = np.zeros(len(flat), bool)
    if selected_contact is not None:
        contact = selected_contact; instance = contact.get('instance_id')
        if contact.get('capture_id') != source['capture_id'] or not isinstance(instance, str) or not instance.startswith(source['capture_id'] + ':'):
            raise ValueError('Selected contact must identify a proposal from this exact RGB-D capture')
        mask = np.asarray(contact['surface_mask']); surface = np.asarray(contact['cad_surface_points_cell_m'], float); corridor = np.asarray(contact['contact_bounds_cell_m'], float)
        tolerance = float(contact.get('shell_tolerance_m', .002))
        if mask.dtype != bool or mask.shape != depth.shape or surface.ndim != 2 or surface.shape[1] != 3 or not 10 <= len(surface) <= 50000 or not np.isfinite(surface).all():
            raise ValueError('Selected contact requires a boolean pixel mask and observed-pose class CAD surface samples')
        if corridor.shape != (2, 3) or not np.isfinite(corridor).all() or np.any(corridor[0] >= corridor[1]) or np.any(corridor[1] - corridor[0] > .15) or not .0001 <= tolerance <= .003:
            raise ValueError('Selected contact requires a bounded corridor and at-most3mm surface shell')
        distance = cKDTree(surface).query(points)[0] if len(points) else np.array([])
        allowed_pixels = mask[vv, uu] & (distance <= tolerance) & ((points >= corridor[0]) & (points <= corridor[1])).all(1)
        selected[np.unique(flat[allowed_pixels])] = 1
    unselected[np.unique(flat[~allowed_pixels])] = 1
    selected[unselected == 1] = 0
    if prior is not None:
        if not isinstance(prior, OccupancySnapshot) or prior.shape != shape or not np.array_equal(prior.bounds_cell_m, bounds) or prior.voxel_size_m != voxel_size_m or prior.depth_margin_m != depth_margin_m:
            raise ValueError('Fusion requires the same declared cell grid and uncertainty margins')
        old_sources = prior.sources; _fresh(old_sources, max_age_s, now_utc)
        if any((s['scene_epoch'], s['cell_frame'], s['replay']) != (source['scene_epoch'], source['cell_frame'], source['replay']) for s in old_sources) or source['capture_id'] in {s['capture_id'] for s in old_sources}:
            raise ValueError('Fuse unique fresh captures only within one unchanged stock epoch/cell/source mode')
        old = prior.states.ravel(); states[(old == FREE) & (states == UNKNOWN)] = FREE; states[old == OCCUPIED] = OCCUPIED
        old_unselected = np.frombuffer(prior._unselected_hits, np.uint8)
        unselected |= old_unselected
        if instance == prior.selected_contact_instance_id:
            selected |= np.frombuffer(prior._selected, np.uint8)
        else:
            unselected[old == OCCUPIED] = 1
        selected[(unselected == 1) | (states != OCCUPIED)] = 0
        sources = old_sources + [source]
    return OccupancySnapshot(tuple(map(tuple, bounds)), shape, voxel_size_m, depth_margin_m,
                             states.tobytes(), selected.tobytes(), unselected.tobytes(), _canonical(sources), instance)
