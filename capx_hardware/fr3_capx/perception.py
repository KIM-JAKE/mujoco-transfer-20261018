"""Lightweight CaP-X perception adapters for FR3 Native Hand.

API compatibility reference: CaP-X (MIT), commit
53e9966d7a8e2fa7494676772bccc35280f5c0ed, capx/integrations/vision,
capx/integrations/franka/control.py and capx/utils/{camera,depth}_utils.py.
This is an independent implementation, not a copy of model weights or services.

All distances are metres. A matrix T_world_local maps local coordinates INTO
world coordinates (column vectors). Returned quaternions are unit WXYZ.
Camera coordinates are OpenCV optical: +X right, +Y down, +Z forward.
Grasp poses target the native_hand body origin, never an implicit Panda TCP.
No function here moves cameras, moves robots, or confirms a physical hold.

Backend contract: call(operation: str, payload: dict) -> dict. Unsupported
operations must raise NotImplementedError or return {"supported": False}.
capture returns rgb, depth_m, intrinsics, T_world_camera, frame='camera_optical',
depth_unit='m', calibration_id, observation_id, timestamp_s, and provenance.
Every response has provenance={source: <allowed sensor/estimator>, oracle: False}
and derived responses echo the same calibration_id, observation_id, timestamp_s.
Metadata is checked, but a trusted backend is still responsible for truthfully
reporting sensor lineage; a wrapper cannot authenticate arbitrary model output.
SAM/OWL/Molmo/ContactGraspNet wrappers are client-ready, not model-qualified.
"""

from __future__ import annotations

import inspect
import copy
import math
import time
from typing import Any, Callable

import numpy as np

from .backend import UnsupportedOperation


class PerceptionError(ValueError):
    """Invalid, stale, uncalibrated, or privileged perception evidence."""


class PerceptionUnavailable(NotImplementedError):
    """The backend or model service does not implement this operation."""


def validate_pose(pose: Any) -> np.ndarray:
    """Validate a finite proper rigid 4x4 transform; never repair bad input."""
    out = np.asarray(pose, dtype=float)
    if out.shape != (4, 4) or not np.isfinite(out).all():
        raise PerceptionError("pose must be a finite 4x4 transform")
    if not np.allclose(out[3], [0, 0, 0, 1], atol=1e-8, rtol=0):
        raise PerceptionError("pose has an invalid homogeneous last row")
    rotation = out[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0):
        raise PerceptionError("pose rotation must be orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6, rtol=0):
        raise PerceptionError("pose rotation must be right handed")
    return out.copy()


def pose_from_xyz_wxyz(position: Any, quaternion_wxyz: Any) -> np.ndarray:
    """Construct T_parent_local from XYZ metres and a unit WXYZ quaternion."""
    xyz = np.asarray(position, dtype=float)
    quat = np.asarray(quaternion_wxyz, dtype=float)
    if xyz.shape != (3,) or not np.isfinite(xyz).all():
        raise PerceptionError("position must have three finite metre values")
    if quat.shape != (4,) or not np.isfinite(quat).all():
        raise PerceptionError("quaternion must contain four finite WXYZ values")
    if not np.isclose(np.linalg.norm(quat), 1.0, atol=1e-6, rtol=0):
        raise PerceptionError("quaternion must already be unit length; no silent normalization")
    w, x, y, z = quat
    out = np.eye(4)
    out[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                   [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                   [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    out[:3, 3] = xyz
    return validate_pose(out)


def pose_to_xyz_wxyz(pose: Any) -> tuple[np.ndarray, np.ndarray]:
    """Convert a proper rigid transform to XYZ metres and WXYZ quaternion."""
    matrix = validate_pose(pose)
    # Local import avoids importing a model, ROS, simulator, or a renderer.
    from scipy.spatial.transform import Rotation
    xyzw = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    wxyz = xyzw[[3, 0, 1, 2]]
    if wxyz[0] < 0:
        wxyz = -wxyz
    return matrix[:3, 3].copy(), wxyz


def validate_intrinsics(intrinsics: Any) -> np.ndarray:
    """Validate pinhole K; distortion must have been removed before this API."""
    k = np.asarray(intrinsics, dtype=float)
    if (k.shape != (3, 3) or not np.isfinite(k).all()
            or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1], atol=1e-9, rtol=0)
            or abs(k[0, 1]) > 1e-9 or abs(k[1, 0]) > 1e-9):
        raise PerceptionError("invalid rectified pinhole intrinsics")
    return k.copy()


def _depth(depth: Any) -> np.ndarray:
    d = np.asarray(depth, dtype=float)
    if d.ndim != 2 or not d.size:
        raise PerceptionError("depth must be a nonempty HxW array in metres")
    return d


def _rgb(image: Any) -> np.ndarray:
    """Accept native uint8 or an exact JSON integer RGB representation."""
    rgb = np.asarray(image)
    if (isinstance(image, (list, tuple)) and rgb.dtype.kind in "ui"
            and rgb.size and np.all((rgb >= 0) & (rgb <= 255))):
        rgb = rgb.astype(np.uint8)
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise PerceptionError("RGB must be HxWx3 uint8 or exact JSON byte integers")
    return rgb


def deproject_pixel_to_camera(pixel: tuple[int, int], depth: float,
                             intrinsics: Any) -> np.ndarray:
    """CaP-X compatible pixel (x,y), axial depth in metres -> optical XYZ."""
    k = validate_intrinsics(intrinsics)
    p = np.asarray(pixel, dtype=float)
    if p.shape != (2,) or not np.isfinite(p).all() or not math.isfinite(depth) or depth <= 0:
        raise PerceptionError("pixel must be finite and axial depth positive and finite")
    return np.array([(p[0]-k[0, 2])*depth/k[0, 0],
                     (p[1]-k[1, 2])*depth/k[1, 1], depth])


def depth_to_pointcloud(depth: Any, intrinsics: Any, subsample_factor: int = 1,
                        depth_clip_range: tuple[float, float] = (0.015, 20.0),
                        filter_invalid: bool = True) -> np.ndarray:
    """CaP-X compatible rectified axial depth metres -> optical Nx3 points.

    Invalid depths are excluded by default. With filter_invalid=False the
    sampled pixel order is retained, including invalid positions. Odd image
    dimensions work with subsampling; K is applied to actual source pixels.
    """
    d, k = _depth(depth), validate_intrinsics(intrinsics)
    if isinstance(subsample_factor, bool) or not isinstance(subsample_factor, int) or subsample_factor < 1:
        raise PerceptionError("subsample_factor must be a positive integer")
    near, far = depth_clip_range
    if not np.isfinite([near, far]).all() or not 0 <= near < far:
        raise PerceptionError("invalid depth clip range")
    yy, xx = np.mgrid[0:d.shape[0]:subsample_factor, 0:d.shape[1]:subsample_factor]
    z = d[::subsample_factor, ::subsample_factor].reshape(-1)
    with np.errstate(invalid="ignore"):
        points = np.column_stack(((xx.ravel()-k[0, 2])*z/k[0, 0],
                                  (yy.ravel()-k[1, 2])*z/k[1, 1], z))
    if filter_invalid:
        return points[np.isfinite(points).all(axis=1) & (z >= near) & (z <= far) & (z > 0)]
    return points


def depth_color_to_pointcloud(depth: Any, img: Any, intrinsics: Any,
                              subsample_factor: int = 1,
                              depth_clip_range: tuple[float, float] = (0.015, 20.0)
                              ) -> tuple[np.ndarray, np.ndarray]:
    """CaP-X compatible aligned RGB-D -> optical points and RGB in [0,1]."""
    d, rgb = _depth(depth), np.asarray(img)
    if rgb.shape != (*d.shape, 3) or rgb.dtype != np.uint8:
        raise PerceptionError("RGB must be aligned HxWx3 uint8")
    points = depth_to_pointcloud(d, intrinsics, subsample_factor, depth_clip_range, False)
    near, far = depth_clip_range
    valid = np.isfinite(points).all(axis=1) & (points[:, 2] >= near) & (points[:, 2] <= far) & (points[:, 2] > 0)
    colors = rgb[::subsample_factor, ::subsample_factor].reshape(-1, 3).astype(float)/255
    return points[valid], colors[valid]


def obs_get_rgb(obs: dict[str, Any]) -> dict[str, np.ndarray]:
    """CaP-X compatible recursive extraction of camera RGB image dictionaries."""
    result = {}
    for name, value in obs.items():
        if isinstance(value, dict):
            images = value.get("images")
            if isinstance(images, dict) and "rgb" in images:
                result[name] = np.asarray(images["rgb"])
            else:
                result.update(obs_get_rgb(value))
    return result


class PerceptionApi:
    """Sensor-only API registry using an injected, explicit operation backend.

    No model is loaded at import or construction. A fresh image and matching
    calibration are mandatory. Grasp scoring is a candidate proposal, never
    evidence that the fingers have closed or that the pin survived transport.
    """

    SOURCES = frozenset({"rgbd", "real_rgbd", "rendered_rgbd", "recorded_rgbd",
                         "rgbd_estimate", "vision", "segmentation_model", "grasp_model"})

    def __init__(self, backend: Any, *, max_age_s: float = 2.0,
                 clock: Callable[[], float] = time.time) -> None:
        if not callable(getattr(backend, "call", None)):
            raise TypeError("backend must implement call(operation, payload)")
        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError("max_age_s must be positive")
        self.backend, self.max_age_s, self.clock = backend, float(max_age_s), clock
        self._observation = None
        self.last_pose = None
        self.last_grasp_candidate = None

    def _call(self, operation: str, payload: dict) -> dict:
        try:
            result = self.backend.call(operation, payload)
        except (NotImplementedError, UnsupportedOperation) as exc:
            raise PerceptionUnavailable(f"backend does not implement {operation}") from exc
        if not isinstance(result, dict):
            raise PerceptionError(f"{operation} must return a dictionary")
        if result.get("supported") is False:
            raise PerceptionUnavailable(f"backend does not implement {operation}")
        if result.get("error"):
            raise PerceptionUnavailable(f"{operation}: {result['error']}")
        return result

    def _evidence(self, record: dict, observation: dict | None = None) -> None:
        provenance = record.get("provenance")
        if not isinstance(provenance, dict) or provenance.get("source") not in self.SOURCES:
            raise PerceptionError("missing/unsupported sensor provenance; simulator body poses are forbidden")
        if provenance.get("oracle") is not False or any(
                provenance.get(key) for key in ("privileged", "ground_truth", "body_pose", "simulator_body_pose", "sim_body_pose")):
            raise PerceptionError("oracle/privileged simulator body-pose evidence is forbidden")
        if provenance.get("kind") in {"oracle", "ground_truth", "simulator_body_pose", "sim_body_pose", "privileged"}:
            raise PerceptionError("oracle/privileged provenance kind is forbidden")
        for key in ("observation_id", "calibration_id"):
            if not isinstance(record.get(key), str) or not record[key].strip():
                raise PerceptionError(f"missing {key}")
        timestamp = record.get("timestamp_s")
        if isinstance(timestamp, bool) or not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp):
            raise PerceptionError("timestamp_s must be a finite wall-clock timestamp")
        age = self.clock() - timestamp
        if age < -0.1 or age > self.max_age_s:
            raise PerceptionError("perception evidence is stale or from the future")
        if observation is not None:
            for key in ("observation_id", "calibration_id", "timestamp_s"):
                if record[key] != observation[key]:
                    raise PerceptionError(f"derived response has mismatched {key}")

    def capture(self, camera: str = "default") -> dict:
        """Capture aligned RGB-D with explicit rectified calibration and lineage."""
        self._observation = None
        result = self._call("capture", {"camera": camera})
        self._evidence(result)
        if result.get("frame") != "camera_optical" or result.get("depth_unit") != "m":
            raise PerceptionError("capture must explicitly use camera_optical coordinates and metre depth")
        if result.get("rectified") is not True or result.get("rgb_depth_aligned") is not True:
            raise PerceptionError("RGB-D must be explicitly rectified and RGB/depth aligned")
        depth = _depth(result.get("depth_m"))
        rgb = _rgb(result.get("rgb"))
        if rgb.shape != (*depth.shape, 3) or rgb.dtype != np.uint8:
            raise PerceptionError("capture RGB must be aligned HxWx3 uint8")
        valid_depth = depth[np.isfinite(depth) & (depth > 0)]
        if not valid_depth.size or np.any(valid_depth > 20):
            raise PerceptionError("no usable metre depths or depth unit/range mismatch")
        checked = dict(copy.deepcopy(result), rgb=rgb.copy(), depth_m=depth.copy(),
                       intrinsics=validate_intrinsics(result.get("intrinsics")),
                       T_world_camera=validate_pose(result.get("T_world_camera")))
        self._observation = checked
        return copy.deepcopy(checked)

    def get_observation(self, camera: str = "default") -> dict:
        """Capture using CaP-X's nested camera observation schema, plus lineage."""
        obs = self.capture(camera)
        xyz, wxyz = pose_to_xyz_wxyz(obs["T_world_camera"])
        return {camera: {"images": {"rgb": obs["rgb"], "depth": obs["depth_m"][..., None]},
                         "intrinsics": obs["intrinsics"], "pose": np.r_[xyz, wxyz],
                         "calibration_id": obs["calibration_id"], "observation_id": obs["observation_id"]},
                "timestamp_s": obs["timestamp_s"], "provenance": obs["provenance"]}

    def _current(self) -> dict:
        if self._observation is None:
            self.capture()
        self._evidence(self._observation)
        return self._observation

    def _request(self, operation: str, observation: dict, **payload: Any) -> dict:
        self._evidence(observation)
        result = self._call(operation, {"observation_id": observation["observation_id"],
                                      "calibration_id": observation["calibration_id"], **payload})
        self._evidence(result, observation)
        return result

    def get_object_pose(self, object_name: str, return_bbox_extent: bool = False
                        ) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
        """Estimate object world XYZ/WXYZ and optional full oriented box lengths.

        Requires a measured orientation; unobserved orientation is rejected
        rather than substituted with an identity or a guessed down quaternion.
        Symmetry-related orientation ambiguity belongs in the backend result.
        """
        obs = self.capture()
        result = self._request("get_object_pose", obs, object_name=self._name(object_name),
                               return_bbox_extent=bool(return_bbox_extent), frame="world")
        if result.get("frame") != "world" or result.get("orientation_observed") is not True:
            raise PerceptionError("object pose needs a world frame and observed orientation")
        matrix = self._result_pose(result)
        extent = None
        if return_bbox_extent:
            extent = np.asarray(result.get("bbox_extent"), dtype=float)
            if extent.shape != (3,) or not np.isfinite(extent).all() or np.any(extent <= 0):
                raise PerceptionError("bbox_extent must be three positive full lengths in metres")
        self.last_pose = dict(result, pose=matrix.copy())
        xyz, wxyz = pose_to_xyz_wxyz(matrix)
        return xyz, wxyz, extent

    @staticmethod
    def _name(name: str) -> str:
        if not isinstance(name, str) or not name.strip():
            raise PerceptionError("object_name must be a nonempty string")
        return name

    @staticmethod
    def _result_pose(result: dict) -> np.ndarray:
        if "pose" in result:
            return validate_pose(result["pose"])
        return pose_from_xyz_wxyz(result.get("position"), result.get("quaternion_wxyz"))

    def sample_grasp_pose(self, object_name: str) -> tuple[np.ndarray, np.ndarray]:
        """Propose a world Native Hand grasp XYZ/WXYZ; does NOT confirm a hold.

        The backend must have converted its network gripper convention to
        native_hand using an explicit calibrated transform. No Panda +120mm
        or other hardcoded end-effector offset is inherited from CaP-X.
        """
        obs = self.capture()
        result = self._request("sample_grasp_pose", obs, object_name=self._name(object_name),
                               frame="world", tool_frame="native_hand", proposal_only=True)
        if result.get("frame") != "world" or result.get("tool_frame") != "native_hand":
            raise PerceptionError("grasp must explicitly target world Native Hand pose")
        if result.get("hold_confirmed") not in (None, False):
            raise PerceptionError("a grasp prediction cannot confirm a physical hold")
        matrix = self._result_pose(result)
        self.last_grasp_candidate = dict(result, pose=matrix.copy(), proposal_only=True, hold_confirmed=False)
        return pose_to_xyz_wxyz(matrix)

    def transform_pose(self, pose: Any, source_frame: str, target_frame: str = "world") -> np.ndarray:
        """Transform T_source_local between calibrated camera_optical and world.

        Uses the current capture's T_world_camera, never a camera look-at or
        movable simulator viewpoint. Unknown or base/Hand frames are rejected.
        """
        matrix = validate_pose(pose)
        if source_frame not in {"world", "camera_optical"} or target_frame not in {"world", "camera_optical"}:
            raise PerceptionError("only calibrated world/camera_optical frames are registered")
        obs = self._current()
        if source_frame == target_frame:
            return matrix
        transform = obs["T_world_camera"]
        return validate_pose((transform if source_frame == "camera_optical" else np.linalg.inv(transform)) @ matrix)

    def _image(self, image: Any) -> tuple[np.ndarray, dict]:
        rgb, obs = _rgb(image), self._current()
        if rgb.dtype != np.uint8 or rgb.shape != obs["rgb"].shape or not np.array_equal(rgb, obs["rgb"]):
            raise PerceptionError("image must match the current calibrated RGB-D observation")
        return rgb, obs

    def segment(self, image: Any, text_prompt: str = "", *, model: str = "sam3",
                box_prompt: Any = None, point_coords: Any = None, max_masks: int | None = None,
                **model_options: Any) -> list[dict]:
        """Return checked segmentation proposals from a configured model backend."""
        rgb, obs = self._image(image)
        if max_masks is not None and (isinstance(max_masks, bool) or not isinstance(max_masks, int) or max_masks < 1):
            raise PerceptionError("max_masks must be a positive integer")
        result = self._request("segment", obs, image=rgb, model=model, text_prompt=text_prompt,
                               box_prompt=box_prompt, point_coords=point_coords, max_masks=max_masks,
                               model_options=model_options)
        rows = result.get("results")
        if not isinstance(rows, list):
            raise PerceptionError("segment backend must return a results list")
        checked = []
        for row in rows:
            mask = np.asarray(row.get("mask"))
            if mask.shape != rgb.shape[:2] or mask.dtype.kind not in "bui" or not np.isin(mask, [0, 1]).all():
                raise PerceptionError("segmentation mask must be binary and match the captured image")
            score = self._score(row.get("score"))
            checked.append(dict(row, mask=mask.astype(bool), score=score))
        return checked[:max_masks] if max_masks else checked

    @staticmethod
    def _score(value: Any) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float, np.number)) or not np.isfinite(value) or not 0 <= value <= 1:
            raise PerceptionError("model score must be a finite value in [0,1]")
        return float(value)

    def init_sam3(self, checkpoint_path=None, device="cuda", model_type="vit_l"):
        """CaP-X SAM3 signature; returns backend client, without loading weights."""
        return lambda image, text_prompt, box_prompt=None: self.segment(
            image, text_prompt, model="sam3", box_prompt=box_prompt,
            checkpoint_path=checkpoint_path, device=device, model_type=model_type)

    def init_sam2(self, model_name="facebook/sam2.1-hiera-large", device="cuda", points_per_batch=64):
        """CaP-X SAM2 signature; returns backend client, without loading weights."""
        return lambda rgb, max_masks=None, box=None: self.segment(
            rgb, model="sam2", box_prompt=box, max_masks=max_masks,
            model_name=model_name, device=device, points_per_batch=points_per_batch)

    def init_sam3_point_prompt(self, device="cuda"):
        """CaP-X SAM3 point client; returns list of mask/score dictionaries."""
        return lambda image, point_coords: self.segment(image, model="sam3", point_coords=point_coords, device=device)

    def init_sam2_point_prompt(self, model_name="facebook/sam2.1-hiera-large", device="cuda"):
        """CaP-X SAM2 point client; returns (scores, NxHxW boolean masks)."""
        def segment_point(image, point_coords):
            rows = self.segment(image, model="sam2", point_coords=point_coords, model_name=model_name, device=device)
            shape = np.asarray(image).shape[:2]
            return [row["score"] for row in rows], np.stack([row["mask"] for row in rows]) if rows else np.empty((0, *shape), dtype=bool)
        return segment_point

    def init_owlvit(self, model_name="google/owlv2-large-patch14-ensemble", device="cpu", threshold=0.05):
        """CaP-X OWL-ViT client; predictions still require sensor lineage."""
        self._score(threshold)
        def detect(rgb, texts=None):
            image, obs = self._image(rgb)
            result = self._request("detect", obs, model="owlvit", model_name=model_name,
                                   device=device, threshold=threshold, image=image, texts=texts)
            rows = result.get("detections")
            if not isinstance(rows, list):
                raise PerceptionError("detect backend must return detections list")
            for row in rows:
                score = self._score(row.get("score"))
                box = np.asarray(row.get("box"), dtype=float)
                if box.shape != (4,) or not np.isfinite(box).all() or np.any(box[:2] > box[2:]):
                    raise PerceptionError("invalid detection XYXY box")
                if (np.any(box[:2] < 0) or box[2] > image.shape[1] or box[3] > image.shape[0]):
                    raise PerceptionError("detection XYXY box lies outside the captured image")
                if not isinstance(row.get("label"), str):
                    raise PerceptionError("detection needs a label")
            return [row for row in rows if row["score"] >= threshold]
        return detect

    def init_molmo(self, model_name="allenai/Molmo2-8B", base_url=None, api_key=None):
        """CaP-X Molmo client; credentials remain in backend configuration."""
        if api_key is not None:
            raise PerceptionError("configure model credentials on the backend, never in tool payloads")
        def point(image, objects=None):
            rgb, obs = self._image(image)
            if not objects:
                return {}
            result = self._request("point", obs, model="molmo", model_name=model_name,
                                   image=rgb, objects=objects, base_url=base_url)
            points = result.get("points")
            if not isinstance(points, dict):
                raise PerceptionError("point backend must return a points dictionary")
            checked = {}
            for name in objects:
                point = points.get(name, (None, None))
                if tuple(point) == (None, None):
                    checked[name] = (None, None)
                    continue
                p = np.asarray(point, dtype=float)
                if p.shape != (2,) or not np.isfinite(p).all() or not (0 <= p[0] < rgb.shape[1] and 0 <= p[1] < rgb.shape[0]):
                    raise PerceptionError("Molmo point is outside the captured image")
                checked[name] = tuple(float(x) for x in p)
            return checked
        return point

    def _grasp_arrays(self, result: dict, *, coordinate_frame: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if result.get("frame") != coordinate_frame or result.get("tool_frame") != "native_hand":
            raise PerceptionError("grasp backend must explicitly convert its gripper frame to native_hand")
        if result.get("hold_confirmed") not in (None, False):
            raise PerceptionError("grasp inference does not confirm hold")
        grasps = np.asarray(result.get("grasps"), dtype=float)
        scores = np.asarray(result.get("scores"), dtype=float)
        contacts = np.asarray(result.get("contact_pts"), dtype=float)
        if grasps.size == 0:
            grasps = np.empty((0, 4, 4))
        if grasps.ndim != 3 or grasps.shape[1:] != (4, 4) or scores.shape != (len(grasps),):
            raise PerceptionError("grasps Nx4x4 and scores N must have matching lengths")
        for matrix, score in zip(grasps, scores):
            validate_pose(matrix)
            self._score(score)
        if contacts.shape != (len(grasps), 3) or not np.isfinite(contacts).all():
            raise PerceptionError("contact_pts must be finite Nx3 in the declared frame")
        return grasps, scores, contacts

    def init_contact_graspnet(self, device="cuda", checkpoint_path=None):
        """CaP-X ContactGraspNet client; returns camera-frame Native Hand poses.

        The raw network gripper frame is deliberately not accepted. The backend
        must declare tool_frame=native_hand after applying its calibrated tool
        mapping. These outputs remain candidate poses, not grasp validation.
        """
        def plan(depth, cam_K, segmap, segmap_id, local_regions=True,
                 filter_grasps=True, skip_border_objects=False, z_range=None,
                 forward_passes=2, max_retries=10):
            obs = self._current()
            d, k = _depth(depth), validate_intrinsics(cam_K)
            if not np.array_equal(d, obs["depth_m"], equal_nan=True) or not np.array_equal(k, obs["intrinsics"]):
                raise PerceptionError("grasp depth/K must match the current calibrated observation")
            labels = np.asarray(segmap)
            if labels.shape != d.shape or labels.dtype.kind not in "bui" or not np.any(labels == segmap_id):
                raise PerceptionError("segmap must be an aligned integer label map containing segmap_id")
            result = self._request("plan_grasps", obs, depth_m=d, intrinsics=k, segmap=labels,
                                   segmap_id=segmap_id, local_regions=local_regions,
                                   filter_grasps=filter_grasps, skip_border_objects=skip_border_objects,
                                   z_range=[0.2, 2.0] if z_range is None else z_range,
                                   forward_passes=forward_passes, max_retries=max_retries,
                                   model="contact_graspnet", device=device, checkpoint_path=checkpoint_path,
                                   frame="camera_optical", tool_frame="native_hand", proposal_only=True)
            return self._grasp_arrays(result, coordinate_frame="camera_optical")
        return plan

    def init_contact_graspnet_point_clouds(self):
        """CaP-X cloud client; accepts only clouds tied to current RGB-D evidence.

        Both clouds must be finite camera-optical XYZ metres. The backend must
        check their lineage against observation_id rather than assuming that
        arbitrary caller-provided coordinates came from the camera.
        """
        def plan(pc_full, pc_segment, segmap_id=1, local_regions=True,
                 filter_grasps=True, forward_passes=2, max_retries=10):
            obs = self._current()
            clouds = []
            for cloud in (pc_full, pc_segment):
                points = np.asarray(cloud, dtype=float)
                if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all() or np.any(points[:, 2] <= 0):
                    raise PerceptionError("point clouds must contain finite camera-optical XYZ metre points")
                clouds.append(points)
            from scipy.spatial import cKDTree
            observed = depth_to_pointcloud(obs["depth_m"], obs["intrinsics"])
            if not len(observed):
                raise PerceptionError("current RGB-D has no usable points")
            if np.any(cKDTree(observed).query(clouds[0], k=1)[0] > 1e-7):
                raise PerceptionError("pc_full must contain only points derived from current RGB-D")
            if np.any(cKDTree(clouds[0]).query(clouds[1], k=1)[0] > 1e-7):
                raise PerceptionError("pc_segment must be a subset of pc_full")
            result = self._request("plan_grasps_point_clouds", obs, pc_full=clouds[0], pc_segment=clouds[1],
                                   segmap_id=segmap_id, local_regions=local_regions, filter_grasps=filter_grasps,
                                   forward_passes=forward_passes, max_retries=max_retries,
                                   frame="camera_optical", tool_frame="native_hand", proposal_only=True)
            return self._grasp_arrays(result, coordinate_frame="camera_optical")
        return plan

    def functions(self) -> dict[str, Callable]:
        """Return callable tools in CaP-X's standard registry format."""
        names = ("capture", "get_observation", "get_object_pose", "sample_grasp_pose", "transform_pose",
                 "segment", "init_sam2", "init_sam3", "init_sam2_point_prompt", "init_sam3_point_prompt",
                 "init_owlvit", "init_molmo", "init_contact_graspnet", "init_contact_graspnet_point_clouds")
        result = {name: getattr(self, name) for name in names}
        result.update({fn.__name__: fn for fn in (obs_get_rgb, deproject_pixel_to_camera,
                                                depth_to_pointcloud, depth_color_to_pointcloud)})
        return result

    def combined_doc(self) -> str:
        """Aggregate signatures and documentation without importing CaP-X."""
        return "\n\n".join(f"{name}{inspect.signature(fn)}\n{inspect.getdoc(fn) or ''}"
                            for name, fn in self.functions().items())
