# Fixed-supply RGB-D field export

Use the accepted trial's frozen source for the simulation-tested detector, not similarly named legacy estimator entrypoints. Existing `isaac_port/fieldkit/vision.py` fits near configured stock priors and explicitly retains ring axial orientation at the prior; it is not the current signed-face RGB-D detector.

Actual reusable camera/detection files in `isaac_port/native_hand_layout/roas-table`:
- `capture_realsense.py`: persistent `RealSenseStream`; alignedRGB/depth, SDK deprojected `points_camera_m`, metricDepth, K and measured `T_cell_camera`; warm camera once, drop queued frames and bound acquisition.
- `rgbd_scattered_rings.py`, `rgbd_geometry.py`, `rgbd_annular_cap_gate.py`: initial/held signed annular cap detection; metric center/axis/diameter, size class, asymmetricCAD face distinction and observed-free-space veto. CPU implementation remains complete fallback.
- `rgbd_fast_tracker.py`, `rgbd_pin_axes.py`, `rgbd_ring_translation.py`: existing supported pin and constrained fresh-tracking methods, invoked under the same image freshness/annular-pose conditions as the accepted frozen worker.
- Dependencies: NumPy/SciPy and `isaac_port/fieldkit/perception.py` plus `isaac_port/fieldkit/assets/part-models.npz`. Optional GPU service requires Warp and the GPU scorer helper; it may receive public packet/calibration/image-derived seeds only.

Real camera public packet contract is exactly `rgb`, `depth_m`, `points_camera_m`, `K`, `T_cell_camera`. No renderer state, actor transform, segmentationID or random scene seed is needed. Class CAD and calibrated original shelf ROI are static allowed inputs. A held-ring ROI seed is computed from current robotSDK TCP FK times the last camera-measured hand-to-ring transform. Initial fixed lookup requests the next CaP class (`ring2`, then `ring1`), without an object identity or hidden location.

Simulation's `realtime_camera_worker.py`, ray camera, cached geometry and body snapshots are image-formation infrastructure. They must not replace camera acquisition in a field adapter. In particular, `fixture_depth_m` in simulation is an empty-fixture CAD depth render. Field use may omit it or use a real recorded empty-fixture depth calibrated to the actual fixed camera; it cannot use private object coordinates.

Camera calibration needed on site: RealSense serial/stream mode, measured camera-to-cell transform, left/right robotbase transforms, nativeHand TCP, source shelfROI/floor and insertion jig frame. Current `capture_realsense.py` validates the extrinsic rigid transform and publishes frame number/hardware timestamp/acquisition time. No physical RealSense is connected here, so reflective-metal depth quality, hand-eye error and accepted detector threshold settings still require the actual camera check.

Field onboarding should separate camera-only observation (no robot commands), single-part slow commissioned pickup, handover, insertion, and complete cycle. Stored robot paths from simulation are planning priors; exporting them does not certify the real scene or robot limits. Export the public branch/admission helper and source hashes alongside the final accepted trace. Current GPU socket implementation is a local service transport, not a complete fieldSDK actuator adapter.
