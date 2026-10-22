# Optional independent GPU2 RGB-D service

This is a new service and socket. The existing budget6-v2 service remains unchanged and running.

- Container: isaac-jake-gpu-2-capacity-ac370e58.
- Container PID:121868; ready.json records identity and source fingerprint.
- Socket: /workspace/project/mujoco/.local/rgbd-gpu2-fast-spread/perception.sock.
- Source fingerprint:94af0e71ce65c7b6a1798f9667fccc1112fa0df65463162fafe30338641d9304.
- Apply config-delta.json. Also overlay the existing CAMERA-worker rgbd_ring_translation.py with the packaged file to activate its separate refinement optimization. Its SHA-256 is90ccf9c121439c4e672e15f730370d8a7190fbff960477ef5e40c91945b174de.

The detector skips neighbor-graph construction only when the existing global-plane branch never consumes that graph. For RGB uint8 only, exact channel maximum-minus-minimum avoids conversion of a whole image to float64 and a strided reduction. Non-uint8/non-three-channel images keep the original expression. No proposal counts, random stream, thresholds, point-cloud values, signed-face/class/occlusion gates, depth inputs or camera settings change. All original detector outputs and mask bits are retained. Images/masks/poses are not cached. Immutable CAD models and existing GPU proposal buffers retain their existing caches.

Frozen executable Python dependencies and CAD samples are listed in frozen-source-manifest.json; NumPy/SciPy/Warp use the same existing container environment. The translation candidate is packaged/fingerprinted but executed by the camera worker, not by this detector service. Changing it therefore requires a new camera-worker source fingerprint and restart through the normal immutable launch mechanism.

rpc-qualification.json compares the unchanged oldGPU2service to this new independent service on seven actual484 public packets. Every detector result except its processing-time metadata, every class/signed axis/center, and every mask bit match exactly. detector-refinement-qualification.json also checks exact worker refinement results and the non-global graph branch. These are isolated per-image measurements, not a full-task runtime or hardware benchmark. Complete new IsaacSim execution remains required.

Read-only profile artifacts are in ../rgbd-profile485. Original masks and confidence gates were not relaxed.

Cleanup: inside the above container, verify /proc/121868/cmdline contains this directory's server.py, then terminate ONLY PID121868. Remove ONLY this directory's perception.sock after the process has stopped. Do not kill/restart the old budget6-v2 service or unlink its socket. Startup refuses to replace an existing optional-service socket.
