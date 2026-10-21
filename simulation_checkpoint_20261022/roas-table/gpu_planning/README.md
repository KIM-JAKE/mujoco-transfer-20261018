# Optional native FR3 GPU proposals

This source package runs a persistent cuRobo IK service on an explicitly assigned GPU. It never commands a robot. CPU planning remains the default. Enable `gpu_ik_proposals` only when the service is ready; missing service, timeout, malformed response or model mismatch returns to the existing CPU search.

The shipped reference model is the actual production RIGHT FR3 base, seven A/axis/B joint frames, native-hand TCP and joint bounds exported from the Isaac robot calibration. It is not the Panda or Robotiq model. It contains no loose-object scene coordinates. The client independently hashes the live robot's frames/TCP/bounds, rejects a mismatched service and checks each proposed solution with its own CPU forward kinematics and bounds. A seed is a search hint only. Never replay reference joints as commands.

## Start in an isolated environment

A CUDA-capable Python environment is required. Tested base image: Python3.12, Torch2.11+cu128, Warp1.16, NumPy/SciPy/trimesh/PyYAML. The repository's vendored cuRobo is used directly; no global or shared installation is needed. `requirements-gpu.txt` supplies only missing helper packages. CUDA extension build products and caches stay outside Git.

When installed at `isaac_port/native_hand_layout/roas-table/gpu_planning`, `bootstrap.sh` finds the project root. For another location set `M80_PROJECT_ROOT`. In the assigned GPU container:

```bash
export M80_PROJECT_ROOT=/workspace/project/mujoco
export M80_GPU_PLANNING_HOME="$M80_PROJECT_ROOT/.local/curobo-gpu2"
export M80_CUROBO_SOURCE="$M80_PROJECT_ROOT/keti_m80_mujoco_20261013/duo_fr3_2f85_20261013/vendor/curobo"
# On another calibrated robot export its current base/TCP/joint-frame JSON first.
export M80_GPU_ROBOT_JSON=/path/to/current/robot-kinematics.json
./bootstrap.sh
export PYTHONPATH="$M80_GPU_PLANNING_HOME/site:$M80_CUROBO_SOURCE${PYTHONPATH:+:$PYTHONPATH}"
python3 ik_server.py
```

The server uses container CUDA0. Select the authorized physical GPU when launching its container; this script does not select or reserve host GPUs. Do not start a second server against an occupied socket. The default socket is `$M80_GPU_PLANNING_HOME/ik.sock`, permissions0660. Logs are in the same local directory. The socket directory must be shared with the Isaac/field client. Configure the canonical `fr3_gpu_ik.py` client socket and a bounded timeout (default0.5s; maximum2s). The server warms its graphs before exposing the socket; startup compile time is separate from per-request time and must be included if measuring cold task wall time.

`M80_GPU_URDF` optionally overrides the generated URDF path. Export with `export_model.py` after changing robot calibration. The generated URDF's effort/velocity placeholders are only cuRobo metadata; original FR3 hardware bounds and the existing trajectory retimer are authoritative.

## Required final acceptance

GPU IK uses TCP target + current measured robot joints only. It does not certify collisions. Every candidate must still pass the existing full native-hand/robot geometry, original fixed CAD, current other-arm encoders, camera-derived stock obstacles and held payload checks. Every issued retimed curve must pass the existing complete path/FR3 speed, acceleration, jerk and actual-state guards. Endpoint success is not task success or hardware certification. Continuous Cartesian segments retain the existing local IK solver to avoid joint jumps.

The reference evidence reports32 random FK samples with maximum position error0.258µm and rotation error0.291µrad. Runtime trials256/257 accepted12/11 GPU pregrasp proposals without CPU IK fallback. Those are endpoint acceleration observations, not full-task acceptance.

## Experimental route proposals — default off

`build_route_model.py` and `route_server.py` are experimental source only. Set `M80_GPU_COLLISION_NPZ` to a freshly exported collision-model archive from the matching production scene, then run the builder before the route server. The archive must contain the native PhysX mesh indices used by this project; an unrelated asset order is unsupported. The builder creates local fitted spheres and a fixed-CAD mesh. These generated assets are intentionally excluded from this package/Git. Do not use a stale mesh after changing the scene.

The sphere geometry is an approximation fitted to the actual FR3/native-hand collision meshes. It can miss or overestimate features. Current other-arm encoder envelopes and RGB-D stock bounds are provided per request; a held-payload envelope is also included. The route remains a proposal and must pass the unchanged original full collision checks and exact issued-curve retiming. Insert/contact strokes do not use the approximate GPU world as an authority.

Current route evidence is an81-point proposal at about5.7s, with original PhysX acceptance still pending; it has not met the100ms target and is not enabled in production. Do not enable `gpu_route_proposals` until the bounded client integration and original full-path check are reviewed. The active IK service is independent and must remain available while route work continues.

No `.so`, Torch package, CUDA cache, credentials, socket, logs or large scene mesh belongs in this source package.
