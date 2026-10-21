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

## Optional checked route proposals — default off

RIGHT and LEFT carry proposals now have bounded opt-in clients. The model uses the same native FR3 chain, approximate native collision spheres, an 8mm fixed-CAD ESDF and current robot/RGB-D obstacles. The coarse model is deliberately an optimizer approximation. The original complete self, own-payload, opposite-arm, fixed-CAD and completed-part registry checks remain authoritative, including all issued FR3 command guards. No insertion contact stroke is certified by the GPU world.

Generate local assets using the matching current production collision archive, which must use this project's native-body mesh layout:

```bash
export M80_GPU_COLLISION_NPZ=/path/to/current/camera-local-models.npz
python3 build_route_model.py
python3 native_witness_proposal.py
python3 build_route_esdf.py
cp robot-route-prior.json robot-route-prior-left.json "$M80_GPU_PLANNING_HOME/"
python3 route_server.py
# In a separate process, using the same environment/assets:
python3 route_dual_server.py
```

The workers have separate `route.sock` and `route-dual.sock`; neither replaces the independent IK worker. Do not start another worker on an occupied socket. Enabling LEFT requires `gpu_route_proposals=true`, `gpu_left_route_proposals=true`, `gpu_dual_arm_reference=true`; the optional `gpu_left_route_socket` is distinct from the RIGHT socket. Missing service, mismatch, timeout or original-checker rejection retains CPU fallback. The service must be warmed before per-request timing; cold startup belongs in end-to-end task wall time.

LEFT uses a rigid world/TCP gauge of the same intrinsic joint chain. Frames, axes and bounds must match; current arm FK and exact model/endpoint echoes remain checked. `left-gauge-readiness.json` records nominal parity. Fixed-geometry cache pose and current observed obstacle envelopes are transformed into the reference world for each request. The dual worker uses native boundary witnesses and optional carried-class surface witnesses to avoid filling annular holes with a sphere. These are optimistic proposal hints, never clearance certificates. They can miss a collision and require the final original checker.

Robot joint priors are optimizer seeds only. Endpoints are retargeted to each current encoder/goal query and the entire curve is optimized and independently checked. No archived command stream or loose-object pose is executed. The packaged scene-specific priors are not field calibration.

Static LEFT proof286 accepted one source279 camera/encoder carry query:178ms GPU,429 issued commands,410 current same-arm/own-payload checks with no rejection, and original fixed/opposite-arm/full CAD acceptance. Planned motion duration was0.892s and static proof wall6.49s. No motion was executed. Earlier installed parts were not recreated in that static proof; live operation must inherit the completed-part CAD registry. This is not a full task or field-hardware pass. RIGHT proofs271/273 predated the exact same-arm guard and only certify their documented fixed/pair/payload/FR3 scope; all current proposals require the new self guard.

The service never changes physical geometry, mass, contact, force, hardware bounds or command caps. GPU geometry files and scene caches must be rebuilt after model changes. No `.so`, Torch package, CUDA cache, credentials, socket, logs or large scene mesh belongs in this source package.
