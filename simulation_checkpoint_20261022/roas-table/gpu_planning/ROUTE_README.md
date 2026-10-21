# Optional GPU carry proposals: verified static path, physical trial pending

The assigned GPU2 service now uses the actual native-FR3 frame-chain model, an8mm ESDF cache of the original fixed CAD, current encoder/RGB-D obstacle envelopes, native palm/finger fitted spheres plus boundary witnesses, and a camera-estimated carried-part envelope. It is an approximate proposal engine. The original complete PhysX/native fixed/pair/payload checks and FR3 command/actual-state guards remain the authority.

`robot-route-prior.json` contains only a recorded ROBOT joint curve and calibration hash. It is an optimizer seed. Its endpoints are retargeted to each current encoder/goal query, and the entire result is optimized against the current world and rechecked. No archived command stream or object pose is executed. A missing/mismatched/invalid prior causes a proposal failure and CPU fallback. LEFT remains CPU fallback. It is not a field-calibrated artifact.

Generate local assets after setting `M80_GPU_COLLISION_NPZ` to the matching production collision archive:

```bash
python3 build_route_model.py
python3 native_witness_proposal.py
python3 build_route_esdf.py
cp robot-route-prior.json "$M80_GPU_PLANNING_HOME/robot-route-prior.json"
python3 route_server.py
```

Use the environment/PYTHONPATH described by the parent GPU planning README. Source files are portable; generated meshes, sphere models, ESDF.npy, sockets and logs stay in M80_GPU_PLANNING_HOME and outside Git. Rebuild fixed geometry after scene changes. ESDF cache manifest records source CAD hash and resolution. Its grid dimensions are explicitly rounded to the known integer shape to prevent float32 metadata truncation in the vendor voxel accessor; no vendor source is patched.

Static trial271 proves one real recorded right-ring carry query:338ms GPU/368ms round trip,142 current encoder/RGB-D envelopes, original full PhysX raw path clear and all5792 float32 retimed commands clear under unchanged FR3 limits. No robot or workpiece moved during that proof. This is neither physical-task nor field-hardware acceptance. Full route timings under simultaneous trials are not benchmark comparisons.

The original81-node corner-stop retimer spent12.06s. The staged continuous adapter first validates the actual CubicSpline and every joint extremum with the original checker; only then does it return existing VerifiedCubicRoute metadata. All issued commands still undergo original FR3 array validation. Geometry samples retain float32 extrema/endpoints and original angular resolution rather than rechecking duplicate stationary ticks. Retiming-only calculation gives2.76s; continuous original PhysX acceptance and actual motion are pending. The pickup metadata change is gated by both gpu_route_proposals and stability_serial, preserving its full issued aperture/pair sweep and flag-off behavior.

Keep GPU route enablement opt-in. Unavailable service, model mismatch, bounds violation, timeout or original geometry rejection falls back to CPU. Do not enable the continuous adapter without its static/issued-curve check and subsequent physical test.

Static trial273 also APPROVED: exact continuous cubic541samples and original full PhysX/native fixed/pair/payload checks passed;862 issuedfloat32 states were all differentiated by the unchangedFR3 retimer, with691 geometry samples retaining extrema/endpoints. GPU256ms/host286ms, plannedphysicalduration1.79375s, diagnosticwall8.4366s. No physical motion was performed. Continuous metadata is now statically accepted; fulltaskGPUphysicaltrials remain pending.
