# Isaac PhysX research checkpoint

This is the current research source and recorded evidence, **not a standalone Isaac scene or the field executable**. Start the field setup with [FIELD_RGBD_20261022.md](../FIELD_RGBD_20261022.md) and `../field_demo_random_20261022/`. These scripts also need the existing server CAD, generated query meshes, model/calibration files and earlier run inputs; those large assets are not duplicated here.

The current random-bin policy has **no accepted full three-part result or real-hardware validation**. It uses10 physically dropped mixed rings, native Franka Hands and image-estimated poses. Private actor state serves only image formation/evaluation, never pose-control input. The current scene also uses a7.6mm pin-shaft collision approximation against a nominal8mm bore; real part tolerance/insertion feasibility must be established at the actual machine. This is not evidence of original CAD real-world assembly clearance.

## Runtime integration

- GPU2 runs exact-reference cuRobo IK, RIGHT routes and isolated LEFT-reference routes. Proposals use8mm ESDF/current RGB-D and encoder obstacle envelopes, then original full native CAD/self/payload/pair/installed-part checks and FR3 command retiming. Rejected/unavailable GPU proposals retain CPU branches. Portable service source/reference data are in `roas-table/gpu_planning/`; sockets, CUDA binaries and mesh caches are excluded.
- GPU2/3 run Warp RGB-D ray-camera workers with unchanged CPU PhysX. Warp rendering is not RTX photoreal rendering. Simulation also uses movable annular-view banks to expose a held cap. These are not proof that one fixed physical RealSense sees all insertion/retention views. The field runner uses actual fixed/calibrated RealSense RGB-D/SDK inputs rather than these simulation image workers; a missing real view remains a failed observation.
- Known Hand surface returns use captured robot TCP/aperture and a narrow official-CAD surface band. Unobserved stock is not replaced with actor truth. A separate conservative residual-occupancy implementation exists in the field package; its remaining Hand/UNKNOWN pre-close blockers are not hidden by the simulation point-cloud result.
- Authored same-arm collision exclusions replace the earlier broad own-prefix omission. Only same-body/direct authored joint/FilteredPairs exemptions apply. Proximal native fingers, palm and arm are checked against held CAD; intended distal pad contact alone is exempt. The current AABB broadphase matched840 original FCL queries including first rejection identities, reducing that isolated self-check cost by about1.4x.
- Released parts are registered from fresh RGB-D into a shared completed-part query world, protecting subsequent motion of both arms/payloads. No actor attachments or object teleports create successful assembly.
- A typed direct-insertion planning failure **before RIGHT carry starts** may choose the existing checked handover skill with the freshest measured grasp. Sensor/execution failures are not caught as planning alternatives.
- The experimental `measured_loaded_contact_geometry` option distinguishes a finite-force finger spring target from actual held encoder geometry. It requires fresh bilateral RGB-D/CAD contact and100ms stationary encoders, checks the exact continuous per-finger interval, monitors encoders before/after every control step with47.5mm/s lookahead, and disarms before release. Only carrier-finger/held-CAD queries use it; palm/arm/fixed/other-arm/installed-part and ordinary opening sweeps remain checked. Physical294/296 passed measured-bound carry;294 retained the small ring but failed its strict100µm large-ring prealignment gate (last169µm/.00245rad), while296 was stopped at984µm proximal reserve below the unchanged1mm requirement. The tiny closing-axis projection in298 did not solve this. Physical299 tests a distinct true-axial outer-diameter receiver family, toolZ parallel to ring axis and2/1.5mm COMoffset, with all live checks retained.

## Evidence and current failures

Component records are under `component-evidence/`; exact configurations/source hashes accompany tests. Physical success is separate from a static query or an expected failed full-task wrapper.

| Record | Verified scope |
| --- | --- |
|273 | One continuous RIGHT route:256ms GPU/286ms round trip,862 issued commands checked,1.794s planned movement; no motion in this static diagnostic. |
|278 | Actual empty-arm park1.408s and one overhead RGB-D frame; large-ring approach/40–70mm lift geometric component, no full grasp/task/hardware certificate. |
|286 | One loaded LEFT static route:178ms GPU,429 issued commands checked,0.892s planned movement. Prior installed part was absent from this particular static proof. |
|287 | Completed-part regression rejects the recorded approaching palm before it kicks the placed large ring, while accepting its clear release pose. |
|291 |840 original/broadphase FCL queries give identical pass/fail and first rejection results. |
|274/280/281 | All or most insertion commands were reached, but withdrawal collision or later loss of an installed ring made overall assembly fail. These are not three-part successes. |
|288 | After large-ring placement, no small-ring grasp/hook passed the native palm versus basket0670 check. |
|289/292 | Large-ring handover succeeded; carry was rejected because the force spring command was treated as achieved aperture. The measured-interval fix is under physical test. |
|290 | A directly insertable carry/fixture route was unavailable before motion. The new typed handover alternative is under physical295 test. |
|293 | New random seed7829 reached a physical hook but the actual finger velocity exceeded the reserved limit; execution stopped. |
|297/300 | Bounded local-rim redistribution tests;297 had no admitted route,300 separated a first-contact sample-count heuristic but found no admitted approach IK. No actual pinch was issued, so these are not physical-impossibility or grasp-failure proofs. |

End-to-end timing includes camera/recognition/planning waits and retention. Parallel reliability runs are explicitly ineligible for speed comparisons; planned subsecond route durations are not a20/30-second whole-task result. Historical failed experiments are preserved for diagnosis, not deployment defaults.
