# Cache / collision component dependencies

The source bundle must contain these components; absolute `.local` paths below identify the current server installation, not a requirement for a different machine's directory layout.

| Component | Required source/data | Runtime |
|---|---|---|
| Stored RIGHT / LEFT fields | `fixed_canonical_plan.py`, `fixed_small_left_plan.py`, their executors, `fixed_saved_routes.py`; original robot FK/calibration and goal checks | Python/NumPy; immutable public actuator command fields |
| Static proof compiler | `fixed_static_route_certificate.py`, `payload_class_geometry.py`, original CAD scene + convex class CAD, `exact_collision_proof_cache.py` | Isaac Sim PhysX/ USD, SciPy, python-fcl |
| Native SELF / own-payload proof | `fixed_own_route_certificate.py`, scoped `same_arm_clearance.py` hooks, original own/placed clearance modules | CPU original FCL convex distance; includes model/backend/source and complete domain identity |
| GPU exact pair client | `gpu_pair_binary_protocol.py`, wrapper + binary + legacyJSON clients, pair schedule/path scopes, serial scope and original aperture helpers | NumPy, local AF_UNIX socket; CPU original collision fallback |
| GPU exact pair server | `.local/collision-gpu2/binary/pair_binary_server.py`, its protocol module, parent `.local/collision-gpu2/pair_kernel.py` | PyTorch CUDA float64; current authorized GPU2; socket `.local/collision-gpu2/pair-binary.sock` |

The binary server uses the unchanged conservative pair kernel. Every request is bounded to16000 exact states per RPC; later chunk failure discards ALL previously returned clear bits. Only a complete exact-state CLEAR proof can replace a pair query. Unknown/error goes back to original CPU geometry checks. Server unavailable is slower, not an authorized clearance.

Proof JSON lives in `.local/fixed-route-cache`; it may be rebuilt before first camera capture. Metadata covers original CAD, source/FCL/backend, robot and world calibration, exact command bytes, nominal measured grasp, width and uncertainty domains. Different source, environment, robot base, fixture geometry or domain invalidates reuse. Source and compiler must ship; cached JSON alone is insufficient. First-time compilation/startup is separate from the reported first-capture-to-final-retention cycle.

RIGHT now prepares TWO independent original-CAD grasp domains: nominal public G, and nominal G with an additional180° local-Z rotation. The detector and raw public G remain unchanged. This is not a mesh symmetry claim or hidden normalization. Each domain runs its own stricter offline original-CAD static and own-robot envelope checks; unmatched public G uses the original planner/guards. In particular, a truly different chord-grasp family can still miss both domains.

At runtime the actual encoder entry, original current counterpart/apertures, installed observed parts, measured loaded-gripper interval, current RGB-D G, contact and FR3 checks remain required. The cached middle is not permission for blind hardware replay or a claim that an unobserved grasp remains inside its envelope continuously. Hardware RGB-D/calibration, gripper and clamp response still require field qualification.
