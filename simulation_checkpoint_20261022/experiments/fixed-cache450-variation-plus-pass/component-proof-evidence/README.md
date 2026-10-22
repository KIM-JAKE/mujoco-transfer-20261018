This archive records simulator MODEL and offline evaluator evidence. It is not a full-task success result or hardware qualification. Runtime control uses RGB-D, public robot commands/encoders and fixed CAD only. Physical workpiece transforms appear only in the explicitly marked retrospective evaluators below.

The saved-middle architecture compiles original PhysX static-mesh collision checks before the first camera capture. The current approach/entry bridge, current opposite arm, own-payload and observed installed-part checks, contact guards and FR3 command/encoder limits remain live. Exact canonical samples only: no rounded joint lookup. Finger-width and held-CAD uncertainty receive conservative convex envelopes, with full-stroke fixture envelopes for all9 prismatic clamp/Button bodies. Geometry, both robot bases, fixed anchors, model sources, class-CAD identity and calibration seal invalidate incompatible cache entries.

Evidence:

- large-static-compile-model.json: all1415 accepted432 LARGEcarry samples covered against2793 original fixed colliders plus9 fixture sweeps, width[.071,.076],3mm/.02rad held-G envelope. One-time MODEL compile62.49s, before cycle capture; no task actuation. Later seal hardening adds both fixed native base meshes to the identity inventory (2795).
- native-static-negative-and-perturbations.json: a newly authored noncolliding100mm planning query cube correctly rejects physical shelf0641; changes to planning-query meshes do not affect the static-world seal.60 original PhysX checks at12 saved poses under random allowed G/width variation pass.
- class-local-geometry-stability.json:48 LARGE,48 SMALL and4 PIN source meshes. Composing only child-to-class-root USD transforms agrees with inverse-world geometry to≤2.24e-16m and gives bit-identical local geometry across20 arbitrary root poses. No physical actor is moved.
- fixed-both-base-seal-invalidation.json: planning-query edits leave the seal unchanged; moving the opposite fixed base1mm invalidates it. Both native link0 CAD/poses and their fixed-joint anchor chain are included.
- public-command-field-provenance.json: accepted432 public command fields, exact sample ranges, byte hashes, original dt and actual/command aperture ranges. PIN camera axis-frame grasp requires axis_to_body_grasp before CAD collision use.
- pin-static-compile-model.json: all1563 PIN field samples statically certified under correct Hand-root→CAD-body nominal grasp and width[.0075,.014]. A diagnostic JSON formatting error happened after the certificate and canonical field had been written; coverage was independently verified against every exact input key. No full-task claim.

Actual held-CAD volume audit:

- rectangular-G-domain-diagnostic.json deliberately records that retrospective actual orientations can exceed the rectangular3mm/.02rad parameter assumptions; arbitrary circular-ring yaw also changes full CAD-frame rotation without changing the annular object volume.
- actual-held-CAD-volume-containment.json resolves this geometrically. For every recorded frame and every one of48 actual convex CAD pieces, every actual vertex has a witness in ONE nominal piece whose Chebyshev displacement is inside that nominal piece's original cube dilation, with≥1nm inward reserve. Convexity then contains the entire actual convex piece, and their union contains the entire actual object. Piece matching handles arbitrary circular-ring yaw without treating it as a sensed control variable.
- Accepted432 LEFT saved entry+carry interval:180 recorded frames,8640/8640 entire-piece proofs; minimum inward slack0.3849mm. Its recorded interval includes the stopped entry during preflight planning and the720 issued moving steps.
-443 RIGHT saved carry:353 recorded frames,16944/16944 entire-piece proofs; minimum inward slack2.5599mm.
- This proves the recorded geometries, not every unrecorded instant or an arbitrary future trajectory. Runtime keeps all physical/contact guards.446 recompiles a wider.06rad domain as a stricter offline geometric check, rather than loosening a runtime collision test.

Binary GPU pair transport:

- binary-native-positive-mask-comparison.json uses production moving-arm proxies (10 each, fixedlink0 excluded exactly as Checker; base remains in FK) plus48 held LARGE CAD pieces.7075/7075 masks are CLEAR and identical between legacy JSON and binary NumPy transport.16 original CPU SAT/LP spotchecks are all CLEAR.
- Component timing in that fixture: legacy12.005s client/7.555s server versus binary1.613s client/.778s server; original float64 kernel unchanged. Exact transform data shrinks61.58MB to5.345MB by referencing identical per-body matrices, not by rounding or omitting bodies. This is a component measurement, not whole-cycle or hardware performance.
- binary-native-touching-104cases.json:104 nativefinger+ring/touching/original1mm margin/reserve cases,0 falseCLEAR. GPU80/CPU81 among96 native cases and GPU4/CPU5 among8 boundary cases preserve conservative CPU fallback.
- binary-protocol-rejections.json: invalid index, wrong dtype, altered margin and unknown geometry produce fallback-required errors and no CLEAR mask.

See SHA256.json for the exact archived JSON records. Final full-task video/results must be attached separately from an actual successful run.
