# Fixed PIN transport: read-only reuse assessment

The fixed holder permits a saved PIN transport middle, using the existing original-static and original-native/own certificate machinery. This is a candidate, not a geometry-qualified path or measured speedup. No simulator run, GPU job, source mutation or actor-pose input was used. The source is the public494/495 recorded command/encoder and RGB-D output. Those two whole cycles ran concurrently; all timing comparisons are descriptive.

## Measured opportunity

|Recorded case|494 nominal|495 MINUS|
|---|---:|---:|
|Last postlift RGB-D confirmation to first carry-issued row|6.622s|9.847s|
|Final empty-LEFT closure row to first carry row|2.932s|6.383s|
|Existing current actual+command LEFT terminal proof, within the preceding gap|1.775s|1.963s|
|Remaining transport planning/retiming upper bound|1.157s|4.420s|
|Transport endpoint GPU IK client time|0.0298s|0.0296s|
|Actual transport issued rows at480Hz|636|920|
|Transport physical duration|1.325s|1.917s|
|Stored robot hint accepted|No|No|

Both runs already avoid a physical LEFT parking move. The current proof is still mandatory. A carry cache cannot save all6.6–9.8s; the remaining1.16–4.42s also contains live original route/terminal checks that must remain unless independently certified. IK-only caching would save about30ms. Executing a494-like middle could reduce495's physical carry by at most the observed0.592s difference before adding required bridges, but that cross-run proposal has NOT been checked and is not a performance claim. Later fresh-G terminal insertion checks/camera servo remain outside this candidate.

The two last post-preload public measured grasps differ0.31548mm in translation and2.82307mrad in full rotation, including the original unobservable-yaw gauge preserved by direct_pin_skill. Their command starts differ up to56.50mrad per joint; their chosen transport endpoints differ up to410.10mrad due to different redundant IK branches. Neither is a literal unchanged whole saved path. Saved pixels or actor coordinates are unnecessary.

## Minimal implementation candidate, if parent elects to proceed

1. Archive the exact494 public RIGHT carry command field (including its actual source dt, FR3 position-dependent95%PBV, acceleration9.5rad/s² and jerk4750rad/s³ audit). Keep original geometry/source/model/world/calibration seals. Compile static and native/own envelopes for that exact field and its original interpolation samples before the measured cycle. Runtime misses fall back to current planner; a stored row is not automatically CLEAR.
2. Use the CURRENT post-preload fresh RGB-D/SDK confirmation, unchanged. Preserve its signed axis, center and existing yaw gauge. Convert its physical-axis midpoint grasp to CAD body grasp exactly with pin_native_contact.axis_to_body_grasp, then the existing103.4mm Hand/TCP transform. A PIN-specific domain must be independently proved. Do not reuse a ring certificate or assume PIN axial symmetry. The observed494/495 deltas suggest a small independently proved domain may cover both, but no tolerance value is qualified here. Large grasp slips correctly miss that domain.
3. Preserve the actual encoder start, current command start, all actual/command aperture bounds and any loaded-contact binding. The archived aperture9.5mm command versus12.5556mm achieved gap shows why one10.5mm planning width cannot substitute for the complete runtime width proof. Current LEFT actual and command q, current LEFT widths, current payload and both installed ring obstacles remain live.
4. Fully check a live entry bridge from current actual/command q to the saved field's exact start, then the entire frozen middle against current counterpart/payload/installed parts and all original issued-array guards before any saved row. Only static/native/own proof rows that match the exact field/hash/domain may reuse their corresponding CLEAR proofs. Unknown rows use original checks. Do not merely join to a nearest sample or retime away certificate identity.
5. The saved end is only a proposal. Recompute the desired Hand preinsert from CURRENT measured G and calibrated physical-axis goal,45mm below the jig. Fully check the exit bridge and original full stroke/release/withdraw branch with30mrad joint-limit margin before transport admission. Retain the later fresh public preinsert images,75µm radial/3mrad axis/150µm shaft gates, current-G terminal reproof, hardware caps and physical retention.30mrad is the terminal joint-limit reserve, not the3mrad insertion alignment gate.
6. Any entry/exit/current-counterpart/domain failure must preserve the existing original route planner. After partial issuance, keep the existing safe failure/replanning semantics; never silently retry an unqualified branch.

## Recommendation

A compact PIN-specific saved-middle adapter is reasonable AFTER the active grip change supplies a stable measured-G domain; it may remove several seconds of route/guard work and branch variation. It is lower value than the large ring saved fields and does not by itself approach20s end-to-end. Preserve all fresh terminal/width/pose gates and report prep cost separately. This assessment does not authorize a new motion: no cross-case path collision qualification or replay has been performed.
