# Isaac PhysX research checkpoint

This archive contains research source and measured component evidence, **not a standalone Isaac scene or a verified field cycle**. The field executable and onboarding are in [FIELD_RGBD_20261022.md](../FIELD_RGBD_20261022.md). Server CAD, generated query meshes, previous run inputs and calibration assets are still required to run these Isaac scripts.

Current policy:10 loose mixed rings, native Franka Hands, RGB-D recognition and encoder-based control; private actor poses serve image formation/evaluation only. No physical attachments or object teleports produce assembly. **348 is the first accepted full three-part PhysX result for the current policy, on one randomseed446. Total656.96s wall/85.35s physics; generalized reliability,20/30s completion and actual hardware remain unverified.** The pin collision shaft is7.6mm against a nominal8mm bore, so real original-CAD insertion tolerance remains unverified.

## Runtime

GPU2 provides exact-reference cuRobo IK/RIGHT/LEFT route proposals; pickup pregrasp now tests GPU endpoints before lazily computing the unchanged50-seed CPU fallback; GPU2/3 provide actual Warp RGB-D triangle-ray images. Original authored-CAD self/payload/other-arm/fixed/installed-part checks and FR3 retiming remain final authority. Warp is not RTX rendering. Virtual held-part view banks are not proof of one fixed physical RealSense's visibility.

The measured loaded-aperture option uses actual stationary encoders plus fresh image-derived G for carrier-finger/held-CAD checks; finite-force spring targets are not treated as achieved geometry. Palm/arm/fixed/opposite-arm/installed-part checks retain full requested widths. Encoder lookahead/expiry is monitored at each control step. The100ms encoder window now starts before the final image and continues through delivery, preserving the250ms capture-age gate.

Typed RIGHT pre-carry planning failure can select a checked handover before loaded motion begins. Observed completed parts are registered as obstacles to both arms. Alternative receiver angle, donor preorientation and double-handover features are experimental flags; model-only feasibility is not physical task success.

## New-layout and computation follow-ups

361(seed447) completed SMALL insertion, then released an uncertain LARGE grasp back into the bin, re-picked and handed it to LEFT; the loaded-finger geometry interval stopped its next carry (517.35s, fullFAIL).362(seed448) completed LARGE insertion but found no SMALL hook/rearrangement route (92.38s, fullFAIL). These do not extend accepted-layout reliability.371 is testing a larger, already permitted, continuously swept finger-uncertainty interval with all geometric and speed guards, plus [numeric FK368](experiments/numeric-fk368/README.md); no result yet.

Stationary full-scene CPU-dispatch364/365 found4 worker threads slower than the existing0 (7.246s vs1.119s for one simulated second); the existing setting is retained. These are no-motion performance components, so their aggregate task verdict is intentionallyFALSE.

587 field source adds an optional conservative whole-link/palm convex self check with all original uncertainty, not a field success. Recorded default20mrad/UNKNOWN admission is still blocked. Actual survey records already support known static NON-bin geometry; they do not erase uncertainty in unseen basket stock.

## Accepted357 smooth-carry follow-up

[1x motion excerpts with explicit CUT cards](media/all3-pass357-recorded-excerpts-1x.mp4), [full470.50s wall-clock replay](media/all3-pass357-recorded-full-wall-1x.mp4).

[Exact source](experiments/full-pass357/README.md) and [acceptance summary](component-evidence/native-roas-table-scatter-stable-v357-smooth-GPU-carry-all3-446/acceptance-summary.json): same randomseed446 passes all three parts again,470.50wall/59.07physics seconds. Actual initial large-ring carry1.86875seconds, with original hardware/geometry guards. Two small-ring rearrangements were needed.20/30seconds, new-layout reliability and hardware remain unverified.

## Accepted three-part assembly348

[Measured acceptance summary](component-evidence/native-roas-table-scatter-stable-v348-gpu-carry-no-greedy-final-retreat-all3-446/acceptance-summary.json), [exact source](experiments/full-pass348/README.md), [1x excerpts with explicit cuts](media/all3-pass348-recorded-excerpts-1x.mp4), [full656.96s wall-clock replay](media/all3-pass348-recorded-full-wall-1x.mp4). LARGE/SMALL/PIN final position errors0.629/0.425/0.759mm and axis errors1.582/9.721/15.240mrad pass original1.5mm/50mrad gates. After0.3125s actual final emptyRIGHT lateral retreat, the original160mm Hand clearance and0.55s physics retention passed. Existing force, joint/finger velocity, acceleration and jerk checks were retained. Three local small-ring rearrangements were needed; they are shown in the excerpts. No hidden attachment or workpiece teleport produced assembly.

The independent449 random layout350 still failed before pickup. One success is not a generalization result. Field565 camera-only screening is a separate, more conservative implementation and still blocked; the additional original-CAD/mount/UNKNOWN diagnostics are under results. No real robot was connected.

## Actual evidence

| Record | Scope |
| --- | --- |
|273/286 | Static cuRobo route queries:256/178ms GPU proposal, original issued geometry/FR3 checks; not full task timing. |
|291 |840 identical original/AABB self-check outcomes, isolated self-check speedup only. |
|299 |Six axial receiver final-IK searches found no branch; legacy receiver later blocked by original0540 at final stroke. |
|304 |Both rings inserted; pin precision failed. Full task failed519.3s. |
|305/307 |25deg receiver physically handed over. Corrected encoder/image binding passed; actual later grip roll prevented assembly. |
|306/310 |One local-rim short lift passed; repeat/composition lost the grasp. Generic contact certificate stayedFALSE. No field/assembly admission. |
|308 |PIN-only actual insertion/release/withdraw passed27.77s; feedback gain0.5 plus100ms physical encoder-position admission. Strict75um/.003rad prealignment was preserved. |
|309 |Donor preorientation before transfer did not remove actual receiver grip rolling; original1mm proximal reserve rejected motion. |
|311b |720 empty/loaded full self/payload queries match exact FCL convex-boundary indexing; no useful speedup, not installed. |
|312 |Combined pin feedback with random-bin full task; LARGE free alignment failed473.8s. Not an all3 success. |
|314/314b |Original finger CAD decomposition, same source/config, both15mm lift+release components passed. Generic hull contact certificate remainedFALSE; no outside-bin or field admission. Settled grasp differs from306, so this is not proof of causality. |
|316b/316c |Stationary CPU/GPU step+readback comparison only:0.5s simulation took27.812s with requested GPU dynamics vs0.562s CPU. Current pipeline keeps CPU physics; cuRobo and image workers remain GPU-backed. No movement/task proof. |
|317/320 |Double-handover held-ring endpoint blocked by actual parked LEFT. Exact original-model causes are recorded; prospective temporary-holder feasibility is not executed transfer success. |
|318 |25mm in-bin shift/release passed, but subsequent ordinary outer pickup still palm/basket blocked. Generic contact certificateFALSE. |
|319 |Actual one-rim extraction rose132.099mm total, lowest class-CAD15.327mm above lip, then checked return/release. Generic contact certificateFALSE, SIM mechanism only. Driver final JSON serialization failed after saved traces; independent evaluator preserves this limitation. No field/assembly admission. |
|324/327 |Half-gain large-ring free alignment did not solve the center gate: both full trials failed (546.97/446.26s).327 used real GPU pregrasp endpoints and physically picked/lifted both parts; final large radial187.5um still exceeded100um despite0.981mrad axis. Not an all3 pass or timing comparison. |
|329 |Identical recorded image/class subsets are cached by full point bytes and exact measured pose, with full array-equality guard.48 native-swept finger/palm queries preserve all4 boolean combinations and same-array mutation invalidation; isolated component3.13x faster. No collision/filter/age margins changed; not whole-cycle speedup. |
|331 |Actual RANDOM seed446 inserted LARGE+SMALL+PIN. All final part pose gates and0.55s retention passed. Aggregate FAILED: emptyRIGHT Hand root154.6mm fromSMALL vs original160mm requirement. Total801.38s wall/63.14s physics, not20/30s. This is a useful assembly milestone, **not a full accepted pass**. |
|333/334/341 |333 seed445 and334 translation-only alignment still failed;341 repeat of446 stopped before carry because no fixture/carry route was approved within budget. Neither proves random reliability. |
|339b |Exact FCL broadphase skips projecting unused full vertices/normals.5310 transformed OBB triples bitwise equal,90/90 full loaded checker results and known self-collision rejection identical. Isolated checker1.544x faster; not cycle-time speedup. Installed in research source. |
|343 |First actual axial-face temporary LEFT transfer retained the ring after RIGHT release/clear. PositiveRIGHT second receiver approach was rejected before motion by original guards. No double-transfer or complete-task pass. |
|345b |Deferred exact pair-CAD vertex/normal projection until an unchanged broadphase survivor needs it.6120 full-geometry arrays equal,48 SAT comparisons equal, four90-query full loaded-checker batches identical including known negative. Additional isolated1.325x over339b; not whole-cycle speedup.345 was explicitly stopped for redundant comparisons and replaced by this bounded component. |
|347 |On actual341 RGBD grasp/stock and encoders, disabling only optional greedy shortcut search admitted the original full checked cuRobo carry/fixture route in14.39s. Original341 had spent~135s and rejected carry. This is model-only diagnosis, not motion or a comparable full-cycle result; physical348 composition is pending. |

The full-CAD FCL accept-only acceleration experiment303 also preserved outcomes but gave no useful loaded speedup; it is not enabled. Current field565 packet replay remains blocked before approach/Hand dispatch by trueUNKNOWN clearance even after two virtual views. Field safety admission and SIM point-cloud checks are distinct implementations; neither result is relabeled as success in the other.

## Video and source

[Full801.4s wall-clock video](media/all3-inserted331-recorded-full-wall-1x.mp4) includes camera, recognition and planning waits at1x. [Three-part insertion excerpts](media/all3-inserted331-recorded-excerpts-1x.mp4) show actual331 recorded PhysX with explicit waiting cuts, original wall timestamps and1x retained motions. The permanent banner preserves **FINAL EMPTY-HAND CLEARANCE FAILED**. [Provenance](media/all3-inserted331-replay-manifest.json) preserves801.38s total and lack of hardware/generalization proof. All three parts met final pose/retention criteria, while the original aggregate160mm hand-clearance criterion remained failed.

[PIN-only recorded video](media/pin-component308-recorded-physx-1x-captioned.mp4) shows actual recorded PhysX at1x monotonic wall time, including camera/compute waits. It is a separate component, not a three-part cycle. [Replay provenance](media/pin-component308-replay-manifest.json) and component records preserve that scope. Only text/reticle overlays were added; no robot/object poses or playback timing were changed.

The PIN feedback experiment overrides are in `experiments/pin-feedback308/`; source/config hashes accompany component records. Canonical source may contain later default-off experiments and is not the exact source of every historical run. Parallel reliability runs are ineligible for cycle-time comparisons. Old policy results and planned subsecond routes do not establish20/30s current whole-task performance.
