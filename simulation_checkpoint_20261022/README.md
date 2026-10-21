# Isaac PhysX research checkpoint

This archive contains research source and measured component evidence, **not a standalone Isaac scene or a verified field cycle**. The field executable and onboarding are in [FIELD_RGBD_20261022.md](../FIELD_RGBD_20261022.md). Server CAD, generated query meshes, previous run inputs and calibration assets are still required to run these Isaac scripts.

Current policy:10 loose mixed rings, native Franka Hands, RGB-D recognition and encoder-based control; private actor poses serve image formation/evaluation only. No physical attachments or object teleports produce assembly. **No accepted full three-part result or real-hardware validation exists for the current policy.** The pin collision shaft is7.6mm against a nominal8mm bore, so real original-CAD insertion tolerance remains unverified.

## Runtime

GPU2 provides exact-reference cuRobo IK/RIGHT/LEFT route proposals; GPU2/3 provide actual Warp RGB-D triangle-ray images. Original authored-CAD self/payload/other-arm/fixed/installed-part checks and FR3 retiming remain final authority. Warp is not RTX rendering. Virtual held-part view banks are not proof of one fixed physical RealSense's visibility.

The measured loaded-aperture option uses actual stationary encoders plus fresh image-derived G for carrier-finger/held-CAD checks; finite-force spring targets are not treated as achieved geometry. Palm/arm/fixed/opposite-arm/installed-part checks retain full requested widths. Encoder lookahead/expiry is monitored at each control step. The100ms encoder window now starts before the final image and continues through delivery, preserving the250ms capture-age gate.

Typed RIGHT pre-carry planning failure can select a checked handover before loaded motion begins. Observed completed parts are registered as obstacles to both arms. Alternative receiver angle, donor preorientation and double-handover features are experimental flags; model-only feasibility is not physical task success.

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

The full-CAD FCL accept-only acceleration experiment303 also preserved outcomes but gave no useful loaded speedup; it is not enabled. Current field561 packet replay remains blocked before approach/Hand dispatch by trueUNKNOWN clearance even after two virtual views. Field safety admission and SIM point-cloud checks are distinct implementations; neither result is relabeled as success in the other.

## Video and source

[PIN-only recorded video](media/pin-component308-recorded-physx-1x-captioned.mp4) shows actual recorded PhysX at1x monotonic wall time, including camera/compute waits. It is a separate component, not a three-part cycle. [Replay provenance](media/pin-component308-replay-manifest.json) and component records preserve that scope. Only text/reticle overlays were added; no robot/object poses or playback timing were changed.

The PIN feedback experiment overrides are in `experiments/pin-feedback308/`; source/config hashes accompany component records. Canonical source may contain later default-off experiments and is not the exact source of every historical run. Parallel reliability runs are ineligible for cycle-time comparisons. Old policy results and planned subsecond routes do not establish20/30s current whole-task performance.
