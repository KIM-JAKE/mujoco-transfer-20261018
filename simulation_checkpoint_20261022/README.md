# Isaac PhysX research source checkpoint

This directory captures the current research scripts and honest failure reports. It is **not** the field executable. Start with `../FIELD_RGBD_20261022.md` and `../field_demo_random_20261022/`.

The scripts use the existing server scene/CAD, generated query meshes, models, calibration and previous run inputs. Those server assets are not duplicated here; copying these scripts alone does not recreate a runnable Isaac scene. Historical experiments remain in the source folder and are not deployment defaults. Exact trial source hashes/configuration accompany each report.

GPU2 supplies cuRobo IK proposals, validated against exact exported USD FK/joint limits and the original PhysX/pair/payload checker. GPU3 uses Warp ray tracing for RGB-D images. Warp is not RTX photoreal rendering. CPU PhysX remains unchanged. Only RGB-D and robot encoder observations enter pose control. Free actor state is retained privately for image generation/evaluation.

Current-policy random full-task reliability is NOT demonstrated. v260: deep pin grasp remained stable through direct insertion/release/withdraw, then ring pregrasp cross-arm contact aborted. v261: near-COM pin grip rolled and was rejected. v262: deep pin hold remained measurable, but axial insertion IK failed. None is a complete three-part success. v258/259 expose respectively a transient camera timeout and a subsequently fixed end-tip radius classification bug. Their failed outcomes are preserved.

The next change adds explicit predictive other-arm geometry and checks the actual issued pregrasp curve/aperture; new trials263/264 are in progress. Actual field hardware has not been connected/tested.

## Next source checkpoint

The Hand surface self-filter uses recorded capture TCP/encoder aperture and official native Hand mesh surfaces only (1.2mm surface band); it never erases the enclosing Hand volume or uses loose actor poses. Four archived late-close false palm-obstacle cases are resolved. The newest445 physical run266 picked/transferred both rings, but failed its large-ring prealignment. Run268 rejects a pin terminal branch before carry; full-task failure reports are retained. Direct class selection also avoids duplicate preview/replanning.

Portable cuRobo GPU2 service sources and exact USD-derived reference model are under `roas-table/gpu_planning/`. GPU route proposals use approximate cached8mm ESDF plus current encoder/RGB-D obstacle envelopes; all original PhysX/native Hand/other-arm/payload checks and FR3 retiming remain authoritative. Default-off route support is being tested, not presented as an accepted physical route. CUDA binaries, mesh caches and runtime sockets are excluded.

## GPU continuous-route checkpoint (v273)

One recorded carry query passed the original full PhysX checks for the continuous cubic and the geometry of the final float32 command curve; all 862 issued commands were checked against the existing FR3 limits. GPU proposal 0.256s, round trip 0.286s, planned physical duration 1.79375s, complete static diagnostic 8.4366s. This trial did not command physical robot motion. Native self-collision exclusions and empty-arm actuator impulses are under investigation; do not treat this as a complete physical-cycle or hardware certificate.

`direct_pin_skill.py` optionally explores axially symmetric pin orientations without moving the calibrated physical axis goal. Full-bin initial images can use explicit bounded pixel/field-of-view configuration. These are research integrations under physical testing. Full task/hardware success remains false where recorded. The unchanged field release remains under `field_demo_random_20261022/`.

## Authored same-arm clearance and actual overhead capture

The omitted nonadjacent link5/link7 pair caused the empty-arm impulse in272. Added authored-CAD same-arm clearance (exact authored filtered pairs/direct joint exclusions only), including aperture sweeps and intended-fixture robot checks. It rejects the archived pre-impact/impact states and retains all862 states of273. Trial278 physically parks successfully in1.40833s and captures one fixed overhead RGB-D frame. That single frame supports a large-ring approach/40–70mm lift geometric component; whole-task/real-hardware acceptance remains unproven.

LEFT GPU IK uses strictly matched intrinsic-chain/base/TCP reference transforms and independently validates all returned joints against originalLEFT FK/bounds. Frozen current-policy physical runs are still testing grasp generalization and post-insertion withdrawal. Earlier274 confirms pin/large-ring retention and small-ring depth, but stops on an empty-arm withdrawal collision, so its full-task pass remains false.
