# Isaac PhysX research source checkpoint

This directory captures the current research scripts and honest failure reports. It is **not** the field executable. Start with `../FIELD_RGBD_20261022.md` and `../field_demo_random_20261022/`.

The scripts use the existing server scene/CAD, generated query meshes, models, calibration and previous run inputs. Those server assets are not duplicated here; copying these scripts alone does not recreate a runnable Isaac scene. Historical experiments remain in the source folder and are not deployment defaults. Exact trial source hashes/configuration accompany each report.

GPU2 supplies cuRobo IK proposals, validated against exact exported USD FK/joint limits and the original PhysX/pair/payload checker. GPU3 uses Warp ray tracing for RGB-D images. Warp is not RTX photoreal rendering. CPU PhysX remains unchanged. Only RGB-D and robot encoder observations enter pose control. Free actor state is retained privately for image generation/evaluation.

Current-policy random full-task reliability is NOT demonstrated. v260: deep pin grasp remained stable through direct insertion/release/withdraw, then ring pregrasp cross-arm contact aborted. v261: near-COM pin grip rolled and was rejected. v262: deep pin hold remained measurable, but axial insertion IK failed. None is a complete three-part success. v258/259 expose respectively a transient camera timeout and a subsequently fixed end-tip radius classification bug. Their failed outcomes are preserved.

The next change adds explicit predictive other-arm geometry and checks the actual issued pregrasp curve/aperture; new trials263/264 are in progress. Actual field hardware has not been connected/tested.
