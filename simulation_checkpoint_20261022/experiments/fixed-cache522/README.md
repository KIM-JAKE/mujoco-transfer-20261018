# Actual522 PIN-only component PASS; not whole-task acceptance

Actual522 completed a PIN-only contact task at **106.566527s wall /32.16875s physics**. **Zero ring tasks** ran and whole-task acceptance remainsfalse. Hardware and the time target are unvalidated. No ring2+ring1+PIN completion or speedup is implied.

The before-contact entry midpoint remains24mm below its calibrated goal: original public class/contact dimensions place the physical tip3.5mm below the fixture bottom, enabling fresh correction before contact. The archived geometry patch, CAD endpoint verification and helper/state tests are in `component-evidence/24mm-before-contact-geometry/`; all original motor/contact/precision limits were retained.

The shared16-image precision budget qualified after14 images (four FAR/ten ENTRY). Final ENTRY error was69.723µm radial,0.498mrad axis and75.728µm shaft error, within original75µm/3mrad/150µm. All ten ENTRY captures succeeded at the first new near-axis view13.

The final insert executed and a fresh full-depth image at view13 independently confirmed the part:345.129degree cap arc,64edge inliers,5877cap points,5.21055mm measured radius and20.784µm circle RMS. `robust_cap_circle_from_pixels=true`, propagatedXY=false and simulator-pose-input=false. Its measured goal-center error217.76µm/axis3.469mrad met the unchanged full-depth1.5mm/50mrad limits. Capture-to-ready was0.17943s. Release, axial withdrawal and a0.5physics-second retention window then executed.

There is **no additional RGB-D exposure after release/retention** in this component: the last public image is the qualified pre-release full-depth packet. Retention is represented separately by actual public robot encoder/command samples and the archived sequence; final part position0.70285mm/axis1.619mrad is explicitly post-run simulator evaluation, never a controller sensor or a fresh-camera claim.

The measured-width carry proof passed in27.0805s before2238issued rows; this is retained overhead, not a speed gain. Current428 frozen input files, including421Python sources, are byte-verified againstexact-source-manifest.json. Archived495 services are unchanged. Seven public RGB-D packets cover preclose, lift, FAR, ENTRY and full-depth; all23 observation records are preserved. Camera frozen519/520 probes are image-only component evidence and not additional physical trials.

No main guide, RESULTS, Git index/commit, live worker, controller source or physical job was modified to archive this result.
