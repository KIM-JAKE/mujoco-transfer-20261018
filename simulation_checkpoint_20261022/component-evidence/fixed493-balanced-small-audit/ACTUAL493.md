# Balanced SMALL receiver and local CAD reuse: actual PLUS493

493 completed all three physical Isaac Sim insertions and final retention in212.539784s whole wall time, including cameras/perception/planning; simulated physics50.233333s. Both20/30second objectives remain unmet. Hardware and hard-realtime control remain untested.

This is a new416Python-file build. Relative to492, only balanced_small_receiver.py, loose_bin_task.py, payload_proxy_content_intern.py and run_realtime_recipe.py differ. Balanced30° receiver targeting, one bounded fresh preclose feedback and exact local CAD reuse were combined; this is not an isolated cause/speed benchmark or the same414-file build that failed492.

The live preclose correction requested141.69µm/7.581mrad, with153.36µm/.196mrad current command/SDK bias, within unchanged2mm/50mrad bounds. A fresh image after correction showed intended receiver grasp achieved within76.89µm/1.356mrad signed-axis difference. Original current-payload, actual/command, native-proximal and future giver-opening preflights admitted the correction before closing. Contact, preload, giver withdrawal and original SMALL stability gates passed.

Public RGB-D plus same-frame SDK TCP show postwithdraw→stable-hold center change2.30µm and original raw-axis comparison0.770mrad (<25mrad). Unit-normalized diagnostic axis angle is0.0549mrad; the controller's threshold calculation was not changed.492 had31.253mrad and then19.343mrad, correctly failing25mrad/5mrad gates.

Later loaded carry changed the measured relative grasp by0.571mm/41.025mrad, within the existing50mrad transport policy. Handover stabilization therefore does not establish that all relative movement was eliminated. Feedback and all original acceptance gates remain necessary.

Actual local CAD counters:57attachments,53interned reuses,4new contents;2,194raw proxy calls,2,074exact hits and120original computations. Invalid, poisoned, digest-mismatch and unsupported counters are zero. Only immutable local geometry is reused; posed collision decisions are not cached. Source/proxy content is revalidated.

The own-payload sibling timers total12.733s across42,880calls; completed-part guard5.244s across55,310calls; USD/PhysX transform/query timers15.529s across40,605calls. These nested diagnostic timings overlap. Different grasp geometry, paths and feedback prevent attributing whole-cycle changes to one feature.

RGB-D:70frames; median capture-to-ready0.234s, p950.416s, maximum1.177s, no3s violation; aggregate inference16.231s. Exact-state and width-sweep GPU receipts retain float64,1mm clearance,1nm outward reserve and original CPU fallback for unknown/context/cache-miss cases. Full counters, hashes and original command/encoder audits are in actual493-audited-profile.json.

No extra hold, force/speed cap increase, threshold relaxation, body attachment, post-initialization workpiece teleport or privileged object-pose controller input was introduced. This one fixed-supply variation pass does not qualify broad random-bin generalization or real robot operation.
