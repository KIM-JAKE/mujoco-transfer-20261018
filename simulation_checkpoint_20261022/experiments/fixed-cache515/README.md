# Actual515 whole-task failure: early-prime integration only

Actual515 failed at **193.992993s wall** during the PIN axial depth-correction preflight: native RIGHT left-finger tip versus original M80 `instance_0554`. No blocked correction row was admitted. This is **not** a whole-task success, a time-target success or a speedup measurement. Hardware remains untested.

The early-prime change produced one actual same-physics-epoch exact-proof hit (epoch6724,744issued samples); other scopes missed and retained the original guard. This demonstrates the narrow cache lifecycle path only. It does not prove whole-cycle reliability, cross-epoch reuse or a causal timing reduction. `early-prime-component-evidence/` contains the source, diff and lifecycle tests; those tests mock native collision math and are not physical qualification.

`code/` and `support-code/` are this run's exact frozen source427files, verified against `exact-source-manifest.json`; the same495 archived GPU service sources are under `service-sources/`. Current515 uses the−1mm collar target. It does not contain the later qualified-full-depth flag or517 preclose-gain change. The failure progress is in `pin_skill_progress` inside `pipeline-results.json`.

`prior-run514-EVALUATION-ONLY/` is explicitly a DIFFERENT prior run. It compares archived physics poses with images after the run and is never controller/sensor input. It shows real axial undertravel together with a bad final-cap estimate, and original robust-circle replay rejects that recorded image. It is diagnostic context, not515 physical qualification or a source change. Public PIN RGB-D packets from515 are separately archived and hashed.

No runtime source, live worker, physics job, global guide, Git index or commit was changed to create this archive.
