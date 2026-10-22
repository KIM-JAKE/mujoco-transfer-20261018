# Actual517 PIN-only failure: .8 preclose gain, original postcarry guard

Actual517 failed at **48.331484s wall** during the first fresh postcarry hold check. The directly measured class-end circle was valid, but public RGB-D/SDK relative-axis drift was **80.917mrad**, exceeding the unchanged **50mrad** limit; center drift was0.49477mm within the unchanged2mm limit. The original guard stopped correction. This is a PIN-only component with no ring task, whole-task success or speedup claim; hardware remains untested.

The preclose .8 gain reached the original300µm gate after three bounded corrections:659.7→418.0→343.8→251.5µm. This is narrow executed evidence only, not proof of general gripping reliability. The−2mm collar target and qualified-depth flag were configured; full-depth qualification was never reached. Exact517 source, diff versus516 and public executed evidence are in `preclose-gain-qualification/`. No independent bounded-gain fixture was available; source validation and original safety gates are retained rather than implying one exists.

`code/` and `support-code/` contain427 verified frozen input files, including420Python sources. Original495 archived services are copied unchanged. Public PIN RGB-D packets are hashed. Later518/519 changes are not included.

The separate camera diagnostic `../../component-evidence/fixed512-513-camera-profile/` rejects the proposedGPU14fallback tier: all four frozen LARGE precision packets already returned their required class with the existingGPU6service. Its timings are component diagnostics, not a whole-cycle gain; no new service was deployed.

No global guide, RESULTS, Git index/commit, live process, physical job or controller source was modified to make this archive.
