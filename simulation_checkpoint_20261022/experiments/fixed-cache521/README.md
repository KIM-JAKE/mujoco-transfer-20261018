# Actual521 PIN-only failure: measured-width proof passed; precision did not

Actual521 failed at **94.819147s wall** under the original16-image total alignment budget. Its last fresh ENTRY image measured **84.316µm** radial error, above the unchanged75µm gate; axis error was0.597mrad within3mrad. The original guard blocked final insertion. No final stroke, qualified full-depth check, release, final withdrawal or retention completion was reached. This is not whole-task success, a PIN insertion completion, a time-target pass or hardware validation.

The new measured-carry-aperture proof DID pass in **26.915693s wall** before the2299-row carry field. It covered the actual/command start and seven total apertures9.3021–13.6979mm, including the original hold band and one-tick reach reserve, with current RGB-D payload G and original other-arm/native/placed/static checks. This narrow proof success does not establish cycle reliability or speedup. The exact receipts are in `pin_measured_carry_aperture_admissions`; the supplied component archival AST proof distinguishes the actual521 implementation.

The ENTRY near-axis view WAS exercised: all11 ENTRY observations succeeded at the first new index13 with the original240degree class-circle gate. This avoids the previously observed oblique view fallback in this particular case, but the changed width proof and physical execution mean its total wall time is not an isolated camera speed comparison. Full-depth near-axis views were not reached.

`code/` and `support-code/` contain428 verified frozen input files, including421Python sources. Service sources are copied from the unchanged495 archive. Seven selected actual public PIN RGB-D packets cover preclose/postlift/FAR and first/intermediate/last ENTRY states; the complete44-image metadata remains inrgb-observations.json. No fabricated or omitted successful completion is implied.

`component-evidence/` preserves ENTRY near-axis image-only frozen520 replay/lifecycle proofs, the bounded ENTRY gain-history-reset integration, and measured-carry-aperture proofs/source. Stage packages are development evidence; authoritative composite source bytes are exclusively undercode/support-code. Later522 before-contact changes are not part of this run. Simulator body poses appear only in explicitly post-run evaluation or camera image formation, never controller/perception measurement inputs.

No main guide, RESULTS, Git index/commit, live job or runtime source was modified to make this archive.
