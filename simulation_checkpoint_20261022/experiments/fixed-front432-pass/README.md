# Fixed-front432: accepted native three-part PhysX cycle

SMALL and LARGE rings from separate fixed spots on the existing shelf, plus PIN from the moved left green holder, physically complete pickup, insertion, finite-force clamp retention and both empty-Hand retreat. Fresh RGB-D automatically determines sizes and signed ring faces; actor traces are evaluation/replay only. No attachment/post-initialization teleport or force/mass/friction/limit/margin relaxation.

The original evaluator reports `full_task_pass=true` and `vision_end_to_end_pass=true`, with **302.6591 total wall seconds /48.3083 physics seconds**. Final position errors LARGE0.487mm, SMALL0.266mm, PIN0.691mm; final0.6385wall/0.5417physics retention and160mm empty-Hand clearance pass. Command/actual FR3/nativeHand audits pass.20/30seconds and hardware operation remain unverified. This is a fixed-supply task, not new random-bin generalization. Concurrent jobs prevent a causal timing claim. CPU planning pauses SIM physics while wall waits count; physics time is not a real robot cycle prediction.

A fully checked0.25s LEFT lead allows actual LEFT SMALL carry to overlap RIGHT LARGE approach. LEFT completes before RIGHT closure in this version; grasping the next LARGE during ongoing LEFT insertion is a subsequent staged policy, not an achievement of432.

245 exact native Python/static input files are copied byte-for-byte in `code/` with SHA-256 in `exact-code-manifest.json`. Server CAD, generated native query meshes, Isaac runtime, public class/robot proposal priors and GPU services are external dependencies. This folder alone is not a standalone scene or field robot executable. The moved green holder is the existing photo-derived proxy with field dimensions unverified. Multi virtual camera visibility and the original SIM PINshaft/jig-fit assumptions remain unverified on hardware.

[Actual1× motion excerpts with explicit waiting CUT cards](../../media/fixed-front432-all3-pass-excerpts-1x.mp4). Warp GPU replay of all34 recorded bodies; not RTX and not a new physics run. Original302.66s wall total, time-target not met and hardware untested are permanently labeled.
