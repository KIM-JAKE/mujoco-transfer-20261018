Keep the current requested CPU PhysX thread count0. The experimental2 setting was slower in this component. No live worker setting was changed.

| Requested count |2.0sim s physics wall | Inclusive control window |
|---|---:|---:|
|0|5.518374s|5.836622s|
|2|31.375473s|33.120631s|

Both fresh-process runs reproduced source494's exact first3142 command rows. The initial2182 rows were untimed warmup; the measured960 rows cover native SMALL closing, hold, breakaway and lift. Original1920Hz PhysX,480Hz control,128/64 solver,30N pad limits, velocity preload, callback/report scope, SDK motor guards and wall pacing remain unchanged. Native SMALL contact occurred on every563 controls after breakaway, with84.5276mm actual lift.

All issued/actual joint and finger q/v,34-body tensor transform arrays, link transforms and contact counts are bit-identical between0/2. Component0 also exactly matches original494's actual q/v and785 complete34-body transform snapshots. Body poses/contact evidence are evaluation-only and never feed commands. Source functions are literal494 extracts, runtime AST-compared against the sealed original under the same Python version. The only setup prefix edits are private output redirection and the requested thread setting/API call before scene attachment.

Cold Kit startup, scene/setup and prefix warmup are outside the timed metric and retained separately. This is a short CPU component experiment, not full-cycle or hardware qualification and not a controlled whole-server benchmark. Other services remain present. Source498 cold setup overlapped approximately the final10 seconds of the2 run; its complete untimed prefix was already6.40x slower before that overlap. Both component processes exited. Live workers remain configured0; their settings, queues and shared primaryCPU were not modified. Requested0's internal semantics are undocumented. No1/4 or repeated0 job was launched.

The two bootstrap errors are retained: a missing private-mirror fieldkit symlink and cross-Python-version AST dump metadata. They were fixed in packaging only; no controller/physics simplification was made. Runtime commands and exact source/seal files are included. Full565MB CAD is represented by original path/SHA, not duplicated.
