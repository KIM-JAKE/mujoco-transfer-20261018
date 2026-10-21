# Source attribution

FR3-v2 arm collision STL, kinematic/inertial YAML and xacro files are copied from
[Franka Robotics franka_description](https://github.com/frankarobotics/franka_description/tree/7aeeddc449edf8d62b594f9e36a81da53e7796f9)
at commit `7aeeddc449edf8d62b594f9e36a81da53e7796f9`.
The upstream `LICENSE` is preserved alongside these files. `provenance.json`
records the exact upstream paths, Git blob hashes and SHA-256 checksums.

`arm-columns.json` and the numeric arrays in `collision-models.npz` are derived
collision envelopes; they are not vendor calibration or a hardware validation.
The triangle-clipped cover can be reproduced with
`scripts/generate-fr3v2-column-cover.py --output /tmp/arm-columns.json`
(developer NumPy/trimesh environment; not required on the field PC).

Native Hand source/licensing is preserved in the adjacent `native-hand/`
directory. The copied numeric finger hull and original M80 ring/pin class-local
CAD have separate source hashes in `derived-provenance.json`; no historical
world placement or simulator actor pose is used as a controller input.
