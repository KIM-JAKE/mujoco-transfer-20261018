# Source environment snapshot

Recorded on the Kakao B200 host on 2026-10-18. This documents the source; AIDAS may require different GPU packages.

| Environment | Source interpreter | Dependency snapshot | Notes |
| --- | --- | --- | --- |
| Main M80/FR3/G2 MuJoCo | Python 3.13.14 | `requirements-main-snapshot.txt` | MuJoCo 3.15.0, python-fcl 0.7.0.11, OSMesa libraries bundled under `keti_m80_mujoco_20261013/renderer/` |
| CuRobo | Python 3.11.11 | `requirements-curobo-snapshot.txt` | Source venv inherited `/opt/conda` system packages, including PyTorch/CUDA. The editable `curobo` source is bundled under `keti_m80_mujoco_20261013/duo_fr3_2f85_20261013/vendor/curobo/`. Recreate against AIDAS drivers; the freeze alone is incomplete. |
| Other-session SAM3 | Python 3.11.11 | `requirements-sam3-snapshot.txt` | Source venv inherited `/opt/conda` system packages. SAM source and cached assets are in `robotics_agent/mujoco_duo_fr3/runtime/`; its external 3.50 GB checkpoint is included as a separate Git LFS archive and restored under `runtime/sam31_checkpoint/`. |

Source host GPU: NVIDIA B200; driver 580.95.05. AIDAS hardware and driver must be checked before GPU tests. The copied virtualenv directories are archival snapshots and contain source-host interpreter paths.
