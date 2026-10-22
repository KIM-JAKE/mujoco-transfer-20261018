# Interpreting stationary positions and nonzero reported joint velocities

The recorded measurements are unchanged: multiple control-step joint-position/rigid-Hand poses remain constant while the reported instantaneous joint velocity is nonzero. **This does not establish sleeping, stale tensors, or frozen integration.** The earlier inference was too strong.

NVIDIA's PhysX engineer explains that TGS reports the velocity from the final internal solver substep. It need not equal joint displacement divided by the outer timestep: corrective constraint velocities can remain nonzero while the frame-end positions reach a steady state. This directly matches the observed position/velocity pattern. [NVIDIA explanation, reply 13](https://forums.developer.nvidia.com/t/dof-velocity-offset-at-rest/205799).

A newer primary PhysX issue documents solver-dependent articulation friction behavior and disagreement between finite-differenced joint positions and reported joint velocity; the maintainer acknowledges the structural behavior. It does not establish that this particular task has the same friction defect. [PhysX issue 498](https://github.com/NVIDIA-Omniverse/PhysX/issues/498).

Consequences: retain the original actual-position FK stationarity criterion; do not introduce a raw-qdot near-zero criterion as a substitute. The sleep/stabilization experiments are diagnostic variants, not proven fixes. Object control remains fresh RGB-D plus measured encoder FK, with the original final geometry/error gates and hardware caps.
