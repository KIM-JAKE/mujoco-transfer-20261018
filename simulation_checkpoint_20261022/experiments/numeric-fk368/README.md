# Numeric USD-frame FK368 (research)

Same original USD base, seven joint frames/axes, TCP transform and native finger transforms.252 joint configurations ×4 native openings compare10,080 body transforms against original Python with maximum element error6.7e-16. A model-base mutation rebuilds the cache.90 original complete loaded collision queries preserve verdicts and first rejections (numeric distances1e-12 tolerance), including the known negative robot self-collision. Original full collision checks remain authoritative; no filtering, contact geometry, force, mass or FR3 limits changed. Full-checker component1.108×; body-transform component8.34×. These are not whole-task speedups. Full physical371 remains pending.

Build with Eigen3.4.0 (`libeigen3-dev3.4.0-2ubuntu2`), g++11.4:

```bash
g++ -O3 -DNDEBUG -std=c++17 -fPIC -shared -static-libstdc++ -static-libgcc -I/path/to/eigen3 numeric_fk.cpp -o libm80_numeric_fk.so
```

Research wrapper currently locates the isolated server library at `.local/fastfk368` via absolute host/container paths; adapt ROOT for another installation. No default field package integration.
