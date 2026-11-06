# Upstream source

- Project: https://github.com/pantor/ruckig
- Version: v0.15.3
- Commit: `37b6e7a4c60f5befd2506741d5f7d0ae7eefb3db`
- License: MIT, preserved in `LICENSE`.

`include/` and `src/` originate from upstream `include/` and `src/ruckig/`.
This is a locally modified **single-precision fork**, not an unmodified upstream
release. Numeric types, templates and constants use `float`; math calls select
FP32 overloads. Profile validation allows 8 epsilon units at limits and 32 at
endpoints, scaled by magnitudes and pre-cancellation integration terms. Positive
tiny time segments remain valid. Polynomial coefficient degeneracy retains its
small absolute threshold: relative `FLT_EPSILON` would discard valid short-time
polynomials. Newton convergence and brake margins use FP32 tolerances.
See `../tests/test_ruckig_c.c` for numerical regressions and streaming tests.
The cloud client, Python bindings, upstream build files, and upstream tests are
not included. `../CMakeLists.txt` builds only the local state-to-state solver,
without `WITH_CLOUD_CLIENT`. No network access is needed to build or run it.
