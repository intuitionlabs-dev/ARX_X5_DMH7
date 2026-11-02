# Upstream source

- Project: https://github.com/pantor/ruckig
- Version: v0.15.3
- Commit: `37b6e7a4c60f5befd2506741d5f7d0ae7eefb3db`
- License: MIT, preserved in `LICENSE`.

`include/` is copied unchanged from upstream `include/`.
`src/` contains the unchanged core files from upstream `src/ruckig/`.
The cloud client, Python bindings, upstream build files, and upstream tests are
not included. `../CMakeLists.txt` builds only the local state-to-state solver,
without `WITH_CLOUD_CLIENT`. No network access is needed to build or run it.
