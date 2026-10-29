# OpenVINS ROS 2 and C++20 compatibility

The submodule remains pinned to
`69488123ed9362dd44b6f28e7f4680abbff1442b`. Jazzy installs `.hpp` headers for
`image_transport`, `tf2_geometry_msgs`, and `cv_bridge`; this upstream revision
includes their older `.h` names. `jazzy_compat.patch` adds `__has_include`
fallbacks in two ROS 2 visualization headers. It preserves the ROS 1 branch,
upstream layout, licenses, estimator behavior, and submodule revision.

The repository's `AGENTS.md` now requires C++20. The pinned revision sets
`CMAKE_CXX_STANDARD` to 14 unconditionally in four packages, overriding a
workspace cache argument. The separate `cxx20.patch` changes those four
assignments and their explanatory comments to 20. It does not change the
existing header patch or either patched header's bytes. This explicit local
compatibility change is recorded in [ADR 0003](../../docs/adr/0003-cxx20-workspace.md).

## Prepare and build

From the repository root, after installing the dependencies in
[`docs/environment.md`](../../docs/environment.md):

```bash
git submodule update --init --recursive
bash docker/openvins/apply_compat.sh
source /opt/ros/jazzy/setup.bash
bash build_workspace.sh
source install/setup.bash
```

Use `/opt/ros/humble/setup.bash` on the Humble target. The build entry point
uses four compiler jobs by default; lower `CMAKE_BUILD_PARALLEL_LEVEL` and
`MAKEFLAGS` together for a memory-limited target. It explicitly requires C++20,
disables compiler extensions, enables `-Wall -Wextra -Wpedantic`, and exports
compile commands for all packages, including generated ROS interfaces. It
accepts colcon selection options, for example `--packages-up-to ov_msckf`.
Root `colcon_defaults.yaml` supplies the standard and warning cache defaults.
The build entry point also injects `cmake/RequireCxx20.cmake`: ROS generators
set generated targets to C++17 explicitly, so a deferred target policy is
required to raise them to C++20 after configuration. Supplying your own
`colcon --cmake-args` replaces defaults; use the build entry point for the full
workspace contract. CMake 3.19 or newer is required for this deferred policy.

The `.h` fallback remains available for older installations. Host verification
uses Jazzy, GCC 13, OpenCV 4.6.0, and Ceres 2.2.0; see ADR 0003 for the C++20
build evidence and unverified target combinations.

The script accepts an optional checkout path, verifies the exact base commit
and both patch checksums, and checks the entire patch set before changing files.
Repeating it recognizes already applied patches using `git apply --reverse --check`.
Conflicting or partially applied patches fail without resetting, cleaning,
checking out files, staging changes, or changing the gitlink. Unrelated local
edits are preserved. The applied patch intentionally appears as a local
submodule modification; record both checksums in capture/build provenance.

## Recorded SHA-256 values

| Artifact | SHA-256 |
| --- | --- |
| `jazzy_compat.patch` | `0a8cbc71ee55baefb673f51f6c2a9fa19b3956891518e2fcd15aaea926e608cf` |
| `cxx20.patch` | `634aec98e1688c4fbfc1ee412fba914d8fff6dc60cfa593e386480cf0f56c6a1` |
| `apply_compat.sh` | `7667902627bf5ac20ad6cff85b9c7f8e4557aeaf29df6d3cdcf702b605db4113` |
| Combined `git diff --binary HEAD` | `5c70ab56ae38bac04b71d69e1710b990ac42f71765892b24a24d90f4bda3b7c2` |
| Patched `ov_core/CMakeLists.txt` | `609f3ae53e8dcacd6369e6d62c07014c4c97aa10f6153f396062cd32fac9bbdb` |
| Patched `ov_eval/CMakeLists.txt` | `39152a7e54066db03d6909b78310ca4f040da37b17eb14a3b4966a5f98ceb4cd` |
| Patched `ov_init/CMakeLists.txt` | `0a906bcdf8c2215e9ae835a7e4005d1191caa94913153c59310feeb76b64c24b` |
| Patched `ov_msckf/CMakeLists.txt` | `970047d4541ddf49ef9db1768a1c5ef22bc61ef4d1a4617e49d393733f27ca2c` |
| Patched `ov_msckf/src/ros/ROS2Visualizer.h` | `d1b14228b0779c81b029b3d6da1fe8494f4712641e4fb20827ac626b87b36b26` |
| Patched `ov_msckf/src/ros/ROSVisualizerHelper.h` | `4594c6a8dde74c95fde48db78b0a07db9f1e6e858e5dff5d7206f659f4a2839e` |

When updating upstream, review and regenerate this patch, its base revision,
and all recorded hashes together. Never silently advance the submodule.
