# ADR 0003: Explicit C++20 workspace and pinned OpenVINS compatibility

Date: 2026-10-29. Status: implementation prepared; build verification pending.

## Requirement and conflict

The current repository `AGENTS.md` requires every C++ target to use C++20 with
`-Wall -Wextra -Wpedantic`. The calibration package previously defaulted to
C++17. OpenVINS commit `69488123ed9362dd44b6f28e7f4680abbff1442b` unconditionally
selected C++14 in `ov_core`, `ov_eval`, `ov_init`, and `ov_msckf`. A command-line
`-DCMAKE_CXX_STANDARD=20` alone cannot override those normal CMake variables.
Generated ROS interface targets also need the workspace setting.
Jazzy's installed `rosidl_typesupport_*` generator CMake files explicitly set
generated target properties to C++17 even when the cache standard is 20.

## Decision

Calibration explicitly requires C++20 with extensions disabled. Root
`colcon_defaults.yaml` sets the same standard, required flag, warning flags,
system Python interpreter, and compile-command export for every CMake package.
`bash build_workspace.sh` supplies these options explicitly and builds packages
sequentially with at most four compiler jobs by default. This entry point avoids
colcon's replacement of default CMake arguments when callers provide their own.
It also injects `cmake/RequireCxx20.cmake` through `CMAKE_PROJECT_INCLUDE`.
The hook uses a deferred, recursive target-property policy after configuration
to require C++20 for every compiled target, including generated interfaces.
It requires CMake >=3.19; the Jazzy host uses 3.28.3. Check this requirement
on the Humble target. Installed ROS generator files and frozen DDS source are
untouched.

The separate, reviewed `docker/openvins/cxx20.patch` changes four CMake standard
assignments and comments at the pinned revision. The older Jazzy header patch
and its two patched headers retain their exact original SHA-256 values. Both
patches are checked before application, can be reapplied safely, and preserve
unrelated edits. No upstream commit, submodule pointer, source formatting,
firmware, DDS configuration, or DDS bridge source is changed by this decision.

`git_baseline` accepts only the exact combined diff and six recorded file hashes,
with both patches committed in an otherwise clean root. The raw Git status and
both patch identities remain in provenance. An unborn or otherwise dirty root
continues to fail the formal capture gate. Patch identities and hashes are in
the [compatibility README](../../docker/openvins/README.md).

## Validation and limits

The required evidence is a successful complete root build on the host plus
inspection of every exported C++ compile command for `-std=c++20`, the required
warnings, and no conflicting later language-standard option. This also checks
generated interface targets. Record command counts, package counts, and the
build log before changing this ADR status to verified.

Humble and target-computer builds remain unverified until run on their actual
toolchain. This build decision does not claim estimator accuracy or hardware
calibration success. No synthetic calibration dataset or self-test is used for
this validation.
