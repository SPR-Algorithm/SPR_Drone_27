# OpenVINS ROS 2 compatibility

OpenVINS remains pinned to `69488123ed9362dd44b6f28e7f4680abbff1442b`.
Its source layout, licenses, algorithm and language requirements remain upstream
owned. The former C++20 patch and recursive workspace target policy have been
removed in accordance with `AGENTS.md`; the original decision is retained in
[ADR 0003](../../docs/adr/0003-cxx20-workspace.md) as history.

Jazzy installs `.hpp` headers for `image_transport`, `tf2_geometry_msgs` and
`cv_bridge`, while this upstream revision includes their older `.h` names.
The existing `jazzy_compat.patch` adds header fallbacks in two visualization
headers. It does not change the C++ standard or estimator behavior.

## Explicit compatibility preparation

`build_workspace.sh` does not modify the submodule. Building project packages
such as `calibration` does not require this patch. If OpenVINS itself is needed
on Jazzy, inspect the patch and explicitly prepare that checkout:

```bash
bash docker/openvins/apply_compat.sh
source /opt/ros/jazzy/setup.bash
bash build_workspace.sh --packages-up-to ov_msckf
```

The script accepts an optional checkout path. It verifies the exact pinned
revision and patch checksum, accepts an already applied patch, and rejects
conflicts without resetting, cleaning, staging or changing the gitlink.
Unrelated edits are preserved, but they do not qualify for the clean-source
exception used by formal capture provenance.

Project C++ targets declare required C++17 and GCC/Clang warnings in their own
CMake files. Third-party targets retain their own requirements. Humble and the
selected RK3576 board still require verification in their actual environments.
Do not reuse a build cache containing the former global C++20 policy; keep old
artifacts separately and use new build/install/log directories.

## Provenance

New capture baselines allow only the exact Jazzy header patch as a local
OpenVINS exception, with the patch committed in an otherwise clean root.
Historical manifests containing the exact former C++20 plus Jazzy patch set
remain readable by the offline calibration tools. They are not accepted as a
new source baseline. Existing modified checkouts are not automatically reverted.

| Artifact | SHA-256 |
| --- | --- |
| `jazzy_compat.patch` and its `git diff --binary HEAD` | `0a8cbc71ee55baefb673f51f6c2a9fa19b3956891518e2fcd15aaea926e608cf` |
| Patched `ov_msckf/src/ros/ROS2Visualizer.h` | `d1b14228b0779c81b029b3d6da1fe8494f4712641e4fb20827ac626b87b36b26` |
| Patched `ov_msckf/src/ros/ROSVisualizerHelper.h` | `4594c6a8dde74c95fde48db78b0a07db9f1e6e858e5dff5d7206f659f4a2839e` |

When updating upstream, review the base revision, patch and provenance hashes
together. Never silently advance the submodule.
