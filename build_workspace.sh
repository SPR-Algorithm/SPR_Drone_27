#!/usr/bin/env bash
# Build selected packages; each package owns its language and compiler options.
set -euo pipefail

workspace_build_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd -- "$workspace_build_root"
if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "Source /opt/ros/jazzy/setup.bash (or the target ROS distribution) first." >&2
  exit 2
fi
for workspace_build_arg in "$@"; do
  if [[ "$workspace_build_arg" == --cmake-args* ]]; then
    echo "Use colcon selection options; set language requirements in the owning package's CMakeLists.txt." >&2
    exit 2
  fi
done
export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-4}
export MAKEFLAGS=${MAKEFLAGS:--j4}
exec colcon build --symlink-install --executor sequential "$@" \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
