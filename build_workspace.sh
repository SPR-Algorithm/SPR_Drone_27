#!/usr/bin/env bash
# Reproducible root build; C++20 also applies to generated interface targets.
set -euo pipefail

workspace_build_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd -- "$workspace_build_root"
if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "Source /opt/ros/jazzy/setup.bash (or the target ROS distribution) first." >&2
  exit 2
fi
for workspace_build_arg in "$@"; do
  if [[ "$workspace_build_arg" == --cmake-args* ]]; then
    echo "Use colcon selection options only; this entry point fixes the required CMake arguments." >&2
    exit 2
  fi
done
bash docker/openvins/apply_compat.sh
export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-4}
export MAKEFLAGS=${MAKEFLAGS:--j4}
exec colcon build --symlink-install --executor sequential "$@" \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_CXX_STANDARD=20 -DCMAKE_CXX_STANDARD_REQUIRED=ON \
  -DCMAKE_CXX_EXTENSIONS=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  "-DCMAKE_PROJECT_INCLUDE=$workspace_build_root/cmake/RequireCxx20.cmake" \
  '-DCMAKE_CXX_FLAGS=-Wall -Wextra -Wpedantic'
