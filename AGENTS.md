# Repository Guidelines

## Project Structure & Module Organization

`Project.md` and recorded ADRs define the contract. ROS 2 packages belong in `src/`. `calibration` provides stereo and camera–IMU tools; `drone_bringup` owns devices and rig configuration. DDS in `px4_bridge` is currently frozen; use the MAVLink calibration profile. Preserve pinned `src/open_vins/`. Keep instructions in `docs/`, decisions in `docs/adr/`, and generated data outside Git. Historical captures remain in `calibration/data/`.

## Build, Test, and Development Commands

Run from the repository root using Bash:

```bash
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
ros2 run tools spr check-rig
```

The explicit Python path avoids selecting a user-installed interpreter without ROS dependencies. Use the corresponding Humble setup on the target computer. Current calibration validation uses real captures; consult package READMEs before hardware launches and select tests accordingly.

## Coding Style and Naming Conventions

- All C++ code across the entire repository must use the **C++20 standard**. Configure CMake targets and build settings consistently to require C++20.
- C++ code uses two-space indentation, `PascalCase` for types, and trailing underscores for private member variables.
- Enable `-Wall -Wextra -Wpedantic` for C++ compilation.
- Python code uses four-space indentation and `snake_case` naming conventions. The existing Python coding standard remains unchanged.
- Keep ROS 2 package names lowercase.
- Follow the conventions of existing helper functions and modules rather than reformatting unrelated code.
- No repository-wide formatter is currently established.
- Preserve OpenVINS' upstream layout, licenses, and existing language-specific formatting conventions. Do not reformat unrelated upstream code. Where repository-wide C++20 requirements conflict with the pinned OpenVINS revision or its build requirements, document the compatibility issue and resolve it explicitly rather than silently modifying upstream code or submodule pointers.

## Testing Guidelines

Register meaningful regressions with CTest/ament so `colcon test` discovers them. Name Python tests `test_*.py`. Check actual 8×11 inner-corner boards with 0.025 m squares, calibration recovery, invalid inputs, transform directions, and time-offset signs. No numerical coverage percentage is established. Use captured images and bags for requested validation; do not generate synthetic calibration datasets. Report hardware, profile, thresholds, and remaining unverified requirements.

## Commit & Pull Request Guidelines

History currently starts with `init`; no scoped convention is established. Use imperative, scoped messages such as `calibration: validate stereo coverage`. PRs should explain the problem, behavior, tests, and requirement IDs. Record contract changes in an ADR; update schemas and documentation together. Make submodule revisions explicit.
