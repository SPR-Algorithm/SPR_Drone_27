# sim
Owner: SilverWolf (integration).

Simulation and replay integration. This package reserves the Project.md boundary; it currently installs
this contract only and starts no node. It is not an implemented flight capability.

Real rosbag2 recording is supplied by drone_bringup; playback uses ros2 bag play <run>/bag --clock. PX4 SITL/Gazebo sensors and sim/live parity remain A1 integration work.

Frames, QoS, rates and topics: Project.md §§4–5. No executable is provided yet.
Build: `colcon build --packages-select sim`.
