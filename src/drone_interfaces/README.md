# drone_interfaces
Version 0.1.0 contains the existing draft messages EstimatorStatus, FlightStatus,
TrajectorySetpoint, and MissionState. This package defines messages only; it has
no node, device access, or publisher, and does not establish a topic's QoS.

Project.md REQ-A0-008 defines the current minimal pose/status requirement.
EstimatorStatus can describe availability and a reset counter, but its presence
does not implement `/spr/vio/status`. Flight and mission messages are retained
drafts for later stages, not implemented flight functions. Public field semantics
still require team agreement; no new messages are added for speculative needs.

Headers carry the producer's ROS sampling/status time. Consumers must know the
producer's time base and frame semantics; messages alone do not enforce either.
TrajectorySetpoint documents ENU/FLU and `world`; yaw is in radians, yaw_rate in
rad/s, and valid_until_s in seconds after header.stamp. Its consumer is not yet
implemented. Build with `colcon build --packages-select drone_interfaces`.
Generated project C++ targets use required C++17 and GCC/Clang warning options.
