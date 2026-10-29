# drone_interfaces
Owner: SilverWolf. Version 0.1.0 draft implementation of Project.md §5.2.
Contains EstimatorStatus, FlightStatus, TrajectorySetpoint, MissionState.
All timestamps use host ROS time; frame and QoS are defined in Project.md §5.
No node or device access. Build: `colcon build --packages-select drone_interfaces`.
Interfaces are not frozen until team review; changes require an ADR.
