# decision
Owner: team member to be assigned.

Decision team integration boundary. This package reserves the Project.md boundary; it currently installs
this contract only and starts no node. It is not an implemented flight capability.

Consume /spr/state/odometry and /spr/flight/status; publish only drone_interfaces messages. Never access devices or px4_msgs.

Frames, QoS, rates and topics: Project.md §§4–5. No executable is provided yet.
Build: `colcon build --packages-select decision`.
