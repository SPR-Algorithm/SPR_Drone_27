# flight_control
Owner: SilverWolf (integration).

Future offboard safety state machine. This package reserves the Project.md boundary; it currently installs
this contract only and starts no node. It is not an implemented flight capability.

A1 requires watchdog/geofence/offboard/failsafe implementation and SITL validation before any flight-control launch is added. No arming or control command is supplied here.

Frames, QoS, rates and topics: Project.md §§4–5. No executable is provided yet.
Build: `colcon build --packages-select flight_control`.
