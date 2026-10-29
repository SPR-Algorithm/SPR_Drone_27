# state
Owner: SilverWolf (integration).

State aggregation and world/odom anchoring. This package reserves the Project.md boundary; it currently installs
this contract only and starts no node. It is not an implemented flight capability.

A1 requires reset-aware state arbitration and sole ownership of dynamic TF. No default/fabricated world-to-VIO transform is published.

Frames, QoS, rates and topics: Project.md §§4–5. No executable is provided yet.
Build: `colcon build --packages-select state`.
