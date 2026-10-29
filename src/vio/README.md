# vio
Owner: SilverWolf (integration).

OpenVINS integration, configuration and playback validation. This package reserves the Project.md boundary; it currently installs
this contract only and starts no node. It is not an implemented flight capability.

The pinned upstream packages are src/open_vins/ov_*. Use the calibration candidate exporter to obtain stereo IR / FC IMU configuration. The D455 single-color template is incompatible. Replay evidence is required before promoting a calibration.

Frames, QoS, rates and topics: Project.md §§4–5. No executable is provided yet.
Build: `colcon build --packages-select vio`.
