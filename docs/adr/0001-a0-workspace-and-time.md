# ADR 0001: A0 workspace and PX4 IMU time semantics
Date: 2026-10-29. Status: implemented for bench calibration; team review pending.
Owner: SilverWolf.

The requested implementation starts from Project.md v0.3. Packages move to a
single root colcon workspace; historical raw captures remain at calibration/data
behind COLCON_IGNORE. OpenVINS remains a Git submodule at its original
69488123ed9362dd44b6f28e7f4680abbff1442b commit, moved to src/open_vins.
px4_msgs is imported via dependencies.repos at the exact release/1.16 commit.

Primary PX4 1.16 code contradicts two assumptions in §4.4:
SensorCombined has no timestamp_sample; its timestamp is the gyro integration
endpoint. Use gyro endpoint minus half gyro_integral_dt. Acceleration has a
separate relative endpoint; interpolate at the common gyro midpoint.
With UXRCE_DDS_SYNCT=1 the DDS serializer already shifts timestamps into agent
host time. The bridge must not apply the offset again. TimesyncStatus offset is
PX4 minus host; record its negative as T_host_px4. The bridge requires DDS
source_protocol=2 and a measured uncertainty <=1 ms; missing or stale evidence
blocks publication. The default firmware DDS topic list may not expose the DDS
TimesyncStatus instance; check this explicitly before formal acquisition.

Camera identity requires both serial domains: this unit's SDK serial is
044322070753, USB descriptor serial is 043323052893 (observed 2026-10-29).
Store both and select the camera using SDK serial; check USB link using its
descriptor serial. They must not be compared as equal.

RealSense 4.58.4 accepts QoS preset DEFAULT (reliable), not a RELIABLE preset;
infra1_qos/infra2_qos and *_info_qos are per-stream parameters. The publisher's
DEFAULT preset depth is 10; project subscriber/recorder depths remain 5. A custom
publisher depth 5 requires a driver change and is not claimed implemented here.

Evidence:
- [PX4 DDS synchronization](https://docs.px4.io/v1.16/en/middleware/uxrce_dds)
- [PX4 v1.16 serializer](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/uxrce_dds_client/dds_topics.h.em)
- [SensorCombined definition](https://github.com/PX4/px4_msgs/blob/392e831c1f659429ca83902e66820d7094591410/msg/SensorCombined.msg)
- [RealSense QoS registration](https://github.com/realsenseai/realsense-ros/blob/4.58.4/realsense2_camera/src/profile_manager.cpp)
- [RealSense QoS presets](https://github.com/realsenseai/realsense-ros/blob/4.58.4/realsense2_camera/src/ros_utils.cpp)

This ADR corrects time interpretation, not the host-time interface. It does not
freeze the draft drone_interfaces messages or approve flight functionality.
