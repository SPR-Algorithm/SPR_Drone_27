# px4_bridge — A0 IMU path

Owner: framework/VIO (SilverWolf). This package subscribes to PX4 DDS messages;
it never opens a serial port, starts an agent, arms, or sends flight commands.
`drone_bringup` owns the agent/device lifecycle and launches this node.

| Direction | Topic | Type | QoS |
| --- | --- | --- | --- |
| Input | `/fmu/out/sensor_combined` | `px4_msgs/msg/SensorCombined` | best effort, volatile, depth 100 |
| Input | `/fmu/out/timesync_status` | `px4_msgs/msg/TimesyncStatus` | best effort, volatile, depth 100 |
| Output | `/spr/imu` | `sensor_msgs/msg/Imu` | best effort, volatile, depth 100 |
| Output | `/spr/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | best effort, depth 5, 1 Hz + transitions |

## Build and run

Use the root workspace and its pinned dependency manifest. `px4_msgs` is pinned to
release/1.16 commit `392e831c1f659429ca83902e66820d7094591410`; firmware must be PX4
1.16.0, and the actual firmware's message definitions must match. No floating
branch is a build input. Both inputs are in PX4 1.16's default DDS topic list.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to px4_bridge \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
# Normally started by drone_bringup; standalone transport debugging only:
ros2 run px4_bridge px4_bridge --ros-args \
  --params-file src/px4_bridge/config/px4.yaml
PYTHONPATH=src/px4_bridge /usr/bin/python3 -m pytest src/px4_bridge/test
colcon test --packages-select px4_bridge
```

The canonical default parameters live in `config/px4.yaml`; bringup loads this
file, then rig overrides for topics, `imu_frame_id`, and rate/gap policy. Parameter
changes require restarting the node. Default `min_rate_hz=200` is a target checked
in diagnostics; the bridge cannot increase the PX4 publication rate. Change the
firmware's DDS stream configuration and measure the real rate before calibration.

## Frames and measurement semantics

`SensorCombined` vectors are already SI units in the board-corrected FRD body
frame. Rotate both by `diag(1,-1,-1)` into `imu_link` (FLU), preserving specific
force including gravity; do not normalize or subtract gravity. Bringup owns the
static `base_link`/`imu_link` transform. This node publishes no TF.

PX4 1.16 `SensorCombined` **has no `timestamp_sample`**. `timestamp` is the gyro
integration end. The common sample time is `timestamp - gyro_integral_dt/2`.
Acceleration time is `timestamp + accelerometer_timestamp_relative -
accelerometer_integral_dt/2`; acceleration is linearly interpolated between
bracketing measurements at the gyro midpoint. It is never extrapolated. Invalid
relative timestamps, clipping, nonfinite values, excessive integration/skew/gaps,
and out-of-order samples are rejected. Interpolation can delay output by one
accelerometer sample. The first unbracketed sample is discarded and counted.

Orientation is unavailable (`orientation_covariance[0]=-1`). All-zero rate/accel
covariance means **unknown**, per `sensor_msgs/Imu`; it does not mean zero noise.
Positive diagonal measurement variances may be configured from a validated IMU
noise model. They stay conservative during interpolation. The `[-1,-1,-1]`
configuration sentinel means unknown; partial unknown entries are invalid.
See the [standard Imu covariance contract](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Imu.html).

## Time protocol and continuous segments

Require `UXRCE_DDS_SYNCT=1`, the agent on the ROS host, `use_sim_time=false`, and
`input_time_domain=agent_synced`. PX4's serializer already adds its session offset
to outgoing `timestamp`/`timestamp_sample`. The bridge explicitly uses an identity
mapping and **does not add TimesyncStatus.estimated_offset again**. That status
offset is PX4-minus-host; its negative is `T_host_px4`. `remote_timestamp` is
already host time. Boot-domain input, absent synchronization, and stale or future
messages fail closed rather than substituting message receipt time.

Publication requires 20 consecutive DDS-source quality samples (default), each
with `RTT/2 + abs(observed_offset-estimated_offset) <= 1000 us`. This observable
packet/filter budget assumes the normal two-way timing model; it is **not a
ground-truth demonstration of <=1 ms physical accuracy**. The real rig still
needs a measured timing report. Status timeout, host clock step, offset jump,
PX4 restart, calibration-count change, and IMU disconnection end the segment,
clear interpolation buffers, and require fresh synchronization. Input downtime
cannot reuse the old offset. Defaults are intentionally conservative and may
prevent output on an unsuitable serial link.

Diagnostics expose `segment_id`, `estimated_offset_us`,
`host_minus_px4_offset_us`, `delta_host_minus_px4_offset_us`,
`round_trip_time_us`, `sync_uncertainty_us`, `sync_ready`, `timestamp_mode`,
frame/covariance policies, rate, and drop counters. The diagnostics recorder
copies these into the run manifest/events. Do not concatenate different segments
into one Kalibr session. Kalibr's residual `timeshift_cam_imu` applies after the
firmware's live host-time mapping; never add the logged offset delta a second
time. For bag replay, play the recorded `/spr/imu` directly under `/clock`; this
live-agent bridge intentionally rejects `use_sim_time=true`.

## Authoritative semantics

These source-based corrections to the draft Project.md §4.4 must remain in the
time ADR and interface documentation:

- [PX4 1.16 SensorCombined definition](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/msg/SensorCombined.msg)
- [VotedSensorsUpdate integration endpoint](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/sensors/voted_sensors_update.cpp)
- [uXRCE timestamp serialization](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/Tools/msg/templates/ucdr/msg.h.em)
- [DDS session offset and resynchronization](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/uxrce_dds_client/uxrce_dds_client.cpp)
- [Timesync offset sign/filter](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/lib/timesync/Timesync.cpp)

Automated tests exercise math, gates, continuity faults, and real ROS message
transport when the generated PX4 dependency is available. Hardware frequency,
serial loss/reconnect, actual clock accuracy, ARM64 latency, and calibrated
camera/FCU mounting remain rig acceptance checks, not claims from synthetic tests.
The ROS publisher tests are disabled by default and require explicit
`SPR_ALLOW_SYNTHETIC_ROS_TESTS=1`; keep that unset during hardware work. They have
not been run under the current real-data-only verification constraint.
