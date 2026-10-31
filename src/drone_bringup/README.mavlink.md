# MAVLink input for offline calibration

This is the retained bench implementation. The current [Project.md](../../Project.md)
requires uXRCE-DDS for the A0 IMU path and real-time OpenVINS; this MAVLink
entrypoint does not satisfy that requirement. Its rate and timing thresholds
remain checks for existing captures, not the current project's acceptance criteria.

`drone_bringup.mavlink_imu.main()` starts `calibration_mavlink_imu`. It owns the
flight-controller USB endpoint with the same `DeviceLock` as other bringup
devices. The supervisor must not hold a second lock for this child. QGC, MAVROS,
and another USB reader must release that endpoint first.

The unified entrypoint currently defaults to the separately validated
`mavlink_calibration` profile. Its rig is `config/rig/d435i_mavlink.yaml`, checked
against `config/rig_mavlink.schema.json`; the DDS rig and schema are unchanged.
After building the workspace and sourcing `install/setup.bash`, provide the
pinned Python dependency directory and record real inputs:

```bash
export SPR_MAVLINK_PYTHONPATH=/path/to/venv/lib/python3.12/site-packages
ros2 run tools spr check-rig --profile mavlink_calibration
ros2 run tools spr record --profile mavlink_calibration --mode imu --duration 20
ros2 run tools spr record --profile mavlink_calibration --mode joint --duration 120
```

The child runs under `/usr/bin/python3` so ROS Jazzy remains ABI-compatible;
`SPR_MAVLINK_PYTHONPATH` supplies only the pinned additional dependencies. A
formal recording requires the reproducible committed baseline. `--experimental`
permits debugging source changes but cannot produce a publishable calibration.

`--mode imu` accepts at least 10 seconds and checks measured IMU/time quality;
it is not an Allan noise estimate. `--mode allan` still requires at least one
hour of stationary data. Joint mode records stereo images, camera information,
IMU, and diagnostics into a finalized, hashed bag. Joint motion is validated
offline and does not inherit the still-image stereo coverage/minimum-pair gate.
`--save-board-images` optionally adds the board PNG sidecar; `--mode stereo`
retains its original still-image quality gates. Keep the board fixed and move
the rigidly attached camera/flight-controller assembly for joint calibration.

The owner requests `HIGHRES_IMU` using transient `MAV_CMD_SET_MESSAGE_INTERVAL`,
reads back its previous interval first, and restores it on normal shutdown.
It writes no parameters, startup scripts, firmware, DDS configuration, or flight
commands. An armed heartbeat aborts acquisition. USB reconnects start a new
clock segment; clock steps, timestamp regressions, and missing synchronization
stop timestamped output until a fresh fit converges.

Use a private Python environment with the pinned `requirements-mavlink.txt` and
ROS dependencies visible. A hardware probe runs the exact same owner:

```bash
PYTHONPATH=src/drone_bringup /path/to/venv/bin/python \
  -m drone_bringup.mavlink_imu --probe-seconds 20 \
  --requested-rate-hz 200 --evidence-dir runs/unique-mavlink-probe
```

The ROS node publishes `sensor_msgs/Imu` to `/spr/calibration/imu` and a diagnostic
named `drone_bringup/mavlink_imu` on `/spr/diagnostics`. Device, baud, rates, topics,
frame, evidence directory, and synchronization thresholds are ROS parameters
listed in `DEFAULTS`. Gyro and acceleration are converted FRD → FLU with
`diag(1,-1,-1)`. Orientation is unavailable; gyro/acceleration covariance is
unknown (all zeros according to the ROS message contract).

PX4 1.16 HIGHRES_IMU contains estimator-bias-subtracted interval averages and
`vehicle_imu.timestamp_sample`. It does **not** transmit integration durations,
the accelerometer sample timestamp, clipping, sensor IDs, or calibration-change
counters. This module preserves the transmitted timestamp and never fabricates
an integration midpoint. Its diagnostics record these limits and identify the
input as `mavlink_calibration_only`.

MAVLink TIMESYNC replies are matched to actual requests, with local monotonic
RTT and a robust low-RTT affine boot-time → host-time fit. Keep `clock_scale`,
`reference_px4_ns`, `reference_host_ns`, `segment_id`, and the complete measured
snapshot with every run. `host_minus_px4_offset_us` is the offset **at that
reference**, not a constant for the whole recording. RTT/residual uncertainty
does not prove physical camera–IMU synchronization accuracy or compensate for
the missing integration periods.

The real MicoAir H743 v2 tests on 2026-10-29 accepted stream-rate requests but
delivered approximately 140–164 Hz for requests of 200, 250, and 1000 Hz. A
successful command ACK is not evidence of 200 Hz delivery. The standalone owner
defaults to a 200 Hz measured rate gate and reports WARN for these captures. The
unified MAVLink test rig explicitly requests 1000 Hz and gates actual delivery
at 140 Hz; the existing DDS rig separately retains its 200 Hz configured check. Actual input
timestamp rate, reception rate, jitter, gaps, clock-fit residual, and RTT remain
available in diagnostics and `status.json`; do not lower a gate to label the
current DDS or VIO requirement complete.

The historical 140 Hz **test** profile uses
`min_rate_hz:=140.0` and `deployment_scope:=mavlink_140hz_test_only` explicitly
for that profile and retains measured rate, jitter, gaps, synchronization fit,
and stationary-motion checks. The `original_dds_200hz_requirement_satisfied`
manifest field is retained for old capture compatibility; the current requirement
does not freeze a universal IMU frequency threshold. This profile does not prove
DDS input or flight deployment readiness.

Captured real TIMESYNC regressions, including a host clock step, run with:

```bash
PYTHONPATH=src/drone_bringup python3 -m pytest \
  src/drone_bringup/test/test_mavlink_time.py
```

Primary implementation evidence:
[PX4 HIGHRES_IMU](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/mavlink/streams/HIGHRES_IMU.hpp),
[VehicleImu FRD units](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/msg/VehicleImu.msg),
[PX4 TIMESYNC](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/mavlink/mavlink_timesync.cpp).
