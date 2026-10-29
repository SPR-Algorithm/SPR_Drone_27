# PX4 1.16 DDS host preparation

Owner: framework/VIO. This directory prepares the real `SensorCombined` path;
it does not operate a flight controller. Device ownership remains with
`drone_bringup`. Installing an agent alone does not change USB MAVLink into DDS.

## Host dependency

```bash
bash docker/px4/build_host_agent.sh
source /opt/ros/jazzy/setup.bash
~/.local/opt/spr-microxrce-agent/2.4.3/bin/MicroXRCEAgent --help
```

`agent-lock.yaml` fixes Agent 2.4.3 at
`73622810d984349b80bbac0ef55fc0b694d62222`, matching PX4's Jazzy guidance.
The script checks the installed FastDDS 2.14.6/FastCDR 2.2.7 Debian pins before
building against their existing ROS ABI. It builds upstream-supported spdlog
1.9.2, with bundled fmt, statically inside the private prefix: system
spdlog 1.12/fmt9 fails to compile this agent's ostream formatting. It never
installs system packages or changes ROS libraries. Superbuild/P2P/CED/tests are
disabled, so CMake cannot fetch floating dependencies. The agent source and
logger hashes are checked, and dirty checkouts are rejected.

Defaults:

- Source/build cache: `~/.cache/spr/microxrce-agent/2.4.3/`.
- Install: `~/.local/opt/spr-microxrce-agent/2.4.3/`.
- Evidence: `build-manifest.txt` and `runtime-libraries.txt` in the install prefix.
- Overrides: `SPR_AGENT_CACHE`, `SPR_AGENT_PREFIX`, `SPR_AGENT_BUILD_JOBS`.

After a dependency update, review the lock and script together. This script is
for the inspected Ubuntu 24.04/Jazzy host; it is not an ARM64/Humble dependency
validation. The official workspace matrix uses Agent 2.4.2 with Humble.

## Firmware transport constraints

The inspected physical path is
`/dev/serial/by-id/usb-MicoAir_MicoAir743v2_0-if00`, currently `/dev/ttyACM0`.
Existing `rm_nav` resources use **USB MAVLink/MAVROS** and are not DDS agents.

Stock PX4 1.16 has two relevant limitations:

1. `dds_topics.h.em` caps every outgoing uORB subscription at 10 ms, about
   100 Hz. A host parameter cannot make this `SensorCombined` stream reach
   the project's 200 Hz target.
2. `TimesyncStatus` is multi-instance; DDS normally subscribes to instance 0.
   If MAVLink publishes first, that instance can describe the wrong clock
   estimator. Bridge acceptance requires `source_protocol=2` (DDS).

`0001-imu-rate-and-dds-timesync-instance.patch` applies only to PX4
`6ea3539157ca358c70a515878b77077af7d4611d` (`v1.16.0`). It uses a 4 ms IMU
notification interval (250 Hz transport ceiling, leaving headroom), preserves
the default interval for other topics, and selects the DDS timesync instance
by its actual source protocol. Missing DDS status fails initialization.
Message definitions and offset conversion remain unchanged.

Example source preparation, separate from any flash operation:

```bash
# In a dedicated PX4 checkout with the project's recorded ARM toolchain:
git checkout --detach 6ea3539157ca358c70a515878b77077af7d4611d
git submodule update --init --recursive
bash /absolute/path/SPR_Drone_27/docker/px4/check_firmware_patch.sh "$PWD"
git apply /absolute/path/SPR_Drone_27/docker/px4/0001-imu-rate-and-dds-timesync-instance.patch
make micoair_h743-v2_default
```

Check the installed board target and exact firmware hash before choosing this
build. MicoAir H743 **v2** maps TELEM2 to `/dev/ttyS3`; the older H743 maps it to
`/dev/ttyS1`. Never copy the older board mapping. This patch has been checked
against immutable upstream source; a successful patch check is not a firmware
compile or hardware test. Keep build logs and source/patch/toolchain hashes
before a separately reviewed upload. Existing local firmware may differ.

## Selecting the real channel

Preferred persistent setup is TELEM2 plus a suitable USB-UART adapter: reserve
that port for DDS, set `UXRCE_DDS_CFG=102`, match `SER_TEL2_BAUD=921600`,
`UXRCE_DDS_SYNCT=1`, and the same `UXRCE_DDS_DOM_ID` as the ROS host. Preserve
QGC/MAVLink on a separate channel. Confirm any existing port assignment first;
do not blindly disable an arbitrary MAVLink instance.

The currently connected USB cable needs an explicit FCU-side switch instead.
`UXRCE_DDS_CFG` has no USB selection, and `SYS_USB_AUTO=2` starts MAVLink.
Auto-detect mode supports MAVLink/NSH/ublox, not XRCE. The following are the
**PX4-side transport commands**, not commands for the Linux host:

```text
cdcacm_autostart stop
mavlink stop -d /dev/ttyACM0
sercon
uxrce_dds_client start -t serial -d /dev/ttyACM0 -b 921600
```

They must run from an independent console or a previously staged delayed SD
script, because stopping USB MAVLink closes its own shell connection. A one-shot
nonpersistent bench script can restore stock USB behavior on reboot; a persistent
`/fs/microsd/etc/extras.txt` setup also needs an explicit USB autostart policy and
recovery procedure. Inspect/backup existing SD scripts first. This repository
does not automatically upload scripts, alter FCU parameters, reboot, or flash.
`UXRCE_DDS_SYNCT=1` and a unique nonzero `UXRCE_DDS_KEY` still apply.

Once the FCU really runs that channel, the **bringup-owned host agent** uses:

```bash
~/.local/opt/spr-microxrce-agent/2.4.3/bin/MicroXRCEAgent serial \
  --dev /dev/serial/by-id/usb-MicoAir_MicoAir743v2_0-if00 -b 921600 -v 4
```

Do not run MAVROS, QGC USB, a shell, and the agent on that same endpoint together.
Bringup supplies the device lock and supervision; the standalone command above
only documents its arguments.

## Required real acceptance

Inspect the actual firmware client status and `/fmu/out/sensor_combined` delivery
rate, verify `/fmu/out/timesync_status.source_protocol==2`, then inspect
`px4_bridge/imu` diagnostics. The bridge must reach its <=1000 us observable
uncertainty gate and remain continuous. Record raw inputs, `/spr/imu`, diagnostics,
firmware/patch/agent hashes, frequency/gaps and reconnect behavior in a real run.
The firmware patch does not prove 200 Hz delivery or <=1 ms physical clock
accuracy. Source IMU integration rate, USB/UART bandwidth and scheduling also
need measurement. Do not relax the bridge gate to label an unsuitable link ready.

Sources:
[PX4 1.16 agent setup](https://docs.px4.io/v1.16/en/middleware/uxrce_dds),
[outgoing DDS template](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/uxrce_dds_client/dds_topics.h.em),
[USB protocol manager](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/drivers/cdcacm_autostart/cdcacm_autostart.cpp),
[MicoAir H743 v2 target](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/boards/micoair/h743-v2/default.px4board).
