#!/usr/bin/env bash
# Check only. Applying/building/uploading firmware is a separate explicit step.
set -euo pipefail
firmware_root=${1:?Usage: check_firmware_patch.sh /path/to/PX4-Autopilot}
firmware_revision=6ea3539157ca358c70a515878b77077af7d4611d
firmware_script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ "$(git -C "$firmware_root" rev-parse HEAD)" != "$firmware_revision" ]]; then
  echo "Firmware checkout must be exactly $firmware_revision (v1.16.0)." >&2
  exit 2
fi
git -C "$firmware_root" apply --check \
  "$firmware_script_dir/0001-imu-rate-and-dds-timesync-instance.patch"
echo 'Patch matches pinned firmware; no source, parameters, or flash changed.'
