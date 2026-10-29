# drone_bringup
Owner: SilverWolf. Sole device-process owner for bench calibration.
Configuration: config/rig/d435i.yaml, validated against rig.schema.json.
The SDK and USB serial numbers differ on this camera; both are required.

```bash
ros2 run tools spr check-rig
ros2 run tools spr record --mode stereo --duration 120 --experimental --show-ui
ros2 run tools spr record --mode joint --duration 120
ros2 run tools spr record --mode allan --duration 3700
```

stereo records IR/camera_info and derived checkerboard PNGs; joint also runs
MicroXRCEAgent and px4_bridge and records /spr/imu plus diagnostics; allan records
IMU only. camera mode records IR without board-gated PNG capture.
Formal runs require an initial Git baseline, real DDS time/rate preflight and
disk space. --experimental allows diagnostic capture but never bypasses final
quality failure status. Capture preview is opt-in.

Start hardware only here; calibration launch is subscriber-only. Cooperative
per-device flock and serial-open checks reject duplicates without killing them.
Preflight verifies actual stream dimensions, encoding, payload, monotonic host
stamps, rates, one publisher and stereo pairing. Runtime observes streams and
supervises children; a child exit or formal quality failure stops all children
in reverse order. SIGINT finalizes the bag. Insufficient pairs/coverage and
empty bags produce FAILED manifests. Logs are local under runs/<run_id>/.

Output camera topics are Project.md §5.1, reliable, subscribers depth5 (driver
DEFAULT preset depth10); IMU best_effort depth100, frame imu_link. Driver TF is
disabled; no artificial static camera-IMU transform is published.
This bench entrypoint does not arm or publish PX4 control commands.
