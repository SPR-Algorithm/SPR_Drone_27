# drone_bringup
Owner: SilverWolf。设备启动、配置与台架采集入口。
当前默认 profile 为 `mavlink_calibration`，配置位于
`config/rig/d435i_mavlink.yaml`，由 `config/rig_mavlink.schema.json` 校验。
相机 SDK 序列号与 USB 描述符序列号不同，两者均需记录。

当前实现保留了既有 MAVLink 标定采集通路；新版 [Project.md](../../Project.md)
要求的 A0 主链路为 uXRCE-DDS → IMU 适配 → 实时 OpenVINS。
本包尚未提供完整实时 VIO 启动组合。配置中的 140/200 Hz、1 ms 同步检查和
标定图像质量门限属于已有采集入口的检查条件，不是新版 A0 的统一验收门槛。
下列命令仍用于现有台架采集；DDS 和实时 VIO 需要分别完成功能验证。
DDS 采集血缘按诊断记录的 `sync_budget_us` 检查时间不确定度；历史记录缺少该字段时沿用 1000 µs，默认配置保持不变。

先完成根工作区构建、加载 overlay，并按 [环境说明](../../docs/environment.md)
配置 `SPR_MAVLINK_PYTHONPATH`。连接设备并释放 QGC/MAVROS 的串口占用后采集：

```bash
ros2 run tools spr check-rig --profile mavlink_calibration
ros2 run tools spr record --profile mavlink_calibration --mode imu --duration 20
ros2 run tools spr record --profile mavlink_calibration --mode stereo --duration 180 --show-ui
ros2 run tools spr record --profile mavlink_calibration --mode allan --duration 3700
ros2 run tools spr record --profile mavlink_calibration --mode joint --duration 180
```

`imu` 检查短时 IMU/时间质量；`allan` 录制静止 IMU，至少一小时才可作为正式噪声分析。
`stereo` 保存 IR、camera_info 和通过清晰度/覆盖门限的派生 PNG。
`joint` 记录双目、camera_info、`/spr/calibration/imu` 与 `/spr/diagnostics`；
固定棋盘，整体移动刚性连接的相机与飞控。运动录包不套用静态双目标定的覆盖率门限。
`--save-board-images --show-ui` 可为 joint 添加棋盘预览及 PNG sidecar。

当前由 `calibration_mavlink_imu` 独占 USB，读取 HIGHRES_IMU，以 TIMESYNC
测量仿射时钟映射。实测频率门限为 140 Hz；请求频率和 ACK 不能代替实测。
正式采集仍需可复现代码基线、预检和磁盘空间；`--experimental` 用于诊断，不能把失败会话标成通过。

详细采集语义见 [README.mavlink.md](README.mavlink.md)，端到端步骤见
[联合标定流程](../../docs/joint-calibration.md)。DDS 配置、Agent、`px4_bridge`
不由当前 profile 启动。历史 [ADR 0002](../../docs/adr/0002-mavlink-calibration-phase.md)
中的“先 MAVLink 标定、后迁移 DDS”安排已由新版 `Project.md` 替代。

Start hardware only here; calibration launch is subscriber-only. Cooperative
per-device flock and serial-open checks reject duplicates without killing them.
Preflight verifies actual stream dimensions, encoding, payload, monotonic host
stamps, rates, one publisher and stereo pairing. Runtime observes streams and
supervises children; a child exit or formal quality failure stops all children
in reverse order. SIGINT finalizes the bag. Insufficient pairs/coverage and
empty bags produce FAILED manifests. Logs are local under runs/<run_id>/.

Output camera topics follow `config/rig/*.yaml`, reliable, subscribers depth5 (driver
DEFAULT preset depth10); IMU is /spr/calibration/imu, best_effort, frame imu_link. Driver TF is
disabled; no artificial static camera-IMU transform is published.
This bench entrypoint does not arm or publish PX4 control commands.
