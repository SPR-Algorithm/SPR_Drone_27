# SPR_Drone_27

Intel RealSense D435i IR 双目 + PX4 飞控 IMU 的视觉里程计项目。
需求真源为 [Project.md](Project.md)，当前实现重点是 **A0 框架和标定**。

## 当前实现

- 根 `src/` colcon 工作区；保留固定版本 OpenVINS 子模块。
- `drone_bringup`：唯一设备入口、MAVLink IMU 采集与时间映射、rig 校验、rosbag2 录制及进程监督。
- `calibration`：8×11 内角点、25 mm 棋盘识别；双目内参/相对外参、留出集/覆盖率/局部不确定度报告；Allan、Kalibr 联合标定及 OpenVINS 导出工具。
- `px4_bridge`：后续 DDS 迁移的冻结实现，当前 MAVLink 标定不启动它或 Agent。
- `diagnostics`：每次会话的 manifest、事件、指标、日志和离线 HTML。
- `drone_interfaces` v0.1 草案；`vio/state/flight_control/sim/planning/decision` 预留包边界，目前无飞行节点。

**当前先使用 MAVLink 完成相机—IMU 联合标定，之后再迁移 DDS。** 选择 `mavlink_calibration`，IMU 话题为 `/spr/calibration/imu`，当前测试要求实测 ≥140 Hz。DDS 不作为当前标定的前置条件，保持冻结；阶段安排见 [ADR 0002](docs/adr/0002-mavlink-calibration-phase.md)。

代码构建或棋盘检测通过，不表示真实联合标定已完成。当前流程仍需要真实 MAVLink 静止记录（Allan ≥1 小时）、运动与独立留出记录、Kalibr 求解和 OpenVINS 回放证据。本入口不解锁、不发布飞行控制命令。

## 构建

```bash
git submodule update --init --recursive
vcs import . < dependencies.repos
source /opt/ros/jazzy/setup.bash
bash build_workspace.sh
source install/setup.bash
```

使用 [环境说明](docs/environment.md) 安装依赖；构建入口会应用固定的 OpenVINS 补丁，并为生成接口强制 C++20。机载目标是 Humble，当前主机验证环境为 Jazzy。`calibration/` 只保留旧采集和旧构建产物，不再是独立工作区。

## 实机采集与标定

```bash
ros2 run tools spr check-rig --profile mavlink_calibration
ros2 run tools spr record --profile mavlink_calibration \
  --mode stereo --duration 180 --experimental --show-ui
ros2 run calibration calibrate --session runs/<run_id>/images/<session> \
  --output runs/<run_id>/intrinsics/attempt_001
ros2 run calibration joint_calibrate.py --help
```

飞控接入、静止/运动/留出数据采集与求解命令见 [MAVLink 联合标定流程](docs/joint-calibration.md)。联合采集需要相机和飞控刚性固定，棋盘保持不动，整体移动组件。

`--experimental` 用于当前开发台架，故障仍记录为 FAILED；正式采集要求代码基线和全部预检。
只通过 `spr record` 打开设备，不能同时另启一个 RealSense 节点。标定板是 **9×12 方格，8×11 内角点**，格边长 0.025 m。保持固定曝光，缓慢改变距离/倾角，覆盖四角并确保完整内角点可见。

[文档索引](docs/index.md) · [标定包说明](src/calibration/README.md) · [贡献指南](AGENTS.md)
