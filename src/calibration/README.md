# calibration

Owner: SilverWolf（框架 / VIO）。ROS 2 Humble/Jazzy 的 D435i 双目采集、相机内参求解与飞控 IMU 联合标定工具。硬件只由 `drone_bringup` 启动；本包订阅数据或离线读取 bag，不打开相机、飞控串口。

## 输入与棋盘格

- 左右 IR 图像：`/camera/infra1/image_rect_raw`、`/camera/infra2/image_rect_raw`，`sensor_msgs/msg/Image`，SensorDataQoS（best effort、volatile）。
- 相机信息：对应 `infra1/camera_info`、`infra2/camera_info`，同样使用 SensorDataQoS；记录实际 optical frame，不改变坐标系。
- 当前联合标定显式选择 `mavlink_calibration`：USB MAVLink `HIGHRES_IMU` → `/spr/calibration/imu`、`imu_link`；`TIMESYNC` 将飞控时间映射到主机。DDS/`px4_bridge` 冻结；离线 CLI 默认仍为 `dds_runtime`，必须显式传入 MAVLink profile。旧 MAVROS 或 RealSense 内置 IMU 不会被追认为本路径测量。
- 棋盘格为 **8×11 内角点，9×12 方格，边长 0.025 m**。物理方格边长必须实测；程序无法从图像推断真实毫米尺寸。检测支持棋盘旋转；SB 检测失败时使用经典棋盘检测与亚像素细化，始终要求完整 88 个角点。

## 构建

在仓库根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
bash build_workspace.sh --packages-up-to calibration
source install/setup.bash
```

C++20，依赖 OpenCV、yaml-cpp、rclcpp、sensor_msgs、message_filters。Python 离线工具的依赖见 `requirements-tools.txt`；Humble 使用相应 ROS setup 文件。

## 真实数据采集

先通过 `drone_bringup` 启动既定 rig 与原始 rosbag2 录制，再启动采集订阅器：

```bash
ros2 launch calibration calib_capture.launch.py \
  output_dir:=runs/session_001/images show_ui:=true
```

正式采集必须固定曝光/增益、关闭 IR 投影仪和内置 IMU，并保持分辨率与运行时一致。PNG、`index.csv`、`meta.yaml` 是派生数据，原始 bag 必须保留。

节点参数包含 `left_image_topic`、`right_image_topic`、`left_info_topic`、`right_info_topic`、`board_cols=8`、`board_rows=11`、`square_size=0.025`。bringup 可将 rig 转成参数字典传入。`expected_width/expected_height` 可限制分辨率；`expected_encoding=mono8`；`max_stereo_delta_sec=0.001` 限制双目时间差，索引记录左右时间戳及差值。

`show_ui=false` 可无界面运行；`opencv_threads=2` 限制 CPU 占用，每秒打印检测状态、模糊度、边距、数量与覆盖率。`input_timeout_sec=10.0` 后没有同步双目输入会报错退出。空会话、少于 `min_pairs=30` 或左图覆盖不足 80% 时退出码为 1。采集不会在第 30 对自动结束；移动棋盘覆盖视野、改变距离和倾角，完成后 Ctrl+C。GUI 支持空格保存、退格删除、R 重置方向锁、Q 退出。手动保存也会重新检测当前图像并检查完整角点、模糊度和边距，只允许绕过姿态多样性与保存间隔；超过 1 秒的旧快照不能手动保存。

## 相机求解与质量门槛

```bash
ros2 run calibration calibrate \
  --session runs/session_001/images/20261029_120000 \
  --output runs/session_001/intrinsics/attempt_001 --verbose
```

默认要求 ≥30 对有效输入；均匀留出 20% 图像，不参与拟合/剔除；最终训练集 ≥20 对、双目 RMS ≤0.5 px、单图 ≤1.0 px、两相机各自 8×6 网格覆盖率 ≥80%。留出集通过左图 PnP 估计棋盘姿态，再用固定双目外参预测右图，应用同样误差门槛。更改 CLI 阈值会写入报告，降低门槛的实验结果不能用于正式验收。

成功输出普通 YAML `camchain.yaml`、`cam0.yaml`、`cam1.yaml`；`T_cn_cnm1` 是 **cam0→cam1**，平移单位米，不填入虚构的 IMU 外参。`report.yaml`/`report.txt` 记录剔除、留出误差、覆盖率、内参局部线性协方差/相关性/标准差。`RED` 表示局部不确定度或条件数告警，需要复查；另外报告 22 维联合局部信息协方差：两相机各 8 个内参、双目旋转 3 维切空间和 3 维平移；每幅图的棋盘姿态由双目联合细化后消元，并输出交叉相关性。旋转标准差为度、平移标准差为毫米；这仍是依赖精确棋盘几何与像素噪声假设的局部估计，不包含系统误差或实测重复性。旧 OpenCV 格式 `meta.yaml` 仍可读取。每次求解使用新的输出目录，避免覆盖历史结果。放宽数量、覆盖率或误差阈值必须显式添加 `--diagnostic`；报告会标记非正式结果且不导出相机 YAML。奇异信息矩阵和负方差不会被填成零不确定度。报告明确标注未执行 bootstrap；完整信息矩阵不可观时外参协方差输出 `null` 与失败原因，正式模式不导出标定文件。

## 合并真实采集会话

同一相机、分辨率、棋盘、固定曝光/增益与工厂参数的补充采集可合并，原始文件保持不变：

```bash
ros2 run calibration combine_sessions.py \
  --session runs/session_001/images/20261029_120000 \
  --session runs/session_002/images/20261029_130000 \
  --output runs/combined_001/images
```

必须先停止采集。工具验证索引与双目 PNG、左右时间戳、记录的图像质量和两次相机配置，按原时间戳排序生成新 ID。默认用相对符号链接；`--copy` 可复制原 PNG 字节以独立归档。`combination_manifest.yaml` 记录每帧来源、哈希和原始会话配置；合并结果仍须通过全部默认标定门槛，不代表已完成标定。

## 飞控 IMU 与联合标定

完整实机操作、数据来源与验收要求见 [联合标定手册](../../docs/joint-calibration.md)。入口为 `ros2 run calibration joint_calibrate.py --help`。提供 `inspect`、`convert`、`allan`、`solve`、`validate`、`prepare-openvins`、`export` 子命令。正式流程需要 ≥1 小时静止 IMU bag、实际运动标定 bag、独立留出 bag、真实 Kalibr PDF 和 OpenVINS 回放证据；缺失时不得发布到 `calibrations/`。

`--profile mavlink_calibration` 必须贯穿所有离线步骤。当前按用户“先用 140 Hz 测试”的要求，仅该 profile 的 Allan、运动和留出包实测频率门槛改为 ≥140 Hz；请求/ACK 不证明实际频率。产物记录 `calibration_phase: mavlink_140hz_test`、`deployment_scope: mavlink_140hz_test_only`、`minimum_observed_imu_rate_hz: 140.0` 和 `original_dds_200hz_requirement_satisfied: false`，不改写 DDS 配置或宣称原 A0 ≥200 Hz 目标通过。Allan ≥1 小时与其他数值/血缘门槛保持不变。

`HIGHRES_IMU` 保留 PX4 偏置补偿区间均值及样本端点时间，消息缺失积分时长，不重造积分中点。时钟拟合误差不能独自证明物理采样同步；实际时差由运动求解及独立留出检查验证。即使本 profile 发布为 `VALIDATED`，使用范围仍限制在上述 MAVLink 测试阶段。

安装离线依赖时使用与 ROS 相同的系统 Python；以下会话名仅为示例，替换为真实录制目录：

```bash
/usr/bin/python3 -m venv --system-site-packages .venv-calibration
source .venv-calibration/bin/activate
pip install -r src/calibration/requirements-tools.txt
ros2 run calibration joint_calibrate.py inspect --profile mavlink_calibration --bag runs/motion_001/bag
ros2 run calibration joint_calibrate.py allan \
  --profile mavlink_calibration --bag runs/static_001/bag \
  --session-manifest runs/static_001/manifest.yaml --output runs/allan_001
ros2 run calibration joint_calibrate.py solve \
  --profile mavlink_calibration \
  --bag runs/motion_001/bag --camchain runs/stereo_001/camchain.yaml \
  --imu runs/allan_001/imu.yaml --allan-report runs/allan_001/allan_report.yaml \
  --session-manifest runs/motion_001/manifest.yaml --output runs/joint_001
ros2 run calibration joint_calibrate.py validate \
  --profile mavlink_calibration --candidate runs/joint_001 \
  --bag runs/heldout_001/bag --session-manifest runs/heldout_001/manifest.yaml
```

静止 bag 必须达到 3600 秒；标定/留出录制时固定棋盘，整体移动相机与飞控，二者安装关系必须刚性固定。`prepare-openvins` 仅生成候选回放输入；实际 OpenVINS 回放通过后，才能向 `export --replay-evidence` 提供真实报告并发布。

## 验证

`colcon test --packages-select calibration` 运行空采集拒绝与 Python 契约测试，不生成棋盘图像或合成标定数据。真实图像检测、求解和导出回归由 `stereo_calibration_real` 覆盖；未设置 `SPR_CALIBRATION_REAL_SESSION` 时明确跳过，设置的目录不存在或不满足默认门槛时失败。

```bash
# 指向已停止采集、满足默认门槛的真实 D435i 会话，包含 meta.yaml、index.csv、left/、right/
export SPR_CALIBRATION_REAL_SESSION="$PWD/runs/<session>/images/<capture>"
colcon test --packages-select calibration --event-handlers console_direct+
colcon test-result --verbose
```

回归在临时目录求解，检查留出误差、双目覆盖、cam0→cam1 变换及协方差，并验证输入文件哈希不变。历史 `--self-test` 和渲染棋盘测试不再注册到默认测试，不用于当前验证。真实图像回归只覆盖相机标定；飞控联合标定仍须实际采集与独立留出验证。
