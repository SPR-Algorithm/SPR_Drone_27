# D435i 双目与飞控 IMU 离线联合标定

本流程实现 `Project.md` 的 REQ-CAL-002/003/004/006/007/008/010/011。求解器使用实际 Kalibr；工具不会生成代替测量的数据或默认 IMU 噪声。硬件由 `drone_bringup` 独占，离线程序不打开设备。

## 当前采用 MAVLink，DDS 冻结

按当前任务约定，联合标定显式选择 **`mavlink_calibration`**：飞控 USB 的 `HIGHRES_IMU` 经 MAVLink `TIMESYNC` 映射到主机时间，发布 `/spr/calibration/imu`、`imu_link`。DDS 的源码、固件、Agent 和 `px4_bridge` 不在本次修改范围内。离线命令不指定 `--profile` 时仍保留原有 `dds_runtime` 合同，避免旧录包或旧脚本被默默解释为新的 MAVLink 流程。

当前用户明确接受“达不到先用 140 Hz 测试”，因此只将本 MAVLink 测试阶段的最低实测频率设为 **140 Hz**；DDS 运行通路及原 ≥200 Hz 目标保持原要求。所有产物显式记录 `calibration_phase: mavlink_140hz_test`、`deployment_scope: mavlink_140hz_test_only`、`minimum_observed_imu_rate_hz: 140.0`、`original_dds_200hz_requirement_satisfied: false`。这不是原 A0/DDS ≥200 Hz 验收通过声明。

用户随后明确“不用完整 allan 分析了”。当前 MAVLink 测试阶段取消完整 Allan 前置要求，允许显式 `--noise-prior` 工程先验；短时噪声测试和整小时静止录制均不再是当前前置条件。DDS 原路径保持完整 Allan 要求，见 [ADR 0005](adr/0005-mavlink-noise-prior.md)。本例外不取消真实运动、独立留出、时间符号、协方差和 OpenVINS 回放检查。

两条路径不互相代替：MAVLink 候选、Allan、留出集、回放和发布产物都必须显式选择同一 profile。即使通过本测试阶段所有检查，也不证明 DDS 采样或时间语义等价，不会自动成为 DDS 运行配置。既有 MAVROS `/mavros/imu/data_raw` 录包仍只能做格式回归。`formal_eligible`/`VALIDATED` 在此只表示通过已标注 profile 的血缘与数值检查，使用范围始终由上述阶段字段限制。

`HIGHRES_IMU` 的数值是 PX4 已做估计器偏置补偿的区间均值，时间戳采用 `vehicle_imu.timestamp_sample` 端点；消息没有积分时长、独立加速度时间、裁剪/标定计数或完整设备标识。程序保留这些限制，不重造“积分中点”，不将其标记为原始未补偿 IMU。TIMESYNC 的 RTT/拟合误差描述时钟映射，不能独自证明相机与 IMU 的物理采样误差 ≤1 ms；仍须实际运动、留出集时间符号和残差检验。

MAVLink header 使用实测仿射时钟映射 `host_ns = reference_host_ns + clock_scale × (px4_ns − reference_px4_ns)`；manifest 保留参考时刻、比例、漂移、RTT 和拟合残差。`T_host_px4_offset_s` 仅表示该参考时刻的局部偏移，不能当成长时间录制的常量补偿。离线程序不会用偏移量重写已映射的消息。

## 环境与入口

先完成根工作区构建并加载 ROS 与工作区环境，再启用独立离线工具环境：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/usr/bin/python3 -m venv --system-site-packages .venv-calibration
.venv-calibration/bin/python -m pip install -r src/calibration/requirements-tools.txt
source .venv-calibration/bin/activate
ros2 run calibration joint_calibrate.py --help
docker image inspect rm-nav-kalibr:local
```

系统包提供 NumPy、OpenCV、PyYAML；`rosbags` 固定版本安装到 venv。Kalibr 镜像必须已经存在，求解时解析为不可变 image ID，以 `--network none` 运行，并记录容器内实际 Kalibr commit。不会隐式下载镜像。实际环境说明见 [environment.md](environment.md)。

## 采集拟合与留出的真实数据

棋盘为 **8 列 × 11 行内角点，9 × 12 方格，边长 0.025 m**。方格边长须实测，图像识别无法验证毫米尺寸。相机与飞控必须刚性固定；拟合和留出数据必须来自同一 MAVLink 采集版本及固定话题/采样语义。当前阶段的采集诊断和实际录包分别验证 IMU ≥140 Hz，配置请求或 ACK 不等于达到频率要求；噪声依据采用 ADR 0005 的阶段例外，时间同步、相机及留出质量门槛保留。

```bash
# 双目内参：移动棋盘，覆盖图像边角、距离、倾斜角。
ros2 run tools spr record --profile mavlink_calibration --mode stereo --duration 180 --show-ui

# 联合：棋盘固定，移动整个相机/飞控组件，充分激励三轴旋转和平移。
ros2 run tools spr record --profile mavlink_calibration --mode joint --duration 180 --show-ui
```

另录一段独立的 `joint` 会话作留出集，目标同样保持固定。避开棋盘对称姿态翻转和运动模糊。保存原始 rosbag2；PNG/CSV 是派生数据。不要把同一录包复制、改 metadata 或重新封装后当作留出数据。

`joint`/`imu` 模式保留各自频率与连续性检查。既有 `allan` 工具仍保留严格测量语义：至少 100 个实际时间戳、最大间隔 ≤1.5 倍中位周期、p99 绝对抖动 ≤10% 中位周期，不补点或重采样。该可选测量工具的通过条件不作为当前工程先验路径的前置条件；当前不追加任何噪声或采样测试。

静止 **≥1 h** 是 DDS 原路径与自愿完整 Allan 测量的本地时长下限，不是当前 MAVLink 的前置条件，也不是相机—IMU 运动录制时长。若未来选择测量路径，仍须检查真实 Allan 曲线的白噪声与随机游走区域，不能把工程先验标成测量结果。

正式采集要求可复现且已提交的 Git 基线。`git.effective_clean` 只允许一条例外：根仓库其余部分干净，OpenVINS gitlink/HEAD 固定在约定基线，实际 diff、两个补丁和六个修改文件的哈希全部与 [兼容补丁](../docker/openvins/README.md) 一致；任何其他修改、暂存或未跟踪文件仍然拒绝。manifest 保留原始 Git status 和 `approved_patches`，不会隐去补丁状态。源码调试可显式使用 `--experimental`；旧数据不会在提交后被追认为正式采集。MAVLink 流程要求实际 `mavlink_timesync` 诊断；原 DDS 流程仍要求自己的 DDS 同步证据，不能靠切换 profile 或重命名话题补齐。

每个 run 的 manifest 包含实际完成录包的文件哈希。相机参数在启动后及结束前通过 `/camera/get_parameters` 读回，并与 rig、实际驱动包版本比对；配置请求本身不等于观测证明。飞控记录实际 `timesync_snapshot`、采集程序 commit、频率、样本语义和时钟映射不确定度。MAVLink 采集器已将 header 映射到主机；离线转换保留 header，不再次添加 `T_host_px4`。DDS 时间约定仍见 [time.md](time.md)。

## 离线求解

下列路径是操作示例中的会话位置，须替换为实际输出目录。每次求解使用新的输出目录。

```bash
ros2 run calibration joint_calibrate.py inspect --profile mavlink_calibration --bag runs/joint_001/bag

ros2 run calibration calibrate \
  --session runs/stereo_001/images/actual_session \
  --output runs/stereo_001/intrinsics

ros2 run calibration joint_calibrate.py solve \
  --profile mavlink_calibration \
  --bag runs/joint_001/bag \
  --camchain runs/stereo_001/intrinsics/camchain.yaml \
  --imu runs/noise_prior_001/imu.yaml \
  --noise-prior runs/noise_prior_001/prior.yaml \
  --session-manifest runs/joint_001/manifest.yaml \
  --output runs/joint_001/solve
```

`inspect` 检查每条消息的类型、frame、时间戳、载荷及流模式。转换同时比较全部消息的记录时间、header 时间、像素/IMU/协方差哈希；逆序、重复时间戳不会被悄悄改写或排序。识别和转换不需要噪声文件，也不改变真实测量。示例 `solve` 需要事先明确提供的先验文件，不会自动填数值。

`solve` 产生 **CANDIDATE**，包含 Kalibr PDF、标准 YAML、完整联合协方差/相关性、输入血缘、工具版本和容器命令。尚未证明硬件标定有效。源录包、所选噪声依据和 bridge 版本的血缘不完整时，候选结果保持不可发布为本阶段已验收标定。

### 显式工程噪声先验

`--noise-prior` 仅适用于 `mavlink_calibration`，与 `--allan-report` 互斥，仍必须提供 `--imu`。先验 YAML 包含 `schema_version: 1`、`basis: engineering_prior`、`calibration_profile: mavlink_calibration`、`noise_model_measured: false`、`original_allan_requirement_satisfied: false`。其 `imu` 映射必须与 `--imu` 文件完全相同，包含实际话题 `/spr/calibration/imu`、已记录运动包的 `update_rate` 和四个正值参数。

`parameter_sources` 为四个参数各记录 `kind`、`reference`、`rationale`、`unit`；数值只在 `imu` 中记录，不另设来源数值。数据手册参数标为 `datasheet_prior`，无法从手册得到的随机游走权重须标为 `engineering_assumption`，写明选择理由；两者均非实测噪声。

| 参数 | Kalibr 连续时间单位 |
| --- | --- |
| `accelerometer_noise_density` | `m/s^2/sqrt(Hz)` |
| `gyroscope_noise_density` | `rad/s/sqrt(Hz)` |
| `accelerometer_random_walk` | `m/s^3/sqrt(Hz)` |
| `gyroscope_random_walk` | `rad/s^2/sqrt(Hz)` |

白噪声密度按 Kalibr 的 `sigma_discrete = density * sqrt(update_rate)` 使用。数据手册的 °/s、µg 单位必须显式换算；PX4 EKF 过程噪声调参值不能未经模型与单位核对直接改名填入。原先验文件及 SHA-256 随候选/导出归档并重新验证，产物必须始终带 `noise_basis: engineering_prior`、`noise_model_measured: false`、`original_allan_requirement_satisfied: false`。先验影响权重和局部协方差，不能把给定权重产生的协方差当成真实噪声已验证。

### 可选测量与未来 DDS 路径

保留的 `allan` 命令要求真实静止 ≥3600 秒、严格采样间隔和可支持 −1/2、+1/2 斜率的拟合区域；不足时失败，不填默认值。未来 DDS 路径仍要求匹配的 `--imu` 与 `--allan-report` 及其采集血缘。DDS 当前冻结；这里保留离线接口说明，不授权新硬件采集、噪声测试或 DDS 切换。

以下仅说明未来恢复 DDS 路径后如何分析已停止的真实静止归档，不是当前待办：

```bash
ros2 run calibration joint_calibrate.py allan \
  --profile dds_runtime --bag runs/dds_static_001/bag \
  --session-manifest runs/dds_static_001/manifest.yaml \
  --output runs/dds_static_001/allan
```

## 留出集与 OpenVINS

```bash
ros2 run calibration joint_calibrate.py validate \
  --profile mavlink_calibration \
  --candidate runs/joint_001/solve --bag runs/joint_heldout_001/bag \
  --session-manifest runs/joint_heldout_001/manifest.yaml

ros2 run calibration joint_calibrate.py prepare-openvins \
  --profile mavlink_calibration \
  --candidate runs/joint_001/solve \
  --estimator-template src/open_vins/config/euroc_mav/estimator_config.yaml
```

留出数据必须与训练记录的时间段和选定话题载荷独立，frame/profile 必须一致，使用记录的双目配对容差，配对率 ≥95%。正式验收还要通过独立采集 manifest 核对 bridge 版本、实际相机、曝光、驱动及录包哈希。检查 ≥30 对检测结果、固定双目外参的重投影误差、三轴陀螺仪与视觉转速匹配、加速度/重力/刚性杆臂一致性。时间符号独立检查遵循 **`t_imu = t_cam + timeshift_cam_imu`**。每次验证保留独立 `validation/attempt_*/`；数值通过后写入带哈希的 `selected.yaml`，失败尝试不覆盖历史；未证明采集血缘的结果仍不能发布。

当前实现的数值门槛：留出 RMS ≤0.5 px、单图 RMS ≤1 px；时间偏移与留出搜索最优值相差 ≤5 ms、陀螺仪向量 RMS ≤0.15 rad/s；加速度向量 RMS ≤0.7 m/s²、重力模长 9.3–10.3 m/s²；局部 Kalibr 标准差分别 ≤2°、2 cm、5 ms。报告暴露不确定度和相关性；局部线性协方差不包含系统误差，这些检查也不能代替飞行振动/温漂验证。

`prepare-openvins` 只在候选目录生成回放输入：双目 `max_cameras: 2`、`use_stereo: true`、`calib_cam_timeoffset: true`、实际 IR 话题和本 profile 的 `/spr/calibration/imu`。模板仅提供估计器调参，所有测得内外参、噪声、时间偏移来自候选结果。`T_cam_imu` 映射 IMU→相机；不改变 Kalibr 符号。cam0/cam1 时间差超过 1 ms 时拒绝单一双目时间偏移导出。OpenCV `%YAML:1.0` 仅在最终 OpenVINS 文件中产生。

## 实际回放证据与发布

本工具不伪造 OpenVINS 回放通过记录。实际回放/evaluator 应生成 `replay_metrics.yaml`，包含：

- `status: PASSED`、实际 `run_id`、`diverged: false`；
- MAVLink 流程还须记录 `calibration_profile: mavlink_calibration` 及上面的四个阶段/范围字段，与候选及实际回放话题一致；
- `config_sha256`，与候选 `openvins_export.yaml` 完全一致；实际停止的 `bag_path` 及 `bag_sha256`，不得是训练 bag；
- `reference.source` 和 `reference.accuracy`；有真值时测量 `metrics.ate_rmse_m`，门槛 ≤0.3 m；
- `artifacts` 中的 `estimated_trajectory`、`replay_log`、`replay_command`、`evaluation_report`，每项均有真实 `path` 和 `sha256`。命令文件采用 YAML `argv` 列表；有真值还须 `reference_trajectory`；
- 没有真值时 `reference.source: none`，禁止 ATE/RPE。须测量 `closed_loop_drift_m`、`repeatability_m`，提供经评审的 `proxy_limits`、`proxy_acceptance: true` 和 `acceptance_adr` 实际文件/哈希。**当前尚未批准代理指标门槛；没有对应决策不能据此正式发布。**

```bash
ros2 run calibration joint_calibrate.py export \
  --profile mavlink_calibration \
  --candidate runs/joint_001/solve \
  --replay-evidence runs/replay_001/replay_metrics.yaml \
  --destination calibrations
```

发布会重新核对采集、所选噪声依据（工程先验或完整实测 Allan）、留出集、协方差、配置及回放文件哈希，并复制小型回放证据。本测试阶段验收产物使用不可覆盖的 `calibrations/mavlink_calibration_<UTC>_<commit>_<inputhash>/`，始终保留 `mavlink_140hz_test_only` 使用范围；工程先验产物同时保留未测量和原 Allan 要求未满足的标记，不代表 DDS 原目标通过。原始大录包保留在 `runs/`。缺失实际数据、未通过门槛或血缘不符时退出失败，不发布占位标定。

发布时还会实际读取回放 bag 的两路图像和所选 profile 的 IMU，核对消息类型、frame、图像 profile 与实测 IMU 频率（本 MAVLink 阶段 ≥140 Hz），并与训练流比较消息/载荷指纹及 header 时间范围。重新封装训练 bag、更改容器元数据或只平移原消息时间戳，不能作为独立回放数据。检查得到的 `selected_topic_inspection` 会写入归档的回放证据；这些流检查不单独证明物理 IMU 来源或实际 VIO 精度，仍须真实回放及上述采集/评估证据。

## 已有真实数据的转换验证

`runs/offline_conversion_20261029T115739Z/` 保存既有 2026-10-28 录包的转换证据：左右图像各 1821 条，IMU 6078 条，总计 9720 条，转换前后全部语义载荷/时间/数量一致，实际 Noetic `rosbag info` 可读。该旧包的 IMU 来自 MAVROS，仅用于格式回归；约 60.8 秒的数据不足 Allan 门槛，也未证明棋盘或联合标定成功。报告仅归档小型证据，没有复制 1.4 GB 录包。

2026-10-30 使用 D435i 与飞控真实 MAVLink 通路完成两段运动采集，源码基线为 `81cf1a0`，相机为固定曝光的 rectified mono8、848×480@30：

| 用途 | 原始 run | 每路图像数 | IMU 数 / 实测平均频率 |
| --- | --- | --- | --- |
| 联合拟合，约 180 s | `runs/20261030T105433_ec6df7ad/` | 5421 | 29734 / 164.53 Hz |
| 独立留出，约 120 s | `runs/20261030T105859_cb4cfaad/` | 3622 | 19879 / 164.60 Hz |

各 run 的 `selected_topic_inspection.json` 和 `readiness_report.yaml` 保存消息、时间及约 1 Hz 抽样棋盘检查。训练抽样 181 对中 78 对、留出抽样 120 对中 59 对在左右图像均识别到全部 88 个内角点；这不是全部帧的检测率。留出的 `independence_report.yaml` 只证明录制独立性。

`runs/20261030T111102_8c748ce8/` 保存训练包的无损 ROS1 转换、相机/棋盘配置、原采集 manifest 副本，以及重新读取真实留出包的检查。`training_conversion.yaml` 比较全部选定消息的数量、记录/header 时间戳、像素和 IMU 载荷哈希；原始 ROS2 包保留。该目录仅是离线输入准备，没有噪声参数或联合标定结果。以后调用当前 `solve` 包装器时仍传入**原始 ROS2 bag**及其原 manifest：包装器自行转换，直接传预转换包会因采集输入哈希不同而失去原始血缘。

当前缺少实测噪声、Kalibr 联合求解、数值留出检验及 OpenVINS 回放证据，尚未达到联合标定验收。上述生成数据由 Git 忽略，其他开发机须取得原始归档；相机节点的已测启动环境见 [environment.md](environment.md)。

依据：[Kalibr YAML 约定](https://github.com/ethz-asl/kalibr/wiki/Yaml-formats)、[IMU 噪声模型](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)、[相机—IMU 标定](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)。
