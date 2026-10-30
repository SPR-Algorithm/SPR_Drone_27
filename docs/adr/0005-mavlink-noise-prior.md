# ADR 0005: 当前 MAVLink 测试阶段显式采用工程噪声先验

日期：2026-10-30。状态：依据用户明确指令实施。负责人：SilverWolf。

## 决定与范围

用户先暂缓 IMU 噪声/采样测试，随后明确“不用完整 allan 分析了”。因此当前
`mavlink_calibration` 不再以短时噪声测试或 ≥1 h 静止录制作为联合求解和本阶段
验收的前置条件。允许用户显式提供可追溯的工程噪声先验，不生成或补造传感器数据，
不自动填默认噪声。此决定覆盖 ADR 0002、ADR 0004 中当前阶段仍必须完成 Allan 的旧约定。

DDS 源码、配置、固件及切换流程继续冻结，原 DDS 完整 Allan 要求不变。
当前候选、留出和回放仍须统一选择 `--profile mavlink_calibration`，实测 IMU ≥140 Hz，
使用范围保留 `mavlink_140hz_test_only` 和 `original_dds_200hz_requirement_satisfied: false`。
这不取消阶段目标，也不表示联合求解、留出或 OpenVINS 已完成。

## 显式先验接口与血缘

`solve` 仍需要 `--imu <imu.yaml>`；当前例外另须 `--noise-prior <prior.yaml>`，与
`--allan-report` 互斥。先验文件必须有：

- `schema_version: 1`、`basis: engineering_prior`、`calibration_profile: mavlink_calibration`；
- `noise_model_measured: false`、`original_allan_requirement_satisfied: false`；
- `imu`：与 `--imu` 文件完全相同，含 `/spr/calibration/imu` 的 `rostopic`、
  已记录运动包的 `update_rate` 和四个正值噪声参数；
- `parameter_sources`：每个参数的 `kind`、`reference`、`rationale`、`unit`，数值只在 `imu` 中保存。

| 参数 | 单位 | 来源种类与说明 |
| --- | --- | --- |
| `accelerometer_noise_density` | `m/s^2/sqrt(Hz)` | 手册依据标为 `datasheet_prior`，记录原值、换算与安全系数的理由 |
| `gyroscope_noise_density` | `rad/s/sqrt(Hz)` | 手册依据标为 `datasheet_prior`，显式将 °/s 换成 rad/s |
| `accelerometer_random_walk` | `m/s^3/sqrt(Hz)` | 没有对应测量时标为 `engineering_assumption`，给出独立选择理由 |
| `gyroscope_random_walk` | `rad/s^2/sqrt(Hz)` | 没有对应测量时标为 `engineering_assumption`，给出独立选择理由 |

先验原文件与 SHA-256 随候选和导出归档并重新核对，修改后不能沿用旧血缘。
产物必须同时记录 `noise_basis: engineering_prior`、`noise_model_measured: false`、
`original_allan_requirement_satisfied: false`，禁止将先验写成实测 Allan 结果。
本 ADR 不内置具体噪声值，也不创建可被误当测量的模板文件。

## 已有硬件依据与限制

2026-10-29 的实际 NSH 记录
`runs/mavlink_source_inspection_20261029T131400Z/nsh_transcript.txt` 表明当时所选
加速度计 ID 为 `6946834`（Type `0x6a`，SPI2），陀螺仪 ID 为 `6684690`
（Type `0x66`，SPI2）；次 IMU 的两者 ID 均为 `3604506`（Type `0x37`，SPI3）。
[PX4 v1.16.0 类型定义](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/drivers/drv_sensor.h)
将这三种类型分别映射为 BMI088 accelerometer、BMI088 gyroscope、BMI270。
[板卡厂商](https://micoair.com/flightcontroller_micoair743v2/)亦说明装有 BMI088 与 BMI270。
这证明历史查询时的选择；当前录制的 `HIGHRES_IMU` 没有这些设备 ID，不能据此声称
已逐帧验证当前选择，也不能忽略 PX4 的滤波、区间平均和偏置补偿。

[Bosch BMI088 产品资料](https://www.bosch-sensortec.com/en/products/motion-sensors/imus/bmi088/)
可作为手册先验的引用；典型白噪声指标不等于这块板的实测噪声，亦不能推导零偏随机游走。
既有 NSH 参数快照
`runs/fcu_inspection_20261029T123130Z/followup/nsh_transcript.txt` 中的
`EKF2_ACC_NOISE=0.3500`、`EKF2_GYR_NOISE=0.0150`、`EKF2_ACC_B_NOISE=0.0030`、
`EKF2_GYR_B_NOISE=0.0010` 是观测到的板载 EKF 调参值，不能未经模型/单位核对直接
改名成 Kalibr 连续时间噪声。Kalibr 将白噪声密度乘以 `sqrt(update_rate)` 得到离散标准差；
不同模型与单位的数值不能仅因字段名称接近而等同。

## 验收与历史数据

独立实际留出的重投影、时间偏移符号、陀螺仪/加速度残差、联合协方差、
OpenVINS 配置及真实回放门槛全部保留。工程先验影响局部协方差，报告必须披露这种
模型依赖，数值通过仍不能声明物理噪声、振动、温漂或飞行适用性已验证。
完整 Allan 工具保留用于自愿测量和未来 DDS 路径；当前不要求追加噪声采集或测试。

原始运动训练 `runs/20261030T105433_ec6df7ad` 与独立留出
`runs/20261030T105859_cb4cfaad` 的采集基线仍为 `81cf1a0`，保留原 manifest、
原始 bag、哈希与初步 readiness/independence 报告，不追改历史状态或把录制成功当标定成功。
后续实现、先验选择及求解产物另记自身代码基线、来源与不可变哈希。
