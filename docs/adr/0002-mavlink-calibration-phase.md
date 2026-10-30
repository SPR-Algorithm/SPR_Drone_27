# ADR 0002: MAVLink 140 Hz 标定测试阶段与 DDS 冻结

日期：2026-10-29。状态：依据用户明确指令实施。负责人：SilverWolf。

用户在本轮实际开发中依次指定“先使用 mavlink 进行 imu 和相机的标定”、
“dds 的部分冻结不去修改”和“达不到先用 140hz 测试”。这些指令覆盖
Project.md 中当前阶段只能使用 DDS、最低 200 Hz 的实施路径；原运行时验收目标保留。

## 当前路径

- `spr record --profile mavlink_calibration` 使用独立 rig/schema，当前 CLI 默认选择此 profile。
- 设备仍为原 USB MAVLink，不启动 MicroXRCEAgent，不执行 DDS SD 暂存脚本，不写持久参数或固件。
- `calibration_mavlink_imu` 是串口唯一所有者，发布 `/spr/calibration/imu`，FLU、`imu_link`、主机 ROS 时间。
- 读取真实 `HIGHRES_IMU`，用匹配请求和往返时间的 MAVLink TIMESYNC 拟合仿射时钟；不得虚构积分中点或物理同步精度。
- 当前测试门限为实测 ≥140 Hz，诊断与 bag 独立检查；请求频率不是实测频率。
- 产物明确写入 `mavlink_140hz_test_only`，不得标成 DDS 运行通路验收。
- `joint` 采集原始双目、camera_info、IMU 与诊断；双目静态采集的 30 对/80%覆盖门限不用于结束运动录包。离线联合求解独立检查棋盘检测、三轴激励、留出误差和时间符号。

## 保留的质量要求

相机/飞控刚性安装，固定棋盘后整体移动组件。噪声必须来自真实静止数据，
不足一小时不作为正式 Allan 结果，不能填默认噪声。原始 bag、实际时间同步诊断、
驱动参数读回、输入和代码哈希必须保留。主机跳时、逆序、断流必须结束连续段并报错。

DDS 源码、配置、固件补丁及暂存脚本保持现状；重新启用必须由用户另行指示。
此前已上传的一次性 SD 脚本没有执行，USB 没有因该脚本切换。MAVLink 结束时恢复采集前的瞬时消息间隔。

## 真实证据

`runs/mavlink_ros_validation_140_20261029T131846Z/summary.json`：22 秒收到
3244 条实际 ROS IMU，159.28 Hz、严格递增时间戳、平均重力模长 9.80693 m/s²。
软件拟合不确定度约 219 µs；这不代表测得的物理相机—IMU 同步精度。

## 2026-10-30 阶段安排重申

用户再次明确：当前先用 MAVLink 完成相机—IMU 联合标定，后续再改 DDS。
因此当前完整流程以 MAVLink 静止、运动、留出数据和回放验证为准，DDS 不作为当前标定的前置条件。
采集入口默认 MAVLink；离线命令显式使用 `--profile mavlink_calibration`，保持数据来源可追溯。

DDS 迁移属于后续独立阶段：核对同一物理 IMU、frame/轴向、单位、采样时刻、频率与时间映射，
再用 DDS 留出数据检查外参与时间偏移。现有 MAVLink 标定结果保留来源标记，不能直接改标签当作 DDS 验收通过。
