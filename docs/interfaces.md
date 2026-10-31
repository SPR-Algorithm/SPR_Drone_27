# A0 接口实现状态

接口需求以 [Project.md](../Project.md) 第 3–4 节为准。下表描述现有实现；旧 MAVLink 阶段记录在 [ADR 0002](adr/0002-mavlink-calibration-phase.md)，其阶段安排已被新需求基线替代，DDS 主链路仍待迁移。

| 接口 | 当前实现 | 说明 |
| --- | --- | --- |
| IR1/IR2 Image、CameraInfo | RealSense 实机已发布 | Y8/mono8，848×480@30，可靠发布；见 rig |
| `/spr/calibration/imu` | 当前 MAVLink 标定输入 | HIGHRES_IMU、FLU/imu_link、主机 ROS 时间；当前测试门限实测 ≥140 Hz |
| `/spr/imu` | 后续 DDS 通路，冻结 | 不由当前 MAVLink profile 发布；尚未完成 DDS 实机验收 |
| `/spr/diagnostics` | 当前 MAVLink 时间/频率诊断 | best_effort；bringup 读取实际拟合、RTT、频率和连续段，保存到 manifest |
| 自定义四类消息 | drone_interfaces 0.1.0 已生成 | 草案，未团队冻结 |
| VIO/状态聚合/offboard | 尚无项目运行节点 | 相关功能尚未实现 |

RealSense DEFAULT QoS 的发布队列深度为 10；图像/录制订阅队列为 5。
共享契约的深度差异及时间语义修正记录在 ADR 0001。`calibration` 的采集订阅器当前使用 SensorDataQoS，可兼容可靠图像发布；正式 VIO 使用可靠订阅。

所有硬件进程必须由 drone_bringup 统一管理。规划/决策代码不得直接使用 px4_msgs、打开设备或发布 PX4 指令。坐标和时间不得在下游重复转换。

迁移 DDS 时须重新验证消息采样时刻、frame、频率、协方差和时间映射，再用新通路留出数据验证标定；修改话题名本身不能证明两条路径等价。
