# A0 接口实现状态

接口真源仍是 Project.md §§4–5；此页记录当前落地范围。

| 接口 | 当前实现 | 说明 |
| --- | --- | --- |
| IR1/IR2 Image、CameraInfo | RealSense 实机已发布 | Y8/mono8，848×480@30，可靠发布；见 rig |
| `/spr/imu` | bridge 程序和消息依赖已构建 | 必须先完成 FCU DDS 链路和频率配置，尚无本机实测 |
| `/spr/diagnostics` | bridge 同步诊断 | best_effort；bringup 读取并记录到 manifest |
| 自定义四类消息 | drone_interfaces 0.1.0 已生成 | 草案，未团队冻结 |
| VIO/state/offboard | 尚无项目运行节点 | 预留包不可当作飞行功能 |

RealSense DEFAULT QoS 的发布队列深度为 10；图像/录制订阅队列为 5。
共享契约的深度差异及时间语义修正记录在 ADR 0001。`calibration` 的采集订阅器当前使用 SensorDataQoS，可兼容可靠图像发布；正式 VIO 使用可靠订阅。

所有硬件进程必须由 drone_bringup 统一管理。规划/决策代码不得直接使用 px4_msgs、打开设备或发布 PX4 指令。坐标和时间不得在下游重复转换。
