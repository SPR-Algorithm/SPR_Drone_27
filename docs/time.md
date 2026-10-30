# 时间约定

跨包消息使用主机 ROS 时间。RealSense 4.58.4 日志会报告实际 frame 时间域；HARDWARE_CLOCK 经驱动映射为 ROS 时间，仍需监测重置/倒退。不存在的 global_time_enabled 参数不能作为保证。

## 当前 MAVLink 标定

HIGHRES_IMU 保留 PX4 传送的 `vehicle_imu.timestamp_sample`，不重构消息中不存在的积分中点。MAVLink TIMESYNC 匹配请求、测量单调时钟 RTT，以仿射模型映射到主机 ROS 时间：

`host_ns = reference_host_ns + clock_scale × (px4_ns − reference_px4_ns)`。

manifest 保存参考时刻、比例、局部偏移、拟合残差、RTT 与连续段。`T_host_px4_offset_s` 是参考时刻的局部偏移，不能作为全程常量再加到已映射的 header。软件拟合不确定度不能独自证明物理相机—IMU 同步精度。

`t_imu = t_cam + timeshift_cam_imu`。必须使用 Kalibr 返回的符号，并用独立留出数据检查。重连、主机跳时和无效时钟映射会结束连续段；不同连续段不能无记录地拼成一个正式联合标定会话。当前录包回放直接使用 `/spr/calibration/imu` 中已映射的 header，不重复补偿。

## 后续 DDS 迁移，当前冻结

PX4 1.16 开启 UXRCE_DDS_SYNCT 后 DDS 序列化已完成主机时间映射。已有桥采用 `agent_synced`，验证 DDS TimesyncStatus 的协议、RTT、偏移稳定性和 ≤1 ms 预算，不重复加偏移。SensorCombined.timestamp 为积分终点，输出时间取陀螺积分中点，加速度插值到相同时刻。这些采样语义与当前 HIGHRES_IMU 路径不同，后续迁移必须用真实 DDS 数据重新验证时间偏移和留出误差。

真实 `/spr/imu` bag 回放直接使用已经映射的时间戳；live bridge 拒绝 use_sim_time。SITL 的独立 `/clock` 路径尚待 A1 实装，不得将 epoch 时间当仿真时间。
