# 时间约定

跨包消息使用主机 ROS 时间。RealSense 4.58.4 日志会报告实际 frame 时间域；HARDWARE_CLOCK 经驱动映射为 ROS 时间，仍需监测重置/倒退。不存在的 global_time_enabled 参数不能作为保证。

PX4 1.16 开启 UXRCE_DDS_SYNCT 后 DDS 序列化已完成主机时间映射。桥采用 `agent_synced`，验证 DDS TimesyncStatus 的协议、RTT、偏移稳定性和 ≤1 ms 预算，不重复加偏移。SensorCombined.timestamp 为积分终点，输出时间取陀螺积分中点，加速度插值到相同时刻。

`t_imu = t_cam + timeshift_cam_imu`。必须使用 Kalibr 返回的符号，并用独立留出数据检查。桥的重连和偏移跳变会结束连续段；不同连续段不能无记录地拼成一个正式联合标定会话。实测同步诊断写入 manifest。

真实 `/spr/imu` bag 回放直接使用已经映射的时间戳；live bridge 拒绝 use_sim_time。SITL 的独立 `/clock` 路径尚待 A1 实装，不得将 epoch 时间当仿真时间。
