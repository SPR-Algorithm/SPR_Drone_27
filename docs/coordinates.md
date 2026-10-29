# 坐标约定

遵循 `T_A_B`：把 B 中的点变换到 A 中。米、弧度、秒为基本单位。

- 相机 optical：x 右、y 下、z 前。保持驱动真实 optical frame。
- `/spr/imu`：`imu_link`，FLU；PX4 原始 FRD 仅在 `px4_bridge` 内转换，矩阵 diag(1,-1,-1)。
- `T_cn_cnm1`：cam0 到 cam1；`T_cam_imu`：IMU 到相机；其逆为 `T_imu_cam`。
- 无联合标定时不得生成单位矩阵假装已测外参。
- 后续 VIO 为 `vio_odom`；`state` 负责 `world/odom/base_link` 动态 TF。A0 采集不发布伪造的世界变换，RealSense 驱动 TF 关闭。

联合采集须刚性固定相机和飞控、固定标定板，再整体移动组件；移动板并保持相机/IMU 不动无法激励联合标定。
