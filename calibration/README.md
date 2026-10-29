# 已迁移到根工作区

源码与 ROS 包已从 `calibration/src/spr_calibration` 迁到
[`src/calibration`](../src/calibration/README.md)，包名为 `calibration`。

请在仓库根目录构建并使用 `ros2 run tools spr record ...`。
旧的 `build/`、`install/`、`log/` 不再使用，`data/` 中历史采集保留不变。
本目录通过 `COLCON_IGNORE` 排除，不能继续构建旧包或 source 旧 overlay。

参阅 [根 README](../README.md) 和 [标定说明](../src/calibration/README.md)。
