# 文档索引

- [需求与架构契约](../Project.md)：范围、接口、阶段与验收。
- [环境与构建](environment.md)：版本、依赖和启动环境。
- [坐标约定](coordinates.md)、[时间约定](time.md)。
- [双目标定](../src/calibration/README.md)、[设备入口](../src/drone_bringup/README.md)。
- [MAVLink 联合标定流程](joint-calibration.md)、[当前阶段决策](adr/0002-mavlink-calibration-phase.md)。
- [PX4 DDS 准备](../docker/px4/README.md)：当前硬件阻塞及构建证据。
- [ADR 0001](adr/0001-a0-workspace-and-time.md)：迁移和 PX4 时间语义修正。
- [接口草案](../src/drone_interfaces/README.md)：未冻结，需团队评审。

真实数据在 `runs/<run_id>/`。先看 `report.html`、`manifest.yaml` 和首个 ERROR；再查看对应组件日志。标定报告位于每次求解的新输出目录。FAILED 会话保留用于诊断，不能当作验收通过。

本阶段没有正式飞行入口。`flight_control`、`state` 和 SITL 包只是架构边界；其 README 明确尚未实现的运行能力。
