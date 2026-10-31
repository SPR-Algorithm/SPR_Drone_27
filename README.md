# SPR_Drone_27

面向 RoboMaster 无人机的 ROS 2 视觉导航项目。当前处于 A0 阶段，目标是使用双目相机、PX4 飞控 IMU 和 OpenVINS 实现可用的实时 VIO。开发测试使用 D435i 左右红外相机，最终相机方案为 D430 模组与 D4 视觉处理板。

需求、阶段范围和功能验收以 [Project.md](Project.md) 为准，开发与验证遵循 [AGENTS.md](AGENTS.md)。

## 目录结构

ROS 2 功能包统一位于 `src/`，按实际职责组织：

```text
SPR_Drone_27/
├── src/
│   ├── drone_bringup/       # 设备启动、运行配置、预检与数据录制
│   ├── calibration/         # 双目标定、相机—IMU 标定与参数导出
│   ├── px4_bridge/          # PX4 DDS 消息接入、IMU 时间与坐标适配
│   ├── open_vins/           # 固定版本的上游算法与 ROS 2 接口（Git 子模块）
│   ├── drone_interfaces/    # 项目公共消息定义
│   ├── diagnostics/        # 运行记录、事件、指标与报告
│   ├── tools/              # spr 辅助命令入口
│   └── external/           # dependencies.repos 导入的外部依赖（Git 忽略）
├── docker/                 # 外部工具环境、兼容补丁与准备脚本
├── docs/                   # 环境、接口、标定与历史决策说明
├── dependencies.repos      # 外部依赖版本清单
└── build_workspace.sh      # 根工作空间构建入口
```

A0 优先复用 OpenVINS 已有 ROS 2 节点；只有实际需要语义转换或状态输出时，才增加定位适配代码。原有 `vio`、`state`、`flight_control`、`planning`、`decision`、`sim` 六个纯占位包已移除。后续定位、导航、规划、控制和仿真模块按实际功能需要建立，阶段目标见 `Project.md` 第 6 节。

包内脚本、配置和功能验证入口保留在所属包中；有明确跨模块复用需求时，再提取到共享目录。`build/`、`install/`、`log/`、`runs/` 等本地产物不纳入源代码结构。

## 构建与运行入口

开发主机使用 Ubuntu 24.04 / ROS 2 Jazzy；目标机载环境为 ROS 2 Humble，具体系统镜像待确认。依赖安装、固定版本和环境准备见 [环境与构建](docs/environment.md)。在已准备依赖的开发主机上，按需构建目标包及其依赖，例如：

```bash
source /opt/ros/jazzy/setup.bash
bash build_workspace.sh --packages-up-to calibration
source install/setup.bash
```

查看当前源码中的 ROS 2 包：

```bash
colcon list --base-paths src
```

- [设备启动与录制](src/drone_bringup/README.md)：当前采集入口及配置。
- [相机与联合标定](src/calibration/README.md)：功能入口、输入与输出。
- [PX4 DDS IMU 适配](src/px4_bridge/README.md)：已有通信适配实现。
- [文档索引](docs/index.md)：详细操作说明与历史记录。

## 当前实现与需求基线的差异

自主维护的 C++ 目标已明确使用 C++17、必需标准和 GCC/Clang 警告选项，根构建入口不再强制第三方语言标准或自动修改 OpenVINS。Python 包继续使用四空格缩进、ROS 参数和现有模块划分。OpenVINS 的 Jazzy 头文件兼容准备见 [兼容说明](docker/openvins/README.md)，仅构建本项目功能包无需应用该补丁。

采集入口仍默认使用 MAVLink 标定通路，联合求解仍依赖 Kalibr 容器；DDS 主链路、原生联合标定和厂商噪声参数流程尚需按需求推进。已有流程的频率、同步和标定质量检查条件不等同于新版 A0 的统一验收门槛；具体差异见各功能包 README。旧说明与新版 `Project.md` 冲突时，以需求基线为准。

目录与包的存在不代表相应功能已完成；实时 VIO、公共位姿与最小状态以及目标机载平台仍需实际实现和人工功能验收。
