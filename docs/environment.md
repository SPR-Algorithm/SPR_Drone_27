# 环境与构建

已观察主机：Ubuntu 24.04 x86_64 / ROS 2 Jazzy / 系统 Python 3.12；OpenCV 4.6，realsense-ros 4.58.4，驱动加载 librealsense 2.58.4。D435i SDK 序列号 044322070753、USB 描述符序列号 043323052893、固件 5.17.3.10；二者不同属已观察事实，均记录在 rig 中。

OpenVINS gitlink 固定 69488123ed9362dd44b6f28e7f4680abbff1442b；PX4 消息由 `dependencies.repos` 固定 release/1.16 的完整 SHA。不要改成浮动 master。`src/external` 为 vcstool 导入区，不提交其生成的嵌套仓库。

```bash
source /opt/ros/jazzy/setup.bash
git submodule update --init --recursive
vcs import . < dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
CMAKE_BUILD_PARALLEL_LEVEL=2 bash build_workspace.sh
source install/setup.bash
```

`build_workspace.sh` 应用固定的 OpenVINS 兼容补丁并注入 C++20 target policy，包括 ROS 自动生成接口；不要用仅设置 Python 路径的普通构建命令代替。Humble 主机替换 setup 路径；ARM64/Humble 尚需实际 runner/目标板验证，不能用本机 Jazzy 构建代替。

## 当前 MAVLink 采集依赖

当前联合标定不启动 DDS。为 MAVLink 采集器安装单独的 Python 依赖：

```bash
/usr/bin/python3 -m venv --system-site-packages .venv-sensors
.venv-sensors/bin/python -m pip install -r src/drone_bringup/requirements-mavlink.txt
export SPR_MAVLINK_PYTHONPATH="$(.venv-sensors/bin/python -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
ros2 run tools spr check-rig --profile mavlink_calibration
```

采集子进程固定使用 `/usr/bin/python3`，通过上述路径读取固定版本的附加依赖；虚拟环境被 Git 忽略。`check-rig` 只校验配置，连接相机/飞控后的真实频率和时间质量仍需实测。

## 离线标定依赖

离线工具使用隔离环境，避免覆盖 ROS Python：

```bash
/usr/bin/python3 -m venv --system-site-packages .venv-calibration
source .venv-calibration/bin/activate
pip install -r src/calibration/requirements-tools.txt
```

离线 venv 可明确调用 `.venv-calibration/bin/python src/calibration/scripts/joint_calibrate.py ... --profile mavlink_calibration`；先 source ROS 与根 overlay，保证 ROS 消息可导入。不要把系统解释器的包缺失误判为相机故障。

联合求解读取本地 Kalibr Docker 镜像并解析到不可变镜像 ID；现有主机 `rm-nav-kalibr:local` 对应 Kalibr commit `1f60227442d25e36365ef5f72cd80b9666d73467`。新机器的可复现容器分发、ARM64 和完整 CI 仍需补齐；目前不宣称干净机器全链路验收完成。

DDS 主机 Agent 的锁定版本及准备材料位于 `docker/px4/`，当前阶段冻结，不是 MAVLink 标定的安装或运行前置条件。
