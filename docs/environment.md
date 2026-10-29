# 环境与构建

已观察主机：Ubuntu 24.04 x86_64 / ROS 2 Jazzy / 系统 Python 3.12；OpenCV 4.6，realsense-ros 4.58.4，驱动加载 librealsense 2.58.4。D435i SDK 序列号 044322070753、USB 描述符序列号 043323052893、固件 5.17.3.10；二者不同属已观察事实，均记录在 rig 中。

OpenVINS gitlink 固定 69488123ed9362dd44b6f28e7f4680abbff1442b；PX4 消息由 `dependencies.repos` 固定 release/1.16 的完整 SHA。不要改成浮动 master。`src/external` 为 vcstool 导入区，不提交其生成的嵌套仓库。

```bash
source /opt/ros/jazzy/setup.bash
git submodule update --init --recursive
vcs import . < dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install --executor sequential \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Humble 主机替换 setup 路径；ARM64/Humble 尚需实际 runner/目标板验证，不能用本机 Jazzy 构建代替。

离线工具使用隔离环境，避免覆盖 ROS Python：

```bash
/usr/bin/python3 -m venv --system-site-packages .venv-calibration
source .venv-calibration/bin/activate
pip install -r src/calibration/requirements-tools.txt
```

`ros2 run` 安装脚本的解释器是系统 Python。需要离线 venv 时，明确调用 `.venv-calibration/bin/python src/calibration/scripts/joint_calibrate.py ...`，并先 source ROS 与根 overlay，保证 ROS 消息可导入。不要把系统解释器的包缺失误判为相机故障。

联合求解读取本地 Kalibr Docker 镜像并解析到不可变镜像 ID；现有主机 `rm-nav-kalibr:local` 对应 Kalibr commit `1f60227442d25e36365ef5f72cd80b9666d73467`。新机器的可复现容器分发、ARM64 和完整 CI 仍需补齐；目前不宣称干净机器全链路验收完成。

主机 Agent 的锁定版本、私有构建脚本和实际证据见 `docker/px4/`；该准备不会修改飞控协议或固件。
