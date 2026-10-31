# 环境与构建

已观察主机：Ubuntu 24.04 x86_64 / ROS 2 Jazzy / 系统 Python 3.12；OpenCV 4.6，realsense-ros 4.58.4，驱动加载 librealsense 2.58.4。D435i SDK 序列号 044322070753、USB 描述符序列号 043323052893、固件 5.17.3.10；二者不同属已观察事实，均记录在 rig 中。

OpenVINS gitlink 固定 69488123ed9362dd44b6f28e7f4680abbff1442b；PX4 消息由 `dependencies.repos` 固定 release/1.16 的完整 SHA。不要改成浮动 master。`src/external` 为 vcstool 导入区，不提交其生成的嵌套仓库。

```bash
source /opt/ros/jazzy/setup.bash
git submodule update --init --recursive
vcs import . < dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
CMAKE_BUILD_PARALLEL_LEVEL=2 bash build_workspace.sh --packages-up-to tools
source install/setup.bash
```

`build_workspace.sh` 保留各包自己的构建要求，不自动应用第三方补丁。自主维护的 C++ 目标在各包 CMake 中明确要求 C++17 和 GCC/Clang 警告；OpenVINS 与 `px4_msgs` 保留上游构建要求。仅构建目标功能时优先传入 `--packages-up-to <包名>`。

固定 OpenVINS 版本在 Jazzy 上需要头文件兼容准备，构建它之前按 [兼容说明](../docker/openvins/README.md) 显式处理；不要仅为统一语言标准修改第三方。曾使用 C++20 全局配置的旧构建目录应另行保留，本轮使用独立 build/install/log 目录，避免继承缓存中的旧编译选项。Humble 主机替换 setup 路径；ARM64/Humble 尚需实际目标板验证，不能用本机 Jazzy 构建代替。

## 当前 MAVLink 采集依赖

现有联合标定采集入口使用 USB MAVLink；它是保留的历史流程，当前 `Project.md` 的 A0 主链路已改为 uXRCE-DDS。下列步骤用于现有 MAVLink 采集器的独立 Python 依赖：

```bash
/usr/bin/python3 -m venv --system-site-packages .venv-sensors
.venv-sensors/bin/python -m pip install -r src/drone_bringup/requirements-mavlink.txt
export SPR_MAVLINK_PYTHONPATH="$(.venv-sensors/bin/python -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
ros2 run tools spr check-rig --profile mavlink_calibration
```

采集子进程固定使用 `/usr/bin/python3`，通过上述路径读取固定版本的附加依赖；虚拟环境被 Git 忽略。`check-rig` 只校验配置，连接相机/飞控后的真实频率和时间质量仍需实测。

### 已实测的主机 CycloneDDS 临时环境

根提交 `81cf1a08d443fef5f12951be3af306ca67020b46` 下，真实 MAVLink 联合采集 [180 s 训练会话](../runs/20261030T105433_ec6df7ad/report.html) 和 [120 s 独立会话](../runs/20261030T105859_cb4cfaad/report.html) 均使用此环境。训练会话保存了相机、IMU 发布器及 recorder 的实际进程环境 [observed_rmw_environment.json](../runs/20261030T105433_ec6df7ad/observed_rmw_environment.json)，以及五个官方 `amd64.deb` 的版本和 SHA256 [rmw_packages.json](../runs/20261030T105433_ec6df7ad/rmw_packages.json)。这些运行数据不在 Git 中，复现时需另行保留。

以下命令假定已配置 ROS 2 Jazzy 官方 APT 源；只下载并局部解包，不要求全局安装。固定版本可能已从滚动仓库移除；此时应恢复与上述 SHA256 一致的缓存包，不能默默改用新版本。

```bash
spr_cyclone_dir=/tmp/spr-cyclonedds
mkdir -p "$spr_cyclone_dir/debs" "$spr_cyclone_dir/root"
(
  cd "$spr_cyclone_dir/debs" || exit 1
  apt download \
    ros-jazzy-cyclonedds=0.10.5-1noble.20260225.142613 \
    ros-jazzy-iceoryx-binding-c=2.0.6-1noble.20260225.140829 \
    ros-jazzy-iceoryx-hoofs=2.0.6-1noble.20260225.055330 \
    ros-jazzy-iceoryx-posh=2.0.6-1noble.20260225.135341 \
    ros-jazzy-rmw-cyclonedds-cpp=2.2.4-1noble.20260911.095700 || exit 1
  # 此专用目录只保留上面五个固定版本的 amd64.deb。
  for spr_deb in ./*_amd64.deb; do
    dpkg-deb --extract "$spr_deb" "$spr_cyclone_dir/root" || exit 1
  done
)
```

在仓库根目录的新 Bash 中，先完成前述工作区构建及 `.venv-sensors` 准备，再设置以下实测启动环境；保留 ROS 原有 `PYTHONPATH`：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
spr_cyclone_prefix=/tmp/spr-cyclonedds/root/opt/ros/jazzy
export AMENT_PREFIX_PATH="$spr_cyclone_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="$spr_cyclone_prefix/lib/x86_64-linux-gnu:$spr_cyclone_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export SPR_MAVLINK_PYTHONPATH="$(.venv-sensors/bin/python -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
```

两个库目录都须加入；从同一 shell 执行 [MAVLink 采集命令](joint-calibration.md#采集拟合与留出的真实数据)，让所有子进程继承相同环境。关闭该 shell 并移除临时解包目录即可撤销。此选择只作用于主机 ROS 2 通信，不改变飞控通信配置。早先 FastDDS 录包缺口的根因尚未证实；上述采集成功也不代表联合标定验收通过。

## 离线标定依赖

离线工具使用隔离环境，避免覆盖 ROS Python：

```bash
/usr/bin/python3 -m venv --system-site-packages .venv-calibration
source .venv-calibration/bin/activate
pip install -r src/calibration/requirements-tools.txt
```

离线 venv 可明确调用 `.venv-calibration/bin/python src/calibration/scripts/joint_calibrate.py ... --profile mavlink_calibration`；先 source ROS 与根 overlay，保证 ROS 消息可导入。不要把系统解释器的包缺失误判为相机故障。

独立的 ROS 1 转换验证属于准备工作；后续 `solve --bag` 仍传原始 ROS 2 bag。转换包的文件哈希不同，不能与原采集 manifest 直接配对使用。

联合求解读取本地 Kalibr Docker 镜像并解析到不可变镜像 ID；现有基镜像 `rm-nav-kalibr:local` 对应 Kalibr commit `1f60227442d25e36365ef5f72cd80b9666d73467`。实际求解发现该镜像的 bsplines 缺少 CHOLMOD 直接链接，须先按 [容器修复说明](../docker/kalibr/README.md) 离线构建 `spr-kalibr-cholmod:local`；这是当前求解默认镜像，原镜像保留。新机器的完整容器构建/分发、ARM64 和 CI 仍需补齐；目前不宣称干净机器全链路验收完成。

DDS 主机 Agent 的锁定版本及准备材料位于 `docker/px4/`，不属于现有 MAVLink 采集的前置条件；新版 A0 的 DDS 通路需单独准备与验证。
