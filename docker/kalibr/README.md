# 本地主机 Kalibr 链接修复

真实联合求解发现原镜像 `rm-nav-kalibr:local` 的 `libbsplines.so` 调用
`cholmod_solve` 等符号，却没有链接 CHOLMOD。它能读取 bag，但不能导入实际求解器。
补丁为固定 Kalibr commit `1f60227442d25e36365ef5f72cd80b9666d73467`
的 bsplines 目标补上直接依赖，重新链接已有数值实现。

同一真实录包还暴露上游协方差路径对偏置样条的通用 Jacobian 调用不受支持。
仓库适配器在收敛状态重建完整解析 Hessian，保留全部残差、偏置正则项和非零信息。
若存在行列系数严格全零、且不含任何待标定参数的完整无用边界变量块，
仅在协方差线性系统中排除这些块，并将原矩阵奇异及排除坐标明确写入报告。
其余干扰变量以零阻尼进行一次严格 CHOLMOD LL′ 分解，再解选定参数的单位列，
得到保留信息支撑上的 8×8 边缘协方差；不声称奇异原矩阵有逆。
弱非零信息、耦合零空间或剩余非正定系统不能通过这种处理获得协方差。
不会再优化、只逆标定子块、删除正则项或填补不可观方向。
此镜像构建 C++20 的小型桥接模块，开启 `-Wall -Wextra -Wpedantic`。
选定列求解同时避开上游选定逆实现的 `int(r*n+c)` 缓存键：实际 Hessian 为
73420 维时该键会溢出，不能将它用于完整协方差恢复。

```bash
DOCKER_BUILDKIT=0 docker build --network none --pull=false \
  -t spr-kalibr-cholmod:local docker/kalibr
docker image inspect spr-kalibr-cholmod:local --format '{{.Id}}'
```

Dockerfile 固定已存在的本地基镜像 ID，并验证源 commit；依赖原镜像中的构建缓存。
它不是干净机器上的完整容器构建方案。构建无网络，不安装包、不下载源码，
不修改宿主机或 OpenVINS 子模块。新镜像用独立标签保存，保留原镜像和失败日志。
构建中的全新 Python 进程实际导入 bsplines 与 IMU—相机求解器；另有十个纯矩阵
回归检查边缘化、参数顺序、精确零支撑诊断、弱/耦合/负方向处理及实际 73420 维稀疏矩阵的边界行为，
不生成传感器数据。分解强制采用 LL′，不将允许负枢轴的 LDL′ 成功状态当作完整 Hessian 正定证明。

联合求解显式选择 `--image spr-kalibr-cholmod:local`；工具继续将标签解析成
不可变 image ID，保存实际 Kalibr commit、命令和输入哈希。补丁不改变优化模型、
测量、时间戳或验收门槛；噪声先验仍须单独声明。候选同时归档完整的
`covariance_method.yaml`、求解前写入的 `covariance_structure.yaml` 与局部模型依赖说明。
详见 [操作手册](../../docs/joint-calibration.md)。
