# UAV-Simulator-for-IoV Docker 镜像配置文档

本文档说明如何在 **Ubuntu Noble 24.04** 环境构建一份包含 ROS 2 Jazzy Jalisco、Gazebo Harmonic、OSQP、RLQP、OMPL、Fast-Planner 和 Stable Baselines3 的容器镜像，以支持多无人机 IoV 仿真与强化学习调参研究。

## 1. 构建镜像前的准备

* **安装 Docker/Podman**：在宿主机上需预先安装 Docker Engine 或 Podman，并确保当前用户具有构建权限。
* **获取 Dockerfile**：本仓库内包含 `UAV_Simulator_Dockerfile` 和 `entrypoint.sh`，构建镜像前请确保两个文件位于同一目录。

## 2. Dockerfile 说明

`UAV_Simulator_Dockerfile` 以 `ubuntu:24.04` 为基础镜像，主要步骤如下：

1. 设置系统区域和时区，安装基础开发工具。
2. 添加 ROS 2 Jazzy 及 Gazebo 的 apt 源，并安装 `ros-jazzy-desktop`、`gz-sim7`（Gazebo Harmonic）等包。
3. 安装 OMPL (`ros-jazzy-ompl`) 以及 Fast-Planner 所需的依赖库（Eigen3、PCL、OpenMP、Boost）。
4. 通过 pip 安装 `osqp`[^1] 和 `stable-baselines3`[^2]。
5. 克隆 RLQP 仓库并以开发模式安装，RLQP 用于对 ADMM 罚参数进行强化学习调参[^3]。
6. 克隆 Fast-Planner 源码到 `/fast_planner_ws/src/Fast-Planner` 并利用 `colcon` 编译，Fast-Planner 提供四旋翼轨迹搜索与 B 样条优化[^4]。
7. 最后复制 `entrypoint.sh` 脚本，该脚本用于在容器启动时自动加载 ROS 2 及 Fast-Planner 的环境。

## 3. 构建镜像

在含有 Dockerfile 的目录下执行以下命令即可构建镜像（假设 Dockerfile 名为 `UAV_Simulator_Dockerfile`）：

```bash
docker build -f UAV_Simulator_Dockerfile -t uav-iov-sim:latest .
```

> **提示**：构建过程需要下载 ROS、Gazebo、Fast-Planner 等较大的依赖，请保持网络通畅。

构建成功后，可以通过以下命令查看镜像：

```bash
docker images | grep uav-iov-sim
```

## 4. 运行容器

运行容器时需要挂载 X11 或使用 `gzclient` 实现图形界面，以下示例以交互模式启动容器并允许使用 Gazebo 图形界面：

```bash
docker run --rm -it \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --privileged \
  uav-iov-sim:latest
```

* 如需挂载仿真场景或源码，可通过 `-v` 参数挂载宿主机路径到容器。例如将 OSM 转换生成的 Gazebo 世界文件挂载到 `/home/ros2_ws/src/worlds`。

进入容器后可执行以下命令验证环境：

```bash
# 启动 Gazebo Harmonic
gz sim

# 初始化 ROS 工作空间
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker

# 检查 Fast-Planner 库和 OSQP
tools:
ros2 pkg list | grep fast_planner
python3 -c "import osqp; import stable_baselines3; import rlqp"
```

## 5. 注意事项

* **Gazebo 与 ROS 2 版本**：Dockerfile 中安装的 `gz-sim7` 与 Jazzy 对应的版本为 Gazebo Harmonic，若未来版本更新，请调整包名称。
* **RLQP 策略训练**：镜像默认仅安装 RLQP 框架，用户需自行在容器中运行训练脚本并保存模型[^5]。
* **图形加速**：如果宿主机使用 NVIDIA GPU，可通过 `--gpus all` 参数启用 GPU 加速；使用 CuOSQP 需要额外安装 CUDA 工具链。
* **安全性**：容器以 root 权限运行，如需限制权限，可在 Dockerfile 中创建普通用户并切换运行身份。

## 6. 无法在当前环境直接构建镜像的说明

当前对话环境无法安装 Docker 或 Podman，因此无法直接执行 `docker build` 创建实际镜像。然而，提供的 Dockerfile 已包含所有必要的配置步骤，用户可以将其复制到本地开发机器上按照上述方法构建镜像。如需进一步定制（例如添加 PX4 仿真、MoveIt2 等），可在 Dockerfile 中添加对应安装步骤。

---

[^1]: [OSQP 官方网站](https://osqp.org/)

[^2]: [Stable Baselines3 文档](https://stable-baselines3.readthedocs.io/en/master/)

[^3]: [RLQP README](https://raw.githubusercontent.com/BerkeleyAutomation/rlqp/master/README.md)

[^4]: [Fast-Planner README](https://raw.githubusercontent.com/HKUST-Aerial-Robotics/Fast-Planner/master/README.md)

[^5]: RLQP 策略训练脚本说明
