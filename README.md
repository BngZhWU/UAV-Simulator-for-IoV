# UAV Simulator for IoV — CUDA/cuDNN + ROS 2 Jazzy + Gazebo Harmonic
This document describes how to build a container image on **Ubuntu Noble 24.04** that includes ROS 2 Jazzy Jalisco, Gazebo Harmonic, OSQP, RLQP, OMPL, Fast-Planner, and Stable Baselines3, supporting multi-UAV IoV simulation and reinforcement learning parameter tuning research with CUDA/CuDNN support.

> **Note:** This configuration has been tested and works on a Windows environment.
## 1. Prerequisites

* **Install Docker/Podman**: Make sure [Docker Engine](https://docs.docker.com/engine/install/) or Podman is installed on the host machine and that your user has permission to build images.
* **Obtain the Dockerfile**: The repository contains `UAV_Simulator_Dockerfile` and `entrypoint.sh`. Confirm both files are in the same directory before building.

```bash
git clone https://github.com/BngZhWU/UAV-Simulator-for-IoV.git
cd UAV-Simulator-for-IoV
```

## 2. Build

```bash
docker build -f UAV_Simulator_Dockerfile -t uav-iov-sim:cuda .
```

> The Dockerfile uses `nvidia/cuda:12.8.1-cudnn-devel-ubuntu24.04`.
> Replace the tag if your host driver requires a different CUDA version.

## 3. Run (GPU + X11)

### Linux/X11

```bash
xhost +local:root   # Allow local X clients

docker run --rm -it \
  --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/home/ros/.XAuthority:ro \
  --name uav_iov_sim \
  uav-iov-sim:cuda bash
```

Inside the container:

```bash
nvidia-smi           # GPU listed
gz --version         # Gazebo Harmonic
ros2 --version       # ROS 2 Jazzy
gz sim               # Launch Gazebo GUI
```

### Wayland Tips

If you use Wayland, start an XWayland session or set:

```bash
export GDK_BACKEND=x11
```

## 4. Typical Workflow

```bash
source /opt/ros/jazzy/setup.bash
source /fast_planner_ws/install/setup.bash

ros2 run demo_nodes_cpp talker
```

## 5. Image Contents

* CUDA 12.x + cuDNN development libraries
* ROS 2 Jazzy desktop + OMPL
* Gazebo Harmonic (`gz-harmonic`)
* Fast‑Planner built with `colcon`
* RLQP installed in editable mode
* Python packages: OSQP, Stable-Baselines3 (+ extra)

## 6. Notes & Rationale

* **ros2-apt-source** is the recommended way to install ROS 2 on Ubuntu 24.04 (Noble).
* **gz-harmonic** metapackage ensures a consistent Gazebo Harmonic install.
* Using NVIDIA's `cudnn-devel` base image exposes CUDA/cuDNN without manual install.

## 7. Troubleshooting

| Issue                   | Fix                                                                                       |
| ----------------------- | ----------------------------------------------------------------------------------------- |
| `nvidia-smi` not found  | Run container with `--gpus all` and ensure NVIDIA Container Toolkit is installed on host. |
| Gazebo GUI doesn’t open | Allow X11 (`xhost +local:root`), mount `/tmp/.X11-unix`, and verify `$DISPLAY`.           |
| Driver/CUDA mismatch    | Use a base image tag matching your host driver.                                           |

## 8. Common Git Commands

```bash
# 1. Clone a remote repository
git clone https://github.com/<username>/<repository>.git

# 2. Enter the repository directory
cd <repository>

# 3. Pull updates from the main branch
git pull origin main  # or 'master' if your default branch is named 'master'

# 4. Fetch and list all remote branches
git fetch
git branch -a

# 5. Switch to an existing branch
git checkout <branch-name>

# 6. Create and switch to a new branch
git checkout -b <new-branch-name>

# After making changes:
# Stage all changes
git add .

# Commit with a message
git commit -m "Brief description of changes"

# 7. Push local branch to GitHub
git push origin <branch-name>

# For first-time pushes of new branches:
git push -u origin <new-branch-name>
```

**Notes:**

* `clone`: Copies the entire remote repository locally.
* `pull`: Fetches and merges changes from the remote.
* `fetch` + `branch -a`: Updates remote refs without merging; lists all branches.
* `checkout`: Switches branches; `-b` creates and switches.
* `add` + `commit`: Stages and commits changes locally.
* `push`: Uploads commits to the remote repository.

---

[^1]: [OSQP Official Website](https://osqp.org/)

[^2]: [Stable Baselines3 Documentation](https://stable-baselines3.readthedocs.io/en/master/)

[^3]: [RLQP README](https://raw.githubusercontent.com/BerkeleyAutomation/rlqp/master/README.md)

[^4]: [Fast-Planner README](https://raw.githubusercontent.com/HKUST-Aerial-Robotics/Fast-Planner/master/README.md)

[^5]: Instructions for RLQP strategy training scripts
