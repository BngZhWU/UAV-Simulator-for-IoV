# UAV-Simulator-for-IoV Docker Image Configuration Guide

This document describes how to build a container image on **Ubuntu Noble 24.04** that includes ROS 2 Jazzy Jalisco, Gazebo Harmonic, OSQP, RLQP, OMPL, Fast-Planner, and Stable Baselines3, supporting multi-UAV IoV simulation and reinforcement learning parameter tuning research.

> **Note:** This configuration has been tested and works on a Windows environment.

## 1. Prerequisites

* **Install Docker/Podman**: Make sure [Docker Engine](https://docs.docker.com/engine/install/) or Podman is installed on the host machine and that your user has permission to build images.
* **Obtain the Dockerfile**: The repository contains `UAV_Simulator_Dockerfile` and `entrypoint.sh`. Confirm both files are in the same directory before building.

  ```bash
  git clone https://github.com/BngZhWU/UAV-Simulator-for-IoV.git
  cd UAV-Simulator-for-IoV
  ```

## 2. Dockerfile Overview

The `UAV_Simulator_Dockerfile` uses `ubuntu:24.04` as its base image and performs the following steps:

1. Configure system locale and timezone, and install essential development tools.
2. Add the ROS 2 Jazzy and Gazebo repositories, then install `ros-jazzy-desktop` and `gz-sim7` (Gazebo Harmonic).
3. Install OMPL (`ros-jazzy-ompl`) and dependencies for Fast-Planner (Eigen3, PCL, OpenMP, Boost).
4. Use pip to install `osqp`[^1] and `stable-baselines3`[^2].
5. Clone the RLQP repository and install it in development mode; RLQP is used for reinforcement learning–based tuning of ADMM penalty parameters[^3].
6. Clone the Fast-Planner source into `/fast_planner_ws/src/Fast-Planner` and build with `colcon`; Fast-Planner provides quadrotor trajectory search and B-spline optimization[^4].
7. Copy the `entrypoint.sh` script to auto-source ROS 2 and Fast-Planner environments on container startup.

## 3. Building the Image

In the directory containing the Dockerfile, run:

```bash
docker build -f UAV_Simulator_Dockerfile -t uav-iov-sim:latest .
```

> **Tip:** The build will download large dependencies (ROS, Gazebo, Fast-Planner). Ensure a stable internet connection.

After a successful build, verify the image with:

```bash
docker images | grep uav-iov-sim
```

## 4. Running the Container

To run with graphical support (X11 or `gzclient`), start the container in interactive mode:

```bash
docker run --rm -it \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --privileged \
  uav-iov-sim:latest
```

* To mount simulation worlds or source code, add `-v` flags. For example, mount OSM-generated Gazebo world files to `/home/ros2_ws/src/worlds`.

Inside the container, verify the environment:

```bash
# Launch Gazebo Harmonic
gz sim

# Initialize ROS workspace
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker

# Check Fast-Planner and OSQP
ros2 pkg list | grep fast_planner
python3 -c "import osqp; import stable_baselines3; import rlqp"
```

## 5. Notes and Considerations

* **ROS 2 and Gazebo Versions**: The installed `gz-sim7` corresponds to Gazebo Harmonic for ROS 2 Jazzy. Update package names if versions change.
* **RLQP Training**: The image installs the RLQP framework only; run training scripts inside the container and save models as needed[^5].
* **GPU Acceleration**: For NVIDIA GPUs, add `--gpus all` and install CUDA toolkits for CuOSQP.
* **Security**: The container runs as root by default. To restrict permissions, create a non-root user in the Dockerfile and switch to it.
* **Customization**: To add PX4 simulation, MoveIt2, or other tools, extend the Dockerfile with the required installation steps.

## 6. Common Git Commands

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
