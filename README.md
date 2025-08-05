# UAV Simulator for IoV — Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + Aerostack2 + CUDA/cuDNN

This document explains how to build and run a container image on **Ubuntu 24.04 (Noble)** for multi‑UAV IoV research.  The image includes **ROS 2 Jazzy Jalisco**, **Gazebo Harmonic**, **Aerostack2** (built from source), **OSQP**, **Stable‑Baselines3**, and the **RLQP** Python package, all accelerated by CUDA/cuDNN.  With these components you can simulate fleets of UAVs in realistic 3D city environments, run quadratic programming based path planners accelerated by RLQP, and prototype learning‑based trajectory optimisation.

> **Note:** This configuration has been tested and works on a Windows environment.

## 1 Prerequisites

- **Install Docker/Podman** – Make sure [Docker Engine](https://docs.docker.com/engine/install/) or Podman is installed and that your user can run `docker build` and `docker run`.
- **Get the build files** – This repository contains `UAV_Simulator_Dockerfile` and `entrypoint.sh`. Both must be present in the root directory before building.

```
bash git clone https://github.com/BngZhWU/UAV-Simulator-for-IoV.git
cd UAV-Simulator-for-IoV
```

## 2 Build

Build the image using the provided Dockerfile (replace the base tag if your host requires a different CUDA version):

```
bash
docker build -f UAV_Simulator_Dockerfile -t uav-iov-sim:cuda .
```

The build will take some time because it installs ROS 2, Gazebo Harmonic, compiles Aerostack2 from source and creates a Python virtual environment with OSQP, Stable‑Baselines3 and RLQP.

## 3 Run (GPU + X11)

### Linux/X11

```
bash # Allow local X clients to connect
xhost +local:root

docker run --rm -it \
  --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --name uav_iov_sim \
  uav-iov-sim:cuda bash
```

Once inside the container, you can verify that CUDA, Gazebo and ROS 2 are available:

```
bash nvidia-smi          # lists GPUs
ros2 --version      # prints ROS 2 Jazzy version
gz --version        # prints Gazebo Harmonic version
gz sim              # launch the Gazebo GUI
```

On Wayland desktops you may need to run under an XWayland session or set `export GDK_BACKEND=x11` before starting Gazebo.

## 4 Typical Workflow

After starting a container shell, prepare the environment:

```
bash # Source ROS 2 and Aerostack2 overlays
source /opt/ros/jazzy/setup.bash
source /aerostack2_ws/install/setup.bash

# (Optional) initialise your own planning workspace
# source /planning_ws/install/setup.bash

# Launch a simple Aerostack2 simulation or run your own nodes
ros2 launch as2_gazebo_worlds gazebo_single_multirotor.launch.py

# Or open a Python shell and import OSQP/RLQP
python -c "import osqp, rlqp"
```

## 5 Image Contents

The resulting image contains the following key components:

- **CUDA 12.x and cuDNN** – GPU acceleration for neural network training and inference.
- **ROS 2 Jazzy desktop and OMPL** – Core ROS 2 libraries, developer tools and sampling‑based planning library.
- **Gazebo Harmonic** – Latest stable Gazebo simulator for ROS 2.
- **Aerostack2** – Full aerial robotics framework providing motion behaviours, path planning and trajectory generation, compiled from source into `/aerostack2_ws`.
- **Python virtual environment** – Located at `/opt/venv`, containing OSQP, Stable‑Baselines3 and the RLQP Python package.

## 6 Notes & Rationale

- **ros2‑apt‑source** is the recommended way to install ROS 2 on Ubuntu 24.04 and automatically configures GPG keys and package lists.
- **gz‑harmonic** metapackage ensures a consistent installation of Gazebo Harmonic and its dependencies.
- Building **Aerostack2** from source allows us to stay close to upstream developments and use the latest behaviours for path planning and trajectory generation.
- Installing OSQP and RLQP inside a virtual environment avoids conflicts with the system Python (PEP 668) and isolates dependencies for reinforcement learning.

## 7 Troubleshooting

| Issue                      | Fix                                                          |
| -------------------------- | ------------------------------------------------------------ |
| `nvidia-smi` not found     | Run the container with `--gpus all` and install the NVIDIA Container Toolkit on the host. |
| Gazebo GUI doesn't open    | Run `xhost +local:root`, mount `/tmp/.X11-unix` into the container and ensure `$DISPLAY` is set. |
| Base image/driver mismatch | Choose a CUDA base image tag that matches your host GPU driver. |
| ROS commands not found     | Make sure you sourced `/opt/ros/jazzy/setup.bash` before running ROS 2 commands. |



## 8 Acknowledgements

This environment combines the open‑source frameworks **ROS 2 Jazzy**, **Gazebo Harmonic**, **Aerostack2**, **OSQP**, **Stable‑Baselines3** and **RLQP**.  We thank the respective communities for their work.

## Common Git Commands

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
