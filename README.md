# UAV IoV Simulator – ROS 2 Humble + Gazebo (ros\_gz) + Aerostack2 + RLQP + CUDA

A GPU‑enabled, containerized environment for multi‑UAV research. It bundles **ROS 2 Humble** on **Ubuntu 22.04**, **ros\_gz** (Gazebo integration), **Aerostack2** (multi‑rotor behaviors), and a Python venv with **OSQP**, **Stable‑Baselines3**, and **RLQP**. The image also includes the **Intel RealSense SDK** so you can build/run `as2_realsense_interface` and access physical cameras (host DKMS required).

> **Highlights**
>
> * Ready‑to‑run: ROS 2 + ros\_gz + Aerostack2 prebuilt in `/aerostack2_ws`
> * GPU support via NVIDIA Container Toolkit
> * RealSense userspace SDK (`librealsense2`, `-dev`, tools) inside the image
> * PEP 668‑compliant Python venv at `/opt/venv`
> * Sensible defaults for reproducible builds; guidance for WSL2 + usbipd‑win

---

## 0. Prerequisites

* **NVIDIA GPU + Driver** on the host; **NVIDIA Container Toolkit** installed.
* **Docker Desktop** (Linux or Windows/WSL2) with Linux containers.
* **(Optional) RealSense**: install **`librealsense2-dkms`** and udev rules on the **host** (Linux) if you want to access a physical camera.
* **GUI/X11**: Linux uses X11; Windows/WSL2 typically uses WSLg (Wayland/X11) automatically.

---

## 1. What’s inside the image

* **Base OS**: Ubuntu 22.04 from CUDA 12.4.1 images (devel/runtime split)
* **ROS 2**: Humble Desktop, OMPL, colcon extensions, ros‑dev‑tools
* **Gazebo integration**: `ros-${ROS_DISTRO}-ros-gz`, `ros-gz-bridge`, `ros-gz-sim`, `ros-gz-interfaces`
* **Aerostack2**: cloned and built at `/aerostack2_ws`
* **Python venv**: `/opt/venv` with `osqp`, `stable-baselines3[extra]`, `pymap3d`, `rlqp` (pinned commit)
* **RealSense**: `librealsense2` (runtime), `librealsense2-dev` (builder), tools (`realsense-viewer`, `rs-enumerate-devices`)
* **Entrypoint**: `/entrypoint.sh` – sources ROS underlay and Aerostack2 overlay, puts venv on PATH, prints `nvidia-smi` and a RealSense hint if tools are present

---

## 2. Build

```bash
# from repo root
DOCKER_BUILDKIT=1 docker build \
  -f UAV_Simulator_Dockerfile \
  -t uav-iov-sim:humble .
```

**Build arguments (optional):**

```bash
--build-arg ROS_DISTRO=humble \
--build-arg CUDA_IMAGE_DEVEL=nvidia/cuda:12.4.1-cudnn-devel-ubuntu22.04 \
--build-arg CUDA_IMAGE_RUNTIME=nvidia/cuda:12.4.1-cudnn-runtime-ubuntu22.04
```

> **Tips**
>
> * Slow networks: enable Docker BuildKit and avoid rebuilding from scratch; keep the APT/PIP caches.
> * Memory pressure on WSL2: increase `.wslconfig` memory/swap; see Troubleshooting.

---

## 3. Run (GPU + GUI + ROS)

### 3.1 Linux host (X11)

```bash
xhost +local:root  # allow X11

docker run --rm -it --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --name uav_iov_sim uav-iov-sim:humble
```

### 3.2 Windows + WSL2 (WSLg)

1. Ensure Docker Desktop uses the **WSL2** backend and WSLg is active.
2. Typically no extra X11 env is needed; WSLg sets `DISPLAY`/`WAYLAND_DISPLAY`.
3. Run the container from a WSL2 shell:

```bash
docker run --rm -it --gpus all `
  --name uav_iov_sim uav-iov-sim:humble
```

> If GUI tools don’t render, explicitly pass `-e DISPLAY` and mount `/tmp/.X11-unix` as in the Linux example.

### 3.3 Access a physical RealSense camera

**On Linux host:**

1. Install host‑side kernel driver + udev rules (one‑time):

   ```bash
   sudo apt-get update && sudo apt-get install -y librealsense2-dkms
   # or use upstream scripts to install udev rules
   ```
2. Replug the device.
3. Run the container with USB passthrough (choose one):

   ```bash
   # pass all USB buses (simple, broad)
   --device=/dev/bus/usb

   # or pass only the relevant video nodes (more restrictive)
   --device=/dev/video0 --device=/dev/video1
   ```

**Example:**

```bash
docker run --rm -it --gpus all \
  --device=/dev/bus/usb \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --name uav_iov_sim uav-iov-sim:humble
```

**Inside container (verification):**

```bash
rs-enumerate-devices
realsense-viewer
```

**Windows + WSL2 (usbipd‑win):**

1. Install **usbipd‑win** on Windows.
2. In **PowerShell (Admin)**: `usbipd list` → `usbipd bind --busid <BUSID>`
3. Attach to WSL2: `usbipd attach --wsl --busid <BUSID>`
4. Confirm in WSL2: `lsusb`
5. Run the container with `--device=/dev/bus/usb`.

> When a device is attached to WSL2 via usbipd‑win, Windows cannot use it until you detach: `usbipd detach --busid <BUSID>`.

---

## 4. First‑run quickstart (5 minutes)

Inside the container:

```bash
# GPU visibility
nvidia-smi

# ROS health
ros2 doctor --report

# Aerostack2 workspace already built
source /aerostack2_ws/install/setup.bash

# Launch a ros_gz static bridge (basic demo)
ros2 run ros_gz_bridge static_bridge
```

In another terminal (same container):

```bash
# Example: list ROS 2 topics and nodes
ros2 topic list
ros2 node list
```

If you have a RealSense camera passed through:

```bash
rs-enumerate-devices
realsense-viewer   # test streaming
```

---

## 5. Developer workflow

### 5.1 Use the Python venv

```bash
which python
python -V
pip list | grep -E "osqp|stable-baselines3|rlqp"
```

The entrypoint puts `/opt/venv/bin` first on `PATH`.

### 5.2 Mount your own ROS 2 workspace

```bash
# host path: $PWD/planning_ws will be mounted into the container
xhost +local:root

docker run --rm -it --gpus all \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $PWD/planning_ws:/planning_ws \
  --name uav_iov_sim uav-iov-sim:humble
```

Inside the container:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p /planning_ws/src && cd /planning_ws
# copy/clone your packages into /planning_ws/src
rosdep update && rosdep install --from-paths src --ignore-src -r -y
# limit parallelism to avoid memory spikes on WSL2
export CMAKE_BUILD_PARALLEL_LEVEL=4
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source /planning_ws/install/setup.bash
```

### 5.3 ros\_gz bridge patterns

```bash
# 1) Static bridge for common message types
ros2 run ros_gz_bridge static_bridge

# 2) Manual bridge for specific pairs (example)
ros2 run ros_gz_bridge parameter_bridge \
  /camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /imu@sensor_msgs/msg/Imu@gz.msgs.IMU
```

---

## 6. Entrypoint behavior (`/entrypoint.sh`)

* Adds `/opt/venv/bin` to `PATH` if present
* Sources `/opt/ros/$ROS_DISTRO/setup.bash`
* Sources `/aerostack2_ws/install/setup.bash` if it exists
* Prints `nvidia-smi` output if available
* Prints a hint if RealSense tools are installed
* Exports `ROS_DOMAIN_ID` (default `0`) and chains to `CMD`

**Bypass/override:**

```bash
docker run --rm -it --gpus all \
  --entrypoint bash uav-iov-sim:humble
```

---

## 7. Environment variables

* `ROS_DISTRO` – set by the image (default: `humble`)
* `ROS_DOMAIN_ID` – DDS domain isolation (default: `0`)
* `DISPLAY`, `WAYLAND_DISPLAY`, `XDG_RUNTIME_DIR` – GUI; WSLg populates these automatically
* `NVIDIA_VISIBLE_DEVICES`, `NVIDIA_DRIVER_CAPABILITIES` – NVIDIA runtime (often implied by `--gpus all`)
* `CMAKE_BUILD_PARALLEL_LEVEL` – limit CMake/Ninja parallelism during heavy builds

---

## 8. Docker Compose (optional)

```yaml
version: "3.8"
services:
  uav:
    image: uav-iov-sim:humble
    container_name: uav_iov_sim
    runtime: nvidia
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=0
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./planning_ws:/planning_ws
    devices:
      - /dev/bus/usb:/dev/bus/usb
    stdin_open: true
    tty: true
```

Start:

```bash
docker compose up --remove-orphans
```

---

## 9. Troubleshooting & Performance

### 9.1 EOF / gRPC disconnect during long builds

* Symptom: `rpc error: code = Unavailable desc = error reading from server: EOF`
* Cause: Docker daemon/BuildKit restarted or lost connection (often due to OOM)
* Fixes:

  * Split very long `RUN` steps (already applied in Dockerfile)
  * Limit `colcon` parallelism (`--parallel-workers 2..4`) and `CMAKE_BUILD_PARALLEL_LEVEL`
  * Increase WSL2 resources via `~/.wslconfig` (e.g., `memory=24GB`, `swap=16GB`), then `wsl --shutdown`
  * Restart Docker Desktop; retry with `--progress=plain`

### 9.2 RealSense not detected in container

* Ensure host has `librealsense2-dkms` and udev rules; replug the device
* Pass USB devices via `--device=/dev/bus/usb` (or specific `/dev/video*`)
* On WSL2 use `usbipd-win` to attach the USB to the WSL distro

### 9.3 GUI not showing (Linux)

* Ensure `xhost +local:root` and `-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw`

### 9.4 DDS discovery issues across networks

* Use the same `ROS_DOMAIN_ID` for communicating processes; consider Fast DDS QoS profiles if needed

### 9.5 Cleaning up disk space

```bash
docker system df
docker image prune -a
docker builder prune -a
```

---

## 10. Typical research pipeline (inside the container)

1. **Global seed generation (Aerostack2 behaviors)** – produce reference paths
2. **Convex smoothing/refinement (OSQP)** – enforce kinodynamic limits & safety
3. **RL‑assisted solver tuning (RLQP)** – learn/update QP hyper‑parameters
4. **Closed‑loop sim** – run ros\_gz + bridges; visualise in RViz/Gazebo

---

## 11. Version notes / matrix

* **Ubuntu**: 22.04 (Jammy)
* **ROS 2**: Humble
* **CUDA/cuDNN**: from chosen base tags (default 12.4.1)
* **Aerostack2**: shallow clone of upstream (HEAD at build time)
* **RealSense**: from Intel apt repo (Jammy channel)

> Rebuild to pick up newer upstream revisions; pin commits/tags for reproducibility in production.

---

## 12. Common Git commands (cheat sheet)

### 12.1 Setup & identity

```bash
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
git config --global core.editor "code --wait"   # VS Code as editor (optional)
```

### 12.2 Start & clone

```bash
git init                      # init repo in current dir
git clone <url> [dir]         # clone to folder
```

### 12.3 Status, add, commit

```bash
git status                    # what changed
git add <file> ...            # stage files
git add -p                    # stage interactively
git commit -m "feat: message" # conventional commit msg (suggested)
```

### 12.4 Branching & switching

```bash
git branch                    # list branches
git switch -c feat/x         # create & switch
git switch main              # switch to main
```

### 12.5 Sync with remote

```bash
git remote -v                # show remotes
git remote add origin <url>  # add remote

git fetch origin             # fetch branches/tags
git pull --rebase origin main  # update local main with rebase
git push -u origin main        # first push sets upstream
```

### 12.6 Inspect & diff

```bash
git log --oneline --graph --decorate --all
git show <commit>
git diff                      # unstaged changes
git diff --staged             # staged changes
```

### 12.7 Undo & clean up (careful)

```bash
git restore <file>            # discard unstaged changes
git restore --staged <file>   # unstage

git reset --hard HEAD~1       # drop last commit (danger)
```

### 12.8 Stash work in progress

```bash
git stash push -m "wip: note"
git stash list
git stash pop                 # apply and drop
```

### 12.9 Merge & rebase

```bash
git merge feat/x              # merge into current branch
# or rebase for linear history
git rebase main               # replay commits on top of main
```

### 12.10 Tags & releases

```bash
git tag -a v1.0.0 -m "release v1.0.0"
git push origin v1.0.0
```

### 12.11 Submodules (optional)

```bash
git submodule add <url> third_party/<name>
git submodule update --init --recursive
```

### 12.12 Useful aliases (optional)

```bash
git config --global alias.s  "status -sb"
git config --global alias.lg "log --oneline --graph --decorate --all"
```

---

## 13. License & acknowledgements

* ROS 2, ros\_gz, Aerostack2, librealsense, and other components are licensed by their respective owners.
* This image integrates these open‑source projects for research and education.
