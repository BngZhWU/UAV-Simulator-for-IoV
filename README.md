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

## 2. Get the image (two options)

You can either **pull the prebuilt image from Docker Hub** or **build locally**.
In all commands below, use the appropriate image name:

* If you **pulled from Docker Hub**, use `benjiwu/uav-iov-sim:humble`.
* If you **built locally**, use `uav-iov-sim:humble` (or your custom tag).

### 2A. Pull the prebuilt image (recommended)

```bash
docker pull docker.io/benjiwu/uav-iov-sim:humble
```

If you use Docker Compose, make sure your `compose.yaml` points to that image:

```yaml
services:
  uav:
    image: benjiwu/uav-iov-sim:humble
```

### 2B. Build locally (custom)

```bash
# from repo root
DOCKER_BUILDKIT=1 docker build \
  -f UAV_Simulator_Dockerfile \
  -t uav-iov-sim:humble .
```

**Optional build arguments:**

```bash
--build-arg ROS_DISTRO=humble \
--build-arg CUDA_IMAGE_DEVEL=nvidia/cuda:12.4.1-cudnn-devel-ubuntu22.04 \
--build-arg CUDA_IMAGE_RUNTIME=nvidia/cuda:12.4.1-cudnn-runtime-ubuntu22.04
```

> **Tips**
>
> * Slow networks: enable Docker BuildKit and avoid rebuilding from scratch; keep the APT/PIP caches.
> * Memory pressure on WSL2: increase `.wslconfig` memory/swap; see Troubleshooting.

## 3. Run (GPU + GUI + ROS) (GPU + GUI + ROS)

> You may **start manually with `docker run`** or simply use **Compose via `docker compose`**. Both ways are equivalent — Compose just keeps the same flags in a single `compose.yaml` for repeatability.

### 3.1 Linux host (X11)

**Manual (recommend allowing X11 first):**

```bash
xhost +local:root

docker run --rm -it --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device=/dev/bus/usb \
  --name uav_iov_sim uav-iov-sim:humble
```

**Using Compose (same effect):**

```bash
docker compose up           # foreground
# or
docker compose up -d        # background
```

> The provided `compose.yaml` already declares GPU request, USB passthrough and X11 mounts.

### 3.2 Windows + WSL2 (WSLg)

**Manual (PowerShell single‑line; for multi‑line use backtick \` as the line continuation):**

```powershell
docker run --rm -it --gpus all `
  --name uav_iov_sim uav-iov-sim:humble
```

> If GUI apps do not render, explicitly pass `-e DISPLAY` and mount `/tmp/.X11-unix` (WSLg usually sets these automatically).

**Using Compose:**

```bash
docker compose up
```

### 3.3 Access a physical RealSense camera

**Linux host:**

1. Install host‑side kernel driver + udev rules (one‑time):

```bash
sudo apt-get update && sudo apt-get install -y librealsense2-dkms
```

2. Replug the device.
3. Start the container with USB passthrough (see above `--device=/dev/bus/usb`, or use Compose which already includes it).

**Windows + WSL2:**

1. Install **usbipd‑win** on Windows.
2. In elevated PowerShell: `usbipd list` → `usbipd bind --busid <BUSID>` → `usbipd attach --wsl --busid <BUSID>`
3. Confirm `lsusb` in WSL2; then `docker compose up` or manual `docker run`.

**Verify inside the container:**

```bash
rs-enumerate-devices
realsense-viewer
```

---

## 4. First‑run quickstart (5 minutes)

Inside the container:

```bash
nvidia-smi
ros2 doctor --report

# Aerostack2 workspace already built
source /aerostack2_ws/install/setup.bash

# Launch a ros_gz static bridge (basic demo)
ros2 run ros_gz_bridge static_bridge
```

In another terminal (same container):

```bash
ros2 topic list
ros2 node list
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

### 5.2 Mount your own ROS 2 workspace

**Linux:**

```bash
xhost +local:root

docker run --rm -it --gpus all \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $PWD/planning_ws:/planning_ws \
  --name uav_iov_sim uav-iov-sim:humble
```

Container:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p /planning_ws/src && cd /planning_ws
# copy/clone your packages into /planning_ws/src
rosdep update && rosdep install --from-paths src --ignore-src -r -y
# limit parallelism to avoid memory spikes (especially on WSL2)
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
* Sources `/opt/ros/$ROS_DISTRO/setup.bash` and `/aerostack2_ws/install/setup.bash`
* Prints `nvidia-smi` if available, and a hint when RealSense tools are present
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
* `CMAKE_BUILD_PARALLEL_LEVEL` – limit CMake/Ninja parallelism during heavy builds

---

## 8. Troubleshooting & Performance

### 8.1 EOF / gRPC disconnect during long builds

* Split long `RUN` steps; limit `colcon`/CMake parallelism; increase WSL2 memory/swap; restart Docker and re‑try with `--progress=plain`.

### 8.2 RealSense not detected in container

* Ensure host has `librealsense2-dkms` and udev rules; replug the device
* Pass USB via `--device=/dev/bus/usb` (or specific `/dev/video*`)
* On WSL2 use `usbipd-win` to attach the USB to the distro

### 8.3 GUI not showing (Linux)

* Ensure `xhost +local:root` and `-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw`

### 8.4 PowerShell line continuation

* Use the **backtick** (\`) for multi‑line commands; the Linux backslash () will cause errors like `invalid reference format`.

---

## 9. Common Git commands (cheat sheet)

### 9.1 Setup & identity

```bash
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
git config --global core.editor "code --wait"
```

### 9.2 Start & clone

```bash
git init
git clone <url> [dir]
```

### 9.3 Status, add, commit

```bash
git status
git add <file> ...
git add -p
git commit -m "feat: message"
```

### 9.4 Branching & switching

```bash
git branch
git switch -c feat/x
git switch main
```

### 9.5 Sync with remote

```bash
git remote -v
git remote add origin <url>

git fetch origin
git pull --rebase origin main
git push -u origin main
```

### 9.6 Inspect & diff

```bash
git log --oneline --graph --decorate --all
git show <commit>
git diff
git diff --staged
```

### 9.7 Undo & clean up (careful)

```bash
git restore <file>
git restore --staged <file>

git reset --hard HEAD~1
```

### 9.8 Stash work in progress

```bash
git stash push -m "wip: note"
git stash list
git stash pop
```

### 9.9 Merge & rebase

```bash
git merge feat/x
# or linear history
git rebase main
```

### 9.10 Tags & releases

```bash
git tag -a v1.0.0 -m "release v1.0.0"
git push origin v1.0.0
```

### 9.11 Submodules (optional)

```bash
git submodule add <url> third_party/<name>
git submodule update --init --recursive
```

### 9.12 Useful aliases (optional)

```bash
git config --global alias.s  "status -sb"
git config --global alias.lg "log --oneline --graph --decorate --all"
```

---

## 10. License & acknowledgements

* ROS 2, ros\_gz, Aerostack2, librealsense, and other components are licensed by their respective owners.
* This image integrates these open‑source projects for research and education. For commercial use, please check the license of these projects. 
