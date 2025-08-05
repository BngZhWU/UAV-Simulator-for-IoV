#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# CUDA/cuDNN + ROS 2 Jazzy + Gazebo Harmonic container entrypoint
# Sources overlays and forwards to the provided command.
# -----------------------------------------------------------------------------
set -e

# 1) Source ROS 2 environment
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "[entrypoint] WARNING: /opt/ros/${ROS_DISTRO}/setup.bash not found."
fi

# 2) Source Fast-Planner workspace if present
if [[ -f "/fast_planner_ws/install/setup.bash" ]]; then
  source "/fast_planner_ws/install/setup.bash"
fi

# 3) CUDA diagnostics (optional)
if command -v nvidia-smi >/dev/null 2>&1; then
  echo "[entrypoint] nvidia-smi available. GPU devices:"
  nvidia-smi || true
else
  echo "[entrypoint] nvidia-smi not found (did you run with --gpus all?)."
fi

# 4) Ensure RLQP importable
export PYTHONPATH="/opt/rlqp:${PYTHONPATH}"

# 5) Defaults for ROS networking if not set
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

exec "$@"
