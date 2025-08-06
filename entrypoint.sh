#!/usr/bin/env bash
# Entry for ROS 2 Humble + Aerostack2 + RLQP container
set -euo pipefail

# 0) venv on PATH
if [ -x "/opt/venv/bin/python" ]; then
  export PATH="/opt/venv/bin:${PATH}"
fi

# --- temporarily disable 'nounset' for sourcing ROS/Aerostack2 ---
set +u
# 1) Source ROS 2
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "[entrypoint] WARNING: /opt/ros/${ROS_DISTRO:-<undefined>}/setup.bash not found."
fi

# 2) Source Aerostack2 overlay
if [ -f "/aerostack2_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "/aerostack2_ws/install/setup.bash"
fi
# --- restore 'nounset' ---
set -u

# 3) Optional GPU info
if command -v nvidia-smi >/dev/null 2>&1; then
  echo "[entrypoint] nvidia-smi:"
  nvidia-smi || true
fi

# 3b) RealSense tools hint
if command -v rs-enumerate-devices >/dev/null 2>&1; then
  echo "[entrypoint] RealSense tools detected. Pass USB devices to access physical cameras."
fi

# 4) Default DDS domain
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# 5) Chain to CMD
exec "$@"
