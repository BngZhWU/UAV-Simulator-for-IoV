#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Container entrypoint for the UAV IoV simulator.
# Prepares ROS 2 and Aerostack2 overlays, ensures the virtual environment is on
# PATH, prints optional GPU diagnostics and then forwards execution to the
# provided command.  POSIX line endings only; Windows CRLF will break bash.
# -----------------------------------------------------------------------------

# Fail fast, treat unset variables as errors and fail pipelines on first error
set -euo pipefail

# 0) Ensure the virtualenv (PEP 668 compliant) is available on PATH
if [ -x "/opt/venv/bin/python" ]; then
  export PATH="/opt/venv/bin:${PATH}"
fi

# 1) Source ROS 2 underlay
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "[entrypoint] WARNING: /opt/ros/${ROS_DISTRO:-<undefined>}/setup.bash not found."
fi

# 2) Source Aerostack2 overlay if available
if [ -f "/aerostack2_ws/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "/aerostack2_ws/install/setup.bash"
fi

# 3) Source any user-provided planning workspace (e.g. /planning_ws)
if [ -f "/planning_ws/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "/planning_ws/install/setup.bash"
fi

# 4) Optional GPU diagnostics
if command -v nvidia-smi >/dev/null 2>&1; then
  echo "[entrypoint] nvidia-smi available. GPU devices:"
  nvidia-smi || true
else
  echo "[entrypoint] nvidia-smi not found (did you run the container with --gpus?)."
fi

# 5) Defaults for ROS networking if not set
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Forward to the provided command
exec "$@"
