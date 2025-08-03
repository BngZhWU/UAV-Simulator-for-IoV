#!/bin/bash
# Source ROS 2 and Fast-Planner overlays
set -e
# Source ROS 2
source /opt/ros/$ROS_DISTRO/setup.bash
# Source Fast-Planner workspace if built
if [ -f "/fast_planner_ws/install/setup.bash" ]; then
    source /fast_planner_ws/install/setup.bash
fi
exec "$@"
