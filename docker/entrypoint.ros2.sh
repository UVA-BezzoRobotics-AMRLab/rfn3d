#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

LAUNCH_ARGS=()
if [ -n "${INITIAL_GOAL:-}" ]; then
  LAUNCH_ARGS+=("initial_goal:=${INITIAL_GOAL}")
fi

exec ros2 launch rfn3d planner.launch.py "${LAUNCH_ARGS[@]}"
