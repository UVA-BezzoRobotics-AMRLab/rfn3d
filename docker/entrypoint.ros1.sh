#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash

# The ROS1 node needs a running master; start one if none is reachable.
if ! rostopic list >/dev/null 2>&1; then
  roscore &
  until rostopic list >/dev/null 2>&1; do sleep 0.5; done
fi

# NOTE: the planner creates a Gurobi environment on startup, which needs a valid
# Gurobi license at runtime — mount one and set GRB_LICENSE_FILE to actually run.
exec rosrun rfn3d planner
