#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

# NOTE: the planner creates a Gurobi environment on startup, which needs a valid
# Gurobi license at runtime — mount one and set GRB_LICENSE_FILE to actually run.
exec ros2 run rfn3d planner_node
