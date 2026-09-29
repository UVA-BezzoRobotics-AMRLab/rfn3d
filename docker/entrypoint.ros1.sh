#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash

# The ROS1 node needs a running master; start one if none is reachable.
if ! rostopic list >/dev/null 2>&1; then
  roscore &
  until rostopic list >/dev/null 2>&1; do sleep 0.5; done
fi

exec rosrun rfn3d planner
