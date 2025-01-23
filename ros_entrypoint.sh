#!/bin/bash
set -e

ARCH=$(dpkg --print-architecture)
#ROS_DISTRO=humble

if [[ $ARCH == 'amd64' ]]; then
  echo 'DETECTED AMD ARCHITECTURE'
  # souce ros2 environment
  source "/opt/ros/$ROS_DISTRO/setup.bash" --
  source "/ros_ws/install/setup.bash" --  || true
else
  echo 'DETECTED ARM ARCHITECTURE'
  # souce ros2 environment
  source "/opt/ros/$ROS_DISTRO/install/setup.bash" --
  source "/ros_ws/install/setup.bash" --   || true
fi
exec "$@"
