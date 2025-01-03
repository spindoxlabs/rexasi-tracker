#!/bin/bash
set -e

ARCH=$(dpkg --print-architecture)

if [[ $ARCH == 'amd64' ]]; then
  echo 'DETECTED AMD ARCHITECTURE'
  # souce ros2 environment
  source "/ros_ws/install/setup.bash" --
  source "/opt/ros/humble/setup.bash" --
else
  echo 'DETECTED ARM ARCHITECTURE'
  # souce ros2 environment
  source "/ros_ws/install/setup.bash" --
  source "/opt/ros/humble/install/setup.bash" --
fi
exec "$@"
