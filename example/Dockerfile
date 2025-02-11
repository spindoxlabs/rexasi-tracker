FROM rexasi-tracker/base

RUN apt update && apt install -y \
  sqlite3 \
  ros-$ROS_DISTRO-ros2bag \
  ros-$ROS_DISTRO-rosbag2-transport \
  ros-$ROS_DISTRO-rosbag2-storage-default-plugins \
  ros-$ROS_DISTRO-rosbag2-storage-mcap
RUN rm -rf /var/lib/apt/lists/*
