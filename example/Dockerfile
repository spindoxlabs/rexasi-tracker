FROM rexasi-tracker/base

RUN apt update && apt install -y \
  ros-$ROS_DISTRO-ros2bag \
  ros-$ROS_DISTRO-rosbag2-transport \
  ros-$ROS_DISTRO-rosbag2-storage-mcap
RUN rm -rf /var/lib/apt/lists/*

COPY ros_entrypoint.sh /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]