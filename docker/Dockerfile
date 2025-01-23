FROM rexasi-tracker/base

ADD requirements.txt .
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install -r requirements.txt

COPY ros/rexasi_tracker src/rexasi_tracker
COPY ros/rexasi_tracker_msgs src/rexasi_tracker_msgs
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --packages-select rexasi_tracker rexasi_tracker_msgs

