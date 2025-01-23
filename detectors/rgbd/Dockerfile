FROM rexasi-tracker/base

RUN apt remove -y python3-sympy 

ADD detectors/rgbd/requirements.txt .
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install -r requirements.txt

COPY ros/rexasi_tracker_msgs src/rexasi_tracker_msgs
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --packages-select rexasi_tracker_msgs

COPY detectors/rgbd/ros/rgbd_detector src/rgbd_detector
COPY detectors/rgbd/ros/rgbd_detector_msgs src/rgbd_detector_msgs
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --packages-select rgbd_detector rgbd_detector_msgs

