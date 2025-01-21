FROM rexasi-tracker/base

RUN apt remove -y python3-sympy 

ADD detectors/rgbd/requirements.txt .
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install -r requirements.txt

# RUN apt update && apt install -y ros-humble-realsense2-*

# RUN apt remove -y python3-sympy 
# ADD requirements.txt .
# RUN python3 -m pip install --upgrade pip
# RUN python3 -m pip install -r requirements.txt

# # for amd, install pyrealsense2 from pip
# RUN python3 -m pip install pyrealsense2

COPY ros/rexasi_tracker_msgs src/rexasi_tracker_msgs
RUN source /opt/ros/humble/setup.bash && colcon build --packages-select rexasi_tracker_msgs

COPY detectors/rgbd/ros/rgbd_detector src/rgbd_detector
RUN source /opt/ros/humble/setup.bash && colcon build --packages-select rgbd_detector

ENTRYPOINT ["/ros_entrypoint.sh"]
