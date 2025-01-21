FROM rexasi-tracker/base

ADD requirements.txt .
RUN pip install -r requirements.txt

# RUN apt remove -y python3-sympy 
# ADD requirements.txt .
# RUN python3 -m pip install --upgrade pip
# RUN python3 -m pip install -r requirements.txt

# # for amd, install pyrealsense2 from pip
# RUN python3 -m pip install pyrealsense2

COPY ros/rexasi_tracker_msgs rexasi_tracker_msgs
RUN source /opt/ros/humble/setup.bash && colcon build --packages-select rexasi_tracker_msgs

COPY detectors/rgbd/ros/rgbd_detector rgbd_detector
RUN source /opt/ros/humble/setup.bash && colcon build --packages-select rgbd_detector

COPY ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
