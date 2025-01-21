FROM rexasi-tracker/base

ADD requirements.txt .
RUN pip install -r requirements.txt

COPY detectors/lidar/ros/lidar_detector/libs/dr_spaam src/dr_spaam
RUN cd src/dr_spaam && python3 setup.py install

COPY ros/rexasi_tracker_msgs rexasi_tracker_msgs
RUN source /opt/ros/humble/setup.bash && colcon build --packages-select rexasi_tracker_msgs

COPY detectors/lidar/ros/lidar_detector lidar_detector
RUN source /opt/ros/humble/setup.bash && colcon build --packages-select lidar_detector

COPY ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
