FROM rexasi-tracker/base

RUN apt remove -y python3-sympy 

ADD detectors/lidar/requirements.txt .
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install -r requirements.txt

COPY detectors/lidar/ros/lidar_detector/libs/dr_spaam src/dr_spaam
RUN cd src/dr_spaam && python3 setup.py install

COPY ros/rexasi_tracker_msgs src/rexasi_tracker_msgs
RUN source /opt/ros/humble/setup.bash && colcon build --packages-select rexasi_tracker_msgs

COPY detectors/lidar/ros/lidar_detector src/lidar_detector
RUN source /opt/ros/humble/setup.bash && colcon build --packages-select lidar_detector

ENTRYPOINT ["/ros_entrypoint.sh"]
