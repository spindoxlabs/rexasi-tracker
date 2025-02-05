---
sidebar_position: 1
---

# Available detectors


With the Rexasi Tracker come the following, ready to use, people detectors.

## LiDAR detector
The LiDAR detector uses the [DR-SPAAM](https://arxiv.org/abs/2004.14079) model to detect people from 2D LiDAR data.
The implementation is based on [2D_lidar_person_detection](https://github.com/VisualComputingInstitute/2D_lidar_person_detection). \
Code and configuration under [LiDAR folder](https://github.com/spindoxlabs/rexasi-tracker/tree/main/detectors/lidar).


## RGBD detector
The RGBD detector uses both color and depth images to detect the position of people from pose keypoints. \
Code and configuration under [RGBD folder](https://github.com/spindoxlabs/rexasi-tracker/tree/main/detectors/rgbd). \
This implementation consists of two ROS2 nodes:

- **Pose estimation**: for each color frame given as input, this node apply the [YOLOv9 Pose](https://docs.ultralytics.com/it/models/yolov9/) model to detect the body joints of each people in the image. The output is a [Persons](https://github.com/spindoxlabs/rexasi-tracker/blob/main/detectors/rgbd/ros/rgbd_detector_msgs/msg/Persons.msg) message, for each person a [PoseArray](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseArray.html) message contains the list of keypoints with their confidence.

- **Detector**: for each person, this node computes the projection of the keypoints from pixel coordinates to 3D world coordinates, then estimates and outputs the center of the person. The projection is done by extracting the keypoints distance using the depth image data along with the camera intrinsics. The library used to compute the projection is the [librealsense2](https://github.com/IntelRealSense/librealsense).

The RGBD detector considers the camera position centered in the `world` frame. If the camera publishes its own static TF, you can use it by changing the `frame_id` and `optical_frame_id` in the launch configuration ([RGBD launch file](https://github.com/spindoxlabs/rexasi-tracker/blob/main/detectors/rgbd/ros/rgbd_detector/launch/launch.py)), so the detections are transformed with respect to the camera position.