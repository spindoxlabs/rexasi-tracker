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

- **Pose estimation**: for each color frame given as input, this node apply the [YOLOv9 Pose](https://docs.ultralytics.com/it/models/yolov9/) model to detect the body keypoints of each people in the image.

- **Detector**: for each person, this node compute the projection of the keypoints from pixel coordinates to 3D world coordinates, then estimates and output the center of the person.
