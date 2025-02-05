---
sidebar_position: 2
---

# Use a custom detector


The Rexasi Tracker can be used with any custom detector, applied not only to people tracking but any application that requires tracking multiple objects in time. \
The implementation requires [ROS2](https://docs.ros.org/en/humble/). If already implemented in ROS1, it can still be used by adding the [ROS1 Bridge](https://github.com/ros2/ros1_bridge).

The detector must publish a [Detection](https://github.com/spindoxlabs/rexasi-tracker/blob/main/ros/rexasi_tracker_msgs/msg/Detections.msg) message to the `/detections` topic (see the [configuration](../configuration)). The content is:

```
std_msgs/Header header
int32 sensor_id
geometry_msgs/Pose[] centers
float32[] confidences
```