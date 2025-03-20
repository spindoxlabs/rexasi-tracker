---
sidebar_position: 4
---

# Output

The tracker outputs a list of tracks on the `/tracks` topic (see [configuration](./configuration)), containing the following data:
```python
# rexasi_tracker_msgs/Tracks

std_msgs/Header header
# ID of the sensor associated to the last detection
int32 sensor_id
# the center of each detection
geometry_msgs/Pose[] centers
# the velocity vector of each detection
geometry_msgs/Twist[] velocities
# the confidence of each detection [0-1.0]
float32[] confidences
# the ID of each detection
int32[] identities
# the ID of past detections
int32[] dead_identities
```
Additionally, for debugging purposes, markers are published on the `debug/association` topic.
For each detection, an arrow marker is generated, indicating the detection's position and direction based on the velocity vector.