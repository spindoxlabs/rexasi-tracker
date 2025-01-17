import json
import os
import sys

import numpy as np
import rclpy
from dr_spaam.detector import Detector
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())
from people_tracker.utils.dto import SensorData
from people_tracker.utils.misc import get_timestamp
from people_tracker.config.topics.src.lidar import lidar_topics
from people_tracker.config.parameters.src.lidar import lidar_parameters
from people_tracker.libs.transformation_functions import calibration_functions
from people_tracker_interfaces.msg import RexString

class DetectorNode(Node):
    def __init__(self):
        super().__init__("LIDAR", automatically_declare_parameters_from_overrides=True)


        self.debug = (
            True
            if "lidar"
            in self.get_parameter("debug_modules")
            .get_parameter_value()
            .string_array_value
            else False
        )

        self.lidar_topic = self.get_parameter(
            "lidar_topic").get_parameter_value().string_value

        self.is_rotated = (
            self.get_parameter("is_rotated").get_parameter_value().bool_value
        )

        self.skipped = 0
        self.skip_n_frames = lidar_parameters["skip_n_frames"]

        self._detector = Detector(**lidar_parameters["model_params"])
        self._detector.set_laser_fov(lidar_parameters["laser_fov"])

        self.get_logger().info(f"Subscribing to {self.lidar_topic}")
        self.subscription = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self._scan_callback,
            lidar_parameters["scan_rate"],
        )

        self.get_logger().info(f"Publishing to {lidar_topics['output']}")
        self.detections_publisher = self.create_publisher(
            String, lidar_topics["output"], 10
        )

        self.transformation_function = calibration_functions["lidar"]["encode"]
        self.frame_number: int = 0

        # TODO check the computation here
        if not self._detector.is_ready():
            self._detector.set_laser_fov(
                np.rad2deg(
                    lidar_parameters["angle_increment"] * lidar_parameters["ranges"]
                )
            )

    def update_coordinates(self, cc: np.array):
        """
        This function is used to update coordinates based on lidar configuration.
        In this specific case, the lidar is put upside down, so the coordinates must be parsed accordingly.
        """

        # switch coordinates (since the lidar is mirrored) -> x,y = y,x
        cc = cc[:, ::-1]

        # if the lidar is not rotated, set negative x (which track the position of the detection) -> x,y = -x,y
        if not self.is_rotated:
            cc[:, 0] = -cc[:, 0]

        return self.transformation_function(cc)

    def _scan_callback(self, msg):

        if self.skip_n_frames > 0:
            if self.skipped == self.skip_n_frames:
                self.skipped = 0
            else:
                self.skipped += 1
                return

        scan = np.array(msg.ranges)

        # replace zeros, inf and nan with placeholder value
        scan[scan == 0.0] = lidar_parameters["placeholder_value"]
        scan[np.isinf(scan)] = lidar_parameters["placeholder_value"]
        scan[np.isnan(scan)] = lidar_parameters["placeholder_value"]

        points, confidences, _ = self._detector(scan)

        # get points with confidence score > CONF_THRESH
        conf_mask = (confidences >= lidar_parameters["conf_tresh"]).reshape(-1)
        points = points[conf_mask]

        self.get_logger().info(f"Detected {len(points)} people")

        # convert to lidar_data
        coordinates = self.update_coordinates(points)
        lidar_data = SensorData(
            **{
                "frame_number": self.frame_number,
                "idx": lidar_parameters["id"],
                "sensor_type": lidar_parameters["label"],
                "timestamp": get_timestamp(msg=msg),
                "centers": {"coordinates": coordinates},
                "confidence": confidences
            }
        )

        self.publish_detections(lidar_data. msg.header.stamp)

    def publish_detections(self, topic_data, stamp):
        msg = RexString()
        msg.header.stamp = stamp
        msg.data = json.dumps(topic_data.model_dump())
        self.detections_publisher.publish(msg)
        self.frame_number += 1


def main(args=None):
    rclpy.init(args=args)
    detector_node = DetectorNode()
    rclpy.spin(detector_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
