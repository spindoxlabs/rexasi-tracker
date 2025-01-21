import os
import sys
from typing import List
import torch
import numpy as np
import rclpy
from dr_spaam.detector import Detector
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())
from rexasi_tracker_msgs.msg import Detections

class DetectorNode(Node):
    def __init__(self):
        super().__init__("LIDAR", automatically_declare_parameters_from_overrides=True)

        # load parameters
        self.debug = self.get_parameter("debug").get_parameter_value().bool_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.sensor_id = self.get_parameter("sensor_id").get_parameter_value().integer_value
        self.lidar_topic = self.get_parameter("lidar_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.model_ckpt_file = self.get_parameter("model_ckpt_file").get_parameter_value().string_value
        self.model_model = self.get_parameter("model_model").get_parameter_value().string_value
        self.model_stride = self.get_parameter("model_stride").get_parameter_value().integer_value
        self.model_panoramic_scan = self.get_parameter("model_panoramic_scan").get_parameter_value().bool_value
        self.conf_tresh = self.get_parameter("conf_tresh").get_parameter_value().integer_value
        self.laser_fov = self.get_parameter("laser_fov").get_parameter_value().integer_value
        self.placeholder_value = self.get_parameter("placeholder_value").get_parameter_value().integer_value
        self.scan_rate = self.get_parameter("scan_rate").get_parameter_value().integer_value
        self.angle_increment = self.get_parameter("angle_increment").get_parameter_value().integer_value
        self.ranges = self.get_parameter("ranges").get_parameter_value().integer_value
        self.calibration_file = self.get_parameter("calibration_file").get_parameter_value().string_value
        self.shear = self.get_parameter("shear").get_parameter_value().bool_value
        self.scale = self.get_parameter("scale").get_parameter_value().bool_value
        self.perim_length = self.get_parameter("perim_length").get_parameter_value().integer_value
        self.skip_n_frames = self.get_parameter("skip_n_frames").get_parameter_value().integer_value
        self.is_rotated = self.get_parameter("is_rotated").get_parameter_value().bool_value

        self.skipped = 0

        self.gpu_available = torch.cuda.is_available()

        self.get_logger().info("Using %s" % ("GPU" if self.gpu_available else "CPU"))
        self.get_logger().info("Loading model from: %s" % str(self.model_ckpt_file))

        self._detector = Detector(self.model_ckpt_file, self.model_model, self.gpu_available, self.model_stride,self.model_panoramic_scan)
        self._detector.set_laser_fov(self.laser_fov)

        self.get_logger().info(f"Subscribing to {self.lidar_topic}")
        self.subscription = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self._scan_callback,
            self.scan_rate,
        )

        self.get_logger().info(f"Publishing to {self.output_topic}")
        self.detections_publisher = self.create_publisher(
            Detections, self.output_topic, 10
        )

        # TODO check the computation here
        if not self._detector.is_ready():
            self._detector.set_laser_fov(
                np.rad2deg(
                    self.angle_increment * self.ranges
                )
            )

    def transform(self, cc: np.array):
        """
        Apply rotation and translation in order to calibrate the points.
        """
        return (np.dot(self.rotation, cc.T) + self.translation).T

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

        return self.transform(cc)

    def _scan_callback(self, msg):

        if self.skip_n_frames > 0:
            if self.skipped == self.skip_n_frames:
                self.skipped = 0
            else:
                self.skipped += 1
                return

        scan = np.array(msg.ranges)

        # replace zeros, inf and nan with placeholder value
        scan[scan == 0.0] = self.placeholder_value
        scan[np.isinf(scan)] = self.placeholder_value
        scan[np.isnan(scan)] = self.placeholder_value

        points, confidences, _ = self._detector(scan)

        # get points with confidence score > CONF_THRESH
        conf_mask = (confidences >= self.conf_tresh).reshape(-1)
        points = points[conf_mask]

        self.get_logger().info(f"Detected {len(points)} people")

        # convert to lidar_data
        centers = self.update_coordinates(points)

        self.publish_detections(centers, confidences, msg.header.stamp)

    def arrayToPoses(self, list: List):
        poses = []
        for c in list:
            poses.append(self.center_to_pose_msg(c))
        return poses

    def publish_detections(self, centers, confidences, stamp):
        msg = Detections()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = stamp
        msg.sensor_id = self.sensor_id
        msg.centers = self.arrayToPoses(centers)
        msg.confidences = confidences
        self.detections_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    detector_node = DetectorNode()
    rclpy.spin(detector_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    detector_node.destroy_node()
    rclpy.shutdown()
