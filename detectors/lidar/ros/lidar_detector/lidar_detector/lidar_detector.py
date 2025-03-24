import os
import sys
from typing import List
import torch
import numpy as np
import rclpy
from dr_spaam.detector import Detector
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())
from rexasi_tracker_msgs.msg import Detections

TF_DISCARD_TH_SEC=5.0 #sec

class DetectorNode(Node):
    def __init__(self):
        super().__init__("LIDAR", automatically_declare_parameters_from_overrides=True)

        # load parameters
        self.debug = self.get_parameter("debug").get_parameter_value().bool_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.lidar_frame_id = self.get_parameter("lidar_frame_id").get_parameter_value().string_value
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

    def get_timestamp_from_msgstamp(self, stamp) -> int:
        return stamp.sec + stamp.nanosec * pow(10, -9)

    def look_up(self, to_frame_rel, from_frame_rel, stamp):
        try:
            t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
            diff = abs(self.get_timestamp_from_msgstamp(t.header.stamp) - self.get_timestamp_from_msgstamp(stamp)) 
            if diff > TF_DISCARD_TH_SEC:
                self.get_logger().info("Discarding transformation older than 1 sec: abs(%.2f - %.2f) > %f" 
                                       % (self.get_timestamp_from_msgstamp(t.header.stamp),self.get_timestamp_from_msgstamp(stamp),TF_DISCARD_TH_SEC))  
                return None
            return t
        except TransformException as ex:
            self.get_logger().error(
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        return None


    def update_coordinates(self, detections: np.array, stamp):
        transform = self.look_up(self.frame_id, self.lidar_frame_id, stamp)
        
        if transform is None:
            return []

        for detection in detections:
            point = PointStamped()
            point.point.x = float(detection[0])
            point.point.y = float(detection[1])
            point.point.z = 0.0
        
            point_transformed = tf2_geometry_msgs.do_transform_point(point, transform)
            detection[0] = point_transformed.point.x
            detection[1] = point_transformed.point.y

        return detections

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
        centers = self.update_coordinates(points, msg.header.stamp)

        self.publish_detections(centers, confidences, msg.header.stamp)

    def center_to_pose_msg(self, center: tuple):
            pose = Pose()
            pose.position.x = float(center[0])
            pose.position.y = float(center[1])
            return pose

    def array_to_poses(self, list: np.array):
        poses = []
        for c in list:
            poses.append(self.center_to_pose_msg(c))
        return poses

    def publish_detections(self, centers, confidences, stamp):
        msg = Detections()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = stamp
        msg.sensor_id = self.sensor_id
        msg.centers = self.array_to_poses(centers)
        msg.confidences = [float(c) for c in confidences]
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
