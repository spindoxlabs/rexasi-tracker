import os
import struct
import sys
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import Pose

from rgbd_detector_msgs.msg import Persons
from rexasi_tracker_msgs.msg import Detections

from libs.pyrealsense_wrapper import PyRealSenseWrapper, init_camera_intrinsics

FRAME_HEIGHT: int = 480
FRAME_WIDTH: int = 848

class RGBD(Node):
    def __init__(self):
        super().__init__("RGBD", automatically_declare_parameters_from_overrides=True)

        # load parameters
        self.debug = self.get_parameter("debug").get_parameter_value().bool_value
        self.is_rotated = self.get_parameter("is_rotated").get_parameter_value().bool_value
        self.keypoints_topic = self.get_parameter("keypoints_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.rgbd_depth_topic = self.get_parameter("rgbd_depth_topic").get_parameter_value().string_value
        self.rgbd_depth_camera_info_topic = self.get_parameter("rgbd_color_camera_info_topic").get_parameter_value().string_value
        self.min_pose_confidence_score = self.get_parameter("min_pose_confidence_score").get_parameter_value().double_value
        self.skip_depth_range = self.get_parameter("skip_depth_range").get_parameter_value().integer_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.optical_frame_id = self.get_parameter("optical_frame_id").get_parameter_value().string_value
        self.fps = self.get_parameter("fps").get_parameter_value().integer_value
        self.sensor_id = self.get_parameter("sensor_id").get_parameter_value().integer_value

        self.camera_intrinsics = None

        self.pyrealsense_wrapper = PyRealSenseWrapper(self, self.frame_id, self.optical_frame_id)

        self.get_logger().info(
            f"Subscribing to {self.rgbd_depth_camera_info_topic}"
        )
        self.sync = ApproximateTimeSynchronizer([Subscriber(self, Image, self.rgbd_depth_topic),
                                          Subscriber(self, Persons, self.keypoints_topic)], 30, 1/self.fps)
        self.sync.registerCallback(self.handle_input_data)

        self.get_logger().info(
            f"Subscribing to {self.rgbd_depth_camera_info_topic}"
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.rgbd_depth_camera_info_topic, self.handle_camera_info, 10
        )

        self.get_logger().info(
            f"Publishing to {self.output_topic}"
        )
        self.publisher = self.create_publisher(
                Detections, self.output_topic, 10
            )

    def handle_camera_info(self, msg):
            self.camera_intrinsics = init_camera_intrinsics(msg)
            self.destroy_subscription(self.camera_info_sub)

    def person_msg_to_list(self, keypoint_msg: Persons):
        poses = []
        for p in keypoint_msg:
            keypoints = []
            for k in p.poses:
                keypoints.append([k.position.x, k.position.y, k.position.z])
            poses.append(keypoints)
        return poses

    def handle_input_data(self, image_msg: Image, keypoint_msg: Persons):

        if self.camera_intrinsics == None:
            self.get_logger().warn(f"Waiting intrinsics !!!")
            return

        data_array = struct.unpack("<%dH" % (len(image_msg.data) // 2), image_msg.data)
        rgbd_image: np.ndarray = np.array(data_array, dtype=np.uint16).reshape(
            (image_msg.height, image_msg.width)
        )
        rgbd_image: np.ndarray = np.ascontiguousarray(rgbd_image, dtype=np.uint16)

        try:
            # extract pose keypoints as coordinates
            keypoints, confidences = self.compute_keypoints(
                people=self.person_msg_to_list(keypoint_msg.persons),
                rgbd_image=rgbd_image,
                stamp=image_msg.header.stamp
            )

            self.get_logger().debug(f"Keypoints: {keypoints}")

            # from pose keypoints, extract centers
            centers: list = self.get_centers(keypoints)

            self.publish_detections(centers, confidences, image_msg.header.stamp)
        except Exception as e:
            self.get_logger().error(
                f"Error processing data: {e}"
            )
   
    def compute_keypoints(self, people: list, rgbd_image: np.ndarray, stamp = None):
        people_keypoints = []
        confidences = []
        for person in people:
            detection = np.array(person)
            confidence_mask = detection[:,2] >= self.min_pose_confidence_score
            detection_depth = np.array([
                rgbd_image[
                    (
                        int(y) if y != FRAME_HEIGHT else FRAME_HEIGHT - 1,
                        int(x) if x != FRAME_WIDTH else FRAME_WIDTH - 1,
                    )
                ]
                # rgbd_image[(int(x),int(y))]
                for x, y, _ in detection
            ])
            mean_detection_depth = np.mean(detection_depth[confidence_mask])
            depth_mask = abs(detection_depth - mean_detection_depth) > self.skip_depth_range
            # invalidate keypoint if low confidence or depth out of range
            detection[np.logical_not(confidence_mask)] = [np.nan, np.nan, np.nan]
            detection[depth_mask] = [np.nan, np.nan, np.nan]
            poses = []
            confidence = []
            for x, y, conf in detection:

                if np.isnan(x) or np.isnan(y):
                    self.get_logger().debug("Skip keypoint")
                    poses.append([np.nan, np.nan, np.nan])
                    continue
            
                y = int(y) if y != FRAME_HEIGHT else FRAME_HEIGHT - 1
                x = int(x) if x != FRAME_WIDTH else FRAME_WIDTH - 1
                
                try:
                    cc_x, cc_y, cc_z = self.pyrealsense_wrapper.px2cc(
                        x,
                        y,
                        rgbd_image[y, x],
                        intrinsics=self.camera_intrinsics,
                        is_rotated=self.is_rotated,
                        stamp=stamp
                    )
                except Exception as e:
                    self.get_logger().error(
                        f"Error projecting 2d point to 3d: {e}"
                    )
                    poses.append([np.nan, np.nan, np.nan])
                    continue
                if cc_x == 0.0 and cc_y == 0.0 and cc_z == 0.0:
                    poses.append([np.nan, np.nan, np.nan])
                else:
                    poses.append([cc_x, cc_y, cc_z])
                    confidence.append(conf)

            people_keypoints.append(poses)
            confidences.append(np.mean(confidence))

        # filter and return only keypoints that are valid
        valid_keypoints = [x for x in people_keypoints if not np.isnan(x).all()]
        return valid_keypoints, confidences

    def get_centers(self, people):
        centers = []
        for idx,p in enumerate(people):
            # Get mean position (person)
            x, y, _ = np.nanmean(p, axis=0)

            # If nan => skip current loop iteration
            if np.isnan(x).any() or np.isnan(y).any():
                self.get_logger().error(
                    f"Cannot detect center for coordinates ({x}-{y}) for {p}"
                )
                continue

            centers.append((x, y))

        return centers

    def center_to_pose_msg(self, center: tuple):
            pose = Pose()
            pose.position.x = float(center[0])
            pose.position.y = float(center[1])
            return pose

    def array_to_poses(self, list: List):
        poses = []
        for c in list:
            poses.append(self.center_to_pose_msg(c))
        return poses

    def publish_detections(self, centers, confidences, stamp):
        self.get_logger().info("Publishing %d detections" % len(centers))
        msg = Detections()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = stamp
        msg.sensor_id = self.sensor_id
        msg.centers = self.array_to_poses(centers)
        msg.confidences = confidences
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rgbd = RGBD()
    rclpy.spin(rgbd)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rgbd.destroy_node()
    rclpy.shutdown()

