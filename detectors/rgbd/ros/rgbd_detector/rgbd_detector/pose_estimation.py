import os
import urllib.request
import platform

import numpy as np
import rclpy
import torch
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from geometry_msgs.msg import PoseArray, Pose

from rgbd_detector_msgs.msg import Persons

YOLO_MODELS_BASE_URL= "https://github.com/ultralytics/assets/releases/download/v0.0.0/"

class PoseEstimation(Node):
    def __init__(self):
        super().__init__("POSE", automatically_declare_parameters_from_overrides=True)

        # load parameters
        self.debug = self.get_parameter("debug").get_parameter_value().bool_value
        self.yolo_model = self.get_parameter("yolo_model").get_parameter_value().string_value
        self.color_topic = self.get_parameter("rgbd_color_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        torch.cuda.device(device)

        self.isARM = platform.machine() == 'aarch64'

        # Load YOLO model
        self.model = self.get_model(self.yolo_model)

        self.get_logger().info(
            f"Subscribed to {self.color_topic}"
        )
        self.subscriber = self.create_subscription(
            Image, self.color_topic, self.handle_frame_event, 10
        )

        self.get_logger().info(
                f"Publishing to {self.output_topic}"
            )
        self.publisher = self.create_publisher(
                Persons, self.output_topic, 10
            )

    def handle_frame_event(self, msg):

        # Converting image from topic to a cv2 image
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1
        )
        image: np.array = np.ascontiguousarray(image, dtype=np.uint8)
        # Convert bgr to rgb
        image = image[:, :, ::-1]

        self.process(image)

    def handle_frame_event(self, msg):

         # Converting image from topic to a cv2 image
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1
        )
        image: np.array = np.ascontiguousarray(image, dtype=np.uint8)
        # Convert bgr to rgb
        image = image[:, :, ::-1]

        poses = self.get_poses(image)

        if len(poses) == 0:
            self.get_logger().info("No people detected")
            return
        else:
            self.get_logger().info(f"Detected {len(poses)} people")

        # publish keypoints
        self.publish_results(poses, msg.header.stamp)

    def get_poses(self, image: np.array):
        # Apply pose detection to get keypoints
        results = self.model(image, verbose=False)

        # get pose estimation
        pose = results[0].keypoints.xy.cpu().numpy().tolist()

        # if no detections, return
        if len(pose) == 0 or len(pose[0]) == 0:
            return []

        # get confidence of pose estimation
        conf = np.array(results[0].keypoints.conf.cpu())

        # Add confidence to keypoints
        for pidx, person in enumerate(pose):
            for kidx, keypoint in enumerate(person):
                keypoint.append(float(conf[pidx][kidx]))
        return pose

    def get_model(self, model_name):
        path = f"/data/{model_name}"
        if self.isARM:
            model_path = f"{path}.engine"
        else:
            model_path = f"{path}.pt"
        self.get_logger().info("Loading model from: %s" % model_path)
        if not os.path.exists(model_path):
            self.download_and_convert_model(model_name)
        model = YOLO(model_path)
        self.get_logger().info("Done loading model.")
        return model
    
    def download_and_convert_model(self, model_name):
        path = f"/data/{model_name}"
        if not os.path.exists(path):
            self.get_logger().info(
                "Downloading..."
            )
            urllib.request.urlretrieve(f"{YOLO_MODELS_BASE_URL}{model_name}.pt", path)
        if self.isARM:
            self.get_logger().info(
                    "Converting..."
                )
            pt_model = YOLO(path)
            pt_model.export(format="engine")

    def keypoint_to_pose_msg(self, keypoint):
        msg = Pose()
        msg.position.x = float(keypoint[0])
        msg.position.y = float(keypoint[1])
        msg.position.z = float(keypoint[2]) # confidence
        return msg

    def publish_results(self, persons, msg_stamp):
        persons_msg = Persons()
        persons_msg.header.stamp = msg_stamp
        for p in persons:
            keypoint_msg = PoseArray()
            for keypoint in p:
                keypoint_msg.poses.append(self.keypoint_to_pose_msg(keypoint))
            persons_msg.persons.append(keypoint_msg)
        self.publisher.publish(persons_msg)


def main(args=None):
    rclpy.init(args=args)
    pose_estimation = PoseEstimation()
    rclpy.spin(pose_estimation)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose_estimation.destroy_node()
    rclpy.shutdown()
