from people_tracker_interfaces.msg import RexString
import json
import os
import sys
import threading
import urllib.request
import platform

import numpy as np
import rclpy
import torch
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


sys.path.append(os.getcwd())
from people_tracker.utils.dto import SensorData
from people_tracker.utils.misc import save_frame, get_timestamp
from people_tracker.config.topics.src.pose_estimation import rgb_topics
from people_tracker.config.parameters.src.pose_estimation import pose_parameters
from people_tracker.utils.buffer import BlockingDeque


class PoseEstimation(Node):
    def __init__(self):
        super().__init__("POSE", automatically_declare_parameters_from_overrides=True)
        # Set device on cuda
        torch.cuda.set_device(pose_parameters["device_n"])

        self.isARM = platform.machine() == 'aarch64'

        # Load YOLO model
        self.model = self.get_model()

        self.n_cameras = (
            self.get_parameter("n_cameras").get_parameter_value().integer_value
        )
        self.camera_rgb_topics = self.get_parameter(
            "camera_rgb_topics").get_parameter_value().string_array_value

        self.debug = self.get_parameter("debug").get_parameter_value().bool_value

        self.frameToProcess: BlockingDeque = BlockingDeque(
            pose_parameters["buffer_dim"]
        )

        self.cameras = {}
        for cam_idx in range(0, self.n_cameras):
            # match cam_idx with id of the cameras (1 -> camera_1)
            cam_idx = cam_idx + 1

            self.cameras[cam_idx] = {}

            # define camera label
            camera_label = f"camera_{cam_idx}"
            self.get_logger().info(f"+++ Setting {camera_label} up +++")

            # declare subscription for camera_id
            subscriber_name = self.camera_rgb_topics[cam_idx -1]
            self.get_logger().info(
                f"Cam {cam_idx} is subscribing to: {subscriber_name}"
            )
            self.cameras[cam_idx]["subscription"] = self.create_subscription(
                Image, subscriber_name, self.handle_frame_event(cam_idx), 10
            )

            # declare publisher keypoints data for camera_id
            keypoints_publisher = f"{camera_label}/{rgb_topics['output']}"
            self.cameras[cam_idx]["keypoints_publisher"] = self.create_publisher(
                RexString, keypoints_publisher, 10
            )
            self.get_logger().info(
                f"Cam {cam_idx} is publishing to {keypoints_publisher}"
            )
            self.cameras[cam_idx]["frame_number"] = 0

        self.thread = threading.Thread(target=self.process).start()

    def handle_frame_event(self, cam_id: int):
        def frame_event(msg, cam_idx):

            # Converting image from topic to a cv2 image
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1
            )
            image: np.array = np.ascontiguousarray(image, dtype=np.uint8)
            # Convert bgr to rgb
            image = image[:, :, ::-1]

            # if debug is enabled, save frame
            if self.debug:
                save_frame(image, get_timestamp(msg), cam_idx)

            # update frame number
            self.cameras[cam_idx]["frame_number"] += 1  # todo: watch this

            if self.frameToProcess.append(
                [
                    image,
                    self.cameras[cam_idx]["frame_number"],
                    cam_idx,
                    get_timestamp(msg),
                    msg.header.stamp
                ]
            ):
                self.get_logger().debug(
                    "The buffer is full, discarging the oldest frame"
                )

        return lambda msg: frame_event(msg, cam_id)

    def process(self):

        while True:

            (
                imageToProcess,
                frameNumber,
                cam_idx,
                timestamp,
                msg_ts
            ) = self.frameToProcess.popleft()
            pose = self.get_pose(imageToProcess)

            if not pose:
                pose = []
                centers = []
            else:
                # get centers for each pose
                centers = self.get_centers(pose)

            topicData = SensorData(
                **{
                    "frame_number": frameNumber,
                    "sensor_type": "rgb",
                    "idx": cam_idx,
                    "detections": {"pixels": pose},
                    "timestamp": timestamp,
                    "centers": {"pixels": centers},
                }
            )

            # publish keypoints
            self.publish_results(topicData, msg_ts)

    def get_pose(self, image: np.array):
        # Apply pose detection to get keypoints
        results = self.model(image, verbose=False)

        # get pose estimation
        pose = results[0].keypoints.xy.cpu().numpy().tolist()

        # if no detections, return
        if len(pose) == 0 or len(pose[0]) == 0:
            self.get_logger().info("No people detected")
            return
        else:
            self.get_logger().info(f"Detected {len(pose)} people")

        # get confidence of pose estimation
        conf = np.array(results[0].keypoints.conf.cpu())

        # Add confidence to keypoints
        for pidx, person in enumerate(pose):
            for kidx, keypoint in enumerate(person):
                keypoint.append(float(conf[pidx][kidx]))
        return pose

    def get_model(self):
        if self.isARM:
            path = f"{pose_parameters['model_path']}/{pose_parameters['model_name']}.engine"
        else:
            path = f"{pose_parameters['model_path']}/{pose_parameters['model_name']}.pt"
        self.get_logger().info("Loading model from: %s" % path)
        if not os.path.exists(path):
            self.download_and_convert_model()
        model = YOLO(path) #.to(device=torch.device("cuda" if torch.cuda.is_available() else "cpu"))
        self.get_logger().info("Done loading model.")
        return model
    
    def download_and_convert_model(self):
        self.get_logger().info("Downloading and converting model to tensor RT")
        os.makedirs(pose_parameters["model_path"], exist_ok=True)
        path = f"{pose_parameters['model_path']}/{pose_parameters['model_name']}.pt"
        if not os.path.exists(path):
            self.get_logger().info(
                "Downloading..."
            )
            urllib.request.urlretrieve(pose_parameters["model_url"], path)
        if self.isARM:
            self.get_logger().info(
                    "Converting..."
                )
            pt_model = YOLO(path)
            pt_model.export(format="engine")

    def get_centers(self, pose):
        centers = []
        for detection in pose:
            person = np.array(np.array(detection))
            if not np.isnan(person).any():
                x, y, _ = np.nanmean(person, axis=0).astype(int)
                centers.append((x, y))
            else:
                self.get_logger().warning(f"Cannot generate pixel centers {person}")
                return
        return centers

    def publish_results(self, topic_data, msg_ts):
        msg = RexString()
        msg.header.stamp = msg_ts
        msg.data = json.dumps(topic_data.model_dump())
        self.cameras[topic_data.idx]["keypoints_publisher"].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pose_estimation = PoseEstimation()
    rclpy.spin(pose_estimation)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose_estimation.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
