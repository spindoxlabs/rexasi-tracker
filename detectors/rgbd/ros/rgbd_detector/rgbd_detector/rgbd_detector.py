import json
import os
import queue
import struct
import sys

from cv_bridge import CvBridge
import numpy as np
import rclpy
from PIL import Image as PilImage
from PIL import ImageDraw, ImageFont
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from message_filters import ApproximateTimeSynchronizer, Subscriber

sys.path.append(os.getcwd())
from people_tracker_interfaces.msg import RexString
from people_tracker.utils.dto import SensorData
from people_tracker.utils.buffer import BlockingDeque
from people_tracker.utils.misc import save_frame
from people_tracker.config.topics.src.rgbd import rgbd_topics
from people_tracker.config.parameters.src.rgbd import rgbd_parameters
from people_tracker.config.parameters.src.general import cameras_parameters
from people_tracker.config.parameters.src.camera import FPS, FRAME_WIDTH, FRAME_HEIGHT
from people_tracker.config.parameters.src.plotter import plotter_parameters
from people_tracker.libs.pyrealsense_wrapper import PyrealSenseWrapper, init_camera_intrinsics


class RGBD(Node):
    def __init__(self):
        super().__init__("RGBD", automatically_declare_parameters_from_overrides=True)

        # load parameters
        self.debug = self.get_parameter("debug").get_parameter_value().bool_value

        # Define number of cameras
        self.n_cameras = (
            self.get_parameter("n_cameras").get_parameter_value().integer_value
        )
        self.is_rotated = (
            self.get_parameter("is_rotated").get_parameter_value().integer_value
        )

        self.debug_rgbd = (
            True
            if "rgbd"
            in self.get_parameter("debug_modules")
            .get_parameter_value()
            .string_array_value
            else False
        )
        self.get_logger().info(f"Debug RGBD: {self.debug_rgbd}")

        self.camera_depth_topics = self.get_parameter(
            "camera_depth_topics").get_parameter_value().string_array_value
        self.camera_info_topics = self.get_parameter(
            "camera_info_topics").get_parameter_value().string_array_value
        # Set camera bridge
        self.bridge = CvBridge()

        self.pyrealsense_wrapper = PyrealSenseWrapper(self)


        self.cameras = {}
        for cam_idx in range(0, self.n_cameras):
            # match cam_idx with id of the cameras (1 -> camera_1)
            cam_idx = cam_idx + 1

            self.cameras[cam_idx] = {}

            camera_label = f"camera_{cam_idx}"
            self.get_logger().info(f"+++ Setting {camera_label} up +++")

            keypoints_subscriber = f"{camera_label}/{rgbd_topics['input']}"
            self.get_logger().info(
                f"Cam {cam_idx} is subscribing to: {keypoints_subscriber}"
            )

            rgbd_subscriber = self.camera_depth_topics[cam_idx-1]
            self.get_logger().info(
                f"Cam {cam_idx} is subscribing to: {rgbd_subscriber}"
            )

            self.cameras[cam_idx]["tss"] = ApproximateTimeSynchronizer([Subscriber(self, Image, rgbd_subscriber),
                                          Subscriber(self, RexString, keypoints_subscriber)], 30, 1/FPS)
            self.cameras[cam_idx]["tss"].registerCallback(self.handle_camera_data(cam_idx))

            camera_info_subscriber = self.camera_info_topics[cam_idx-1]
            self.get_logger().info(
                f"Cam {cam_idx} is subscribing to {camera_info_subscriber}"
            )

            self.cameras[cam_idx]["camera_info_subscriber"] = self.create_subscription(
                CameraInfo, camera_info_subscriber, self.handle_camera_info(cam_idx), 10
            )

            # Define position publisher, where data will be published:
            # for sensor fusion is better to have a single topic with all the data from the cameras,
            # For inizialize multiple publisher, switch to f"{camera_id}/{OUTPUT_TOPIC}"
            position_publisher = f"{rgbd_topics['output']}"
            self.cameras[cam_idx]["position_publisher"] = self.create_publisher(
                RexString, position_publisher, 10
            )
            self.get_logger().info(
                f"Cam {cam_idx} is publishing to {position_publisher}"
            )

            camera_info_pub = f"{camera_label}/color/camera_info"
            self.cameras[cam_idx]["camera_info_pub"] = self.create_publisher(
                CameraInfo, camera_info_pub, 10
            )

            self.cameras[cam_idx]["rgbd_image_buffer"] = BlockingDeque(
                rgbd_parameters["image_buffer_len"]
            )

            self.cameras[cam_idx]["keypoints_buffer"] = queue.Queue()

            self.cameras[cam_idx]["camera_intrinsics"] = None
            self.cameras[cam_idx]["topic_data"] = []

            self.cameras[cam_idx]["stream_open"]: bool = True

            self.cameras[cam_idx]["translation"] = cameras_parameters[cam_idx][
                "translation"
            ]
            self.cameras[cam_idx]["rotation"] = cameras_parameters[cam_idx]["rotation"]


    def handle_camera_info(self, cam_idx: int):
        def camera_info_callback(msg, cam_idx):
            self.cameras[cam_idx]["camera_intrinsics"] = init_camera_intrinsics(msg)
            self.destroy_subscription(self.cameras[cam_idx]["camera_info_subscriber"])
            # msg.header.frame_id = "camera_1"
            # self.cameras[cam_idx]["camera_info_pub"].publish(msg)


        return lambda msg: camera_info_callback(msg, cam_idx)

    def handle_camera_data(self, cam_idx: int):
        def camera_data_callback(image_msg, keypoint_msg, cam_idx):
            self.process_frame(image_msg, keypoint_msg, cam_idx)

        return lambda image_msg, keypoint_msg: camera_data_callback(image_msg, keypoint_msg, cam_idx)
    

    def process_frame(self, image_msg: Image, keypoint_msg: RexString, cam_idx: int):
        self.get_logger().debug(f"Process cam {cam_idx} data")

        if self.cameras[cam_idx]["camera_intrinsics"] == None:
            self.get_logger().warn(f"Cam {cam_idx}: waiting intrinsics !!!")
            return

        if rgbd_parameters["use_bridge"]:
            rgbd_image: np.ndarray = self.bridge.imgmsg_to_cv2(image_msg, "16UC1")
        else:
            data_array = struct.unpack("<%dH" % (len(image_msg.data) // 2), image_msg.data)
            rgbd_image: np.ndarray = np.array(data_array, dtype=np.uint16).reshape(
                (image_msg.height, image_msg.width)
            )
            rgbd_image: np.ndarray = np.ascontiguousarray(rgbd_image, dtype=np.uint16)

        sensor_data = SensorData(**json.loads(keypoint_msg.data))

        try:
            # extract pose keypoints as coordinates
            keypoints, confidences = self.compute_keypoints(
                cam_idx=cam_idx,
                people=sensor_data.detections.pixels,
                rgbd_image=rgbd_image,
                stamp=image_msg.header.stamp
            )

            self.get_logger().debug(f"Keypoints: {keypoints}")

            # from pose keypoints, extract centers
            centers: list = self.get_centers(keypoints)

            sensor_data.detections.coordinates = keypoints
            sensor_data.centers.coordinates = centers
            sensor_data.sensor_type = "rgbd"
            sensor_data.confidence = confidences

            self.publish(sensor_data, cam_idx, image_msg.header.stamp)
        except Exception as e:
            self.get_logger().error(
                f"Error processing data from camera {cam_idx}: {e}"
            )
            return

        if self.debug_rgbd:
            self.get_logger().info(
                f"timestamp RGBD: {sensor_data.timestamp}"
            )
            self.get_logger().info(
                f"centers RGBD: {sensor_data.centers.coordinates}"
            )

            if self.get_parameter("debug").get_parameter_value().bool_value:
                # draw skeleton
                rgb_image_to_print: np.ndarray = self.draw_skeleton(
                    people=sensor_data.detections.pixels,
                    r_point=3,
                    centers=centers,
                    image=rgbd_image,
                )

                save_frame(
                    rgb_image_to_print,
                    sensor_data.timestamp,
                    cam_idx,
                    folder="keypoints",
                )
   
    def compute_keypoints(self, cam_idx: int, people: list, rgbd_image: np.ndarray, stamp = None):
        people_keypoints = []
        confidences = []
        for person in people:
            detection = np.array(person)
            confidence_mask = detection[:,2] >= rgbd_parameters["min_pose_confidence_score"]
            detection_depth = np.array([
                rgbd_image[
                    (
                        int(y) if y != FRAME_HEIGHT else FRAME_HEIGHT - 1,
                        int(x) if x != FRAME_WIDTH else FRAME_WIDTH - 1,
                    )
                ]
                for x, y, _ in detection
            ])
            mean_detection_depth = np.mean(detection_depth[confidence_mask])
            depth_mask = abs(detection_depth - mean_detection_depth) > rgbd_parameters["skip_depth_range"]
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
                        cam_idx,
                        x,
                        y,
                        rgbd_image[y, x],
                        intrinsics=self.cameras[cam_idx]["camera_intrinsics"],
                        is_rotated=self.is_rotated,
                        stamp=stamp
                    )
                except Exception as e:
                    self.get_logger().error(
                        f"Error transform data for camera {cam_idx}: {e}"
                    )
                    poses.append([np.nan, np.nan, np.nan])
                    continue
                if cc_x == 0.0 and cc_y == 0.0 and cc_z == 0.0:
                    poses.append([np.nan, np.nan, np.nan])
                else:
                    poses.append([cc_x, cc_y, cc_z])
                    confidence.append(conf) # normalize with number of keypoints?

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

    def publish(self, sensor_data: SensorData, cam_idx: int, stamp):
        data = json.dumps(sensor_data.model_dump())
        self.get_logger().debug("Publishing \n%s" % data)
        msg = RexString()
        msg.header.stamp = stamp
        msg.data = data
        self.cameras[cam_idx]["position_publisher"].publish(msg)

    def draw_skeleton(self, people, r_point, image, centers) -> np.ndarray:

        image = PilImage.fromarray(image)
        draw = ImageDraw.Draw(image)
        font_path = os.path.join(os.getcwd(), "qt", "fonts", "DejaVuSans.ttf")
        font = ImageFont.truetype(font_path, size=16)

        # for person, center in zip(people, centers):
        for person in people:
            center = []
            color = 255

            # draw keypoints
            for keypoint in person:
                top_left = (keypoint[0] - r_point, keypoint[1] - r_point)
                bottom_right = (keypoint[0] + r_point, keypoint[1] + r_point)
                draw.ellipse(xy=[top_left, bottom_right], fill=color)

            # draw joints
            for n, connection in enumerate(plotter_parameters["joint_connections"]):
                if person[connection[0]][2] < 0.5 or person[connection[1]][2] < 0.5:
                    self.get_logger().debug("confidence < 0.5")
                    continue
                joint1 = person[connection[0]]
                joint2 = person[connection[1]]
                if n == 4:
                    # draw position
                    draw.text(
                        tuple(np.array(joint2[:2]).astype(int)),
                        str([round(x, 2) for x in center]),
                        align="left",
                        color=color,
                        font=font,
                    )
                # draw joint
                draw.line(
                    xy=(joint1[0], joint1[1], joint2[0], joint2[1]),
                    fill=color,
                    width=2,
                )
        return np.array(image)


def main(args=None):
    rclpy.init(args=args)
    refsystem = RGBD()

    try:
        rclpy.spin(refsystem)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    refsystem.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
