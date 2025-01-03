import os
import logging
from ament_index_python.packages import get_package_share_directory

logger = logging.getLogger("launch")
CAMERA_N = os.environ.get("CAMERA_N", "2")
DEBUG = os.environ.get("DEBUG", "False").lower()
if DEBUG == "false":
    DEBUG = False
else:
    DEBUG = True
CAMERA_FR_SERIAL_NUMBER = os.environ.get("CAMERA_FR_SERIAL_NUMBER", "'313522303246'")
CAMERA_FL_SERIAL_NUMBER = os.environ.get("CAMERA_FL_SERIAL_NUMBER", "'318122301365'")
CAMERA_1_RGB_TOPIC = os.environ.get(
    "CAMERA_1_RGB_TOPIC", "/wheelchair/RGBDCamera_right/rgb"
)
CAMERA_2_RGB_TOPIC = os.environ.get(
    "CAMERA_2_RGB_TOPIC", "/wheelchair/RGBDCamera_left/rgb"
)
CAMERA_1_DEPTH_TOPIC = os.environ.get(
    "CAMERA_1_DEPTH_TOPIC", "/wheelchair/RGBDCamera_right/depth"
)
CAMERA_2_DEPTH_TOPIC = os.environ.get(
    "CAMERA_2_DEPTH_TOPIC", "/wheelchair/RGBDCamera_left/depth"
)
CAMERA_1_INFO_TOPIC = os.environ.get(
    "CAMERA_1_INFO_TOPIC", "/wheelchair/RGBDCamera_right/camera_info"
)
CAMERA_2_INFO_TOPIC = os.environ.get(
    "CAMERA_2_INFO_TOPIC", "/wheelchair/RGBDCamera_left/camera_info"
)
CAMERA_1_FRAME_ID=os.environ.get(
    "CAMERA_1_FRAME_ID", "front_right_camera_color_frame"
)
CAMERA_2_FRAME_ID=os.environ.get(
    "CAMERA_2_FRAME_ID", "front_left_camera_color_frame"
)
CAMERA_1_OPTICAL_FRAME_ID=os.environ.get(
    "CAMERA_1_OPTICAL_FRAME_ID", "front_right_camera_color_optical_frame"
)
CAMERA_2_OPTICAL_FRAME_ID=os.environ.get(
    "CAMERA_2_OPTICAL_FRAME_ID", "front_right_camera_color_optical_frame"
)
WHEELCHAIR_FRAME_ID=os.environ.get(
    "WHEELCHAIR_FRAME_ID", "base_link"
)
ODOMETRY_FRAME_ID=os.environ.get(
    "ODOMETRY_FRAME_ID", "odom"
)
X_FORWARD=os.environ.get(
    "X_FORWARD", "true"
)

LIDAR_TOPIC = os.environ.get("LIDAR_TOPIC", "/wheelchair/LiDAR/laser_scan")
if CAMERA_N == "1":
    PARAMS = {
        "n_cameras": 1,
        "camera_rgb_topics": [CAMERA_1_RGB_TOPIC],
        "camera_depth_topics": [CAMERA_1_DEPTH_TOPIC],
        "camera_info_topics": [CAMERA_1_INFO_TOPIC],
        "lidar_topic": LIDAR_TOPIC,
        "debug": DEBUG,
        "debug_modules": [""],
        "camera_frame_id": [CAMERA_1_FRAME_ID],
        "camera_optical_frame_id": [CAMERA_1_OPTICAL_FRAME_ID],
        "lidar_frame_id": "lidar",
        "wheelchair_frame_id": WHEELCHAIR_FRAME_ID,
        "odometry_frame_id": ODOMETRY_FRAME_ID,
        "x_forward": True if X_FORWARD == "true" else False
    }
else:

    PARAMS = {
        "n_cameras": 2,
        "camera_rgb_topics": [CAMERA_1_RGB_TOPIC, CAMERA_2_RGB_TOPIC],
        "camera_depth_topics": [CAMERA_1_DEPTH_TOPIC, CAMERA_2_DEPTH_TOPIC],
        "camera_info_topics": [CAMERA_1_INFO_TOPIC, CAMERA_2_INFO_TOPIC],
        "lidar_topic": LIDAR_TOPIC,
        "debug": DEBUG,
        "debug_modules": [""],
        "camera_frame_id": [CAMERA_1_FRAME_ID, CAMERA_2_FRAME_ID],
        "camera_optical_frame_id": [CAMERA_1_OPTICAL_FRAME_ID, CAMERA_2_OPTICAL_FRAME_ID],
        "lidar_frame_id": "lidar",
        "wheelchair_frame_id": WHEELCHAIR_FRAME_ID,
        "odometry_frame_id": ODOMETRY_FRAME_ID,
        "x_forward": True if X_FORWARD == "true" else False
    }

SENSOR_SPECIFIC_PARAMS = {"is_rotated": True}
LIDAR_ARGUMENTS = [
    os.path.join(
        get_package_share_directory("sick_scan_xd"), "launch/sick_tim_7xx.launch"
    ),
    "hostname:=192.168.1.10",
]
REALSENSE_CONFIG = "/ros_ws/people_tracker/config/realsense_params.json"
