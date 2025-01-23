from launch_ros.actions import Node
from launch import LaunchDescription

pose_parameters = {
    "debug": "true",
    "rgbd_color_topic": "/camera_1/color/image_raw",
    "output_topic": "/keypoints",
    "yolo_model": "yolov8n-pose"
}

rgbd_parameters = {
    "debug": True,
    "sensor_id": 2,
    "frame_id": "world",
    "optical_frame_id": "color_optical_frame",
    "is_rotated": False,
    "rgbd_color_camera_info_topic": "/camera_1/color/camera_info",
    "rgbd_depth_topic": "/camera_1/depth/image_rect_raw",
    "rgbd_depth_camera_info_topic": "/camera_1/depth/camera_info",
    "output_topic": "/detections",
    "keypoints_topic": pose_parameters["output_topic"],
    "min_pose_confidence_score": 0.7,
    "skip_depth_range": 2000,
    "fps": 30
}


def generate_launch_description():

    pose_estimation= Node(
        package='rgbd_detector',
        executable='pose_estimation',
        name='pose_estimation',
        parameters=[{**pose_parameters}],
    )

    rgbd_detector= Node(
        package='rgbd_detector',
        executable='rgbd_detector',
        name='rgbd_detector',
        parameters=[{**rgbd_parameters}],
    )

    return LaunchDescription(
        [
            pose_estimation,
            rgbd_detector
        ]
    )
