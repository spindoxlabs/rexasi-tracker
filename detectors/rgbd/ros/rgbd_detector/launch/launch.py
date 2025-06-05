from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

pose_parameters = {
    "pose_debug": LaunchConfiguration("pose_debug"),
    "rgbd_color_topic": LaunchConfiguration("rgbd_color_topic"),
    "pose_output_topic": LaunchConfiguration("pose_output_topic"),
    "yolo_model": LaunchConfiguration("yolo_model")
}

rgbd_parameters = {
    "rgbd_debug": LaunchConfiguration("rgbd_debug"),
    "sensor_id": LaunchConfiguration("sensor_id"),
    "frame_id": LaunchConfiguration("frame_id"),
    "optical_frame_id": LaunchConfiguration("optical_frame_id"),
    "is_rotated": LaunchConfiguration("is_rotated"),
    "rgbd_color_camera_info_topic": LaunchConfiguration("rgbd_color_camera_info_topic"),
    "rgbd_depth_topic": LaunchConfiguration("rgbd_depth_topic"),
    "rgbd_depth_camera_info_topic": LaunchConfiguration("rgbd_depth_camera_info_topic"),
    "detection_output_topic": LaunchConfiguration("detection_output_topic"),
    "keypoints_topic": LaunchConfiguration("keypoints_topic"),
    "min_pose_confidence_score": LaunchConfiguration("min_pose_confidence_score"),
    "skip_depth_range": LaunchConfiguration("skip_depth_range"),
    "fps": LaunchConfiguration("fps")
}


def generate_launch_description():
    pose_debug_arg = DeclareLaunchArgument("pose_debug", default_value="true")
    rgbd_color_topic_arg = DeclareLaunchArgument("rgbd_color_topic", default_value="/camera_1/color/image_raw")
    pose_output_topic_arg = DeclareLaunchArgument("pose_output_topic", default_value="/keypoints")
    yolo_model_arg = DeclareLaunchArgument("yolo_model", default_value="yolov8n-pose")
    rgbd_debug_arg = DeclareLaunchArgument("rgbd_debug", default_value="True")
    sensor_id_arg = DeclareLaunchArgument("sensor_id", default_value="2")
    frame_id_arg = DeclareLaunchArgument("frame_id", default_value="world")
    optical_frame_id_arg = DeclareLaunchArgument("optical_frame_id", default_value="camera_1_color_optical_frame")
    is_rotated_arg = DeclareLaunchArgument("is_rotated", default_value="False")
    rgbd_color_camera_info_topic_arg = DeclareLaunchArgument("rgbd_color_camera_info_topic", default_value="/camera_1/color/camera_info")
    rgbd_depth_topic_arg = DeclareLaunchArgument("rgbd_depth_topic", default_value="/camera_1/depth/image_rect_raw")
    rgbd_depth_camera_info_topic_arg = DeclareLaunchArgument("rgbd_depth_camera_info_topic", default_value="/camera_1/depth/camera_info")
    detection_output_topic_arg = DeclareLaunchArgument("detection_output_topic", default_value="/detections")
    keypoints_topic_arg = DeclareLaunchArgument("keypoints_topic", default_value="/keypoints")
    min_pose_confidence_score_arg = DeclareLaunchArgument("min_pose_confidence_score", default_value="0.7")
    skip_depth_range_arg = DeclareLaunchArgument("skip_depth_range", default_value="2000")
    fps_arg = DeclareLaunchArgument("fps", default_value="30")


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
            pose_debug_arg,
            rgbd_color_topic_arg,
            pose_output_topic_arg,
            yolo_model_arg,
            rgbd_debug_arg,
            sensor_id_arg,
            frame_id_arg,
            optical_frame_id_arg,
            is_rotated_arg,
            rgbd_color_camera_info_topic_arg,
            rgbd_depth_topic_arg,
            rgbd_depth_camera_info_topic_arg,
            detection_output_topic_arg,
            keypoints_topic_arg,
            min_pose_confidence_score_arg,
            skip_depth_range_arg,
            fps_arg,
            pose_estimation,
            rgbd_detector
        ]
    )
