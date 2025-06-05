
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


lidar_parameters = {
    "debug": LaunchConfiguration("debug"),
    "frame_id": LaunchConfiguration("frame_id"),
    "lidar_frame_id": LaunchConfiguration("lidar_frame_id"),
    "sensor_id": LaunchConfiguration("sensor_id"),
    "lidar_topic": LaunchConfiguration("lidar_topic"),
    "output_topic": LaunchConfiguration("output_topic"),
    "model_ckpt_file": LaunchConfiguration("model_ckpt_file"),
    "model_model": LaunchConfiguration("model_model"),
    "model_gpu": LaunchConfiguration("model_gpu"),
    "model_stride": LaunchConfiguration("model_stride"),
    "model_panoramic_scan": LaunchConfiguration("model_panoramic_scan"),
    "conf_tresh": LaunchConfiguration("conf_tresh"),
    "laser_fov": LaunchConfiguration("laser_fov"),
    "placeholder_value": LaunchConfiguration("placeholder_value"),
    "scan_rate": LaunchConfiguration("scan_rate"),
    "angle_increment": LaunchConfiguration("angle_increment"),
    "ranges": LaunchConfiguration("ranges"),
    "calibration_file": LaunchConfiguration("calibration_file"),
    "shear": LaunchConfiguration("shear"),
    "scale": LaunchConfiguration("scale"),
    "perim_length": LaunchConfiguration("perim_length"),
    "skip_n_frames": LaunchConfiguration("skip_n_frames"),
    "is_rotated": LaunchConfiguration("is_rotated")
}

def generate_launch_description():
    debug_arg = DeclareLaunchArgument("debug", default_value="True")
    frame_id_arg = DeclareLaunchArgument("frame_id", default_value="world")
    lidar_frame_id_arg = DeclareLaunchArgument("lidar_frame_id", default_value="laser_scan")
    sensor_id_arg = DeclareLaunchArgument("sensor_id", default_value="1")
    lidar_topic_arg = DeclareLaunchArgument("lidar_topic", default_value="/scan")
    output_topic_arg = DeclareLaunchArgument("output_topic", default_value="/detections")
    model_ckpt_file_arg = DeclareLaunchArgument("model_ckpt_file", default_value="/data/ckpt_jrdb_ann_ft_dr_spaam_e20.pth")
    model_model_arg = DeclareLaunchArgument("model_model", default_value="DR-SPAAM")
    model_gpu_arg = DeclareLaunchArgument("model_gpu", default_value="True")
    model_stride_arg = DeclareLaunchArgument("model_stride", default_value="1")
    model_panoramic_scan_arg = DeclareLaunchArgument("model_panoramic_scan", default_value="False")
    conf_tresh_arg = DeclareLaunchArgument("conf_tresh", default_value="0.8")
    laser_fov_arg = DeclareLaunchArgument("laser_fov", default_value="270")
    placeholder_value_arg = DeclareLaunchArgument("placeholder_value", default_value="29.99")
    scan_rate_arg = DeclareLaunchArgument("scan_rate", default_value="14")
    angle_increment_arg = DeclareLaunchArgument("angle_increment", default_value="0.00581718236207962")
    ranges_arg = DeclareLaunchArgument("ranges", default_value="811")
    calibration_file_arg = DeclareLaunchArgument("calibration_file", default_value="/ros_ws/people_tracker/config/lidar_calibration/calibration_lidar.yml")
    shear_arg = DeclareLaunchArgument("shear", default_value="False")
    scale_arg = DeclareLaunchArgument("scale", default_value="False")
    perim_length_arg = DeclareLaunchArgument("perim_length", default_value="62")
    skip_n_frames_arg = DeclareLaunchArgument("skip_n_frames", default_value="0")
    is_rotated_arg = DeclareLaunchArgument("is_rotated", default_value="False")
   
    lidar_detector= Node(
            package='lidar_detector',
            executable='lidar_detector',
            name='lidar_detector',
            parameters=[{**lidar_parameters}],
    )

    return LaunchDescription(
        [
            debug_arg,
            frame_id_arg,
            lidar_frame_id_arg,
            sensor_id_arg,
            lidar_topic_arg,
            output_topic_arg,
            model_ckpt_file_arg,
            model_model_arg,
            model_gpu_arg,
            model_stride_arg,
            model_panoramic_scan_arg,
            conf_tresh_arg,
            laser_fov_arg,
            placeholder_value_arg,
            scan_rate_arg,
            angle_increment_arg,
            ranges_arg,
            calibration_file_arg,
            shear_arg,
            scale_arg,
            perim_length_arg,
            skip_n_frames_arg,
            is_rotated_arg,
            lidar_detector,
        ]
    )
