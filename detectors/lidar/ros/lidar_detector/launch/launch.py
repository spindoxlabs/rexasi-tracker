
from launch import LaunchDescription
from launch_ros.actions import Node

lidar_parameters = {
    "debug": True,
    "frame_id": 'world',
    "lidar_frame_id": "laser_scan",
    "sensor_id": 1,
    "lidar_topic": "/scan",
    "output_topic": "/detections",
    "model_ckpt_file": "/data/self_supervised_person_detection/ckpt_jrdb_ann_drow3_e40.pth",
    "model_model": "DROW3",
    "model_gpu": True,
    "model_stride": 1,
    "model_panoramic_scan": False,
    "conf_tresh": 0.8,
    "laser_fov": 270,
    "placeholder_value": 29.99,
    "scan_rate": 14,
    "angle_increment": 0.00581718236207962,
    "ranges": 811,
    "calibration_file": "/ros_ws/people_tracker/config/lidar_calibration/calibration_lidar.yml",
    "shear": False,
    "scale": False,
    "perim_length": 62,
    "skip_n_frames": 0,
    "is_rotated": False
}

def generate_launch_description():
   
    lidar_detector= Node(
            package='lidar_detector',
            executable='lidar_detector',
            name='lidar_detector',
            parameters=[{**lidar_parameters}],
    )

    return LaunchDescription(
        [
            lidar_detector,
        ]
    )
