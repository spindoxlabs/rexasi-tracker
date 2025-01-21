import sys
import os

import launch_ros.actions

from launch import LaunchDescription

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())
from rgbd_detector.config.config import rgbd_parameters

def generate_launch_description():
   
    pose_estimation = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=["/ros_ws/rgbd_detector/rgbd_detector/pose_estimation.py"],
        parameters=[{**rgbd_parameters}],
    )

    rgbd_detector = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=["/ros_ws/rgbd_detector/rgbd_detector/rgbd_detector.py"],
        parameters=[{**rgbd_parameters}],
    )

    return LaunchDescription(
        [
            pose_estimation,
            rgbd_detector
        ]
    )
