import logging
import os
import sys

import launch_ros.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())
from lidar_detector.config.config import lidar_parameters

def generate_launch_description():
   
    lidar_detector= launch_ros.actions.Node(
        executable=sys.executable,
        arguments=["/ros_ws/lidar_detector/lidar_detector/lidar_detector.py"],
        parameters=[{**lidar_parameters}],
    )

    return LaunchDescription(
        [
            lidar_detector,
        ]
    )
