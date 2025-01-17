import sys

import launch_ros.actions

from launch import LaunchDescription


def generate_launch_description():
   
    pose_estimation = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=["/ros_ws/people_tracker/people_tracker/pose_estimation.py"],
        parameters=[PARAMS],
    )

    rgbd_detector = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=["/ros_ws/people_tracker/people_tracker/rgbd.py"],
        parameters=[{**PARAMS, **SENSOR_SPECIFIC_PARAMS}],
    )

    return LaunchDescription(
        [
            pose_estimation,
            rgbd_detector
        ]
    )
