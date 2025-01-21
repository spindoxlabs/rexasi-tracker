
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    tracker = Node(
            package='rexasi_tracker',
            executable='tracker',
            name='tracker',
            parameters=[],
    )

    track_fusion = Node(
            package='rexasi_tracker',
            executable='track_fusion',
            name='track_fusion',
            parameters=[],
    )

    return LaunchDescription(
        [
            tracker,
            track_fusion,
        ]
    )
