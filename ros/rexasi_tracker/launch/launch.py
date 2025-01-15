import logging
import os
import sys

import launch_ros.actions
from launch import LaunchDescription


def generate_launch_description():

    tracker = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=["rexasi_tracker/rexasi_tracker/tracker.py"],
        parameters=[],
    )

    track_fusion = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=["rexasi_tracker/rexasi_tracker/track_fusion.py"],
        parameters=[],
    )

    try:
        # TestNode(logger=logger).run_tests()
        logging.info(f"Tests passed. Launching nodes")
    except Exception as e:
        logging.getLogger("launch").error(f"Test failed. {e}")
        sys.exit()

    return LaunchDescription(
        [
            tracker,
            track_fusion,
        ]
    )
