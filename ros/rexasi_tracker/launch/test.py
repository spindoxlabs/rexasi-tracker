import logging
import os
import sys

import launch_ros.actions
from launch import LaunchDescription

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())

from people_tracker.test.test_node import TestNode
from people_tracker.config.parameters.src.launch import logger, PARAMS


def generate_launch_description():
    

    tracker = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=["/ros_ws/people_tracker/people_tracker/tracker.py"],
        parameters=[PARAMS],
    )

    track_fusion = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=["/ros_ws/people_tracker/people_tracker/track_fusion.py"],
        parameters=[PARAMS],
    )


    try:
        TestNode(logger=logger).run_tests()
        logger.info(f"Tests passed. Launching nodes")
    except Exception as e:
        logging.getLogger("launch").error(f"Test failed. {e}")
        sys.exit()

    return LaunchDescription(
        [

            tracker,
            track_fusion,

        ]
    )
