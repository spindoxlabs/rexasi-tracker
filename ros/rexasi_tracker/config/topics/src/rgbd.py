import os
import sys

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())

from rexasi_tracker.config.topics.src.pose_estimation import (
    POSE_ESTIMATION_OUTPUT_TOPIC,
)

# ++++ rgbd node topics ++++
RGBD_OUTPUT_TOPIC = "position/rgbd/px2cc"

rgbd_topics = {"input": POSE_ESTIMATION_OUTPUT_TOPIC, "output": RGBD_OUTPUT_TOPIC}
