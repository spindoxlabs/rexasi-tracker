import os
import sys

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())

from rexasi_tracker.config.topics.src.pose_estimation import (
    POSE_ESTIMATION_OUTPUT_TOPIC,
)

# ++++ stereo node topics ++++
STEREO_OUTPUT_TOPIC = "position/stereo/px2cc"

stereo_topics = {"input": POSE_ESTIMATION_OUTPUT_TOPIC, "output": STEREO_OUTPUT_TOPIC}
