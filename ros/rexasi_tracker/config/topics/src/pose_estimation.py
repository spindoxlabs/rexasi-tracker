import os
import sys

# if os.getcwd() not in sys.path:
#     sys.path.append(os.getcwd())

from people_tracker.config.topics.src.realsense import RGB_IMAGE_TOPIC

# ++++ pose estimation node topics ++++
POSE_ESTIMATION_OUTPUT_TOPIC = "position/keypoints"

rgb_topics = {"input": RGB_IMAGE_TOPIC, "output": POSE_ESTIMATION_OUTPUT_TOPIC}
