import os
import sys

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())

from rexasi_tracker.config.topics.src.rgbd import RGBD_OUTPUT_TOPIC
from rexasi_tracker.config.topics.src.stereo import STEREO_OUTPUT_TOPIC
from rexasi_tracker.config.topics.src.lidar import LIDAR_OUTPUT_TOPIC

#  ++++ tracker node topics ++++
TRACKER_OUTPUT_TOPIC = "position/px2cc/tracked"

tracker_topics = {
    "input": {
        "rgbd": RGBD_OUTPUT_TOPIC,
        "stereo": STEREO_OUTPUT_TOPIC,
        "lidar": LIDAR_OUTPUT_TOPIC,
    },
    "output": TRACKER_OUTPUT_TOPIC,
}
