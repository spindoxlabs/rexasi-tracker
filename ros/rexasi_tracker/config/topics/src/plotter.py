import os
import sys

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())

from rexasi_tracker.config.topics.src.tracker import TRACKER_OUTPUT_TOPIC
from rexasi_tracker.config.topics.src.sensor_fusion import SENSORFUSION_OUTPUT_TOPIC
from rexasi_tracker.config.topics.src.realsense import RGB_IMAGE_TOPIC

# +++ plotter ++++
PLOTTER_OUTPUT_TOPIC = "plot/image"

PLOTTER_TELEMETRY_TOPIC = "plot/telemetry"

PLOTTER_LIDAR_RVIZ_OUTPUT = "plot/lidar/rviz"

PLOTTER_LIDAR_OUTPUT = "plot/lidar"

PLOTTER_STEREO_OUTPUT = "plot/stereo_vision"

plotter_topics = {
    "input": {
        "rgb": RGB_IMAGE_TOPIC,
        "detection": SENSORFUSION_OUTPUT_TOPIC,
        "stereo": TRACKER_OUTPUT_TOPIC,
        "lidar": TRACKER_OUTPUT_TOPIC,
        "telemetry": TRACKER_OUTPUT_TOPIC,
    },
    "output": {
        "rgb": PLOTTER_OUTPUT_TOPIC,
        "stereo": PLOTTER_STEREO_OUTPUT,
        "lidar": {"3d": PLOTTER_LIDAR_OUTPUT, "rviz": PLOTTER_LIDAR_RVIZ_OUTPUT},
        "telemetry": PLOTTER_TELEMETRY_TOPIC,
    },
}
