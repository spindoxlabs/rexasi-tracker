import os
import sys

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())

from rexasi_tracker.config.topics.src.realsense import RGB_IMAGE_TOPIC

CALIBRATION_INTRINSICS_OUTPUT_TOPIC = "calibration/intrinsics"
CALIBRATION_EXTRINSICS_OUTPUT_TOPIC = "calibration/extrinsics"

calibration_topics = {
    "input": RGB_IMAGE_TOPIC,
    "output": {
        "intrinsics": CALIBRATION_INTRINSICS_OUTPUT_TOPIC,
        "extrinsics": CALIBRATION_EXTRINSICS_OUTPUT_TOPIC,
    },
}
