import os
import sys

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())

from rexasi_tracker.config.topics.src.tracker import TRACKER_OUTPUT_TOPIC

# ++++ sensor fusion node topics ++++
SENSORFUSION_OUTPUT_TOPIC: str = "position/px2cc/tracked/fused"
SENSORFUSION_OUTPUT_FUSED_TRACKS_TOPIC: str = "position/fused_tracks"

sensor_fusion_topics = {
    "input": TRACKER_OUTPUT_TOPIC,
    "output": SENSORFUSION_OUTPUT_TOPIC,
    "output_fused_track": SENSORFUSION_OUTPUT_TOPIC,
}
