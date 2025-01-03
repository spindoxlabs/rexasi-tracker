import os
import sys

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())

from people_tracker.config.topics.src.sensor_fusion import SENSORFUSION_OUTPUT_TOPIC

# ++++ CC2PX node topics  ++++
CC2PX_OUTPUT_TOPIC = "position/px2cc/tracked/fused/cc2px"

cc2px_topics = {"input": SENSORFUSION_OUTPUT_TOPIC, "output": CC2PX_OUTPUT_TOPIC}
