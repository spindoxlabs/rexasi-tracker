import os
import sys

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())
# from rexasi_tracker.config.parameters.src.camera import FPS
# from rexasi_tracker.config.parameters.src.lidar import SCAN_RATE
FPS=15
SCAN_RATE=15


"""
distance function:  define distance function
distance threshold: maximum distance for a match
initialization_delay: number of frames to wait before start
hit counter max: how long an object can live without getting matched to any detections
reid hit counter max: how long an object can live without getting matched to any id
pointwise hit counter max: This argument defines how large the inertia for each point of a tracker can grow.
"""

DISTANCE_FUNCTION: str = "euclidean"
DISTANCE_THRESHOLD: float = 0.8

INITIALIZATION_DELAY: int = int(FPS / 4)
HIT_COUNTER_MAX: int = int(FPS / 2)
REID_HIT_COUNTER_MAX: int = int(FPS / 2)
POINTWISE_HIT_COUNTER_MAX: int = int(FPS / 2)

INITIALIZATION_DELAY_LIDAR: int = 3  # int(SCAN_RATE / 4)  # 3
HIT_COUNTER_MAX_LIDAR: int = 4  # int(SCAN_RATE / 2)  # 7
REID_HIT_COUNTER_MAX_LIDAR: int = int(SCAN_RATE / 2)
POINTWISE_HIT_COUNTER_MAX_LIDAR: int = int(SCAN_RATE / 2)

PERIOD: int = 1
N_TIMESTAMPS: int = 20
MEASURE_UNIT: str = "coordinates"

tracker_parameters = {
    "model": {
        "rgbd": {
            "distance_function": DISTANCE_FUNCTION,
            "distance_threshold": DISTANCE_THRESHOLD,
            "initialization_delay": INITIALIZATION_DELAY,
            "hit_counter_max": HIT_COUNTER_MAX,
            # "reid_hit_counter_max": REID_HIT_COUNTER_MAX,
            "pointwise_hit_counter_max": POINTWISE_HIT_COUNTER_MAX,
        },
        "stereo": {
            "distance_function": DISTANCE_FUNCTION,
            "distance_threshold": DISTANCE_THRESHOLD,
            "initialization_delay": INITIALIZATION_DELAY,
            "hit_counter_max": HIT_COUNTER_MAX,
            # "reid_hit_counter_max": REID_HIT_COUNTER_MAX,
            "pointwise_hit_counter_max": POINTWISE_HIT_COUNTER_MAX,
        },
        "lidar": {
            "distance_function": DISTANCE_FUNCTION,
            "distance_threshold": DISTANCE_THRESHOLD,
            "initialization_delay": INITIALIZATION_DELAY_LIDAR,
            "hit_counter_max": HIT_COUNTER_MAX_LIDAR,
            # "reid_hit_counter_max": REID_HIT_COUNTER_MAX_LIDAR,
            "pointwise_hit_counter_max": POINTWISE_HIT_COUNTER_MAX_LIDAR,
        },
    },
    "period": PERIOD,
    "n_timestamps": N_TIMESTAMPS,
    "measure_unit": MEASURE_UNIT,
}
