import os
import sys

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())
# from rexasi_tracker.config.parameters.src.general import STREAM_OPEN
STREAM_OPEN = True
# from rexasi_tracker.config.parameters.src.camera import FPS 
# from rexasi_tracker.config.parameters.src.sensors import LIDAR_ID, CAMERA_1_ID, CAMERA_2_ID, STEREO_ID
FPS=15
LIDAR_ID=0
CAMERA_1_ID = 1
CAMERA_2_ID= 2 
STEREO_ID = -1

DISTANCE_FUNCTION: str = "euclidean"
DISTANCE_THRESHOLD: int = 1
INITIALIZATION_DELAY: int = int(FPS / 3)
HIT_COUNTER_MAX: int = int(FPS / 2)
REID_HIT_COUNTER_MAX: int = int(FPS / 2)
BUFFER_SIZE: int = 32
TRACKS_DISTANCE_THRESHOLD: float = 1.0
HUNGARIAN_THRESHOLD: float = 0.5  # meters

sensor_fusion_parameters = {
    "model": {
        "distance_function": DISTANCE_FUNCTION,
        "distance_threshold": DISTANCE_THRESHOLD,
        "initialization_delay": INITIALIZATION_DELAY,
        "hit_counter_max": HIT_COUNTER_MAX,
        "reid_hit_counter_max": REID_HIT_COUNTER_MAX
    },
    "tracks_distance_threshold": TRACKS_DISTANCE_THRESHOLD,
    "hungarian_threshold": HUNGARIAN_THRESHOLD,
    "camera": {
        "fps": FPS,
        "width": 640,
        "height": 480
    },
    "is_stream_open": STREAM_OPEN,
    "buffer_size": BUFFER_SIZE,
}

kalman_parameters = {
    "R_std": {
        LIDAR_ID: {"x": 0.001, "y": 0.001},
        CAMERA_1_ID: {"x": 0.001, "y": 0.001},
        CAMERA_2_ID: {"x": 0.001, "y": 0.001},
        STEREO_ID: {"x": 0.001, "y": 0.001}
    },

    "Q_std": {
        LIDAR_ID: {"x": 0.04, "y": 0.04},
        CAMERA_1_ID: {"x": 0.04, "y": 0.04},
        CAMERA_2_ID: {"x": 0.04, "y": 0.04},
        STEREO_ID: {"x": 0.04, "y": 0.04}
    }
}

default_fusion_parameters = {
    "tracks_distance_threshold": TRACKS_DISTANCE_THRESHOLD,
    "hungarian_threshold": HUNGARIAN_THRESHOLD,
}

default_kalman_parameters = {
        "R_std": {"x": 0.001, "y": 0.001},
        "Q_std": {"x": 0.04, "y": 0.04}
    }

sensor_exclusion = {
    # LIDAR_ID: {
    #     "condition": "gt",
    #     "value": 2
    # },
    # CAMERA_1_ID: {
    #     "condition": "lt",
    #     "value": 1.5
    # },
    # CAMERA_2_ID: {
    #     "condition": "lt",
    #     "value": 1.5
    # },
    # STEREO_ID: {
    #     "condition": "lt",
    #     "value": 1.5   
    # }
}
