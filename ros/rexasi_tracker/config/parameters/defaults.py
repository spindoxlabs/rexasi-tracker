
CONFIG_FILE="rexasi_tracker/config/config.yml"
CONFIG_SCHEMA_FILE="rexasi_tracker/config/parameters/config_schema.py"

MAX_SENSORS_NR=100

FPS=15

X_FORWARD=1

default_tracker_parameters = {
        "distance_function": "euclidean",
        "distance_threshold": 0.8,
        "initialization_delay": int(FPS / 4),
        "hit_counter_max": int(FPS / 2),
        "pointwise_hit_counter_max": int(FPS / 2),
    }

default_kalman_parameters = {
        "R_std": {"x": 0.001, "y": 0.001},
        "Q_std": {"x": 0.04, "y": 0.04}
    }

sensor_exclusion = {}