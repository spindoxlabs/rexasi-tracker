import logging

import numpy as np
import yaml
from torch import cuda

STREAM_OPEN: bool = True
CONFIG_FILE: str = "/ros_ws/rexasi_tracker/config/config.yml"
CALIBRATION_FILE: str = (
    "/ros_ws/rexasi_tracker/config/camera_calibration/calibration_camera.yml"
)
USE_PYREALSENSE: bool = True
DEVICE: str = "cuda" if cuda.is_available() else "cpu"
DEVICE_N: int = 0 if cuda.is_available() else -1


def load_camera_config(cam_idx: int, logger):
    with open(CALIBRATION_FILE, "r") as file:
        config = yaml.safe_load(file)
    try:
        camera_config = config[f"camera_{cam_idx}"]
    except Exception as e:
        logger.error(f"{e}")
        exit()
    return camera_config


def load_config():
    with open(CONFIG_FILE, "r") as file:
        config = yaml.safe_load(file)
    return config


def load_camera_translation(cam_idx: int, logger: logging.Logger = logging.getLogger(), calib_file: str = CALIBRATION_FILE):
    with open(calib_file, "r") as file:
        config = yaml.safe_load(file)
    try:
        camera_config = config[f"camera_{cam_idx}"]
    except Exception as e:
        logger.error(
            f"Error loading calibration for camera {cam_idx} from config {config}: {e}"
        )
        exit(1)
    # Get translation array
    camera_translation = np.array(
        [camera_config["T1"], camera_config["T2"], camera_config["T3"]]
    )
    return camera_translation


def load_camera_rotation(cam_idx: int, logger: logging.Logger = logging.getLogger()):
    with open(CALIBRATION_FILE, "r") as file:
        config = yaml.safe_load(file)
    try:
        camera_config = config[f"camera_{cam_idx}"]
    except Exception as e:
        logger.error(
            f"Error loading calibration for camera {cam_idx} from config {config}: {e}"
        )
        exit(1)
    # Get rotation matrix
    camera_rotation = np.array(
        [
            [camera_config["R11"], camera_config["R12"], camera_config["R13"]],
            [camera_config["R21"], camera_config["R22"], camera_config["R23"]],
            [camera_config["R31"], camera_config["R32"], camera_config["R33"]],
        ]
    )
    return camera_rotation


def load_camera_height(cam_idx: int, logger=logging.getLogger()):
    camera_config = load_camera_config(cam_idx, logger)
    return camera_config["T3"]


# cameras_parameters = {
#     cam_idx: {
#         "translation": load_camera_translation(cam_idx),
#         "rotation": load_camera_rotation(cam_idx),
#         "height": load_camera_height(cam_idx),
#     }
#     for cam_idx in [x["cam_idx"] for x in load_config()["cameras"]]
# }
