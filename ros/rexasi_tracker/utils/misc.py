import os
import shutil
from pathlib import Path

import cv2
import numpy as np

SAVE_EVALUATION_DATA = True
OUTPUT_PATH = f"/data/output/debug/"
EVALUATION_FILE = f"CAMERA_detection.csv"

try:
    if os.path.exists(OUTPUT_PATH):
        shutil.rmtree(f"{OUTPUT_PATH}")
    Path(f"{OUTPUT_PATH}").mkdir(parents=True, exist_ok=True)
except Exception as e:
    print(e)


def save_evaluation_data(
    cam_idx: int,
    cc: list,
    ids: list,
    frame_number: int,
    timestamp: int,
    sensor_type: str,
    name: str = "generic",
    confidence: float = 1.0,
) -> None:
    path = f"{OUTPUT_PATH}detections/{sensor_type}"
    Path(f"{path}").mkdir(parents=True, exist_ok=True)
    evaluation_file_name = EVALUATION_FILE.replace("CAMERA", f"{name}_{cam_idx}")
    if evaluation_file_name not in os.listdir(path):
        with open(
            f"{path}/{evaluation_file_name}",
            "w",
            encoding="utf-8",
        ) as file:
            file.write("frame;id;x;y;timestamp\n")
        file.close()

    with open(f"{path}/{evaluation_file_name}", "a+", encoding="utf-8") as file:
        if len(ids) == 0:
            file.write(f"{frame_number};;;;{timestamp}\n")
        for center, identity in zip(cc, ids):
            # this format is the one used by the MOT CHALLENGE
            file.write(
                f"{frame_number};{identity};{center[0]};{center[1]};{timestamp}\n"
            )
    file.close()


def save_frame(frame: np.ndarray, timestamp: int, cam_idx: int, folder: str = "rgb"):
    path = f"{OUTPUT_PATH}/frames/{folder}/camera_{cam_idx}"
    Path(f"{path}").mkdir(parents=True, exist_ok=True)
    cv2.imwrite(f"{path}/{str(timestamp).ljust(11, '0')}.png", frame)

def get_timestamp_from_msgstamp(stamp) -> int:
    return stamp.sec * pow(10, 9) + stamp.nanosec

def get_timestamp(msg) -> int:
    return get_timestamp_from_msgstamp(msg.header.stamp)
