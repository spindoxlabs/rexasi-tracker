import os
import shutil
from pathlib import Path

import cv2
import numpy as np

OUTPUT_PATH = "/data/output/calibration/matrices/"


def my_estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs):
    """
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    """
    marker_points = np.array(
        [
            [-markerLength / 2, markerLength / 2, 0],
            [markerLength / 2, markerLength / 2, 0],
            [markerLength / 2, -markerLength / 2, 0],
            [-markerLength / 2, -markerLength / 2, 0],
        ],
        dtype=np.float32,
    )
    trash = []
    rvecs = []
    tvecs = []
    for c in corners:
        nada, R, t = cv2.solvePnP(
            marker_points, c, cameraMatrix, distCoeffs, False, cv2.SOLVEPNP_IPPE_SQUARE
        )
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash


def delete_temporary_artifacts(n_cams: int):
    try:
        if os.path.exists(OUTPUT_PATH):
            shutil.rmtree(f"{OUTPUT_PATH}")
        for cam_idx in range(n_cams):
            Path(f"{OUTPUT_PATH}/{cam_idx + 1}").mkdir(parents=True, exist_ok=True)
    except Exception as e:
        print(e)


def save_matrices(matrix: np.array, cc: np.array, cam_idx: int):
    label = f"xy=({round(cc[0], 2)},{round(cc[2], 2)}).npy"
    with open(f"{OUTPUT_PATH}/{cam_idx}/{label}", "wb") as f:
        np.save(f, matrix)
