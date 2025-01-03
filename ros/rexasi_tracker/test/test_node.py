import logging
import os
import sys

import numpy as np

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())

from rexasi_tracker.config.parameters.src.general import load_camera_translation
from rexasi_tracker.libs.transformation_functions import back_project_wrapper
from rexasi_tracker.test.test_data import TEST_BACKPROJECT_DATA
from rexasi_tracker.config.parameters.src.general import DEVICE, DEVICE_N
from rexasi_tracker.libs.transformation_functions import lidar_calibration_wrapper

CALIBRATION_TEST_FILE: str = (
    "/ros_ws/people_tracker/config/camera_calibration/calibration_camera_test.yml"
)

class TestNode:
    # todo: +++++ update, populate and test +++++
    """
    This node is used to test the calibration files for backproject.
    """

    def __init__(self, logger):
        self.logger = logger

        # Define number of cameras
        self.n_cameras = 2

        # Define backproject
        self.back_project = back_project_wrapper

    def test_cuda(self):
        assert DEVICE == "cuda", "Device is not running on cuda!"
        self.logger.info(f"[CHECK] Cuda detected")

    def test_parameters(self):
        assert self.n_cameras == 2, "Only 2 cameras are supported!"
        self.logger.info(f"[CHECK] Cameras number")

    def test_transform_function(self):
        # todo: add pyrealsense transformation
        for cam_idx in range(0, self.n_cameras):
            # match cam_idx with id of the cameras (1 -> camera_1)
            cam_idx = cam_idx + 1

            translation = load_camera_translation(cam_idx=cam_idx, calib_file = CALIBRATION_TEST_FILE)

            # check test data for a specific camera is available
            if cam_idx not in TEST_BACKPROJECT_DATA:
                self.logger.error(f"No data available for camera {cam_idx}.")
                raise NotImplementedError

            # check if the calibration is available for a specific camera
            if cam_idx == 1:
                calibration = self.back_project.Cal1
            elif cam_idx == 2:
                calibration = self.back_project.Cal2
            else:
                self.logger.error(f"No calibration data for {cam_idx}")
                raise ModuleNotFoundError

            for d in TEST_BACKPROJECT_DATA[cam_idx]:
                # get calibration data and run evaluation
                img_x, img_y, expected_x, expected_y = d
                res = calibration.run(img_y, img_x)

                # compute position on the line (s)
                Z = 0
                s = (Z - translation[2]) / float(res[2] - translation[2])

                # compute xy coordinates in the World
                X = translation[0] + s * (res[0] - translation[0])
                Y = translation[1] + s * (res[1] - translation[1])

                if round(X) != expected_x or round(Y) != expected_y:
                    self.logger.error(
                        "\n!!! Test failed for cam %d: %d,%d -> %.2f,%.2f,%.2f (expected: %d,%d,0)\n"
                        % (cam_idx, img_x, img_y, X, Y, Z, expected_x, expected_y)
                    )
                    raise ValueError
                else:
                    self.logger.info(f"[CHECK] Camera {cam_idx} calibration")

    def test_lidar_calibration(self):
        v0, calibration_points = lidar_calibration_wrapper.load_calibration_data()
        trans = lidar_calibration_wrapper.transform(v0.T).T
        assert np.allclose(calibration_points, trans, atol=0.05) == True
        self.logger.info(f"[CHECK] Lidar calibration")

    def run_tests(self):
        self.logger.info("+++ STARTING TESTS +++")

        # Run tests
        self.test_cuda()
        self.test_parameters()
        self.test_transform_function()
        self.test_lidar_calibration()

        self.logger.info("+++ ALL TEST WERE SUCCESSFULL! +++")
