
lidar_parameters = {
    "lidar_topic": "/scan",
    "model_params": {
        "ckpt_file": "/data/self_supervised_person_detection/ckpt_jrdb_ann_drow3_e40.pth",
        "model": "DROW3",
        "gpu": True,
        "stride": 1,
        "panoramic_scan": False,
    },
    "conf_tresh": 0.8,
    "laser_fov": 270,
    "placeholder_value": 29.99,
    "scan_rate": 14,
    "angle_increment": 0.00581718236207962,
    "ranges": 811,
    "calibration_file": "/ros_ws/people_tracker/config/lidar_calibration/calibration_lidar.yml",
    "shear": False,
    "scale": False,
    "perim_length": 62,
    "skip_n_frames": 0
}
