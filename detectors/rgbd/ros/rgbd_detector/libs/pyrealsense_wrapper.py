
import pyrealsense2 as rs
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformException
import tf2_geometry_msgs
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def init_camera_intrinsics(camera_info):
    intrinsics = rs.intrinsics()
    intrinsics.width = camera_info.width
    intrinsics.height = camera_info.height
    intrinsics.ppx = camera_info.k[2]
    intrinsics.ppy = camera_info.k[5]
    intrinsics.fx = camera_info.k[0]
    intrinsics.fy = camera_info.k[4]
    # _intrinsics.model = cameraInfo.distortion_model
    intrinsics.model = rs.distortion.none
    intrinsics.coeffs = [i for i in camera_info.d]
    return intrinsics

def get_timestamp_from_msgstamp(stamp) -> int:
    return stamp.sec * pow(10, 9) + stamp.nanosec

class PyRealSenseWrapper:
    def __init__(self, node, ref_frame_id, optical_frame_id):
        self.node = node
        self.logger = rclpy.logging.get_logger('PyRealSenseWrapper')
        self.y180 = np.array([[-1,  0,  0],
                         [0,  1,  0],
                         [0,  0,  -1]])
        self.x90 = np.array([[1,  0,  0],
                        [0,  0,  1],
                        [0,  -1,  0]])
        self.transforms = { "ts": 0, "frame_id": optical_frame_id, "ref_to_camera": None, "camera_to_ref": None}
        self.ref_frame_id = ref_frame_id
        if self.node:
            self.frames = []
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def look_up(self, to_frame_rel, from_frame_rel, stamp):
        try:
            t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, stamp)
                            #rclpy.time.Time()) #,timeout=rclpy.duration.Duration(seconds=1.0))
            # self.logger.debug("Transform from '%s' to '%s': %s" % (from_frame_rel, to_frame_rel, str(t)))
            return t
        except TransformException as ex:
            self.logger.error(
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        return None

    def get_transform(self, to_camera: bool, stamp = None):
        if stamp:
            timestamp = get_timestamp_from_msgstamp(stamp)
            if self.transforms["ts"] == timestamp:
                # use cached transform
                return self.transforms["ref_to_camera" if to_camera else "camera_to_ref"]
            self.transforms["ts"] = timestamp
        if to_camera:
            self.transforms["ref_to_camera"] = self.look_up(self.transforms["frame_id"], self.ref_frame_id, stamp)
        else:
            self.transforms["camera_to_ref"] = self.look_up(self.ref_frame_id, self.transforms["frame_id"], stamp)
        return self.transforms["ref_to_camera" if to_camera else "camera_to_ref"]
    
    def cc2px(self, x, y, z=0.0, **kwargs):
        try:
            point = PointStamped()
            point.point.x = x
            point.point.y = y
            point.point.z = z

            t = self.get_transform(True, kwargs["stamp"])
            if t is None:
                return 0, 0
            
            point_transformed = tf2_geometry_msgs.do_transform_point(point, t)

            pixels = rs.rs2_project_point_to_pixel(kwargs["intrinsics"], [point_transformed.point.x*1000, point_transformed.point.y*1000, point_transformed.point.z*1000])

            return int(pixels[0]), int(pixels[1])
        except Exception as e:
            self.logger.error(f"Error computing cc2px: {e}")
            return 0, 0

    def px2cc(self, x, y, depth, **kwargs):
        try:
            x, y, z = rs.rs2_deproject_pixel_to_point(
                    kwargs["intrinsics"], [x, y], depth)
            
            if x == 0 and y == 0 and z == 0:
                return x, y, z

            point = PointStamped()
            point.point.x = x/1000
            point.point.y = y/1000
            point.point.z = z/1000

            t = self.get_transform(False, kwargs["stamp"])
            if t is None:
                return 0, 0, 0

            point_transformed = tf2_geometry_msgs.do_transform_point(point, t)
            return point_transformed.point.x, point_transformed.point.y, point_transformed.point.z
        except Exception as e:
            self.logger.error(f"Error computing px2cc: {e}")
            return 0, 0, 0

