import json
import os
import sys

import numpy as np
import rclpy
from norfair import Detection
from norfair import Tracker
from norfair.filter import OptimizedKalmanFilterFactory
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

sys.path.append(os.getcwd())
from rexasi_tracker.utils.dto import SensorTrackedData
from rexasi_tracker.utils.misc import save_evaluation_data
from rexasi_tracker.config.topics.src.tracker import tracker_topics
from rexasi_tracker.config.parameters.src.tracker import tracker_parameters, default_tracker_parameters
from rexasi_tracker_msgs.msg import RexString

DEBUG_MARKERS_TOPIC = "/debug/norfair"
COLORS = [(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 1.0, 1.0)]
MAX_SENSORS_NR=100

class Norfair(Node):
    """
    Code: https://github.com/tryolabs/norfair
    Documentation: https://tryolabs.github.io/norfair/2.2/reference/
    """

    def __init__(self):
        super().__init__(
            "TRACKER", automatically_declare_parameters_from_overrides=True
        )

        self.markers_publisher = self.create_publisher(
            MarkerArray, DEBUG_MARKERS_TOPIC, 10
        )

        self.debug = self.get_parameter("debug").get_parameter_value().bool_value

        # SUBSCRIPTIONS
        # subscriber node that will read data from a specific topic
        # in these case, every time message is sent to IMAGE_CENTERS_TOPIC, self.msg_event is triggered
        self.rgbd_subscriber = self.create_subscription(
            RexString, f"{tracker_topics['input']['rgbd']}", self.msg_event, 10
        )
        self.get_logger().info(f"Subscribed to {tracker_topics['input']['rgbd']}")

        self.stereo_subscriber = self.create_subscription(
            RexString, f"{tracker_topics['input']['stereo']}", self.msg_event, 10
        )
        self.get_logger().info(f"Subscribed to {tracker_topics['input']['stereo']}")

        self.lidar_subscriber = self.create_subscription(
            RexString, f"{tracker_topics['input']['lidar']}", self.msg_event, 10
        )
        self.get_logger().info(f"Subscribed to {tracker_topics['input']['lidar']}")

        # PUBLISHERS
        # publisher node that will publish data to a specific topic
        # in these case, the data is sent to TRACKER_TOPIC
        self.publisher = self.create_publisher(
            RexString, f"{tracker_topics['output']}", 10
        )
        self.get_logger().info(f"Publishing to {tracker_topics['output']}")

        # CAMERA TRACKERS
        # every camera has its tracker, and the number of cameras is stated in the parameter n_cameras from the launcher
        self.n_cameras = (
            self.get_parameter("n_cameras").get_parameter_value().integer_value
        )
        self.cameras = {}

        # keep track of hit_counters
        self.current_tracks_id = {}

    def get_parameters(self, sensor_id):
        # TODO get from config file or use defaults
        return default_tracker_parameters

    def add_sensor(self, sensor_id):
        self.cameras[sensor_id] = {
                "tracker": Tracker(
                    **self.get_parameters(sensor_id),
                    filter_factory=OptimizedKalmanFilterFactory(), # filter for reid
                ),
                "tracked_objects": [],
            }
        self.current_tracks_id[sensor_id] = set()

    def msg_event(self, msg):
        """
        This event is triggered every time a msg is published on INPUT_TOPIC.
        The resulting data will be published in OUTPUT_TOPIC, using TrackerData utils.
        Input:
            SensorData without identities
        Output:
            SensorData with identities
        """

        # LOAD CENTERS FROM TOPIC INPUT_TOPIC
        # apply TrackerData utils to the msg data in order to parse values
        tracker_data = SensorTrackedData(**json.loads(msg.data))
        # skips void detections
        if tracker_data.timestamp == 0:
            return

        self.get_logger().debug(
            f"Recieving message: {len(tracker_data.centers.coordinates)} from {tracker_data.idx}"
        )
        # if idx not exists, skip (this might be yielded but void camerdata data such as
        # Cameradata(cam_idx=0, timestamp=0, detections=[[[]]])
        if tracker_data.idx not in self.cameras:
            if len(self.cameras.entries()) > MAX_SENSORS_NR:
                self.get_logger().error("Reached max sensor number limit (%d)" % MAX_SENSORS_NR)
                return
            self.get_logger().info("Adding sensor with index: %d" % tracker_data.idx)
            self.add_sensor(tracker_data.idx)

        # parse detection data according to MEASURE_UNIT variable
        cam_idx = tracker_data.idx
        if tracker_parameters["measure_unit"] == "coordinates":
            centers = tracker_data.centers.coordinates
        else:
            centers = tracker_data.centers.pixels

        # GENERATE DETECTIONS
        # generate detection for each value inside tracker_data.centers
        detections = [
            Detection(
                points=np.array([np.array(center), np.array(center)]).reshape(2, 2),
                data=tracker_data.confidence[idx]
            )
            for idx, center in enumerate(centers)
        ]

        # TRACK OBJECTS
        # track each detection using the trackers
        self.cameras[cam_idx]["tracked_objects"] = self.cameras[cam_idx][
            "tracker"
        ].update(detections)  # , period=tracker_parameters["period"])

        # # save and output tracked objects hit counter
        tracks_id = set()
        for tracked_object in self.cameras[cam_idx]["tracked_objects"]:
            tracks_id.add(str(tracked_object.id))
        dead_tracks_id = self.current_tracks_id[cam_idx].difference(tracks_id)
        self.current_tracks_id[cam_idx] = tracks_id

        # reset centers in topic data
        tracker_data.centers.coordinates = []
        tracker_data.confidence = []

        for tracked_object in self.cameras[cam_idx]["tracked_objects"]:
            # if tracked_object.hit_counter > 0:
            tracker_data.identities.append(tracked_object.id)
            point = tuple(tracked_object.last_detection.points[0])
            tracker_data.centers.coordinates.append(point)
            tracker_data.confidence.append(tracked_object.last_detection.data)
            # else:
            # tracker_data.dead_identities.append(tracked_object.id)
        tracker_data.dead_identities = list(dead_tracks_id)

        if self.debug:
            self.get_logger().info("Tracked %s" % str(tracker_data.centers.coordinates))
            self.publish_debug_markers(
                tracker_data.identities, tracker_data.centers.coordinates, COLORS[cam_idx])

        self.publish(tracker_data, msg.header.stamp)

    def get_track_marker(self, identity, center, color):
        markers = []
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = 3
        marker.id = identity
        marker.lifetime = rclpy.duration.Duration(seconds=0.3).to_msg()
        # marker.action = Marker.DELETE
        # Set the scale of the marker
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.0
        # Set the color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        # Set the pose of the marker
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = 0.1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        markers.append(marker)
        text_marker = Marker()
        text_marker.header.frame_id = "/map"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.type = 9
        text_marker.id = identity * 1000
        text_marker.text = f"{identity}"
        text_marker.lifetime = rclpy.duration.Duration(
            seconds=0.3).to_msg()
        # marker.action = Marker.DELETE
        # Set the scale of the marker
        text_marker.scale.x = 1.0
        text_marker.scale.y = 1.0
        text_marker.scale.z = 0.2
        # Set the color
        text_marker.color.r = 0.0
        text_marker.color.g = 0.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0
        # Set the pose of the marker
        text_marker.pose.position.x = center[0]
        text_marker.pose.position.y = center[1]
        text_marker.pose.position.z = 0.3
        text_marker.pose.orientation.x = 0.0
        text_marker.pose.orientation.y = 0.0
        text_marker.pose.orientation.z = 0.0
        text_marker.pose.orientation.w = 1.0
        markers.append(text_marker)
        return markers

    def publish_debug_markers(self, identities, centers, color):
        marker_array = MarkerArray()
        for id, c in zip(identities, centers):
            marker_array.markers.extend(self.get_track_marker(id, c, color))
        self.markers_publisher.publish(marker_array)

    def publish(self, data: SensorTrackedData, stamp) -> None:
        if self.debug:
            save_evaluation_data(
                data.idx,
                data.centers.coordinates,
                data.identities,
                data.frame_number,
                data.timestamp,
                sensor_type=data.sensor_type,
                name=f"{self.get_name()}_{data.sensor_type}",
            )
        msg = RexString()
        msg.header.stamp = stamp
        msg.data = json.dumps(data.model_dump())
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    tracker = Norfair()
    rclpy.spin(tracker)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
