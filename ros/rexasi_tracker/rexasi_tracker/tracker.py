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
from geometry_msgs.msg import Pose

sys.path.append(os.getcwd())
from rexasi_tracker.config.topics.defaults import TRACKER_INPUT_TOPIC, TRACKER_OUTPUT_TOPIC
from rexasi_tracker.utils.misc import save_evaluation_data, load_yaml, validate_yaml
from rexasi_tracker.config.parameters.defaults import MAX_SENSORS_NR, default_tracker_parameters, CONFIG_FILE, CONFIG_SCHEMA_FILE
from rexasi_tracker_msgs.msg import Detections, Tracks

DEBUG_MARKERS_TOPIC = "/debug/norfair"
COLORS = [(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 1.0, 1.0)]

class Norfair(Node):
    """
    Code: https://github.com/tryolabs/norfair
    Documentation: https://tryolabs.github.io/norfair/2.2/reference/
    """

    def __init__(self):
        super().__init__(
            "TRACKER", automatically_declare_parameters_from_overrides=True
        )

        self.config = load_yaml(CONFIG_FILE)
        valid, err = validate_yaml(self.config, CONFIG_SCHEMA_FILE)
        if not valid:
            self.get_logger().error("Wrong configuration file: %s" % str(err))
            sys.exit(-1)


        self.debug = self.get_parameter("debug").get_parameter_value().bool_value

        # SUBSCRIPTIONS
        # subscriber node that will read data from a specific topic
        # in these case, every time message is sent to IMAGE_CENTERS_TOPIC, self.msg_event is triggered
        self.subscriber = self.create_subscription(
            Detections, TRACKER_INPUT_TOPIC, self.msg_event, 10
        )
        self.get_logger().info(f"Subscribed to {TRACKER_INPUT_TOPIC}")

        # PUBLISHERS
        # publisher node that will publish data to a specific topic
        # in these case, the data is sent to TRACKER_TOPIC
        self.publisher = self.create_publisher(
            Tracks, TRACKER_OUTPUT_TOPIC, 10
        )
        self.get_logger().info(f"Publishing to {TRACKER_OUTPUT_TOPIC}")

        self.sensors = {}

        # keep track of hit_counters
        self.current_tracks_id = {}

    def get_parameters(self, sensor_id):
        try:
            return self.config["sensors"][sensor_id]["tracker_parameters"]
        except:
            pass
        return default_tracker_parameters

    def add_sensor(self, sensor_id):
        self.sensors[sensor_id] = {
                "tracker": Tracker(
                    **self.get_parameters(sensor_id),
                    filter_factory=OptimizedKalmanFilterFactory(), # filter for reid
                ),
                "tracked_objects": [],
            }
        self.current_tracks_id[sensor_id] = set()

    def msg_event(self, tracker_data):
        """
        This event is triggered every time a msg is published on INPUT_TOPIC.
        The resulting data will be published in OUTPUT_TOPIC, using TrackerData utils.
        Input:
            SensorData without identities
        Output:
            SensorData with identities
        """

        self.get_logger().debug(
            f"Receiving message: {len(tracker_data.centers)} from sensor ID {tracker_data.sensor_id}"
        )
        # if idx not exists, skip (this might be yielded but void camerdata data such as
        # Cameradata(cam_idx=0, timestamp=0, detections=[[[]]])
        if tracker_data.sensor_id not in self.sensors:
            if len(self.sensors.entries()) > MAX_SENSORS_NR:
                self.get_logger().error("Reached sensor number limit (%d)" % MAX_SENSORS_NR)
                return
            self.get_logger().info("Adding sensor with index: %d" % tracker_data.sensor_id)
            self.add_sensor(tracker_data.sensor_id)

        # parse detection data according to MEASURE_UNIT variable
        centers = tracker_data.centers

        # GENERATE DETECTIONS
        # generate detection for each value inside tracker_data.centers
        detections = [
            Detection(
                # points=np.array([np.array(center), np.array(center)]).reshape(2, 2),
                points=np.array([[center.position.x,center.position.y],[center.position.x,center.position.y]]).reshape(2, 2),
                data=tracker_data.confidences[idx]
            )
            for idx, center in enumerate(centers)
        ]

        # TRACK OBJECTS
        # track each detection using the trackers
        self.sensors[tracker_data.sensor_id]["tracked_objects"] = self.sensors[tracker_data.sensor_id][
            "tracker"
        ].update(detections)

        tracks_id = set()
        for tracked_object in self.sensors[tracker_data.sensor_id]["tracked_objects"]:
            tracks_id.add(str(tracked_object.id))
        dead_tracks_id = self.current_tracks_id[tracker_data.sensor_id].difference(tracks_id)
        self.current_tracks_id[tracker_data.sensor_id] = tracks_id

        # output tracks
        tracks = Tracks()
        tracks.header = tracker_data.header
        tracks.sensor_id = tracker_data.sensor_id

        for tracked_object in self.sensors[tracker_data.sensor_id]["tracked_objects"]:
            tracks.identities.append(tracked_object.id)
            point = tuple(tracked_object.last_detection.points[0])
            center = Pose()
            center.position.x = point[0]
            center.position.y = point[1]
            tracks.centers.append(center)
            tracks.confidences.append(tracked_object.last_detection.data)
        tracks.dead_identities = list(dead_tracks_id)

        if self.debug:
            self.get_logger().info("Tracked %s" % str(tracks.centers))
            self.publish_debug_markers(
                tracks.identities, tracks.centers, COLORS[tracker_data.sensor_id])

        self.publish(tracks)

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
        marker.pose.position.x = center.position.x
        marker.pose.position.y = center.position.y
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
        text_marker.pose.position.x = center.position.x
        text_marker.pose.position.y = center.position.y
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

    def publish(self, tracks: Tracks) -> None:
        # if self.debug:
        #     save_evaluation_data(
        #         tracks.sensor_id,
        #         tracks.centers,
        #         tracks.identities,
        #         tracks.frame_number,
        #         tracks.timestamp,
        #         sensor_type=tracks.sensor_type,
        #         name=f"{self.get_name()}_{tracks.sensor_type}",
        #     )
        self.publisher.publish(tracks)


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
