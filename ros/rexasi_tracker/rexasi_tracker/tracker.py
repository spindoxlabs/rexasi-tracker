import os
import sys
import numpy as np
import rclpy
from norfair import Detection
from norfair import Tracker
from norfair.filter import OptimizedKalmanFilterFactory
from rclpy.node import Node
from geometry_msgs.msg import Pose

sys.path.append(os.getcwd())
from rexasi_tracker.utils.misc import load_yaml
from rexasi_tracker.config.parameters.defaults import MAX_SENSORS_NR, default_tracker_parameters, CONFIG_FILE, CONFIG_SCHEMA_FILE
from rexasi_tracker_msgs.msg import Detections, Tracks


class Norfair(Node):
    """
    Code: https://github.com/tryolabs/norfair
    Documentation: https://tryolabs.github.io/norfair/2.2/reference/
    """

    def __init__(self):
        super().__init__(
            "TRACKER", automatically_declare_parameters_from_overrides=True
        )

        valid, config, err = load_yaml(CONFIG_FILE, CONFIG_SCHEMA_FILE)
        if not valid:
            self.get_logger().error("Wrong configuration file: %s" % str(err))
            sys.exit(-1)
        self.config = config

        self.get_logger().debug("Loaded configuration: %s" % str(self.config))

        self.debug = self.config["general"]["debug"]

        # SUBSCRIPTIONS
        input_topic = self.config["topics"]["tracker_input_topic"]
        self.subscriber = self.create_subscription(
            Detections, input_topic, self.msg_event, 10
        )
        self.get_logger().info(f"Subscribed to {input_topic}")

        # PUBLISHERS
        output_topic = self.config["topics"]["tracker_output_topic"]
        self.publisher = self.create_publisher(
            Tracks, output_topic, 10
        )
        self.get_logger().info(f"Publishing to {output_topic}")

        self.sensors = {}

        # keep track of hit_counters
        self.current_tracks_id = {}

    def get_tracker_parameters(self, sensor_id):
        for s in self.config["sensors"]:
            if s["sensor_id"] == sensor_id:
                return  s["tracker_parameters"]
        return default_tracker_parameters

    def add_sensor(self, sensor_id):
        self.sensors[sensor_id] = {
                "tracker": Tracker(
                    **self.get_tracker_parameters(sensor_id),
                    filter_factory=OptimizedKalmanFilterFactory(), # filter for reid
                ),
                "tracked_objects": [],
            }
        self.current_tracks_id[sensor_id] = set()

    def msg_event(self, tracker_data):
        self.get_logger().debug(
            f"Receiving message: {len(tracker_data.centers)} from sensor ID {tracker_data.sensor_id}"
        )
        # INIT A SENSOR TRACKER IF NOT SEEN YET
        if tracker_data.sensor_id not in self.sensors:
            if len(self.sensors.entries()) > MAX_SENSORS_NR:
                self.get_logger().error("Reached sensor number limit (%d)" % MAX_SENSORS_NR)
                return
            self.get_logger().info("Adding sensor with index: %d" % tracker_data.sensor_id)
            self.add_sensor(tracker_data.sensor_id)

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

        # OUTPUT TRACKS
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
