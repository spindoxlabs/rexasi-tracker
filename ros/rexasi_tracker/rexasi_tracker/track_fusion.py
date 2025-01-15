import json
import os
import sys
from typing import  List, Tuple
from scipy.optimize import linear_sum_assignment

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from scipy.linalg import block_diag
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
from transforms3d._gohlketransforms import quaternion_from_euler
from geometry_msgs.msg import Pose, Twist

if os.getcwd() not in sys.path:
    sys.path.append(os.getcwd())
from rexasi_tracker.config.topics.defaults import TRACKER_OUTPUT_TOPIC, FUSION_OUTPUT_TOPIC
from rexasi_tracker.utils.dto import AssociationTrack, SensorTrack
from rexasi_tracker.utils.misc import get_timestamp, save_evaluation_data, load_yaml, validate_yaml
from rexasi_tracker.config.parameters.defaults import X_FORWARD, sensor_exclusion, default_kalman_parameters,default_fusion_parameters,CONFIG_FILE, CONFIG_SCHEMA_FILE
from rexasi_tracker_msgs.msg import Tracks

DEBUG_MARKERS_TOPIC = "/debug/association"
DEBUG_CSV_FILE = "/data/sensor_fusion.csv"
COLORS = [(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 1.0, 1.0)]


class TrackFusion(Node):
    def __init__(self):
        super().__init__("FUSION", automatically_declare_parameters_from_overrides=True)

        self.config = load_yaml(CONFIG_FILE)
        valid, err = validate_yaml(self.config, CONFIG_SCHEMA_FILE)
        if not valid:
            self.get_logger().error("Wrong configuration file: %s" % str(err))
            sys.exit(-1)

        self.frame_number = 0

        self.distance_threshold = self.get_fusion_parameters()["tracks_distance_threshold"]
        self.hungarian_threshold = self.get_fusion_parameters()["hungarian_threshold"]

        self.noMeasure = 2
        self.noQuantity = 4

        self.debug = self.get_parameter(
            "debug").get_parameter_value().bool_value

        if self.debug:
            self.markers_publisher = self.create_publisher(
                MarkerArray, DEBUG_MARKERS_TOPIC, 10
            )

        self.identity_index = 0
        self.sensor_tracks: List[SensorTrack] = []
        self.associated_tracks: List[AssociationTrack] = []
        
        self.test()

        self.reset_data()

        # create subscriber
        self.get_logger().info(f"Subscribed to {TRACKER_OUTPUT_TOPIC}")
        self.subscriber = self.create_subscription(
            Tracks, TRACKER_OUTPUT_TOPIC, self.tracks_callback, 10
        )

        # create publisher
        # self.get_logger().info(f"Publishing {FUSION_OUTPUT_TOPIC}")
        # self.publisher = self.create_publisher(
        #     String, FUSION_OUTPUT_TOPIC, 10
        # )
        # self.get_logger().info(
        #     f"Creating {sensor_fusion_topics['output_fused_track']}")
        # self.fused_tracks_publisher = self.create_publisher(
        #     String, sensor_fusion_topics["output_fused_track"], 10
        # )

        self.get_logger().info(f"Publishing to {FUSION_OUTPUT_TOPIC}")
        self.output_publisher = self.create_publisher(
            Tracks, FUSION_OUTPUT_TOPIC, 10
        )

    def get_kalman_parameters(self, sensor_id):
        try:
            return self.config["sensors"][sensor_id]["kalman_parameters"]
        except:
            pass
        return default_kalman_parameters
    
    def get_fusion_parameters(self):
        if "fusion_parameters" in self.config:
            if "tracks_distance_threshold" in self.config["fusion_parameters"] \
                and "hungarian_threshold" in self.config["fusion_parameters"]:
                return self.config["fusion_parameters"]
        return default_fusion_parameters

    def reset_data(self):
        self.identity_index = 0
        self.sensor_tracks: List[SensorTrack] = []
        self.associated_tracks: List[AssociationTrack] = []
        self.x_forward = X_FORWARD

    def center_to_pose_msg(self, center: Tuple):
        pose = Pose()
        pose.position.x = float(center[0])
        pose.position.y = float(center[1])
        return pose
    
    def vel_to_twist_msg(self, vel: Tuple):
        twist = Twist()
        twist.linear.x = float(vel[0])
        twist.linear.y = float(vel[1])
        return twist

    def arrayToPoses(self, list: List):
        poses = []
        for c in list:
            poses.append(self.center_to_pose_msg(c))
        return poses

    def test(self):
        self.get_logger().info("+++++++ Starting tests +++++")

        self.x_forward = False

        topic_data_1 = Tracks()
        topic_data_1.header.stamp = self.get_clock().now().to_msg()
        topic_data_1.sensor_id = 1
        topic_data_1.identities = [1, 2]
        topic_data_1.dead_identities = []
        topic_data_1.centers = self.arrayToPoses([(-1, 5), (1, 5)])
        topic_data_1.confidences = [0.8, 0.9]

        self.process_sensor_data(topic_data_1)

        topic_data_2 = Tracks()
        topic_data_2.header.stamp = self.get_clock().now().to_msg()
        topic_data_2.sensor_id = 2
        topic_data_2.identities = [3, 4, 5]
        topic_data_2.dead_identities = []
        topic_data_2.centers = self.arrayToPoses([(-0.8, 5.1), (1.2, 4.9), (0, 3)])
        topic_data_2.confidences = [0.8, 0.6, 0.9]

        self.process_sensor_data(topic_data_2)

        assert len(self.associated_tracks) == 3
        assert len(self.sensor_tracks) == 5
        assert self.sensor_tracks[0].fused_track_ref == 1
        assert self.sensor_tracks[1].fused_track_ref == 2
        assert self.sensor_tracks[2].fused_track_ref == 1
        assert self.sensor_tracks[3].fused_track_ref == 2
        assert self.sensor_tracks[4].fused_track_ref == 3

        topic_data_3 = Tracks()
        topic_data_3.header.stamp = self.get_clock().now().to_msg()
        topic_data_3.sensor_id = 1
        topic_data_3.identities = [1, 2]
        topic_data_3.dead_identities = []
        topic_data_3.centers = self.arrayToPoses([(-0.8, 4.7), (1.1, 4.6)])
        topic_data_3.confidences = [0.8, 0.9]

        self.process_sensor_data(topic_data_3)

        assert len(self.associated_tracks) == 3
        assert len(self.sensor_tracks) == 5
        assert self.associated_tracks[0].center == (-0.8, 4.7)
        assert self.associated_tracks[1].center == (1.1, 4.6)

        topic_data_4 = Tracks()
        topic_data_4.header.stamp = self.get_clock().now().to_msg()
        topic_data_4.sensor_id = 2
        topic_data_4.identities = []
        topic_data_4.dead_identities = [3,4,5]
        topic_data_4.centers = self.arrayToPoses([])
        topic_data_4.confidences = []

        self.process_sensor_data(topic_data_4)

        assert len(self.associated_tracks) == 2
        assert len(self.sensor_tracks) == 2

        self.get_logger().info("+++++++ Tests successful +++++")
        self.reset_data()

    def print_tracks(self):
        self.get_logger().info("Fused tracks (%d)" % len(self.associated_tracks))
        for track in self.associated_tracks:
            self.get_logger().info("  Fused track: %s" % str(track))
        self.get_logger().info("Sensor tracks (%d)" % len(self.sensor_tracks))
        for track in self.sensor_tracks:
            self.get_logger().info("  Sensor track: %s" % str(track))

    def kalman_init(self, sensor_id):

        # Init Kalman filter
        kf = KalmanFilter(dim_x=self.noQuantity, dim_z=self.noMeasure)

        # Set parameters
        kf.F = np.identity(4)
        if self.noQuantity == 2:
            kf.F[0, 1] = 0
        elif self.noQuantity == 4:
            kf.F[2, 3] = 0

        kf.u = 0.
        kf.H = np.array([[1., 0., 0., 0.],
                        [0., 0., 1., 0.]])

        kf.R = np.eye(2) * [self.get_kalman_parameters(sensor_id)["R_std"]["x"] **
                             2, self.get_kalman_parameters(sensor_id)["R_std"]["y"]**2]

        q_x = Q_discrete_white_noise(
            dim=self.noMeasure, dt=0, var=self.get_kalman_parameters(sensor_id)["Q_std"]["x"]**2)
        q_y = Q_discrete_white_noise(
            dim=self.noMeasure, dt=0, var=self.get_kalman_parameters(sensor_id)["Q_std"]["y"]**2)
        kf.Q = block_diag(q_x, q_y)

        kf.x = np.array([[0., 0., 0., 0.]]).T

        # Set high value of uncertainty (the first position is unknown)
        kf.P = np.eye(4) * 500.

        return kf

    def kalman_predict_and_update(self, track: AssociationTrack, only_predict: bool):
        if only_predict:
            track.kalman_filter.kf.predict()
            return
        # PREDICT
        # Update F,Q with current dt
        dt = (track.timestamp - track.kalman_filter.last_ts) / pow(10, 9)
        track.kalman_filter.kf.F[0, 1] = dt
        track.kalman_filter.kf.F[2, 3] = dt
        q_x = Q_discrete_white_noise(
            dim=self.noMeasure, dt=dt, var=self.get_kalman_parameters(track.current_sensor_id)["Q_std"]["x"]**2)
        q_y = Q_discrete_white_noise(
            dim=self.noMeasure, dt=dt, var=self.get_kalman_parameters(track.current_sensor_id)["Q_std"]["y"]**2)
        track.kalman_filter.kf.Q[0:2, 0:2] = q_x
        track.kalman_filter.kf.Q[2:4, 2:4] = q_y
        track.kalman_filter.kf.predict()
        # UPDATE
        # Update R
        track.kalman_filter.kf.R[0,
                                 0] = self.get_kalman_parameters(track.current_sensor_id)["R_std"]["x"]**2
        track.kalman_filter.kf.R[1,
                                 1] = self.get_kalman_parameters(track.current_sensor_id)["R_std"]["y"]**2
        track.kalman_filter.kf.update(np.asarray(track.center))
        track.kalman_filter.last_ts = track.timestamp

    def associate_sensor_tracks(self, distance_matrix, sensor_id):
        # filter distance matrix keeping only non associated tracks
        association_matrix, sensor_indices, fused_indices = self.compute_association_matrix(
            distance_matrix, sensor_id)
        if len(sensor_indices) > 0:
            self.get_logger().debug("Association matrix: \n%s \n[row_idx %s, col_idx %s]" % (
                str(association_matrix), str(sensor_indices), str(fused_indices)))

        # association matrix
        h_matrix, h_row_indices, h_col_indices = self.compute_hungarian(
            association_matrix)
        if len(h_row_indices) > 0:
            self.get_logger().debug("Hungarian matrix: \n%s \nrow indices: [%s], col indices: [%s]" % (
                str(h_matrix), str(h_row_indices), str(h_col_indices)))

        # associate tracks
        self.associate_new_tracks(
            h_row_indices, h_col_indices,
            sensor_indices, fused_indices,
            sensor_id)

        self.update_association_data()

    def process_sensor_data(self, tracks_data: Tracks):

        if len(tracks_data.identities) == 0 and len(tracks_data.dead_identities) == 0:
            self.get_logger().info("No identities")
            return

        self.add_sensor_tracks(tracks_data)

        distance_matrix = self.compute_distance_matrix()
        if distance_matrix is not np.nan:
            self.get_logger().debug("Distance matrix: \n%s" % str(distance_matrix))

        if self.has_distant_tracks(distance_matrix):
            # clear all associations if exists d(TSNj,TF) > th
            # and riassociate for each sensor
            self.dissociate_all_tracks()
            sensors = set()
            for s in self.sensor_tracks:
                sensors.add(s.sensor_id)
            for s_id in sensors:
                self.associate_sensor_tracks(distance_matrix, s_id)
                distance_matrix = self.compute_distance_matrix()
        else:
            self.associate_sensor_tracks(distance_matrix, tracks_data.sensor_id)

        self.remove_reference_to_dead_tracks()
        self.remove_fused_with_no_references()

    def tracks_callback(self, tracks_data: Tracks):

        self.process_sensor_data(tracks_data)

        self.publish_output(tracks_data.header.stamp, tracks_data.header.frame_id)

        if self.debug:
            self.publish_markers()

        self.get_logger().info("Tracks: %d" % (len(self.associated_tracks)))

        # if self.debug:
        #     centers = []
        #     identities = []
        #     for t in self.associated_tracks:
        #         if tracks_data.sensor_id == t.current_sensor_id:
        #             centers.append(
        #                 (t.kalman_filter.kf.x.T[0][0], t.kalman_filter.kf.x.T[0][2]))
        #             identities.append(t.identity)
        #     if len(centers) > 0:
        #         save_evaluation_data(
        #             0,
        #             centers,
        #             identities,
        #             self.frame_number,
        #             topic_data.timestamp,
        #             sensor_type="kalman_fusion",
        #             name=f"{self.get_name()}",
        #         )
        #     self.frame_number += 1

    def angle_between_vectors_np(self, v1_u, v2_u):
        angle_rad = np.arctan2(np.cross(v1_u, v2_u), np.dot(v1_u, v2_u))
        angle_deg = np.degrees(angle_rad)
        return angle_rad, angle_deg

    def get_track_marker(self, identity, center, velocity, confidence, frame_id):
        marker_lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        markers = []
        #add circle
        circle_marker = Marker()
        circle_marker.header.frame_id = frame_id
        circle_marker.header.stamp = self.get_clock().now().to_msg()
        circle_marker.type = 2
        circle_marker.id = identity
        circle_marker.lifetime = marker_lifetime
        circle_marker.scale.x = 0.1
        circle_marker.scale.y = 0.1
        circle_marker.scale.z = 0.0
        circle_marker.pose.position.x = center[0]
        circle_marker.pose.position.y = center[1]
        circle_marker.pose.position.z = 0.05
        circle_marker.pose.orientation.w = 1.0
        circle_marker.color.r = 0.0
        circle_marker.color.g = 1.0
        circle_marker.color.b = 0.0
        circle_marker.color.a = 1.0
        markers.append(circle_marker)

        # add arrow
        arrow_marker = Marker()
        arrow_marker.header.frame_id = frame_id
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.type = 0
        arrow_marker.id = identity*100
        arrow_marker.lifetime = marker_lifetime

        # comput velocity value and angle
        velocity_abs = np.abs(velocity)
        vel_abs = np.sqrt(pow(velocity_abs[0],2)+pow(velocity_abs[1],2))
        angle_rad, angle_deg = self.angle_between_vectors_np([1,0],[velocity[0],velocity[1]])
        vel_quat = quaternion_from_euler(0,0,angle_rad,'syzx')
        
        # Set the scale of the arrow_marker (normalize wrt max person velocity)
        max_vel = 1.5 # m/s
        arrow_marker.scale.x = vel_abs/max_vel
        arrow_marker.scale.y = 0.05
        arrow_marker.scale.z = 0.0
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0
        arrow_marker.pose.position.x = center[0]
        arrow_marker.pose.position.y = center[1]
        arrow_marker.pose.position.z = 0.0
        arrow_marker.pose.orientation.x = vel_quat[0]
        arrow_marker.pose.orientation.y = vel_quat[1]
        arrow_marker.pose.orientation.z = vel_quat[2]
        arrow_marker.pose.orientation.w = vel_quat[3]
        markers.append(arrow_marker)
        # add identity text
        text_marker = Marker()
        text_marker.header.frame_id = frame_id
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.type = 9
        text_marker.id = identity * 10000
        text_marker.text = f"(id:{identity})(conf:{round(confidence,1)})(vel_angle:{round(angle_deg)},vel:[{round(velocity[0],1)},{round(velocity[1],1)}])"
        text_marker.lifetime = marker_lifetime
        text_marker.scale.x = 1.0
        text_marker.scale.y = 1.0
        text_marker.scale.z = 0.1
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0
        text_marker.pose.position.x = center[0]
        text_marker.pose.position.y = center[1]
        text_marker.pose.position.z = 0.2
        text_marker.pose.orientation.x = 0.0
        text_marker.pose.orientation.y = 0.0
        text_marker.pose.orientation.z = 0.0
        text_marker.pose.orientation.w = 1.0
        markers.append(text_marker)
        return markers

    def sensor_track_id(self, track: SensorTrack):
        return f"S{track.sensor_id}_ID{track.identity}"

    def publish_markers(self):
        marker_array = MarkerArray()
        for f in self.associated_tracks:
            if len(f.sensor_track_refs) == 0:
                self.get_logger().error("No sensor tracks associated")
                continue
            center = self.get_position(f.kalman_filter.kf)
            vel = self.get_velocity(f.kalman_filter.kf)
            marker_array.markers.extend(self.get_track_marker(
                f.identity, center, vel, f.confidence, frame_id='world'))
        self.markers_publisher.publish(marker_array)

    def publish_output(self, stamp, frame_id):
        tracks = Tracks()
        tracks.header.stamp = stamp
        tracks.header.frame_id = frame_id
        for a in self.associated_tracks:
            tracks.identities.append(a.identity)
            center = self.get_position(a.kalman_filter.kf)
            vel = self.get_velocity(a.kalman_filter.kf)
            tracks.pose = self.center_to_pose_msg(center)
            tracks.velocity = self.vel_to_twist_msg(vel)
            tracks.confidences.append(a.confidence)
        self.output_publisher.publish(tracks)

    def get_position(self, kf: KalmanFilter):
        return [kf.x.T[0][0], kf.x.T[0][2]] 

    def get_velocity(self, kf: KalmanFilter):
        return [kf.x.T[0][1], kf.x.T[0][3]] 
    
    def dissociate_all_tracks(self):
        self.get_logger().info("Disassociating all tracks")
        for f in self.associated_tracks:
            f.sensor_track_refs = []
        for s in self.sensor_tracks:
            s.fused_track_ref = 0

    def associate_tracks(self, sensor_track: SensorTrack, fused_track: AssociationTrack):
        index = self.sensor_track_id(sensor_track)
        self.get_logger().info("Associate sensor track %s to %d"
                               % (index,
                                  fused_track.identity))
        fused_track.sensor_track_refs.append(sensor_track)
        sensor_track.fused_track_ref = fused_track.identity

    def to_exclude(self, center: Pose, sensor_id: int):
        # orientation: 0 = X forward, 1 = Y forward
        distance = center.position.x if self.x_forward else center.position.y
        if sensor_id in sensor_exclusion:
            if sensor_exclusion[sensor_id]["condition"] == "gt" and distance > sensor_exclusion[sensor_id]["value"]:
                return True
            if sensor_exclusion[sensor_id]["condition"] == "lt" and distance < sensor_exclusion[sensor_id]["value"]:
                return True
        return False

    def add_sensor_tracks(self, tracks_data: Tracks):
        self.get_logger().debug("Add identities %s (dead: %s) from sensor %d" % (
            str(tracks_data.identities), str(tracks_data.dead_identities), tracks_data.sensor_id))
        for sIndex, id in enumerate(tracks_data.identities + tracks_data.dead_identities):
            if id in tracks_data.identities and self.to_exclude(tracks_data.centers[sIndex], tracks_data.sensor_id):
                self.get_logger().info("Excluding Id %d of sensor %d" % (id, tracks_data.sensor_id))
                continue
            found = False
            for tIndex, s in enumerate(self.sensor_tracks):
                if s.sensor_id == tracks_data.sensor_id and s.identity == id:  # update existing sensor track
                    found = True
                    if id in tracks_data.dead_identities:
                        s.dead = True
                    else:
                        self.sensor_tracks[tIndex].timestamp = get_timestamp(tracks_data)
                        self.sensor_tracks[tIndex].center = (tracks_data.centers[sIndex].position.x, tracks_data.centers[sIndex].position.y)
                        self.sensor_tracks[tIndex].confidence = tracks_data.confidences[sIndex]
                    break
            if not found and id not in tracks_data.dead_identities:  # add new sensor track
                new_track = SensorTrack()
                new_track.identity = id
                new_track.sensor_id = tracks_data.sensor_id
                new_track.center = (tracks_data.centers[sIndex].position.x, tracks_data.centers[sIndex].position.y)
                new_track.timestamp = get_timestamp(tracks_data)
                new_track.confidence = tracks_data.confidences[sIndex]
                self.sensor_tracks.append(new_track)
                self.get_logger().debug("Add new sensor track %s" % str(new_track))

    def get_new_identity(self):
        self.identity_index += 1
        return self.identity_index

    def compute_distance_matrix(self):
        if len(self.sensor_tracks) == 0 or len(self.associated_tracks) == 0:
            return np.nan
        matrix = np.full(
            (len(self.sensor_tracks), len(self.associated_tracks)), np.inf)
        for sIndex, s in enumerate(self.sensor_tracks):
            for fIndex, f in enumerate(self.associated_tracks):
                # center = [f.kalman_filter.kf.x.T[0]
                #           [0], f.kalman_filter.kf.x.T[0][2]]
                a = np.array(s.center)
                b = np.array(f.center)
                matrix[sIndex, fIndex] = np.linalg.norm(a - b)
        return matrix

    def has_distant_tracks(self, distance_matrix):
        if distance_matrix is np.nan:
            return False
        for sIndex, s in enumerate(self.sensor_tracks):
            for fIndex, f in enumerate(self.associated_tracks):
                if s.fused_track_ref != f.identity:
                    continue
                if distance_matrix[sIndex, fIndex] > self.distance_threshold:
                    self.get_logger().info("Distance between %s and %d is %.3f > %.3f"
                                           % (self.sensor_track_id(s), f.identity,
                                              distance_matrix[sIndex, fIndex], self.distance_threshold))
                    return True
        return False

    def compute_association_matrix(self, distance_matrix, sensor_id):
        if distance_matrix is np.nan:
            return np.nan, [], []
        matrix = np.full(distance_matrix.shape, np.inf)
        row_idx = 0
        col_idx = 0
        sensor_indices = []
        fused_indices = []
        added_indices = False
        for sIndex, s in enumerate(self.sensor_tracks):
            if s.dead or s.fused_track_ref != 0 or s.sensor_id != sensor_id:
                continue
            col_idx = 0
            for fIndex, f in enumerate(self.associated_tracks):
                sensors = [r.sensor_id for r in f.sensor_track_refs]
                if s.sensor_id in sensors:
                    continue
                distance = distance_matrix[sIndex, fIndex]
                self.get_logger().debug("[%d,%d] distance %.3f from sensor %s to fused %d"
                                        % (row_idx, col_idx, distance, self.sensor_track_id(s), f.identity))
                matrix[row_idx, col_idx] = distance
                col_idx += 1
                if not added_indices:
                    fused_indices.append(fIndex)
            added_indices = True
            sensor_indices.append(sIndex)
            row_idx += 1
        if row_idx == 0:
            return np.nan, [], []
        return matrix[:len(sensor_indices), :len(fused_indices)], sensor_indices, fused_indices

    def compute_hungarian(self, association_matrix):
        if association_matrix is np.nan:
            return [], [], []
        max_x = association_matrix.shape[0]
        max_y = association_matrix.shape[1]
        size = max_x + max_y
        matrix = np.full((size, size), np.inf)
        matrix[max_x: size, max_y: size] = 0
        matrix[:max_x, : max_y] = association_matrix
        matrix[max_x: size, max_y: size] = 0
        np.fill_diagonal(matrix[0:, max_y:],
                         self.hungarian_threshold)
        np.fill_diagonal(matrix[max_x:, 0:],
                         self.hungarian_threshold)
        try:
            row_indices, col_indices = linear_sum_assignment(matrix)
        except:
            self.get_logger().error("Failed to compute Hungarian")
            return [], []
        return matrix, row_indices, col_indices

    def associate_new_tracks(self, h_row_indices, h_col_indices, sensor_indices, fused_indices, sensor_id):
        # associate matching tracks
        for index, r in enumerate(h_row_indices):
            c = h_col_indices[index]
            if r > len(sensor_indices) - 1:
                continue
            if c > len(fused_indices) - 1:
                continue
            st = self.sensor_tracks[sensor_indices[r]]
            ft = self.associated_tracks[fused_indices[c]]
            self.associate_tracks(st, ft)
        # create fused tracks from remaining sensor tracks
        for sIndex, s in enumerate(self.sensor_tracks):
            if not s.dead and s.fused_track_ref == 0 and s.sensor_id == sensor_id:
                new_identity = self.get_new_identity()
                self.get_logger().info("Add new fused track %d" % new_identity)
                new_track = AssociationTrack()
                new_track.identity = new_identity
                new_track.center = s.center
                new_track.kalman_filter.kf = self.kalman_init(s.sensor_id)
                new_track.confidence = s.confidence
                self.associate_tracks(s, new_track)
                self.associated_tracks.append(new_track)

    def update_association_data(self):
        for f in self.associated_tracks:
            if len(f.sensor_track_refs) == 0:
                continue
            selected = -1
            maxTs = 0
            for index, s in enumerate(f.sensor_track_refs):
                if maxTs < s.timestamp:
                    maxTs = s.timestamp
                    selected = index
            if selected > -1:
                # update with more recent sensor track
                updated = f.timestamp != f.sensor_track_refs[selected].timestamp
                if updated:
                    f.center = f.sensor_track_refs[selected].center
                    f.current_sensor_id = f.sensor_track_refs[selected].sensor_id
                    f.timestamp = f.sensor_track_refs[selected].timestamp
                    f.confidence = f.sensor_track_refs[selected].confidence
                    self.kalman_predict_and_update(f, False)
                # else:
                    # self.kalman_predict_and_update(f, True)

    def remove_reference_to_dead_tracks(self):
        for s in self.sensor_tracks:
            for f in self.associated_tracks:
                if s not in f.sensor_track_refs:
                    continue
                if s.dead:
                    index = self.sensor_track_id(s)
                    self.get_logger().debug("Remove sensor track ref %s from %d" % (index, f.identity))
                    f.sensor_track_refs.remove(s)
        # filter dead tracks
        self.sensor_tracks = [s for s in self.sensor_tracks if not s.dead]

    def remove_fused_with_no_references(self):
        self.associated_tracks = [
            f for f in self.associated_tracks if len(f.sensor_track_refs) > 0]

    # def publish(self, topic_data: SensorTrackedData):
        # """
        # This data is used to publish the results to the subscriber
        # """
        # fused_coordinates = []
        # fused_identities = []
        # for s in self.sensor_tracks:
        #     if s.sensor_id == topic_data.idx and s.fused_track_ref != 0:
        #         fused_identities.append(s.fused_track_ref)
        #         for f in self.associated_tracks:
        #             if f.identity == s.fused_track_ref:
        #                 fused_coordinates.append(f.center)
        #                 break
        # topic_data.fused_identities = fused_identities
        # msg = String()
        # msg.data = json.dumps(topic_data.model_dump())
        # self.publisher.publish(msg)
        # for f in self.associated_tracks:
        #     msg = String()
        #     msg.data = json.dumps(f.model_dump())
        #     self.fused_tracks_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    track_association = TrackFusion()
    rclpy.spin(track_association)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    track_association.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
