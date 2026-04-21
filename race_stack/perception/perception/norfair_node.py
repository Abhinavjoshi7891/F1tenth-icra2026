#!/usr/bin/env python3

import time
import numpy as np
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, HistoryPolicy

from frenet_conversion.frenet_converter import FrenetConverter
from f110_msgs.msg import ObstacleArray, Obstacle, WpntArray, OpponentTrajectories
from visualization_msgs.msg import Marker, MarkerArray
from norfair import Detection, Tracker, OptimizedKalmanFilterFactory
try:
    from norfair.tracker import TrackedObject
except ImportError:
    # Fallback for different norfair versions
    from norfair import TrackedObject
from perception.norfair_helpers import id_to_rgb_color
from copy import deepcopy
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener

class NorfairNode(Node):
    def __init__(self):
        super().__init__('norfair_ros')
        
        # Parameters
        self.declare_parameter('visualize', True)
        self.declare_parameter('input_topic', '/perception/detection/raw_obstacles')
        self.declare_parameter('rate', 40.0)
        
        self.visualize = self.get_parameter('visualize').value
        input_topic = self.get_parameter('input_topic').value
        self.rate_hz = self.get_parameter('rate').value
        
        # State Gates
        self.converter = None
        self.detections = None
        self.opp_trajectories = None
        
        # Norfair Setup Parameters (mirrored from ROS 1 config)
        self.declare_parameter('norfair_setup.distance_function', 'euclidean')
        self.declare_parameter('norfair_setup.distance_threshold', 0.9)
        self.declare_parameter('norfair_setup.hit_counter_max', 15)
        self.declare_parameter('norfair_setup.initialization_delay', 9)
        self.declare_parameter('norfair_setup.pointwise_hit_counter_max', 4)
        self.declare_parameter('norfair_setup.past_detections_length', 4)
        self.declare_parameter('norfair_setup.reid_distance_threshold', 0.1)
        self.declare_parameter('norfair_setup.reid_hit_counter_max', 1000)
        
        # Kalman Filter Parameters
        self.declare_parameter('norfair_setup.kalman_filter.R', 0.074)
        self.declare_parameter('norfair_setup.kalman_filter.Q', 0.00017)
        self.declare_parameter('norfair_setup.kalman_filter.pos_variance', 10.0)
        self.declare_parameter('norfair_setup.kalman_filter.pos_vel_covariance', 0.0)
        self.declare_parameter('norfair_setup.kalman_filter.vel_variance', 1.0)

        # Initialize Tracker
        self.setup_tracker()

        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.tracked_obstacles_pub = self.create_publisher(ObstacleArray, '/perception/obstacles', 5)
        self.tracked_markers_pub = self.create_publisher(MarkerArray, '/perception/static_dynamic_marker_pub', 5)
        self.latency_pub = self.create_publisher(Float32, '/perception/tracking/latency', 10)

        # Subscribers
        self.create_subscription(WpntArray, '/global_waypoints', self.waypoints_cb, 10)
        self.create_subscription(ObstacleArray, input_topic, self.detection_cb, 10)
        self.create_subscription(OpponentTrajectories, "/perception/opp_trajectories", self.opp_trajectories_cb, 10)

        # Main Timer
        self.timer = self.create_timer(1.0 / self.rate_hz, self.pipeline)
        
        self.get_logger().info("Norfair Tracker Node Initialized")

    def setup_tracker(self):
        distance_threshold = self.get_parameter('norfair_setup.distance_threshold').value
        hit_counter_max = self.get_parameter('norfair_setup.hit_counter_max').value
        initialization_delay = self.get_parameter('norfair_setup.initialization_delay').value
        pointwise_hit_counter_max = self.get_parameter('norfair_setup.pointwise_hit_counter_max').value
        past_detections_length = self.get_parameter('norfair_setup.past_detections_length').value
        reid_distance_threshold = self.get_parameter('norfair_setup.reid_distance_threshold').value
        reid_hit_counter_max = self.get_parameter('norfair_setup.reid_hit_counter_max').value

        filter_factory = OptimizedKalmanFilterFactory(
            R=self.get_parameter('norfair_setup.kalman_filter.R').value,
            Q=self.get_parameter('norfair_setup.kalman_filter.Q').value,
            pos_variance=self.get_parameter('norfair_setup.kalman_filter.pos_variance').value,
            pos_vel_covariance=self.get_parameter('norfair_setup.kalman_filter.pos_vel_covariance').value,
            vel_variance=self.get_parameter('norfair_setup.kalman_filter.vel_variance').value,
        )

        self.tracker = Tracker(
            distance_function=self.get_parameter('norfair_setup.distance_function').value,
            distance_threshold=distance_threshold,
            hit_counter_max=hit_counter_max,
            initialization_delay=initialization_delay,
            pointwise_hit_counter_max=pointwise_hit_counter_max,
            past_detections_length=past_detections_length,
            filter_factory=filter_factory,
            reid_distance_function=self.reid_distance_function,
            reid_distance_threshold=reid_distance_threshold,
            reid_hit_counter_max=reid_hit_counter_max
        )

    def reid_distance_function(self, obj1: TrackedObject, obj2: TrackedObject) -> float:
        if self.opp_trajectories is not None and obj2.id in self.opp_trajectories:
            trajectory = self.opp_trajectories[obj2.id]
            dist = np.linalg.norm(obj1.estimate - trajectory, axis=1).min()
            return dist
        return np.linalg.norm(obj1.estimate - obj2.estimate)

    def waypoints_cb(self, msg: WpntArray):
        if self.converter is None:
            wpnt_arr = np.array([[wpnt.x_m, wpnt.y_m, wpnt.psi_rad] for wpnt in msg.wpnts])
            self.converter = FrenetConverter(wpnt_arr[:, 0], wpnt_arr[:, 1], wpnt_arr[:, 2])
            self.track_length = msg.wpnts[-1].s_m + np.linalg.norm(np.array([msg.wpnts[-1].x_m - msg.wpnts[0].x_m, msg.wpnts[-1].y_m - msg.wpnts[0].y_m]))
            self.get_logger().info("Norfair Tracker: initialized FrenetConverter")

    def detection_cb(self, msg: ObstacleArray):
        self.detections = msg
        num_raw = len(msg.obstacles)
        if num_raw > 0:
            self.get_logger().info(f"Norfair Input: Received {num_raw} raw detections", once=False, skip_first=True)

    def opp_trajectories_cb(self, msg: OpponentTrajectories):
        self.opp_trajectories = {}
        for traj in msg.trajectories:
            self.opp_trajectories[traj.id] = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in traj.oppwpnts])

    def pipeline(self):
        if self.converter is None or self.detections is None:
            return

        start = time.perf_counter()
        
        # Process current detections
        detections_msg = deepcopy(self.detections)
        norfair_detections = []
        
        for obstacle in detections_msg.obstacles:
            # Detections are already in Cartesian in the ObstacleArray provided by detect.py
            # But we double check if we need to transform. 
            # In ROS 2 abd_tracker, detect.py publishes in 'map' frame.
            detections_point = self.converter.get_cartesian(obstacle.s_center, obstacle.d_center)
            norfair_detections.append(Detection(points=np.array([[detections_point[0], detections_point[1]]])))

        # Tracker Update
        tracked_objects = self.tracker.update(norfair_detections)
        
        num_tracked = len(tracked_objects)
        self.get_logger().info(f"Norfair Output: Tracking {num_tracked} objects", throttle_duration_sec=1.0)

        # Publish
        self.publish_tracked(tracked_objects, detections_msg.header)
        
        end = time.perf_counter()
        latency_msg = Float32()
        latency_msg.data = float(end - start)
        self.latency_pub.publish(latency_msg)

    def publish_tracked(self, tracked_objects: List[TrackedObject], header):
        # Obstacle Array
        obs_array = ObstacleArray()
        obs_array.header = header
        obs_array.header.frame_id = 'map'

        for obj in tracked_objects:
            x, y = obj.estimate[0, 0], obj.estimate[0, 1]
            vx, vy = obj.estimate_velocity[0, 0], obj.estimate_velocity[0, 1]
            
            sd = self.converter.get_frenet([x], [y])
            v_sd = self.converter.get_frenet_velocities(vx, vy, 0, sd[0])
            
            obs = Obstacle()
            obs.id = int(obj.id)
            obs.size = 0.5
            obs.s_center = float(sd[0])
            obs.d_center = float(sd[1])
            obs.vs = float(v_sd[0] * self.rate_hz)
            obs.vd = float(v_sd[1] * self.rate_hz)
            obs.s_var = float(obj.estimate_variance[0, 0])
            obs.d_var = float(obj.estimate_variance[0, 1])
            obs.is_static = bool(abs(obs.vs) < 0.1)
            
            obs_array.obstacles.append(obs)
        
        self.tracked_obstacles_pub.publish(obs_array)

        # Visualization
        if self.visualize:
            self.publish_markers(tracked_objects, header)

    def publish_markers(self, tracked_objects: List[TrackedObject], header):
        marker_array = MarkerArray()
        for obj in tracked_objects:
            r, g, b = id_to_rgb_color(obj.id)
            x, y = obj.estimate[0, 0], obj.estimate[0, 1]

            # Sphere Marker
            m = Marker()
            m.header = header
            m.header.frame_id = 'map'
            m.id = int(obj.id)
            m.type = Marker.SPHERE
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.5
            m.scale.x = 0.5
            m.scale.y = 0.5
            m.scale.z = 1.0
            m.color.a = 1.0
            m.color.r = float(r)
            m.color.g = float(g)
            m.color.b = float(b)
            m.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
            marker_array.markers.append(m)

            # ID Text Marker
            tm = Marker()
            tm.header = header
            tm.header.frame_id = 'map'
            tm.id = int(obj.id + 1000)
            tm.type = Marker.TEXT_VIEW_FACING
            tm.pose.position.x = float(x)
            tm.pose.position.y = float(y)
            tm.pose.position.z = 1.2
            tm.scale.z = 0.5
            tm.color.a = 1.0
            tm.color.r = 1.0
            tm.color.g = 1.0
            tm.color.b = 1.0
            tm.text = str(obj.id)
            tm.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
            marker_array.markers.append(tm)
            
        self.tracked_markers_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = NorfairNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
