#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from f110_msgs.msg import ObstacleArray, Obstacle, WpntArray
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from frenet_conversion.frenet_converter import FrenetConverter
import numpy as np
import math

class RandomObstaclePublisher(Node):
    """
    ROS 2 Node to publish random static obstacles on the track.
    Ported from legacy f110_utils with added visualization.
    """

    def __init__(self):
        super().__init__('random_obstacle_publisher')

        # ---- Parameters ----
        self.declare_parameter('n_obstacles', 5)
        self.declare_parameter('publish_at_lookahead', False)
        self.declare_parameter('lookahead_distance', 5.0)
        self.declare_parameter('obstacle_width', 0.25)
        self.declare_parameter('obstacle_length', 0.25)
        self.declare_parameter('obstacle_max_d_from_traj', 1.0)
        self.declare_parameter('rnd_seed', 42)
        self.declare_parameter('rate', 25.0)

        self.n_sectors = self.get_parameter('n_obstacles').value + 1
        self.publish_at_lookahead = self.get_parameter('publish_at_lookahead').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.obstacle_width = self.get_parameter('obstacle_width').value
        self.obstacle_length = self.get_parameter('obstacle_length').value
        self.obstacle_max_d_from_traj = self.get_parameter('obstacle_max_d_from_traj').value
        seed = self.get_parameter('rnd_seed').value
        rate_hz = self.get_parameter('rate').value

        # ---- State ----
        self.obstacle_array = []
        self.gb_wpnts = None
        self.ego_s = 0.0
        self.final_s = 0.0
        self.gen = np.random.default_rng(seed)
        self.obstacles_generated = False
        self.converter = None

        # ---- Subscribers & Publishers ----
        self.create_subscription(WpntArray, '/global_waypoints', self.global_waypoints_cb, 10)
        self.create_subscription(Odometry, '/car_state/odom_frenet', self.odom_cb, 10)
        
        self.obstacle_pub = self.create_publisher(ObstacleArray, '/perception/obstacles', 10)
        self.marker_pub   = self.create_publisher(MarkerArray, '/perception/random_obstacle_markers', 10)

        # ---- Main Timer ----
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_cb)

        self.get_logger().info('Random Obstacle Publisher initialized. Waiting for waypoints...')

    def global_waypoints_cb(self, msg):
        """Store waypoints, initialize converter, and generate obstacles once."""
        if self.gb_wpnts is not None:
            return
            
        self.gb_wpnts = msg.wpnts
        self.final_s = msg.wpnts[-1].s_m
        
        # Initialize Frenet Converter for Map visualization
        self.waypoints_xy = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in msg.wpnts])
        self.psi_array = np.array([wpnt.psi_rad for wpnt in msg.wpnts])
        self.converter = FrenetConverter(self.waypoints_xy[:, 0], self.waypoints_xy[:, 1], self.psi_array)
        
        self.get_logger().info(f'Received waypoints. Generating {self.n_sectors-1} obstacles...')
        self.generate_obstacles()
        self.obstacles_generated = True

    def odom_cb(self, msg):
        """Update ego position for lookahead mode."""
        self.ego_s = msg.pose.pose.position.x

    def generate_obstacles(self):
        """Logic from ROS 1: split track into sectors and place one obstacle in each."""
        self.obstacle_array.clear()
        s_spacing = self.final_s / self.n_sectors
        margin = max(0.5, self.obstacle_length)

        for sec in range(self.n_sectors - 1): # n_sectors is n_obstacles + 1
            s_start = (sec + 1) * s_spacing # start from second sector to avoid spawning on ego
            s_end = s_start + s_spacing - margin
            
            # Generate random Frenet coordinates
            ob_s = s_start + (s_end - s_start) * self.gen.random()
            
            # Get closest waypoint for track width limits
            wpt = self.get_closest_waypoint(ob_s)
            track_right = -min(wpt.d_right, self.obstacle_max_d_from_traj)
            track_left = min(wpt.d_left - self.obstacle_width, self.obstacle_max_d_from_traj)

            ob_d_right = track_right + (track_left - track_right) * self.gen.random()

            # Build Obstacle msg
            ob = Obstacle()
            ob.id = sec
            ob.s_start = ob_s
            ob.s_end = ob_s + self.obstacle_length
            ob.d_right = ob_d_right
            ob.d_left = ob_d_right + self.obstacle_width
            ob.s_center = ob_s + self.obstacle_length / 2.0
            ob.d_center = ob_d_right + self.obstacle_width / 2.0
            ob.size = (self.obstacle_width + self.obstacle_length) / 2.0
            ob.vs = 0.0
            ob.vd = 0.0
            ob.is_static = True
            ob.is_visible = True
            ob.is_actually_a_gap = False
            
            self.obstacle_array.append(ob)
        
        self.get_logger().info('Obstacles generated successfully.')

    def get_closest_waypoint(self, s):
        """Find waypoint closest to arc-length s."""
        min_dist = float('inf')
        closest_wpt = self.gb_wpnts[0]
        for wpt in self.gb_wpnts:
            dist = (wpt.s_m - s)**2
            if dist < min_dist:
                min_dist = dist
                closest_wpt = wpt
        return closest_wpt

    def timer_cb(self):
        """Publish the generated obstacles and their markers."""
        if not self.obstacles_generated:
            return

        msg = ObstacleArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'frenet'

        # Filter if lookahead is enabled
        obs_to_pub = []
        if self.publish_at_lookahead:
            for ob in self.obstacle_array:
                # Proper wrapping detection for lookahead
                dist = (ob.s_start - self.ego_s) % self.final_s
                if dist < self.lookahead_distance:
                    obs_to_pub.append(ob)
        else:
            obs_to_pub = self.obstacle_array

        msg.obstacles = obs_to_pub
        self.obstacle_pub.publish(msg)
        
        # Publish Visualization Markers
        if self.converter is not None:
            self.publish_markers(obs_to_pub)

    def publish_markers(self, obstacles):
        """Publish spheres to RViz for each active obstacle."""
        marker_array = MarkerArray()

        for ob in obstacles:
            xy = self.converter.get_cartesian(ob.s_center, ob.d_center)
            
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp    = self.get_clock().now().to_msg()
            m.ns              = "random_obstacles"
            m.id              = ob.id
            m.type            = Marker.CUBE
            m.action          = Marker.ADD
            m.scale.x         = self.obstacle_length
            m.scale.y         = self.obstacle_width
            m.scale.z         = 0.5
            m.color.a         = 0.8
            m.color.r         = 1.0 # Red color for obstacles
            m.color.g         = 0.0
            m.color.b         = 0.0
            m.pose.position.x = float(xy[0])
            m.pose.position.y = float(xy[1])
            m.pose.position.z = 0.25
            m.pose.orientation.w = 1.0
            marker_array.markers.append(m)
            
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RandomObstaclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
