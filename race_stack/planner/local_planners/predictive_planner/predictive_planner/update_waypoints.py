#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from f110_msgs.msg import WpntArray
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String
import time

class WaypointUpdater(Node):
    """
    Update the scaled waypoints to get a better estimate of one's position and speed.
    """
    def __init__(self):
        super().__init__('waypoint_updater')

        # Parameters
        self.declare_parameter('update_waypoints', True)
        self.declare_parameter('speed_offset', 0.0)
        self.declare_parameter('hysteresis_time', 2.0)
        
        self.update_waypoints = self.get_parameter('update_waypoints').value
        self.speed_offset = self.get_parameter('speed_offset').value
        self.hysteresis_time = self.get_parameter('hysteresis_time').value

        # State tracking
        self.state = "GB_TRACK"
        self.gb_track_start_time = None
        self.max_speed_scaled = 10.0
        
        # Data storage
        self.wpnts_scaled_msg = None
        self.wpnts_updated_msg = None
        self.s_points_array = np.array([])

        # Subscribers
        self.create_subscription(Odometry, "/car_state/odom_frenet", self.odom_cb, 10)
        self.create_subscription(String, "/state_machine", self.state_machine_cb, 10)
        self.create_subscription(WpntArray, "/global_waypoints_scaled", self.scaled_wpnts_cb, 10)
        self.create_subscription(WpntArray, "/global_waypoints", self.gb_wpnts_cb, 10)

        # Publishers
        self.wpnts_updated_pub = self.create_publisher(WpntArray, "/global_waypoints_updated", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/updated_waypoints_marker", 10)

        # Timer for main loop (1Hz as per ROS 1)
        self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Waypoint Updater Node Initialized")

    def state_machine_cb(self, data: String):
        self.state = data.data
        if self.state == "GB_TRACK" and self.gb_track_start_time is None:
            self.gb_track_start_time = time.time()
        elif self.state != "GB_TRACK":
            self.gb_track_start_time = None

    def scaled_wpnts_cb(self, msg: WpntArray):
        if self.wpnts_scaled_msg is None:
            self.get_logger().info("Received initial scaled waypoints")
        self.wpnts_scaled_msg = msg
        if self.wpnts_updated_msg is None:
            self.wpnts_updated_msg = msg
            self.s_points_array = np.array([wpnt.s_m for wpnt in msg.wpnts])

    def gb_wpnts_cb(self, msg: WpntArray):
        if len(msg.wpnts) > 0:
            self.max_speed_scaled = max([wpnt.vx_mps for wpnt in msg.wpnts])

    def odom_cb(self, data: Odometry):
        if not self.update_waypoints or self.wpnts_updated_msg is None or self.s_points_array.size == 0:
            return

        ego_position = data.pose.pose.position.x
        ego_speed = data.twist.twist.linear.x
        
        # Find closest waypoint
        ego_approx_indx = np.abs(self.s_points_array - ego_position).argmin()
        
        # Hysteresis check
        if self.state == "GB_TRACK" and self.gb_track_start_time is not None:
            if (time.time() - self.gb_track_start_time) >= self.hysteresis_time:
                self.wpnts_updated_msg.wpnts[ego_approx_indx].vx_mps = float(ego_speed + self.speed_offset)
                
                # Sync first and last if they are the same point (lap closure)
                if ego_approx_indx == 0 or ego_approx_indx == (len(self.wpnts_updated_msg.wpnts) - 1):
                    val = self.wpnts_updated_msg.wpnts[ego_approx_indx].vx_mps
                    self.wpnts_updated_msg.wpnts[0].vx_mps = val
                    self.wpnts_updated_msg.wpnts[-1].vx_mps = val

    def visualize_waypoints(self):
        if self.wpnts_updated_msg is None:
            return
            
        marker_array = MarkerArray()
        for i, wpnt in enumerate(self.wpnts_updated_msg.wpnts):
            if i % 5 != 0: continue # Subsample for performance
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.pose.position.x = wpnt.x_m
            marker.pose.position.y = wpnt.y_m
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = float(wpnt.vx_mps / self.max_speed_scaled)
            marker.pose.position.z = marker.scale.z / 2.0
            marker.color.a = 1.0
            marker.color.g = 1.0
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

    def timer_callback(self):
        if self.wpnts_updated_msg is None:
            return
            
        self.wpnts_updated_msg.header.stamp = self.get_clock().now().to_msg()
        self.wpnts_updated_pub.publish(self.wpnts_updated_msg)
        self.visualize_waypoints()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointUpdater()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
