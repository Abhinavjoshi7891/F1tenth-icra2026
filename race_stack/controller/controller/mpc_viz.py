#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from f110_msgs.msg import WpntArray
from visualization_msgs.msg import Marker, MarkerArray
from frenet_conversion.frenet_converter import FrenetConverter
import numpy as np

class MPCVizNode(Node):
    def __init__(self):
        super().__init__('mpc_viz')
        self.wpnt_sub = self.create_subscription(WpntArray, '/global_waypoints', self.waypoints_cb, 10)
        self.state_sub = self.create_subscription(Float64MultiArray, '/mpc_controller/states', self.states_cb, 10)
        self.pred_pub = self.create_publisher(MarkerArray, '/mpc_controller/predicted_trajectory', 10)
        self.bound_pub = self.create_publisher(MarkerArray, '/mpc_controller/track_boundaries', 10)

        self.fren_conv = None
        self.waypoints = None
        self.model_n_x = 8
        self.get_logger().info("MPC Viz Node Started. Listening for /global_waypoints...")

    def waypoints_cb(self, msg):
        if self.fren_conv is not None:
            return 
        self.get_logger().info(f"Received {len(msg.wpnts)} waypoints. Initializing FrenetConverter...")
        x = np.array([wp.x_m for wp in msg.wpnts])
        y = np.array([wp.y_m for wp in msg.wpnts])
        psi = np.array([wp.psi_rad for wp in msg.wpnts])
        self.fren_conv = FrenetConverter(x, y, psi)
        self.waypoints = msg.wpnts
        self.publish_static_boundaries()

    def states_cb(self, msg):
        if self.fren_conv is None:
            return
        
        total_states = len(msg.data)
        if total_states % 8 == 0:
            self.model_n_x = 8
        elif total_states % 5 == 0:
            self.model_n_x = 5
        
        N_plus_1 = total_states // self.model_n_x
        mpc_s = []
        mpc_n = []
        
        # Track length for wrapping s
        track_len = self.fren_conv.raceline_length

        for i in range(N_plus_1):
            s_val = msg.data[i * self.model_n_x + 0]
            mpc_s.append(s_val % track_len) # Wrap around track length
            mpc_n.append(msg.data[i * self.model_n_x + 1])

        traj_x, traj_y = self.fren_conv.get_cartesian(np.array(mpc_s), np.array(mpc_n))
            
        self.publish_trajectory(traj_x, traj_y)

    def publish_trajectory(self, x_coords, y_coords):
        markers = MarkerArray()
        for i in range(len(x_coords)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "predicted_path"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x_coords[i])
            marker.pose.position.y = float(y_coords[i])
            marker.pose.position.z = 0.5
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            markers.markers.append(marker)
        self.pred_pub.publish(markers)

    def publish_static_boundaries(self):
        if self.waypoints is None:
            return
        markers = MarkerArray()
        for i, wp in enumerate(self.waypoints):
            if i % 10 != 0: continue 
            markers.markers.append(self.create_boundary_marker(wp.x_m, wp.y_m, wp.psi_rad, wp.d_left, i, "left", [0.0, 1.0, 0.0]))
            markers.markers.append(self.create_boundary_marker(wp.x_m, wp.y_m, wp.psi_rad, -wp.d_right, i + 10000, "right", [1.0, 0.0, 0.0]))
        self.bound_pub.publish(markers)

    def create_boundary_marker(self, x, y, psi, d, m_id, ns, color):
        bx = x - d * np.sin(psi)
        by = y + d * np.cos(psi)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = m_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(bx)
        marker.pose.position.y = float(by)
        marker.pose.position.z = 0.2
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = MPCVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
