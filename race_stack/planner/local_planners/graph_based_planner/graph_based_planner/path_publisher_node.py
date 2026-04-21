#!/usr/bin/env python3
"""Parity port of ROS 1 path_publisher.py.

NOTE: Not used in head_to_head (state machine owns /local_waypoints).
Ship it for feature parity with ROS 1 standalone planner.launch.
Do NOT launch it alongside the state machine — they fight for /local_waypoints.
"""
import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from f110_msgs.msg import WpntArray, Wpnt


class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.declare_parameter('path_length', 80)
        self.declare_parameter('loop_rate_hz', 80.0)

        self.path_length  = self.get_parameter('path_length').value
        self.loop_rate_hz = self.get_parameter('loop_rate_hz').value

        self.selected_path = None
        self.sel_action = None
        self.frenet_pose = None
        self.waypoints_original = None
        self.max_s = None
        self.close_to_s0 = False
        self.speed_scaling = 1.0

        self.create_subscription(WpntArray,         '/global_waypoints',      self.waypoint_cb,     10)
        self.create_subscription(Float32MultiArray, '/planner_path',           self.planner_path_cb, 10)
        self.create_subscription(Odometry,          '/car_state/odom_frenet', self.odom_fre_cb,     10)

        self.cmd_pub  = self.create_publisher(WpntArray,   '/local_waypoints',         10)
        self.wpnt_viz = self.create_publisher(MarkerArray, '/local_waypoints/markers', 10)

        self.create_timer(1.0 / self.loop_rate_hz, self._tick)

    def waypoint_cb(self, data: WpntArray):
        self.waypoints_original = np.array([
            [w.s_m, w.d_m, w.x_m, w.y_m, w.d_right, w.d_left,
             w.psi_rad, w.kappa_radpm, w.vx_mps, w.ax_mps2]
            for w in data.wpnts
        ])
        self.max_s = float(np.amax(self.waypoints_original[:, 0]))

    def odom_fre_cb(self, data: Odometry):
        p = data.pose.pose.position
        self.frenet_pose = np.array([p.x, p.y])

    def planner_path_cb(self, data: Float32MultiArray):
        arr = np.asarray(data.data)
        self.selected_path = arr.reshape(data.layout.dim[0].size, data.layout.dim[1].size)
        self.sel_action = data.layout.dim[0].label

    def _find_nearest_s(self, station, arr):
        return int(np.abs(arr - station).argmin())

    def _tick(self):
        if (self.selected_path is None or self.waypoints_original is None
                or self.frenet_pose is None or self.max_s is None):
            return

        self.close_to_s0 = (self.frenet_pose[0] < 0.5
                            or self.frenet_pose[0] > self.max_s - 0.5)

        wpt_arr = WpntArray()
        mrk_arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        wpt_arr.header.stamp = now
        wpt_arr.header.frame_id = 'map'

        path = self.selected_path
        for i, coord in enumerate(path[:self.path_length, :]):
            w = Wpnt()
            w.s_m = (coord[0] + self.frenet_pose[0]) % self.max_s
            w.x_m = coord[1]
            w.y_m = coord[2]
            w.d_m = 0.0
            w.psi_rad     = coord[3]
            w.kappa_radpm = coord[4]
            w.vx_mps      = coord[5] * self.speed_scaling
            w.ax_mps2     = coord[6]
            w.id = i
            idx = self._find_nearest_s(w.s_m, self.waypoints_original[:, 0])
            w.d_left  = self.waypoints_original[idx, 5] - w.d_m
            w.d_right = self.waypoints_original[idx, 4] + w.d_m
            wpt_arr.wpnts.append(w)

            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.type = Marker.SPHERE
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.a = 1.0
            m.color.g = 1.0
            m.id = i
            m.pose.position.x = w.x_m
            m.pose.position.y = w.y_m
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            mrk_arr.markers.append(m)

        self.cmd_pub.publish(wpt_arr)
        self.wpnt_viz.publish(mrk_arr)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PathPublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
