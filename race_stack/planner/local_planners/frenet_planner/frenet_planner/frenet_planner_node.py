"""
Frenet Planner ROS 2 Node.
Direct port of ROS1 frenet_planner_node.cpp.
Uses FrenetConverter from frenet_conversion package (Python).
"""
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from visualization_msgs.msg import MarkerArray, Marker
from f110_msgs.msg import WpntArray, Wpnt, ObstacleArray, OTWpntArray
from builtin_interfaces.msg import Time

from frenet_conversion.frenet_converter import FrenetConverter
from frenet_planner.data_structs import FrenetInitialConditions, FrenetHyperparameters
from frenet_planner.frenet_optimal_trajectory import FrenetOptimalTrajectory
from frenet_planner.frenet_path import FrenetPath


def quaternion_to_yaw(x, y, z, w):
    """Extract yaw from quaternion — replaces tf::Matrix3x3.getRPY."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class FrenetPlannerNode(Node):
    """ROS 2 node wrapping the Frenet Optimal Trajectory planner."""

    def __init__(self):
        super().__init__('frenet_planner')

        # --- Declare ALL parameters with defaults ---
        self._declare_params()

        # --- Load hyperparameters ---
        self.fot_hp = FrenetHyperparameters()
        self.fot_ic = FrenetInitialConditions()
        self._load_hyperparameters()

        self.debug = self.get_parameter('debug').value
        self.measure = self.get_parameter('measure').value
        self.max_rate = self.get_parameter('max_frequency').value
        self.speed_scale = self.get_parameter('frenet_speed_scaling').value
        self.pub_only_if_obstacles = self.get_parameter('pub_only_if_obstacles').value
        self.pub_global_track_advancement = self.get_parameter('pub_global_track_advancement').value
        self.replanning_distance = self.get_parameter('replanning_distance').value

        # --- State ---
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.v_x = 0.0
        self.v_y = 0.0
        self.has_global_trajectory = False
        self.has_obstacles = False
        self.on_path = False
        self.on_path_count = 0
        self.global_path_length = 0.0
        self.max_gb_speed = 10.0
        self.wpnt_cnt = 0
        self.wpt_array = []
        self.last_ot_side = "none"
        self.last_switch_time = self.get_clock().now().to_msg()

        self.frenet_converter: Optional[FrenetConverter] = None
        self.frenet_solver = FrenetOptimalTrajectory(debug_mode=self.debug)
        self.best_frenet_path = FrenetPath()
        self.best_frenet_path.set_hyperparameters(self.fot_hp)

        # --- Subscribers ---
        self.create_subscription(
            WpntArray, '/global_waypoints_scaled', self.global_trajectory_cb, 10)
        self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(
            ObstacleArray, '/perception/obstacles', self.obstacle_cb, 10)

        # --- Publishers ---
        self.local_trajectory_pub = self.create_publisher(
            OTWpntArray, '/planner/avoidance/otwpnts', 1)
        self.local_trajectory_marker_pub = self.create_publisher(
            MarkerArray, 'planner/markers', 1)
        self.obstacle_marker_pub = self.create_publisher(
            MarkerArray, 'planner/obstacle_markers', 1)

        if self.debug:
            self.initial_position_marker_pub = self.create_publisher(
                Marker, 'planner/initial_condition_marker', 1)

        if self.measure:
            self.latency_pub = self.create_publisher(
                Float64, 'planner/frenet/latency', 1)

        # --- Main loop timer ---
        period = 1.0 / self.max_rate
        self.timer = self.create_timer(period, self.main_loop)

        self.get_logger().info('[Planner] Finished initializing Frenet Planner node.')

    # ==================== PARAMETER DECLARATION ====================

    def _declare_params(self):
        """Declare all parameters with defaults matching ROS 1 frenet_planner_params_safe.yaml."""
        self.declare_parameter('debug', False)
        self.declare_parameter('measure', False)
        self.declare_parameter('max_frequency', 20.0)
        self.declare_parameter('frenet_speed_scaling', 0.9)
        self.declare_parameter('pub_only_if_obstacles', False)
        self.declare_parameter('pub_global_track_advancement', True)
        self.declare_parameter('replanning_distance', 1.5)

        # FrenetHyperparameters
        self.declare_parameter('max_speed', 10.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('max_accel', 8.0)
        self.declare_parameter('max_curvature', 1.0)
        self.declare_parameter('max_road_width_l', 1.5)
        self.declare_parameter('max_road_width_r', 1.5)
        self.declare_parameter('d_road_w', 0.05)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('maxt', 1.5)
        self.declare_parameter('mint', 1.0)
        self.declare_parameter('d_sample_t', 0.5)
        self.declare_parameter('d_t_s', 0.25)
        self.declare_parameter('d_t_s_ot', 2.0)
        self.declare_parameter('n_s_sample', 12.0)
        self.declare_parameter('obstacle_clearance', 0.35)
        self.declare_parameter('track_clearance', 0.3)
        self.declare_parameter('k_lat_dev', 0.6)
        self.declare_parameter('k_lat_v', 0.2)
        self.declare_parameter('k_lat_a', 0.0)
        self.declare_parameter('k_lat_j', 0.1)
        self.declare_parameter('k_lon_dev', 0.8)
        self.declare_parameter('k_track_bounds', 0.05)
        self.declare_parameter('k_obstacles', 0.2)
        self.declare_parameter('k_temporal', 1.0)
        self.declare_parameter('k_lat', 0.6)
        self.declare_parameter('k_lon', 1.0)
        self.declare_parameter('k_time', 10.0)
        self.declare_parameter('num_threads', 0)
        self.declare_parameter('ic_avoid_obstacles', False)
        self.declare_parameter('ego_width', 0.3)

    def _load_hyperparameters(self):
        hp = self.fot_hp
        hp.max_speed = self.get_parameter('max_speed').value
        hp.min_speed = self.get_parameter('min_speed').value
        hp.max_accel = self.get_parameter('max_accel').value
        hp.max_curvature = self.get_parameter('max_curvature').value
        hp.max_road_width_l = self.get_parameter('max_road_width_l').value
        hp.max_road_width_r = self.get_parameter('max_road_width_r').value
        hp.d_road_w = self.get_parameter('d_road_w').value
        hp.dt = self.get_parameter('dt').value
        hp.maxt = self.get_parameter('maxt').value
        hp.mint = self.get_parameter('mint').value
        hp.d_sample_t = self.get_parameter('d_sample_t').value
        hp.d_t_s = self.get_parameter('d_t_s').value
        hp.init_d_t_s = self.get_parameter('d_t_s').value
        hp.d_t_s_ot = self.get_parameter('d_t_s_ot').value
        hp.n_s_sample = self.get_parameter('n_s_sample').value
        hp.obstacle_clearance = self.get_parameter('obstacle_clearance').value
        hp.track_clearance = self.get_parameter('track_clearance').value
        hp.k_lat_dev = self.get_parameter('k_lat_dev').value
        hp.k_lat_v = self.get_parameter('k_lat_v').value
        hp.k_lat_a = self.get_parameter('k_lat_a').value
        hp.k_lat_j = self.get_parameter('k_lat_j').value
        hp.k_lon_dev = self.get_parameter('k_lon_dev').value
        hp.k_track_bounds = self.get_parameter('k_track_bounds').value
        hp.k_obstacles = self.get_parameter('k_obstacles').value
        hp.k_temporal = self.get_parameter('k_temporal').value
        hp.k_lat = self.get_parameter('k_lat').value
        hp.k_lon = self.get_parameter('k_lon').value
        hp.k_time = self.get_parameter('k_time').value
        hp.num_threads = self.get_parameter('num_threads').value
        hp.ic_avoid_obstacles = self.get_parameter('ic_avoid_obstacles').value
        hp.ego_width = self.get_parameter('ego_width').value

    # ==================== CALLBACKS ====================

    def odom_cb(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        self.odom_yaw = quaternion_to_yaw(o.x, o.y, o.z, o.w)
        self.v_x = msg.twist.twist.linear.x
        self.v_y = msg.twist.twist.linear.y

    def global_trajectory_cb(self, msg: WpntArray):
        # subsample: only process every 8th message (same as ROS 1)
        if self.wpnt_cnt % 8 != 0:
            self.wpnt_cnt += 1
            return
        self.wpnt_cnt += 1

        self.wpt_array = list(msg.wpnts)
        if not self.wpt_array:
            return
        self.global_path_length = self.wpt_array[-1].s_m
        self.max_gb_speed = max(w.vx_mps for w in self.wpt_array)

        wx, wy, d_right, d_left, v_s = [], [], [], [], []
        for wpt in self.wpt_array:
            # Note: We mutable the waypoint velocity globally here in our memory copy
            wpt.vx_mps *= self.speed_scale
            wx.append(wpt.x_m)
            wy.append(wpt.y_m)
            d_right.append(wpt.d_right - self.fot_hp.track_clearance)
            d_left.append(wpt.d_left - self.fot_hp.track_clearance)
            v_s.append(wpt.vx_mps)

        enable_wrapping = True
        self.frenet_solver.set_global_trajectory(
            wx, wy, d_right, d_left, v_s, enable_wrapping)

        # build FrenetConverter for odom→frenet conversion
        psi_arr = np.array([w.psi_rad for w in self.wpt_array])
        self.frenet_converter = FrenetConverter(
            np.array(wx), np.array(wy), psi_arr)

        self.has_global_trajectory = True

        self.fot_hp.global_path_length = self.global_path_length
        self.frenet_solver.set_hyperparameters(self.fot_hp)
        self.best_frenet_path.set_hyperparameters(self.fot_hp)

    def obstacle_cb(self, msg: ObstacleArray):
        if not self.has_global_trajectory:
            return
        self.has_obstacles = self.frenet_solver.update_obstacles(msg.obstacles)
        self._publish_obstacles(msg.obstacles)

    # ==================== MAIN LOOP ====================

    def main_loop(self):
        if not self.has_global_trajectory:
            return
        if self.pub_only_if_obstacles and not self.has_obstacles:
            return

        start_time = time.perf_counter()

        if self._update_initial_conditions():
            self.on_path = self._find_local_trajectory()
            if not self.on_path:
                self._publish_empty_trajectory()
                self.on_path_count = 0
            elif self.on_path_count < 3:
                self.on_path_count += 1
                self._publish_local_trajectory(self.best_frenet_path)
            else:
                self._publish_local_trajectory(self.best_frenet_path)

        if self.measure:
            elapsed = time.perf_counter() - start_time
            lat_msg = Float64()
            lat_msg.data = elapsed
            self.latency_pub.publish(lat_msg)

    # ==================== PLANNER LOGIC ====================

    def _update_initial_conditions(self) -> bool:
        if not self.has_global_trajectory or self.frenet_converter is None:
            return False

        ic = self.fot_ic

        if not self.on_path:
            # get frenet from odom
            frenet_sd = self.frenet_converter.get_frenet(
                np.array([self.odom_x]), np.array([self.odom_y]))
            frenet_vel = self.frenet_converter.get_frenet_velocities(
                self.v_x, self.v_y, self.odom_yaw, s=float(frenet_sd[0]))

            ic.s0 = float(frenet_sd[0])
            ic.c_d = float(frenet_sd[1])
            ic.c_speed = float(frenet_vel[0])
            ic.c_d_d = float(frenet_vel[1])
            ic.c_d_dd = 0.0
            ic.c_s_dd = 0.0
            self.id_start = int(self.frenet_converter.get_closest_index_by_s(ic.s0))
        else:
            # find closest point on previous best path
            best = self.best_frenet_path
            if not best.x:
                return False
            min_dist = float('inf')
            closest_idx = 0
            for i in range(len(best.x)):
                d_sq = (self.odom_x - best.x[i])**2 + (self.odom_y - best.y[i])**2
                if d_sq < min_dist:
                    min_dist = d_sq
                    closest_idx = i

            if min_dist > self.replanning_distance ** 2:
                frenet_sd = self.frenet_converter.get_frenet(
                    np.array([self.odom_x]), np.array([self.odom_y]))
                frenet_vel = self.frenet_converter.get_frenet_velocities(
                    self.v_x, self.v_y, self.odom_yaw, s=float(frenet_sd[0]))

                ic.s0 = float(frenet_sd[0])
                ic.c_d = float(frenet_sd[1])
                ic.c_speed = float(frenet_vel[0])
                ic.c_d_d = float(frenet_vel[1])
                ic.c_d_dd = 0.0
                ic.c_s_dd = 0.0
                self.id_start = int(self.frenet_converter.get_closest_index_by_s(ic.s0))
            else:
                ic.s0 = best.s[closest_idx] % self.global_path_length
                ic.c_d = best.d[closest_idx]
                ic.c_speed = best.s_d[closest_idx]
                ic.c_d_d = best.d_d[closest_idx]
                ic.c_d_dd = best.d_dd[closest_idx]
                ic.c_s_dd = best.s_dd[closest_idx]

        return True

    def _find_local_trajectory(self) -> bool:
        if not self.has_global_trajectory:
            return False

        ic = self.fot_ic
        hp = self.fot_hp

        # clamp speed
        ic.c_speed = max(hp.min_speed, ic.c_speed)

        if abs(ic.c_d) > 3.0:
            return False

        # get target speed at end of horizon
        s_dist = max(0.5, ic.c_speed * hp.maxt)
        s_dist = (ic.s0 + s_dist) % self.global_path_length

        id_end = self.frenet_converter.get_closest_index_by_s(s_dist)
        ic.target_speed = self.wpt_array[id_end].vx_mps

        self.frenet_solver.update_current_position(ic)

        if self.debug:
            self._publish_initial_position()

        result = self.frenet_solver.update_optimal_trajectory(self.best_frenet_path)
        if result is not None:
            self.best_frenet_path = result
            return True
        else:
            return False

    # ==================== PUBLISHERS ====================

    def _publish_empty_trajectory(self):
        markers = MarkerArray()
        clearing = Marker()
        clearing.action = Marker.DELETEALL
        markers.markers.append(clearing)

        wpts = OTWpntArray()
        wpts.header.frame_id = 'map'
        wpts.header.stamp = self.get_clock().now().to_msg()
        wpts.last_switch_time = self.last_switch_time

        self.local_trajectory_pub.publish(wpts)
        self.local_trajectory_marker_pub.publish(markers)

    def _publish_local_trajectory(self, path: FrenetPath):
        markers = MarkerArray()
        wpts = OTWpntArray()

        clearing = Marker()
        clearing.action = Marker.DELETEALL
        markers.markers.append(clearing)

        clamp_speed = self.max_gb_speed
        local_advancement = 0.0
        hp = self.fot_hp

        for i in range(len(path.x) - 1):
            # marker
            m = Marker()
            m.header.frame_id = 'map'
            m.type = Marker.SPHERE
            m.scale.x = 0.25
            m.scale.y = 0.25
            m.scale.z = 0.25
            m.color.a = 0.5
            m.color.g = 1.0
            m.pose.orientation.w = 1.0
            m.id = i
            m.pose.position.x = path.x[i]
            m.pose.position.y = path.y[i]
            speed_here = min(path.ds[i] / hp.dt, clamp_speed)
            m.pose.position.z = speed_here / self.max_gb_speed
            markers.markers.append(m)

            # waypoint
            wpt = Wpnt()
            wpt.id = i
            if self.pub_global_track_advancement:
                wpt.s_m = path.s[i] % self.global_path_length
            else:
                wpt.s_m = local_advancement
            wpt.d_m = path.d[i]
            if i < len(path.d_right) and path.d_right[i] < 100:
                wpt.d_right = path.d_right[i]
            if i < len(path.d_left) and path.d_left[i] < 100:
                wpt.d_left = path.d_left[i]
            wpt.x_m = path.x[i]
            wpt.y_m = path.y[i]
            wpt.psi_rad = path.yaw[i]
            if i < len(path.c):
                wpt.kappa_radpm = path.c[i]
            wpt.vx_mps = speed_here
            wpt.ax_mps2 = path.s_dd[i]
            wpts.wpnts.append(wpt)
            local_advancement += path.ds[i]

        # Populate OTWpntArray specific fields
        # Determine side (simple heuristic based on average lateral displacement)
        if len(path.d) > 0:
            avg_d = sum(path.d) / len(path.d)
            current_side = "left" if avg_d > 0.0 else "right"
        else:
            current_side = "none"

        # Check for side switch for hysteresis logic in state machine
        if current_side != self.last_ot_side and current_side != "none":
            self.last_switch_time = self.get_clock().now().to_msg()
            self.last_ot_side = current_side

        wpts.header.frame_id = 'map'
        wpts.header.stamp = self.get_clock().now().to_msg()
        wpts.last_switch_time = self.last_switch_time
        wpts.ot_side = current_side
        wpts.side_switch = (current_side != self.last_ot_side)
        
        self.local_trajectory_pub.publish(wpts)
        self.local_trajectory_marker_pub.publish(markers)

    def _publish_obstacles(self, obstacles):
        markers = MarkerArray()
        clearing = Marker()
        clearing.action = Marker.DELETEALL
        markers.markers.append(clearing)

        if self.frenet_converter is None:
            return

        for ob in obstacles:
            for corner_idx, (s_val, d_val) in enumerate([
                (ob.s_start, ob.d_left),
                (ob.s_end, ob.d_left),
                (ob.s_start, ob.d_right),
                (ob.s_end, ob.d_right),
            ]):
                m = Marker()
                m.header.frame_id = 'map'
                m.type = Marker.SPHERE
                m.scale.x = 0.2
                m.scale.y = 0.2
                m.scale.z = 0.2
                m.color.a = 0.7
                m.color.r = 0.1 if ob.is_actually_a_gap else 0.9
                m.color.g = 0.9 if ob.is_actually_a_gap else 0.1
                m.pose.orientation.w = 1.0
                m.id = ob.id * 4 + corner_idx

                cart = self.frenet_converter.get_cartesian(s_val, d_val)
                m.pose.position.x = float(cart[0])
                m.pose.position.y = float(cart[1])
                markers.markers.append(m)

        self.obstacle_marker_pub.publish(markers)

    def _publish_initial_position(self):
        if self.frenet_converter is None:
            return
        m = Marker()
        m.header.frame_id = 'map'
        m.type = Marker.SPHERE
        m.scale.x = 0.3
        m.scale.y = 0.3
        m.scale.z = 0.3
        m.color.a = 1.0
        m.color.b = 1.0
        m.pose.orientation.w = 1.0
        m.id = 101

        ic = self.fot_ic
        cart = self.frenet_converter.get_cartesian(ic.s0, ic.c_d)
        m.pose.position.x = float(cart[0])
        m.pose.position.y = float(cart[1])
        self.initial_position_marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = FrenetPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
