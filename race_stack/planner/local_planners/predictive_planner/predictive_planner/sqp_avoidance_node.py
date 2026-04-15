#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from f110_msgs.msg import Wpnt, WpntArray, ObstacleArray, OTWpntArray, OpponentTrajectory
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Float32MultiArray, Float32, Bool
from rcl_interfaces.msg import SetParametersResult
from scipy.optimize import minimize
from frenet_conversion.frenet_converter import FrenetConverter
from copy import deepcopy

class SQPAvoidanceNode(Node):
    """
    Advanced SQP Avoidance Node (Multi-opponent compatible).
    """
    def __init__(self):
        super().__init__('sqp_avoidance_node')
        
        # State & Data
        self.frenet_state = Odometry()
        self.local_wpnts = None
        self.past_avoidance_d = []
        self.global_waypoints_raw = None
        self.converter = None
        
        # Scaled Waypoints
        self.scaled_wpnts = None
        self.scaled_wpnts_msg = WpntArray()
        self.scaled_vmax = 1.0
        self.scaled_max_idx = 0
        self.scaled_max_s = 1.0
        self.scaled_delta_s = 0.1
        
        # Updated Waypoints
        self.wpnts_updated = []
        self.max_s_updated = 1.0
        self.max_idx_updated = 0
        
        # Obstacles
        self.obs = ObstacleArray()
        self.obs_perception = ObstacleArray()
        self.obs_predict = ObstacleArray()
        self.opp_id = None
        
        # Opponent Trajectory
        self.opponent_waypoints = None
        self.max_opp_idx = 0
        self.opponent_wpnts_sm = None
        
        # Logic Flags
        self.last_ot_side = ""
        self.ot_section_check = True
        
        # Solver Params
        self.min_radius = 0.05
        self.max_kappa = 1.0 / self.min_radius
        self.width_car = 0.28
        
        # Parameters
        self.declare_parameter('evasion_dist', 0.65)
        self.declare_parameter('obs_traj_tresh', 1.5)
        self.declare_parameter('spline_bound_mindist', 0.2)
        self.declare_parameter('lookahead_dist', 15.0)
        self.declare_parameter('avoidance_resolution', 20)
        self.declare_parameter('back_to_raceline_before', 5.0)
        self.declare_parameter('back_to_raceline_after', 5.0)
        self.declare_parameter('measure', False)

        self.update_params()
        self.add_on_set_parameters_callback(self.param_callback)

        # Publishers
        self.mrks_pub = self.create_publisher(MarkerArray, "/planner/avoidance/markers_sqp", 10)
        self.evasion_pub = self.create_publisher(OTWpntArray, "/planner/avoidance/otwpnts", 10)
        self.merger_pub = self.create_publisher(Float32MultiArray, "/planner/avoidance/merger", 10)
        if self.measure:
            self.latency_pub = self.create_publisher(Float32, "/planner/pspliner_sqp/latency", 10)

        # Subscribers
        self.create_subscription(ObstacleArray, "/perception/obstacles", self.obs_perception_cb, 10)
        self.create_subscription(ObstacleArray, "/collision_prediction/obstacles", self.obs_predict_cb, 10)
        self.create_subscription(Odometry, "/car_state/odom_frenet", self.state_cb, 10)
        self.create_subscription(WpntArray, "/global_waypoints_scaled", self.scaled_wpnts_cb, 10)
        self.create_subscription(WpntArray, "/local_waypoints", self.local_wpnts_cb, 10)
        self.create_subscription(WpntArray, "/global_waypoints", self.gb_cb, 10)
        self.create_subscription(WpntArray, "/global_waypoints_updated", self.updated_wpnts_cb, 10)
        self.create_subscription(OpponentTrajectory, "/opponent_trajectory", self.opponent_trajectory_cb, 10)
        self.create_subscription(Bool, "/ot_section_check", self.ot_sections_check_cb, 10)

        # Main Loop (20Hz)
        self.create_timer(0.05, self.loop)
        self.get_logger().info("Advanced SQP Avoidance Node Started")

    def update_params(self):
        self.evasion_dist = self.get_parameter('evasion_dist').value
        self.obs_traj_tresh = self.get_parameter('obs_traj_tresh').value
        self.spline_bound_mindist = self.get_parameter('spline_bound_mindist').value
        self.lookahead = self.get_parameter('lookahead_dist').value
        self.avoidance_resolution = self.get_parameter('avoidance_resolution').value
        self.back_to_raceline_before = self.get_parameter('back_to_raceline_before').value
        self.back_to_raceline_after = self.get_parameter('back_to_raceline_after').value
        self.measure = self.get_parameter('measure').value

    def param_callback(self, params):
        for param in params:
            if param.name == 'evasion_dist': self.evasion_dist = param.value
            elif param.name == 'obs_traj_tresh': self.obs_traj_tresh = param.value
            elif param.name == 'spline_bound_mindist': self.spline_bound_mindist = param.value
            elif param.name == 'lookahead_dist': self.lookahead = param.value
            elif param.name == 'avoidance_resolution': self.avoidance_resolution = param.value
            elif param.name == 'back_to_raceline_before': self.back_to_raceline_before = param.value
            elif param.name == 'back_to_raceline_after': self.back_to_raceline_after = param.value
            elif param.name == 'measure': self.measure = param.value
        return SetParametersResult(successful=True)

    def obs_perception_cb(self, data: ObstacleArray):
        if self.opp_id is not None:
            # Filter out the opponent car, keep static obstacles
            data.obstacles = [o for o in data.obstacles if o.id != self.opp_id or o.is_static]
        else:
            # Keep only the static obstacles
            data.obstacles = [o for o in data.obstacles if o.is_static]
        self.obs_perception = data
        self._update_all_obs()

    def obs_predict_cb(self, data: ObstacleArray):
        self.obs_predict = data
        if len(data.obstacles) > 0:
            self.opp_id = data.obstacles[0].id
        self._update_all_obs()

    def _update_all_obs(self):
        self.obs.header = self.obs_perception.header
        self.obs.obstacles = list(self.obs_perception.obstacles) + list(self.obs_predict.obstacles)

    def state_cb(self, data: Odometry): self.frenet_state = data
    def ot_sections_check_cb(self, data: Bool): self.ot_section_check = data.data

    def gb_cb(self, data: WpntArray):
        self.global_waypoints_raw = np.array([[wpnt.x_m, wpnt.y_m, wpnt.psi_rad] for wpnt in data.wpnts])
        
        # Initialise fallback for wpnts_updated if empty
        if len(self.wpnts_updated) == 0:
            self.wpnts_updated = data.wpnts[:-1]
            self.max_s_updated = self.wpnts_updated[-1].s_m
            self.max_idx_updated = len(self.wpnts_updated)
            
        if self.converter is None:
            self.get_logger().info("Initializing FrenetConverter...")
            self.converter = FrenetConverter(
                self.global_waypoints_raw[:, 0], 
                self.global_waypoints_raw[:, 1],
                self.global_waypoints_raw[:, 2]
            )
            self.get_logger().info("FrenetConverter Initialized.")

    def scaled_wpnts_cb(self, data: WpntArray):
        self.scaled_wpnts = np.array([[wpnt.s_m, wpnt.d_m] for wpnt in data.wpnts])
        self.scaled_wpnts_msg = data
        if len(data.wpnts) > 0:
            self.scaled_vmax = np.max([w.vx_mps for w in data.wpnts])
            self.scaled_max_idx = data.wpnts[-1].id
            self.scaled_max_s = data.wpnts[-1].s_m
            if len(data.wpnts) > 1:
                self.scaled_delta_s = data.wpnts[1].s_m - data.wpnts[0].s_m

    def updated_wpnts_cb(self, data: WpntArray):
        if len(data.wpnts) > 0:
            self.wpnts_updated = data.wpnts[:-1]
            self.max_s_updated = self.wpnts_updated[-1].s_m
            self.max_idx_updated = self.wpnts_updated[-1].id

    def local_wpnts_cb(self, data: WpntArray):
        self.local_wpnts = np.array([[wpnt.s_m, wpnt.d_m] for wpnt in data.wpnts])

    def opponent_trajectory_cb(self, data: OpponentTrajectory):
        self.opponent_waypoints = data.oppwpnts
        self.max_opp_idx = len(data.oppwpnts) - 1
        self.opponent_wpnts_sm = np.array([wpnt.s_m for wpnt in data.oppwpnts])

    def loop(self):
        if self.converter is None or self.scaled_wpnts is None or len(self.wpnts_updated) == 0:
            return

        start_time = time.perf_counter()
        obs = deepcopy(self.obs)
        self.current_d = self.frenet_state.pose.pose.position.y
        self.cur_s = self.frenet_state.pose.pose.position.x
        
        # Pre-process
        obs.obstacles = sorted(obs.obstacles, key=lambda o: o.s_start)
        considered_obs = []
        for o in obs.obstacles:
            dist_to_ego = (o.s_start - self.cur_s) % self.scaled_max_s
            if abs(o.d_center) < self.obs_traj_tresh and dist_to_ego < self.lookahead:
                # Buffering
                o.s_start = (o.s_start - self.evasion_dist) % self.scaled_max_s
                if o.s_end < o.s_start: o.s_end += self.scaled_max_s
                considered_obs.append(o)

        if not considered_obs and len(self.obs_predict.obstacles) > 0:
            considered_obs = [self.obs_predict.obstacles[0]]

        if considered_obs and self.ot_section_check:
            ev_x, ev_y, ev_s, ev_d, ev_v = self.sqp_solver(considered_obs, self.cur_s)
            if len(ev_s) > 0:
                self.merger_pub.publish(Float32MultiArray(data=[float(considered_obs[-1].s_end % self.scaled_max_s), float(ev_s[-1] % self.scaled_max_s)]))
        else:
            mrks = MarkerArray()
            del_marker = Marker()
            del_marker.header.frame_id = "map"
            del_marker.action = Marker.DELETEALL
            mrks.markers.append(del_marker)
            self.mrks_pub.publish(mrks)

        if self.measure:
            self.latency_pub.publish(Float32(data=float(time.perf_counter() - start_time)))

    def sqp_solver(self, considered_obs, curr_s):
        danger = False
        init_obj = self.group_objects(considered_obs)
        
        idx_start = np.abs(self.scaled_wpnts[:, 0] - init_obj.s_start).argmin()
        idx_end = np.abs(self.scaled_wpnts[:, 0] - (init_obj.s_end % self.scaled_max_s)).argmin()
        
        diff = (idx_end - idx_start) % self.scaled_max_idx
        gb_idxs = np.array([(idx_start + i) % self.scaled_max_idx for i in range(diff)])
        # Fallback: if obstacle covers fewer than 20 waypoints, use s_center as anchor
        if len(gb_idxs) < 20:
            gb_idxs = np.array([int(init_obj.s_center / self.scaled_delta_s + i) % self.scaled_max_idx for i in range(20)])
        
        side, init_apex = self._more_space(init_obj, self.scaled_wpnts_msg.wpnts, gb_idxs)
        kappas = np.array([self.scaled_wpnts_msg.wpnts[i].kappa_radpm for i in gb_idxs])
        max_k = np.max(np.abs(kappas))
        outside = "left" if np.sum(kappas) < 0 else "right"

        if side == outside:
            for o in considered_obs: o.s_end += 0.5

        min_s_start, max_s_end = self.scaled_max_s, 0.0
        for o in considered_obs:
            min_s_start = min(min_s_start, o.s_start)
            max_s_end = max(max_s_end, o.s_end)
            if o.d_left > 3 or o.d_right < -3: danger = True

        start_av = max(min_s_start - self.back_to_raceline_before, curr_s)
        end_av = max_s_end + self.back_to_raceline_after

        s_av = np.linspace(start_av, end_av, self.avoidance_resolution)
        self.down_sampled_delta_s = s_av[1] - s_av[0]
        
        wpnt_idxs = np.array([np.abs(self.scaled_wpnts[:, 0] - (s % self.scaled_max_s)).argmin() for s in s_av])
        corr_wpnts = [self.scaled_wpnts_msg.wpnts[i] for i in wpnt_idxs]
        bounds = [(-w.d_right + self.spline_bound_mindist, w.d_left - self.spline_bound_mindist) for w in corr_wpnts]

        # Curvature
        x_pts, y_pts = np.array([w.x_m for w in corr_wpnts]), np.array([w.y_m for w in corr_wpnts])
        dx, dy = np.diff(x_pts), np.diff(y_pts)
        dx = np.where(dx == 0, 1e-6, dx); dy = np.where(dy == 0, 1e-6, dy)
        ddx, ddy = np.diff(dx), np.diff(dy)
        dx, dy = dx[:-1], dy[:-1]
        self.global_traj_kappas = (dx*ddy - dy*ddx) / ((dx**2 + dy**2)**1.5)

        self.obs_dn_idx = []
        self.obs_dn_center_d = []
        self.obs_dn_min_dist = []

        for o in considered_obs:
            o_idx_s = np.abs(s_av - o.s_start).argmin()
            o_idx_e = np.abs(s_av - o.s_end).argmin()
            if o_idx_s < len(s_av) - 2:
                if o.is_static or o_idx_e == o_idx_s or self.opponent_waypoints is None:
                    if o_idx_e == o_idx_s: o_idx_e = o_idx_s + 1
                    idxs = np.arange(o_idx_s, o_idx_e + 1)
                    self.obs_dn_idx.extend(idxs)
                    self.obs_dn_center_d.extend([(o.d_left + o.d_right)/2.0] * len(idxs))
                    self.obs_dn_min_dist.extend([self.width_car + self.evasion_dist] * len(idxs))
                else:
                    idxs = np.arange(o_idx_s, o_idx_e + 1)
                    self.obs_dn_idx.extend(idxs)
                    opp_idxs = [np.abs(self.opponent_wpnts_sm - (s_av[i] % self.max_opp_idx)).argmin() for i in idxs]
                    self.obs_dn_center_d.extend([self.opponent_waypoints[i].d_m for i in opp_idxs])
                    self.obs_dn_min_dist.extend([self.width_car + self.evasion_dist] * len(idxs))

        self.obs_dn_idx = np.array(self.obs_dn_idx).astype(int)
        
        v_ego = self.frenet_state.twist.twist.linear.x
        v_ref = self.wpnts_updated[wpnt_idxs[0] % self.max_idx_updated].vx_mps
        r_speed = min(np.clip(v_ego, 1.0, 6.5), v_ref)
        self.min_radius = np.interp(r_speed, [1.0, 6.0, 7.0], [0.2, 2.0, 4.0])
        self.max_kappa = 1.0 / self.min_radius

        initial_guess = self.past_avoidance_d if len(self.past_avoidance_d) > 0 else np.full(len(s_av), init_apex)
        res = minimize(self.obj_fn, initial_guess, method='SLSQP', bounds=bounds,
                       constraints=[{'type': 'ineq', 'fun': self.eq_cons_ineq}, {'type': 'ineq', 'fun': self.ineq_cons}],
                       options={'ftol': 1e-1, 'maxiter': 50})

        self.get_logger().debug(f"[SQP] Solver: success={res.success} obs_pts={len(self.obs_dn_idx)} danger={danger} msg={res.message}")
        if (res.success or res.status == 9) and len(self.obs_dn_idx) >= 2 and not danger:
            n_pts = int((end_av - start_av) / self.scaled_delta_s)
            s_arr = np.linspace(start_av, end_av, n_pts)
            ev_d = np.interp(s_arr, s_av, res.x)
            ev_s = np.mod(s_arr, self.scaled_max_s)
            cart = self.converter.get_cartesian(ev_s, ev_d)
            ev_v = np.interp(s_arr, s_av, [w.vx_mps for w in corr_wpnts])
            
            msg = OTWpntArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.wpnts = [Wpnt(id=i, s_m=float(s), d_m=float(d), x_m=float(x), y_m=float(y), vx_mps=float(v)) 
                         for i, (x, y, s, d, v) in enumerate(zip(cart[0], cart[1], ev_s, ev_d, ev_v))]
            self.past_avoidance_d = res.x
            self.last_ot_side = "left" if np.mean(ev_d) > 0 else "right"
            self.evasion_pub.publish(msg)
            self.viz(ev_s, ev_d, cart[0], cart[1], ev_v)
            return cart[0], cart[1], ev_s, ev_d, ev_v
        else:
            self.past_avoidance_d = []
            return [], [], [], [], []

    def obj_fn(self, d): return np.sum(d**2)*10 + np.sum(np.diff(np.diff(d))**2)*100 + (np.diff(d)[0]**2)*1000
    def eq_cons(self, d): return [0.02 - abs(d[0] - self.current_d), 0.02 - abs(d[-2]), 0.02 - abs(d[-1])]
    def eq_cons_ineq(self, d):
        """Soft version: penalise large deviations from current position at start, and enforce near-zero at end."""
        # start: d[0] should be close to current_d (within 0.1m)
        # end: d[-2] and d[-1] should be near 0 (back on raceline)
        return np.array([
            0.1 - abs(d[0] - self.current_d),
            0.15 - abs(d[-2]),
            0.15 - abs(d[-1]),
        ])
    def _consecutive_points_constraint(self, d):
        """Prevents the trajectory from tunnelling through an obstacle between sample points."""
        points = d[self.obs_dn_idx]
        violations = []
        for i in range(len(points) - 1):
            same_side = (
                (points[i] > self.obs_dn_center_d[i] and points[i+1] > self.obs_dn_center_d[i+1]) or
                (points[i] < self.obs_dn_center_d[i] and points[i+1] < self.obs_dn_center_d[i+1])
            )
            violations.append(1.0 if same_side else -1.0)
        return violations

    def ineq_cons(self, d):
        dn_idx = self.obs_dn_idx
        # Obstacle clearance constraint
        v1 = np.abs(d[dn_idx] - np.array(self.obs_dn_center_d)) - np.array(self.obs_dn_min_dist)
        # Consecutive points constraint (no tunnelling)
        v2 = self._consecutive_points_constraint(d)
        # Turning radius constraint
        y_p = np.diff(d); y_p = np.where(y_p == 0, 1e-6, y_p)
        y_pp = np.diff(y_p); y_p = y_p[:-1]
        k = (self.down_sampled_delta_s * y_pp) / (self.down_sampled_delta_s**3)
        v3 = self.max_kappa - abs(self.global_traj_kappas - k)
        # First-point smoothness constraint
        v4 = [float(self.down_sampled_delta_s - abs(d[1]-d[0]))]
        return np.concatenate([v1, v2, v3, v4])

    def group_objects(self, obs_list):
        obj = deepcopy(obs_list[0])
        for o in obs_list:
            obj.d_left, obj.d_right = max(obj.d_left, o.d_left), min(obj.d_right, o.d_right)
            obj.s_start, obj.s_end = min(obj.s_start, o.s_start), max(obj.s_end, o.s_end)
        obj.s_center = (obj.s_start + obj.s_end) / 2.0
        return obj

    def _more_space(self, obj, wpnts, idxs):
        l_m = np.mean([wpnts[i].d_left for i in idxs])
        r_m = np.mean([wpnts[i].d_right for i in idxs])
        l_g, r_g = abs(l_m - obj.d_left), abs(r_m + obj.d_right)
        gap = self.evasion_dist + self.spline_bound_mindist
        if r_g > gap and l_g < gap: return "right", min(0.0, obj.d_right - self.evasion_dist)
        if l_g > gap and r_g < gap: return "left", max(0.0, obj.d_left + self.evasion_dist)
        cl, cr = max(0.0, obj.d_left + self.evasion_dist), min(0.0, obj.d_right - self.evasion_dist)
        return ("left", cl) if abs(cl) <= abs(cr) else ("right", cr)

    def viz(self, s, d, x, y, v):
        mrks = MarkerArray()
        for i in range(len(s)):
            if i % 3 != 0: continue
            m = Marker()
            m.header.frame_id, m.header.stamp = "map", self.get_clock().now().to_msg()
            m.type, m.id = Marker.CYLINDER, i
            m.scale.x, m.scale.y = 0.1, 0.1
            m.scale.z = float(v[i]/self.scaled_vmax)
            m.color.a, m.color.g, m.color.r, m.color.b = 1.0, 0.13, 0.63, 0.94
            m.pose.position.x, m.pose.position.y = float(x[i]), float(y[i])
            m.pose.position.z, m.pose.orientation.w = m.scale.z/2.0, 1.0
            mrks.markers.append(m)
        self.mrks_pub.publish(mrks)

def main(args=None):
    rclpy.init(args=args)
    node = SQPAvoidanceNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
