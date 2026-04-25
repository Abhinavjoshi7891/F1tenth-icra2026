#state_machine.py
# CORRECTED VERSION — Faithful port from ROS1 state_machine_node.py
# 
# Fixes applied (referenced to the analysis report):
#   Bug #2:  car_state_frenet_cb now extracts cur_vs from twist.linear.x
#   Bug #3:  state_pub now publishes self.state.value ("GB_TRACK") not str(self.state) ("StateType.GB_TRACK")
#   Bug #1:  _check_ofree for predictive_spliner now checks static obstacles (matches ROS1)
#   Bug #4:  get_splini_wpts now handles "avoidance over more than 1 lap" case
#   Bug #8:  _check_ftg now uses self.state (not self.cur_state)
#   Bug #10: FTG counter reset added when not in TRAILING
#   Bug #11: _check_gbfree guards on head_to_head mode (timetrials_only equivalent)
#   Bug #13: threading.Lock added back for get_splini_wpts safety
#
import os
import yaml
import threading
import rclpy
import numpy as np
from builtin_interfaces.msg import Time
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter
from ament_index_python import get_package_share_directory
from std_msgs.msg import String, Bool, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from f110_msgs.msg import WpntArray, OTWpntArray, ObstacleArray
from visualization_msgs.msg import Marker, MarkerArray
try:
    from vesc_msgs.msg import VescStateStamped
except:
    pass

from state_machine.transitions import dummy_transition, timetrials_transition, head_to_head_transition
from state_machine.state_types import StateType
from state_machine.states import DefaultStateLogic
from stack_master.parameter_event_handler import ParameterEventHandler
from state_machine.state_machine_params import StateMachineParams

def time_to_float(time_instant: Time):
    return time_instant.sec + time_instant.nanosec * 1e-9

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        # PARAMETER DECLARATION
        self.params = StateMachineParams(self)
        
        self.ftg_disabled = False
        self.lock = threading.Lock()  # [Bug #13] Lock for concurrent waypoint access

        # update on parameter changes for rate
        self.add_on_set_parameters_callback(self.params.parameters_callback)
        
        # SUBSCRIPTIONS
        if self.params.test_on_car:
            self.battery_sub = self.create_subscription(VescStateStamped, "/vesc/sensors/core", self.battery_cb, 10)
        else:
            self.battery_level = self.params.volt_threshold + 1
        self.max_speed = None
        self.create_subscription(WpntArray, "/global_waypoints", self.glb_wpnts_cb, 10)
        while self.max_speed is None: # equivalent of wait for message
            self.get_logger().info("Waiting for global waypoints message", throttle_duration_sec=0.5)
            rclpy.spin_once(self)

        
        self.glb_wpnts = None
        self.num_glb_wpnts = 0
        self.track_length = 1
        self.gb_max_idx = 10
        self.create_subscription(WpntArray, "/global_waypoints_scaled", self.glb_wpnts_scaled_cb, 10)
        while self.glb_wpnts is None: # equivalent of wait for message
            self.get_logger().info("Waiting for scaled global waypoints message", throttle_duration_sec=0.5)
            rclpy.spin_once(self)

        
        self.cur_s = None
        self.cur_d = None
        self.cur_vs = 0.0  # [Bug #2] Initialize ego velocity
        self.create_subscription(Odometry, '/car_state/frenet/odom',self.car_state_frenet_cb, 10) # car frenet coordinates
        while self.cur_s is None:
            self.get_logger().info("Waiting for car state frenet message", throttle_duration_sec=0.5)
            rclpy.spin_once(self)

        self.obstacles = []
        if self.params.mode == 'head_to_head':
            self.current_position = None
            self.create_subscription(PoseStamped, '/car_state/pose', self.car_state_cb, 10)
            while self.current_position is None:
                self.get_logger().info("Waiting for car state pose message", throttle_duration_sec=0.5)
                rclpy.spin_once(self)


            self.last_valid_avoidance_wpnts = None
            self.splini_ttl_counter = 0
            self.avoidance_wpnts = None
            self.create_subscription(OTWpntArray, '/planner/avoidance/otwpnts', self.avoidance_cb, 10)

            # Frenet planner waypoints (separate channel, WpntArray on /planner/waypoints)
            self.frenet_wpnts = WpntArray()
            if self.params.overtake_mode == "frenet":
                self.create_subscription(WpntArray, '/planner/waypoints', self.frenet_wpnts_cb, 10)

            # Graph-based planner waypoints (flattened Float32MultiArray on /planner/graph_based_wpnts)
            self.graph_based_wpts = None         # np.ndarray, shape (N, 7): s, x, y, psi, kappa, vx, ax
            self.graph_based_action = None       # str: 'straight'/'left'/'right'/'follow'
            self.gb_wpnts_arr = None             # cached np view of glb_wpnts for d_left/d_right lookup
            if self.params.overtake_mode == "graph_based":
                self.create_subscription(Float32MultiArray, '/planner/graph_based_wpnts',
                                         self.graph_based_cb, 10)

            # Perception obstacles (from /perception/obstacles)
            self.obstacles_perception = []
            self.create_subscription(ObstacleArray, "/perception/obstacles", self.obstacles_cb, 10)

            # Predictive planner extras
            self.obstacles_prediction = []
            self.merger = None
            self.force_trailing = False
            self.emergency_break_horizon = 1.1  # [m]
            self.emergency_break_d = 0.12  # [m]
            self.create_subscription(ObstacleArray, "/collision_prediction/obstacles", self.obstacle_prediction_cb, 10)
            self.create_subscription(Float32MultiArray, "/planner/avoidance/merger", self.merger_cb, 10)
            self.create_subscription(Bool, "/collision_prediction/force_trailing", self.force_trailing_cb, 10)

            # TODO setup things for sectors and overtaking sectors
            self.only_ftg_zones = []
            self.ftg_counter = 0

            self.overtake_zones = []

        # INITIALIZATIONS        
        self.waypoints_dist = 0.1
        self.state = StateType(self.params.initial_state)
        self.local_waypoints = WpntArray()
        self.first_visualization = True
        self.x_viz = 0
        self.y_viz = 0

        # choose how to transition between states
        if self.params.mode == 'dummy':
            self.state_transition = dummy_transition
        elif self.params.mode == 'timetrials':
            self.state_transition = timetrials_transition
        elif self.params.mode == 'head_to_head':
            self.state_transition = head_to_head_transition
        else:
            raise NotImplementedError(f"Mode {self.params.mode} not recognized")
                
        # choose what to do in the different states
        self.state_logic = DefaultStateLogic
            
        # PUBLICATIONS
        self.state_pub = self.create_publisher(String, 'state', 10)
        self.state_marker_pub = self.create_publisher(Marker, 'state_marker', 10)
        self.loc_wpnt_pub = self.create_publisher(WpntArray, 'local_waypoints', 10)
        self.vis_loc_wpnt_pub = self.create_publisher(MarkerArray, 'local_waypoints/markers', 10)
        self.ot_section_check_pub = self.create_publisher(Bool, '/ot_section_check', 10)
        
        # main loop
        self.main_loop = self.create_timer(1/self.params.rate_hz, self.main_loop_callback)

        # set up ot param reading
        self.init_ot_params()

    #############
    # CALLBACKS #
    #############
    def battery_cb(self, msg: VescStateStamped):
        self.battery_level = msg.state.voltage_input

    def glb_wpnts_scaled_cb(self, data: WpntArray):
        """
        Callback function of velocity interpolator subscriber.

        Parameters
        ----------
        data
            Data received from velocity interpolator topic
        """
        self.glb_wpnts = data.wpnts[:-1]  # exclude last point (because last point == first point)
        self.num_glb_wpnts = len(self.glb_wpnts)
        self.track_length = data.wpnts[-1].s_m
        # Get spacing between wpnts for rough approximations
        self.wpnt_dist = data.wpnts[1].s_m - data.wpnts[0].s_m
        self.gb_max_idx = data.wpnts[-1].id
        if self.params.overtake_mode == "graph_based":
            self.gb_wpnts_arr = np.array([
                [w.s_m, w.d_m, w.x_m, w.y_m, w.d_right, w.d_left,
                 w.psi_rad, w.kappa_radpm, w.vx_mps, w.ax_mps2]
                for w in self.glb_wpnts
            ])

    def glb_wpnts_cb(self, data: WpntArray):
        self.max_speed = max([wpnt.vx_mps for wpnt in data.wpnts])

    def car_state_frenet_cb(self, msg: Odometry):
        self.cur_s = msg.pose.pose.position.x
        self.cur_d = msg.pose.pose.position.y
        self.cur_vs = msg.twist.twist.linear.x  # [Bug #2] Extract ego velocity

    def frenet_wpnts_cb(self, data: WpntArray):
        """Frenet planner waypoints — separate channel from spliner."""
        self.frenet_wpnts = data

    def graph_based_cb(self, data):
        """Graph-based planner waypoints — flattened Float32MultiArray."""
        arr = np.asarray(data.data)
        if len(arr) == 0 or len(data.layout.dim) < 2:
            self.graph_based_wpts = None
            return
        self.graph_based_wpts = arr.reshape(data.layout.dim[0].size, data.layout.dim[1].size)
        self.graph_based_action = data.layout.dim[0].label

    def avoidance_cb(self, data: OTWpntArray):
        """Subscribes to avoidance waypoints (spliner or predictive_spliner)"""
        if len(data.wpnts) > 0:
            if self.params.overtake_mode == "predictive_spliner":
                ttl = self.params.pred_splini_ttl
            else:
                ttl = self.params.splini_ttl
            self.splini_ttl_counter = int(ttl * self.params.rate_hz)
            self.avoidance_wpnts = data
        # If empty we don't overwrite the avoidance waypoints

    def obstacles_cb(self, data):
        if len(data.obstacles) != 0:
            self.obstacles_perception = data.obstacles
        else:
            self.obstacles_perception = []
        self.obstacles = self.obstacles_perception + self.obstacles_prediction

    def obstacle_prediction_cb(self, data: ObstacleArray):
        if len(data.obstacles) != 0:
            self.obstacles_prediction = data.obstacles
        else:
            self.obstacles_prediction = []
        self.obstacles = self.obstacles_perception + self.obstacles_prediction

    def merger_cb(self, data: Float32MultiArray):
        self.merger = data.data

    def force_trailing_cb(self, data: Bool):
        self.force_trailing = data.data
    
    def car_state_cb(self, data: PoseStamped):
        x = data.pose.position.x
        y = data.pose.position.y
        rot = Rotation.from_quat([data.pose.orientation.x, data.pose.orientation.y, 
                                       data.pose.orientation.z, data.pose.orientation.w])
        rot_euler = rot.as_euler('xyz', degrees=False)
        theta = rot_euler[2]

        self.current_position = [x, y, theta]
    
    ### Properties to evaluate state transitions
    @property
    def _low_bat(self)->bool:
        if self.battery_level < self.params.volt_threshold:
            return True
        else:
            return False
    
    @property
    def _check_only_ftg_zone(self) -> bool:
        ftg_only = False
        # check if the car is in a ftg only zone, but only if there is an only ftg zone
        if len(self.only_ftg_zones) != 0:
            for sector in self.only_ftg_zones:
                if sector[0] <= self.cur_s / self.waypoints_dist <= sector[1]:
                    ftg_only = True
                    break  # cannot be in two ftg zones
        return ftg_only

    @property
    def _check_close_to_raceline(self) -> bool:
        return np.abs(self.cur_d) < self.params.gb_ego_width_m  # [m]

    @property
    def _check_ot_sector(self) -> bool:
        in_sector = False
        for sector in self.ot_sectors:
            if sector['ot_flag']:
                # Extend end by ~0.5m (5 waypoints) to prevent flicker at boundary
                if (sector['start'] <= self.cur_s / self.waypoints_dist <= (sector['end'] + 5)):
                    in_sector = True
                    break
        self.ot_section_check_pub.publish(Bool(data=in_sector))
        return in_sector

    @property
    def _check_ofree(self) -> bool:
        """Check if the overtaking line is free of obstacles.
        Frenet: reads self.frenet_wpnts (WpntArray from /planner/waypoints).
        Spliner/PS: reads self.last_valid_avoidance_wpnts (OTWpntArray from /planner/avoidance/otwpnts)."""
        o_free = True

        if self.params.overtake_mode == "frenet":
            # Frenet wpts use local arc-length (s_m starts at 0 = ego position).
            # Convert obs global s → local s by subtracting cur_s.
            frenet_wpts = self.frenet_wpnts.wpnts if self.frenet_wpnts is not None else []
            if frenet_wpts:
                horizon = self.params.overtaking_horizon_m
                for obs in self.obstacles:
                    obs_s_global = obs.s_center
                    dist_to_obj = (obs_s_global - self.cur_s) % self.track_length
                    if dist_to_obj < horizon:
                        obs_s_local = dist_to_obj  # frenet s_m is distance from ego
                        obs_d = obs.d_center
                        avoid_wpnt_idx = np.argmin(
                            np.array([abs(w.s_m - obs_s_local) for w in frenet_wpts])
                        )
                        ot_d = frenet_wpts[avoid_wpnt_idx].d_m
                        ot_obs_dist = ot_d - obs_d
                        if abs(ot_obs_dist) < self.params.lateral_width_ot_m:
                            o_free = False
                            self.get_logger().info(f"O_FREE False (frenet), obs dist to ot lane: {ot_obs_dist:.2f} m", throttle_duration_sec=2.0)
                            break
            # No frenet waypoints yet → path is free (don't block overtake)
            return o_free

        elif self.params.overtake_mode in ("spliner", "predictive_spliner"):
            if self.last_valid_avoidance_wpnts is not None and len(self.last_valid_avoidance_wpnts):
                horizon = self.params.overtaking_horizon_m

                # Filter to only obstacles within horizon (and static-only for predictive_spliner)
                candidates = []
                for obs in self.obstacles:
                    if self.params.overtake_mode == "spliner" or obs.is_static:
                        dist_to_obj = (obs.s_center - self.cur_s) % self.track_length
                        if dist_to_obj < horizon:
                            candidates.append((dist_to_obj, obs))

                # Only evaluate the CLOSEST obstacle — the one actively being evaded.
                # Checking all obstacles causes false O_FREE=False when a second obstacle
                # sits on the raceline in the evasion spline's post-apex return section (d→0),
                # producing ot_obs_dist≈0 which is spuriously below lateral_width_ot_m.
                if candidates:
                    candidates.sort(key=lambda x: x[0])
                    _, closest_obs = candidates[0]
                    obs_s = closest_obs.s_center
                    obs_d = closest_obs.d_center
                    avoid_wpnt_idx = np.argmin(
                        np.array([abs(avoid_s.s_m - obs_s) for avoid_s in self.last_valid_avoidance_wpnts])
                    )
                    
                    # Longitudinal check: Ensure the waypoint found is actually near the obstacle in S.
                    # If it's too far (e.g. > 1.0m), it's a stale path or outside the spline window.
                    s_diff = abs(self.last_valid_avoidance_wpnts[avoid_wpnt_idx].s_m - obs_s)
                    
                    if s_diff < 1.0:
                        ot_d = self.last_valid_avoidance_wpnts[avoid_wpnt_idx].d_m
                        ot_obs_dist = ot_d - obs_d
                        if abs(ot_obs_dist) < self.params.lateral_width_ot_m:
                            o_free = False
                            self.get_logger().info(
                                f"O_FREE False, obs dist to ot lane: {ot_obs_dist:.4f} m "
                                f"(closest obs @ s={obs_s:.1f}m d={obs_d:.2f}m | path_d={ot_d:.2f}m)",
                                throttle_duration_sec=2.0
                            )
                    else:
                        # Path doesn't cover this obstacle yet; assume global distance for now
                        self.get_logger().debug(f"O_FREE: Nearest waypoint is {s_diff:.1f}m away from obstacle at S={obs_s:.1f}. Ignoring stale path.")
            return o_free
        elif self.params.overtake_mode == "graph_based":
            # Graph-based planner owns action selection. If it has published a valid path, consider o_free True.
            return self.graph_based_wpts is not None and len(self.graph_based_wpts) > 0
        else:
            self.get_logger().error(f"Unknown overtake planner: {self.params.overtake_mode}")
            return True

    @property
    def _check_gbfree(self) -> bool:
        gb_free = True
        # [Bug #11] Guard: if not in head_to_head, global is always free (matches ROS1 timetrials_only guard)
        if self.params.mode != "head_to_head":
            return True

        horizon = self.params.gb_horizon_m  # Horizon in front of cur_s [m]

        for obs in self.obstacles:
            obs_s = obs.s_center
            gap = (obs_s - self.cur_s) % self.track_length
            if gap < horizon:
                obs_d = obs.d_center
                if abs(obs_d) < self.params.lateral_width_gb_m:
                    gb_free = False
                    self.get_logger().info(f"GB_FREE False, obs dist to gb lane: {obs_d} m", throttle_duration_sec=1.0)
                    break

        return gb_free

    @property
    def _check_enemy_in_front(self) -> bool:
        # [Bug #11] Guard: matches ROS1 timetrials_only guard
        if self.params.mode != "head_to_head":
            return False

        horizon = self.params.gb_horizon_m  # Horizon in front of cur_s [m]
        for obs in self.obstacles:
            gap = (obs.s_start - self.cur_s) % self.track_length
            if gap < horizon:
                return True
        return False

    @property
    def _check_availability_splini_wpts(self) -> bool:
        if self.avoidance_wpnts is None:
            return False
        elif len(self.avoidance_wpnts.wpnts) == 0:
            return False
        # Say no to the ot line if the last switch was less than splini_hyst_timer_sec ago
        elif (
            abs(time_to_float(self.avoidance_wpnts.header.stamp) - time_to_float(self.avoidance_wpnts.last_switch_time))
            < self.params.splini_hyst_timer_sec
        ):
            self.get_logger().debug(f"Still too fresh into the switch...{abs(time_to_float(self.avoidance_wpnts.last_switch_time) - time_to_float(self.get_clock().now().to_msg()))}")
            return False
        else:
            # If the splinis are valid update the last valid ones
            self.last_valid_avoidance_wpnts = self.avoidance_wpnts.wpnts.copy()
            return True

    @property
    def _check_availability_graph_wpts(self) -> bool:
        if self.graph_based_wpts is None:
            return False
        return len(self.graph_based_wpts) > 0

    @property
    def _check_ftg(self) -> bool:
        # If we have been standing still for N seconds inside TRAILING -> FTG
        threshold = self.params.ftg_timer_sec * self.params.rate_hz
        if self.ftg_disabled:
            return False
        else:
            if self.state == StateType.TRAILING and self.cur_vs < self.params.ftg_threshold_speed:
                if self.ftg_counter == 0:
                    self.get_logger().info(f"FTG: Speed {self.cur_vs:.2f}m/s below threshold {self.params.ftg_threshold_speed}m/s. Starting timer...")
                
                self.ftg_counter += 1
                
                # Log timer every second (at rate_hz frequency)
                if self.ftg_counter % int(self.params.rate_hz) == 0:
                    remaining_sec = self.params.ftg_timer_sec - (self.ftg_counter / self.params.rate_hz)
                    self.get_logger().info(f"FTG: timer-- > {remaining_sec:.1f}s remaining")
            else:
                self.ftg_counter = 0

            if self.ftg_counter > threshold:
                self.get_logger().warn("FTG: Kicked in!")
                return True
            else:
                return False

    @property
    def _check_emergency_break(self) -> bool:
        emergency_break = False
        if self.params.overtake_mode == "predictive_spliner":
            obstacles = list(self.obstacles_perception)
            if obstacles and self.local_waypoints.wpnts:
                horizon = self.emergency_break_horizon
                for obs in obstacles:
                    dist_to_obj = (obs.s_start - self.cur_s) % self.track_length
                    if dist_to_obj < horizon:
                        local_wpnt_idx = np.argmin(
                            np.array([abs(w.s_m - obs.s_center) for w in self.local_waypoints.wpnts])
                        )
                        ot_d = self.local_waypoints.wpnts[local_wpnt_idx].d_m
                        ot_obs_dist = ot_d - obs.d_center
                        if abs(ot_obs_dist) < self.emergency_break_d:
                            emergency_break = True
                            self.get_logger().warn("Emergency break triggered!")
                            break
        return emergency_break

    @property
    def _check_force_trailing(self) -> bool:
        return self.force_trailing

    @property
    def _check_on_spline(self) -> bool:
        """True if ego is currently inside the avoidance spline's s-range"""
        if self.last_valid_avoidance_wpnts is not None and len(self.last_valid_avoidance_wpnts) > 0:
            s_start = self.last_valid_avoidance_wpnts[0].s_m
            s_end   = self.last_valid_avoidance_wpnts[-1].s_m
            # Spline wraps around track end
            if s_start > s_end:
                return (self.cur_s > s_start and self.cur_s < self.track_length) or \
                       (self.cur_s < s_end and self.cur_s > 0)
            else:
                return s_start < self.cur_s < s_end
        return False

    @property
    def _check_on_merger(self) -> bool:
        """True if ego is inside the merger region published by the SQP node"""
        if self.merger is not None and len(self.merger) >= 2:
            m0, m1 = self.merger[0], self.merger[1]
            if m0 < m1:
                return m0 < self.cur_s < m1
            elif m0 > m1:
                return self.cur_s > m0 or self.cur_s < m1
        return False

    @property
    def _check_prediction_gbfree(self) -> bool:
        """True if no collision-predicted obstacle is within 10 m ahead"""
        # [Bug #11] Guard for timetrials
        if self.params.mode != "head_to_head":
            return True
        horizon = 10.0
        for obs in self.obstacles_prediction:
            gap = (obs.s_start - self.cur_s) % self.track_length
            if gap < horizon:
                return False
        return True

    ###########
    # HELPERS #
    ###########
    def get_splini_wpts(self) -> WpntArray:
        """Obtain the waypoints by fusing those obtained by spliner with the
        global ones.
        [Bug #4] Restored the "avoidance over more than 1 lap" case from ROS1.
        [Bug #13] Restored threading lock.
        """
        splini_glob = self.glb_wpnts.copy()

        # Handle wrapping
        if self.last_valid_avoidance_wpnts is not None:
            # [Bug #4] Case 1: avoidance spans more than one full lap
            if len(self.last_valid_avoidance_wpnts) > len(splini_glob):
                splini_idxs = [
                    int(s % len(splini_glob))
                    for s in range(
                        int(self.last_valid_avoidance_wpnts[0].s_m / self.waypoints_dist + 0.5),
                        int((self.track_length + self.last_valid_avoidance_wpnts[0].s_m) / self.waypoints_dist + 0.5),
                    )
                ]
            # Case 2: normal non-wrapping
            elif self.last_valid_avoidance_wpnts[-1].s_m > self.last_valid_avoidance_wpnts[0].s_m:
                splini_idxs = [
                    s
                    for s in range(
                        int(self.last_valid_avoidance_wpnts[0].s_m / self.waypoints_dist + 0.5),
                        int(self.last_valid_avoidance_wpnts[-1].s_m / self.waypoints_dist + 0.5),
                    )
                ]
            # Case 3: wrapping around track end
            else:
                splini_idxs = [
                    int(s % len(splini_glob))
                    for s in range(
                        int(self.last_valid_avoidance_wpnts[0].s_m / self.waypoints_dist + 0.5),
                        int((self.track_length + self.last_valid_avoidance_wpnts[-1].s_m) / self.waypoints_dist + 0.5),
                    )
                ]

            with self.lock:  # [Bug #13] Restored lock
                for i, s in enumerate(splini_idxs):
                    splini_glob[s] = self.last_valid_avoidance_wpnts[min(i, len(self.last_valid_avoidance_wpnts) - 1)]

        # If the last valid points have been reset, then we just pass the global waypoints
        else:
            self.get_logger().warn(f"No valid avoidance waypoints, passing global waypoints")
            pass

        return splini_glob

    def get_graph_based_wpts(self):
        """Port of ROS1 state_machine_node.py get_graph_based_wpts (lines 791-816).
        Converts the raw Float32MultiArray rows into a WpntArray, filling d_left/d_right
        from the cached global waypoints array.
        """
        from f110_msgs.msg import WpntArray, Wpnt
        arr = WpntArray()
        if self.graph_based_wpts is None or self.gb_wpnts_arr is None:
            return arr
        for i, coord in enumerate(self.graph_based_wpts[:self.params.n_loc_wpnts, :]):
            w = Wpnt()
            w.s_m         = float((coord[0] + self.cur_s) % self.track_length)
            w.x_m         = float(coord[1])
            w.y_m         = float(coord[2])
            w.d_m         = 0.0
            w.psi_rad     = float(coord[3])
            w.kappa_radpm = float(coord[4])
            w.vx_mps      = float(coord[5])
            w.ax_mps2     = float(coord[6])
            w.id          = i
            idx = int(np.abs(self.gb_wpnts_arr[:, 0] - w.s_m).argmin())
            w.d_left  = float(self.gb_wpnts_arr[idx, 5] - w.d_m)
            w.d_right = float(self.gb_wpnts_arr[idx, 4] + w.d_m)
            arr.wpnts.append(w)
        return arr

    def get_frenet_wpts(self) -> list:
        """Fuse frenet planner waypoints into the global array for the OVERTAKE state.
        Frenet publishes with pub_global_track_advancement=false, so wpt.s_m is local arc-length
        starting from 0. We anchor by adding ego's cur_s so indices map to the correct global slots.
        Falls back to global if no frenet waypoints are available yet.
        """
        frenet_wpts = self.frenet_wpnts.wpnts if self.frenet_wpnts is not None else []
        splini_glob = self.glb_wpnts.copy()

        if not frenet_wpts:
            self.get_logger().warn("No frenet waypoints available, passing global waypoints", throttle_duration_sec=1.0)
            return splini_glob

        n_fwpts = len(frenet_wpts)
        avg_d = sum(w.d_m for w in frenet_wpts) / n_fwpts
        d_min = min(w.d_m for w in frenet_wpts)
        d_max = max(w.d_m for w in frenet_wpts)

        # Anchor frenet local s to ego's current track position
        # wpt.s_m is local arc-length from 0; add cur_s to get global s
        ego_s = self.cur_s
        s_start_global = (ego_s + frenet_wpts[0].s_m) % self.track_length
        s_end_global   = (ego_s + frenet_wpts[-1].s_m) % self.track_length

        self.get_logger().info(
            f'[SM FRENET_WPTS] n={n_fwpts} local_s=[{frenet_wpts[0].s_m:.1f},{frenet_wpts[-1].s_m:.1f}] '
            f'global_s=[{s_start_global:.1f},{s_end_global:.1f}] ego_s={ego_s:.1f} '
            f'avg_d={avg_d:.3f} d_min={d_min:.3f} d_max={d_max:.3f}',
            throttle_duration_sec=1.0
        )

        # Build global index list anchored at ego_s
        n_wpts = len(frenet_wpts)
        start_idx = int(s_start_global / self.waypoints_dist + 0.5)
        splini_idxs = [
            int((start_idx + i) % len(splini_glob))
            for i in range(n_wpts)
        ]

        with self.lock:
            for i, s in enumerate(splini_idxs):
                if 0 <= s < len(splini_glob):
                    splini_glob[s] = frenet_wpts[i]

        return splini_glob

    def init_ot_params(self):
        """Obtain the initial overtaking parameters from the parameter server.
        Then instantiate a parameter event handler to get the updates on those parameters.
        """
        self.parameter_client = self.create_client(GetParameters, '/ot_interpolator/get_parameters')
        self.parameter_client.wait_for_service()
        self.n_ot_sectors = None
        self.ot_param_names = None
        self.ot_sectors = None
        
        request = GetParameters.Request()
        request.names = ['n_sectors']
        future = self.parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.n_ot_sectors = future.result().values[0].integer_value
            self.get_logger().info(f'Got {self.n_ot_sectors} sectors')

            starts = [f'Overtaking_sector{i}.start' for i in range(self.n_ot_sectors)]
            ends = [f'Overtaking_sector{i}.end' for i in range(self.n_ot_sectors)]
            ot_flags = [f'Overtaking_sector{i}.ot_flag' for i in range(self.n_ot_sectors)]
            self.ot_param_names = ot_flags
            self.ot_sectors = []

            request = GetParameters.Request()
            request.names = starts + ends + ot_flags
            self.get_logger().info(f'Getting OT params: {request.names}')
            future = self.parameter_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                for i in range(self.n_ot_sectors):
                    start = future.result().values[i].integer_value
                    end = future.result().values[i + self.n_ot_sectors].integer_value
                    ot_flag = future.result().values[i + 2 * self.n_ot_sectors].bool_value
                    self.ot_sectors.append({'start': start, 'end': end, 'ot_flag': ot_flag})
                self.get_logger().info(f'OT sectors obtained: {self.ot_sectors}')
            else:
                self.get_logger().error(f'Service call failed {future.exception()}')
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')
        
        # once initialization is finished, the event handler can be used
        self.handler = ParameterEventHandler(self)

        for ot_flag in ot_flags:
            self.handler.add_parameter_callback(
                ot_flag,
                'ot_interpolator',
                callback=self.ot_flag_cb
            )

    def ot_flag_cb(self, p: Parameter):
        self.get_logger().info(f'OT flag changed: {p.name} {p.value.bool_value}')
        changed_sector_int = int(p.name.split('.')[0][-1])  # TODO ultra hardcoded ugliness
        self.ot_sectors[changed_sector_int]['ot_flag'] = p.value.bool_value

    def get_ot_params(self):
        self.get_logger().info(f'Getting only ot flags params')
        request = GetParameters.Request()
        request.names = self.ot_param_names
        future = self.parameter_client.call_async(request)
        self.executor.spin_until_future_complete(future)
        if future.result() is not None:
            for i in range(self.n_ot_sectors):
                self.ot_sectors[i]['ot_flag'] = future.result().values[i].bool_value
        else:
            self.get_logger().error(f'Service call failed {future.exception()}')

        self.get_logger().info(f'OT sectors: {self.ot_sectors}')

    ###############
    # PUB HELPERS #
    ###############
    def _pub_local_waypoints(self, wpts: WpntArray):
        loc_markers = MarkerArray()
        loc_wpnts = wpts
        # set stamp to now         
        loc_wpnts.header.stamp = self.get_clock().now().to_msg()
        loc_wpnts.header.frame_id = "map"

        for i, wpnt in enumerate(loc_wpnts.wpnts):
            mrk = Marker()
            mrk.header.frame_id = "map"
            mrk.type = mrk.SPHERE
            mrk.scale.x = 0.15
            mrk.scale.y = 0.15
            mrk.scale.z = 0.15
            mrk.color.a = 1.0
            mrk.color.g = 1.0

            mrk.id = i
            mrk.pose.position.x = wpnt.x_m
            mrk.pose.position.y = wpnt.y_m
            mrk.pose.position.z = wpnt.vx_mps / self.max_speed  # Visualise speed in z dimension
            mrk.pose.orientation.w = 1.0
            loc_markers.markers.append(mrk)

        if len(loc_wpnts.wpnts) == 0:
            self.get_logger().warn("No local waypoints published...", throttle_duration_sec=3.0)
        else:
            self.loc_wpnt_pub.publish(loc_wpnts)

        self.vis_loc_wpnt_pub.publish(loc_markers)
    
    def visualize_state(self, state: StateType):
        """
        Function that visualizes the state of the car by displaying a colored cube in RVIZ.

        Parameters
        ----------
        state
            Current state of the car to be displayed
        """
        if self.first_visualization:
            self.first_visualization = False
            x0 = self.glb_wpnts[0].x_m
            y0 = self.glb_wpnts[0].y_m
            x1 = self.glb_wpnts[1].x_m
            y1 = self.glb_wpnts[1].y_m
            # compute normal vector of 125% length of trackboundary but to the left of the trajectory
            xy_norm = (
                -np.array([y1 - y0, x0 - x1]) / np.linalg.norm([y1 - y0, x0 - x1]) * 1.25 * self.glb_wpnts[0].d_left
            )

            self.x_viz = x0 + xy_norm[0]
            self.y_viz = y0 + xy_norm[1]

        mrk = Marker()
        mrk.type = mrk.SPHERE
        mrk.id = int(1)
        mrk.header.frame_id = "map"
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.color.a = 1.0
        mrk.color.g = 1.0
        mrk.pose.position.x = float(self.x_viz)
        mrk.pose.position.y = float(self.y_viz)
        mrk.pose.position.z = 0.0
        mrk.pose.orientation.w = 1.0
        mrk.scale.x = 1.0
        mrk.scale.y = 1.0
        mrk.scale.z = 1.0

        # Set color and log info based on the state of the car
        if state == StateType.GB_TRACK:
            mrk.color.g = 1.0
        elif state == StateType.OVERTAKE:
            mrk.color.r = 1.0
            mrk.color.g = 1.0
            mrk.color.b = 1.0
        elif state == StateType.TRAILING:
            mrk.color.r = 0.0
            mrk.color.g = 0.0
            mrk.color.b = 1.0
        elif state == StateType.FTGONLY:
            mrk.color.r = 1.0
            mrk.color.g = 0.0
            mrk.color.b = 0.0
        self.state_marker_pub.publish(mrk)

    #############
    # MAIN LOOP #
    #############    
    def main_loop_callback(self):
        self.get_logger().debug(f"Current state: {self.state}")
        old_state = self.state

        # ---- Step 1: State transition ----
        if self.params.force_state:
            self.state = self.params.force_state_choice
        else:
            self.state = self.state_transition(self)
        
        # Log when returning from FTG back to spliner/raceline states
        if old_state == StateType.FTGONLY and self.state in (StateType.GB_TRACK, StateType.TRAILING, StateType.OVERTAKE):
            self.get_logger().info(f"FTG: Back to spliner (new state: {self.state.value})")

        # [Bug #3] Publish state as bare string value ("GB_TRACK", not "StateType.GB_TRACK")
        msg = String()
        msg.data = self.state.value  # <-- THIS IS THE CRITICAL FIX
        self.state_pub.publish(msg)
        self.visualize_state(state=self.state)

        # ---- Step 2: Generate and publish local waypoints ----
        self.local_waypoints.wpnts = self.state_logic(self)
        self._pub_local_waypoints(self.local_waypoints)

        # ---- Step 3: TTL management (Moved back to end per Bug #5 revert) ----
        if self.params.mode == "head_to_head":
            self.splini_ttl_counter -= 1
            if self.params.overtake_mode in ("spliner", "frenet"):
                if self.splini_ttl_counter <= 0:
                    self.last_valid_avoidance_wpnts = None
                    self.avoidance_wpnts = None
                    self.splini_ttl_counter = -1
            elif self.params.overtake_mode == "predictive_spliner":
                if self.splini_ttl_counter <= 0:
                    if not self._check_on_spline:
                        self.avoidance_wpnts = None
                        self.last_valid_avoidance_wpnts = None
                    elif self.splini_ttl_counter <= -int(self.params.pred_splini_ttl * self.params.rate_hz) * 3:
                        self.avoidance_wpnts = None
                        self.splini_ttl_counter = -10

        # [Bug #10] Clear FTG counter if not in TRAILING state (matches ROS1 lines 1401-1403)
        if self.state != StateType.TRAILING:
            self.ftg_counter = 0

        # HUD LOGGING: Print race status once per second
        if len(self.obstacles) > 0 and self.cur_s is not None:
            closest_obs = min(self.obstacles, key=lambda obs: (obs.s_center - self.cur_s) % self.track_length)
            gap = (closest_obs.s_center - self.cur_s) % self.track_length
            self.get_logger().info(
                f"HUD: Ego S: {self.cur_s:.1f}m | Opp S: {closest_obs.s_center:.1f}m | Gap: {gap:.1f}m | State: {self.state.value} | Planner: {self.params.overtake_mode}",
                throttle_duration_sec=1.0
            )

        # OVERTAKE diagnostics: log what d-values are being sent to controller
        if self.state == StateType.OVERTAKE and self.params.overtake_mode == "frenet":
            wpts = self.local_waypoints.wpnts
            frenet_n = len(self.frenet_wpnts.wpnts) if self.frenet_wpnts is not None else 0
            if wpts:
                d_vals = [w.d_m for w in wpts]
                avg_d = sum(d_vals) / len(d_vals)
                self.get_logger().info(
                    f'[SM OVERTAKE] sending {len(wpts)} wpts (frenet_n={frenet_n}) '
                    f'avg_d={avg_d:.3f} d_min={min(d_vals):.3f} d_max={max(d_vals):.3f} '
                    f'd_first={d_vals[0]:.3f} d_last={d_vals[-1]:.3f}',
                    throttle_duration_sec=1.0
                )


# defined as entry point in setup.py:
def main(args=None):
    rclpy.init(args=args)

    state_machine = StateMachine()

    rclpy.spin(state_machine)

    state_machine.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()