import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange, SetParametersResult
from rclpy.parameter import Parameter
from f110_msgs.msg import ObstacleArray, Obstacle, WpntArray, OpponentTrajectory, OppWpnt
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np
from frenet_conversion.frenet_converter import FrenetConverter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class ObstaclePublisher(Node):
    """Publish a dynamic obstacle in the F110 simulator.

    This is a ROS 2 port of the ROS 1 obstacle_publisher package with full
    feature parity, including:
      - Speed profiles: const, raceline, sine, step
      - Live parameter reconfiguration (speed_scaler, sine amplitudes/phases)
      - Always publishes ObstacleArray to /perception/obstacles (fixes ghosting)
      - Optional map-painting in 'lidar' mode (kept from ROS 2 port)
      - Visualization markers always published

    Launch parameters:
      - trajectory:    centerline / min_curv / shortest_path
      - start_s:       Starting s-coordinate along the track [m]
      - speed_scaler:  Scale factor applied to raceline speed (default 0.5)
      - speed_type:    const / raceline / sine / step
      - speed:         Constant speed [m/s] used by 'const' and base of 'sine'
      - type:          virtual / lidar  (visual only, does NOT affect topic publishing)
    """

    def __init__(self):
        super().__init__('obstacle_publisher')

        # ------------------------------------------------------------------ #
        # Static parameters (read once at startup)                           #
        # ------------------------------------------------------------------ #
        self.declare_parameters(
            namespace='',
            parameters=[
                ('trajectory',    'min_curv'),
                ('start_s',       0.0),
                ('type',          'virtual'),
                ('speed_type',    'raceline'),  # const / raceline / sine / step
                ('speed',         1.0),         # used by 'const' and 'sine'
            ]
        )

        # ------------------------------------------------------------------ #
        # Dynamic parameters (can be changed at runtime via ros2 param set)  #
        # ------------------------------------------------------------------ #
        pd_scaler = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=2.0, step=0.01)]
        )
        pd_sine = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=2.0, step=0.01)]
        )
        pd_phase = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(from_value=0.5, to_value=8.0, step=0.1)]
        )
        self.declare_parameter('speed_scaler', 0.5,  pd_scaler)
        self.declare_parameter('ampl_sin1',    0.3,  pd_sine)
        self.declare_parameter('ampl_sin2',    0.0,  pd_sine)
        self.declare_parameter('phase_sin1',   4.0,  pd_phase)
        self.declare_parameter('phase_sin2',   3.0,  pd_phase)
        self.add_on_set_parameters_callback(self._dyn_param_cb)

        # Read static params
        self.waypoints_type = self.get_parameter('trajectory').value
        self.starting_s     = float(self.get_parameter('start_s').value)
        self.opp_type       = self.get_parameter('type').value
        self.speed_type     = self.get_parameter('speed_type').value
        self.base_speed     = self.get_parameter('speed').value

        # Read dynamic params
        self._read_dyn_params()

        # Trajectory → topic mapping (same as ROS1)
        trajectory_map = {
            'min_curv':     '/global_waypoints',
            'shortest_path': '/global_waypoints/shortest_path',
            'centerline':   '/centerline_waypoints',
        }
        if self.waypoints_type not in trajectory_map:
            self.get_logger().error(f'Invalid trajectory type: {self.waypoints_type}')
            return
        self.waypoints_topic = trajectory_map[self.waypoints_type]

        # ------------------------------------------------------------------ #
        # Internal state                                                      #
        # ------------------------------------------------------------------ #
        self.ego_wpnts   = None
        self.max_s       = None
        self.wpnts       = None
        self.converter   = None
        self.car_odom    = None

        # Step-speed profile state
        self._step_timer      = 0.0
        self._step_phase_move = True    # True = moving, False = standing still

        # Map-painting state (lidar mode only)
        self.first_map    = True
        self.original_map = None
        self.current_map  = None
        self.obstacle_size = 2
        self.last_ind     = None

        self.dynamic_obstacle = self._init_dynamic_obstacle()
        self.obj_len = 0.5
        self.counter = 0

        # ------------------------------------------------------------------ #
        # Subscriptions                                                        #
        # ------------------------------------------------------------------ #
        self.create_subscription(Odometry,  '/car_state/odom_frenet', self._odom_cb, 10)
        self.create_subscription(WpntArray, self.waypoints_topic,     self._waypoints_cb, 10)
        self.create_subscription(WpntArray, '/global_waypoints',      self._global_waypoints_cb, 10)

        if self.opp_type == 'lidar':
            qos_map = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            )
            self.create_subscription(OccupancyGrid, '/map', self._map_cb, qos_profile=qos_map)

        # ------------------------------------------------------------------ #
        # Publishers                                                           #
        # ------------------------------------------------------------------ #
        # ALWAYS publish the ObstacleArray — this is the core fix vs old ROS2 port
        self.obstacle_pub      = self.create_publisher(ObstacleArray,     '/perception/obstacles',  10)
        self.marker_pub        = self.create_publisher(MarkerArray,        '/dummy_obstacle_markers', 10)
        self.opponent_traj_pub = self.create_publisher(OpponentTrajectory, '/opponent_waypoints',    10)

        if self.opp_type == 'lidar':
            qos_map = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            )
            self.pub_map = self.create_publisher(OccupancyGrid, '/map', qos_map)

        # Run blocking setup then start timer
        self._loop_setup()

    # ====================================================================== #
    # Dynamic parameter callback                                              #
    # ====================================================================== #
    def _read_dyn_params(self):
        self.speed_scaler = self.get_parameter('speed_scaler').value
        self.ampl_sin1    = self.get_parameter('ampl_sin1').value
        self.ampl_sin2    = self.get_parameter('ampl_sin2').value
        self.phase_sin1   = self.get_parameter('phase_sin1').value
        self.phase_sin2   = self.get_parameter('phase_sin2').value

    def _dyn_param_cb(self, params):
        for param in params:
            if param.name == 'speed_scaler':
                self.speed_scaler = param.value
            elif param.name == 'ampl_sin1':
                self.ampl_sin1 = param.value
            elif param.name == 'ampl_sin2':
                self.ampl_sin2 = param.value
            elif param.name == 'phase_sin1':
                self.phase_sin1 = param.value
            elif param.name == 'phase_sin2':
                self.phase_sin2 = param.value
        self.get_logger().info(
            f'[obstacle_publisher] Params updated: speed_scaler={self.speed_scaler:.2f}, '
            f'ampl_sin1={self.ampl_sin1:.2f}, ampl_sin2={self.ampl_sin2:.2f}, '
            f'phase_sin1={self.phase_sin1:.2f}, phase_sin2={self.phase_sin2:.2f}'
        )
        return SetParametersResult(successful=True)

    # ====================================================================== #
    # Initializer                                                             #
    # ====================================================================== #
    def _init_dynamic_obstacle(self):
        obs = Obstacle()
        obs.id = 1
        obs.d_right = -0.1
        obs.d_left  =  0.1
        obs.is_actually_a_gap = False
        return obs

    # ====================================================================== #
    # Callbacks                                                               #
    # ====================================================================== #
    def _global_waypoints_cb(self, data: WpntArray):
        self.ego_wpnts = data.wpnts[:-1]
        self.max_s     = self.ego_wpnts[-1].s_m
        if self.converter is None:
            pts = np.array([[w.x_m, w.y_m, w.psi_rad] for w in data.wpnts])
            self.converter = FrenetConverter(pts[:, 0], pts[:, 1], pts[:, 2])

    def _waypoints_cb(self, data: WpntArray):
        self.wpnts = data.wpnts[:-1]

    def _odom_cb(self, data: Odometry):
        self.car_odom = data

    def _map_cb(self, data: OccupancyGrid):
        if self.first_map:
            self.original_map = data
            self.current_map  = self.original_map
            self.first_map    = False
        else:
            self.current_map.data = data.data

    # ====================================================================== #
    # Speed profile helpers                                                   #
    # ====================================================================== #
    def _compute_speed(self, base_raceline_speed: float, elapsed_time: float) -> float:
        """
        Returns the instantaneous speed for the virtual opponent based on speed_type.

        speed_type options (mirrors ROS1 README):
          - 'const'    : always self.base_speed  (m/s)
          - 'raceline' : base_raceline_speed * speed_scaler
          - 'sine'     : base_speed + ampl_sin1*sin(2π*t/phase_sin1)
                                    + ampl_sin2*sin(2π*t/phase_sin2)
          - 'step'     : stand still for 2.5 s, then drive at base_speed for 2.5 s
        """
        if self.speed_type == 'const':
            return self.base_speed

        elif self.speed_type == 'raceline':
            return base_raceline_speed * self.speed_scaler

        elif self.speed_type == 'sine':
            v = (self.base_speed
                 + self.ampl_sin1 * np.sin(2 * np.pi * elapsed_time / self.phase_sin1)
                 + self.ampl_sin2 * np.sin(2 * np.pi * elapsed_time / self.phase_sin2))
            return max(0.0, v)

        elif self.speed_type == 'step':
            phase_duration = 2.5  # seconds per phase, matching ROS1
            phase = (elapsed_time % (2 * phase_duration))
            return self.base_speed if phase >= phase_duration else 0.0

        else:
            self.get_logger().warn(f'Unknown speed_type: {self.speed_type}, defaulting to raceline')
            return base_raceline_speed * self.speed_scaler

    # ====================================================================== #
    # Map painting helpers (lidar mode — kept from existing ROS2 port)        #
    # ====================================================================== #
    def _rc_2_ind(self, r, c):
        return r * self.original_map.info.width + c

    def _ind_2_rc(self, ind):
        return [ind // self.original_map.info.width,
                ind  % self.original_map.info.width]

    def _coord_2_cell_rc(self, x, y):
        ox = self.original_map.info.origin.position.x
        oy = self.original_map.info.origin.position.y
        res = self.original_map.info.resolution
        return [int((y - oy) // res), int((x - ox) // res)]

    def _add_lidar_obs(self, ind):
        rc = self._ind_2_rc(ind)
        for i in range(-self.obstacle_size, self.obstacle_size):
            for j in range(-self.obstacle_size, self.obstacle_size):
                idx = self._rc_2_ind(rc[0] + i, rc[1] + j)
                self.current_map.data[idx] = 100
        self.pub_map.publish(self.current_map)

    def _clear_lidar_obs(self, ind):
        rc = self._ind_2_rc(ind)
        for i in range(-self.obstacle_size, self.obstacle_size):
            for j in range(-self.obstacle_size, self.obstacle_size):
                idx = self._rc_2_ind(rc[0] + i, rc[1] + j)
                self.current_map.data[idx] = 0
        self.pub_map.publish(self.current_map)

    # ====================================================================== #
    # Visualization                                                            #
    # ====================================================================== #
    def _publish_markers(self, obstacles):
        """Always publish RVIZ markers (identical to ROS1 behaviour)."""
        marker_array = MarkerArray()
        for obs in obstacles:
            xy = self.converter.get_cartesian(obs.s_center, obs.d_center)
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = self.get_clock().now().to_msg()
            m.type  = Marker.SPHERE
            m.id    = obs.id
            m.scale.x = m.scale.y = m.scale.z = 0.5
            m.color.a = 0.8
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.pose.position.x = float(xy[0])
            m.pose.position.y = float(xy[1])
            m.pose.orientation.w = 1.0
            marker_array.markers.append(m)
        self.marker_pub.publish(marker_array)

    # ====================================================================== #
    # Setup (blocking — runs once before main timer starts)                   #
    # ====================================================================== #
    def _loop_setup(self):
        # 1) Wait for global waypoints (needed for Frenet converter + ego speed)
        while self.ego_wpnts is None:
            self.get_logger().info('Waiting for /global_waypoints...', throttle_duration_sec=1.0)
            rclpy.spin_once(self)

        # 2) If lidar mode, also wait for the map
        if self.opp_type == 'lidar':
            while self.original_map is None:
                self.get_logger().info('Waiting for /map...', throttle_duration_sec=1.0)
                rclpy.spin_once(self)

        # 3) Wait for the chosen trajectory waypoints (may be same as global)
        while self.wpnts is None:
            self.get_logger().info(f'Waiting for {self.waypoints_topic}...', throttle_duration_sec=1.0)
            rclpy.spin_once(self)

        # 4) Build ego speed profile along the track
        s_array = np.array([w.s_m for w in self.ego_wpnts])

        if self.speed_type == 'const':
            ego_speeds = np.full(len(self.ego_wpnts), self.base_speed)
        else:
            ego_speeds = np.array([w.vx_mps * self.speed_scaler for w in self.ego_wpnts])

        # 5) Project opponent trajectory into Frenet frame and resample onto ego s-grid
        opp_frenet = self.converter.get_frenet(
            [w.x_m for w in self.wpnts],
            [w.y_m for w in self.wpnts]
        )
        opp_s = np.array(opp_frenet[0])
        opp_d = np.array(opp_frenet[1])

        sorted_idx      = np.argsort(opp_s)
        opp_s_sorted    = opp_s[sorted_idx]
        opp_d_sorted    = opp_d[sorted_idx]

        resampled_d  = np.interp(s_array, opp_s_sorted, opp_d_sorted)
        resampled_xy = self.converter.get_cartesian(s_array, resampled_d)

        # 6) Build OpponentTrajectory
        self.opponent_wpnts = OpponentTrajectory()
        for i in range(len(s_array)):
            wpnt = OppWpnt()
            wpnt.x_m         = float(resampled_xy[0][i])
            wpnt.y_m         = float(resampled_xy[1][i])
            wpnt.proj_vs_mps = float(ego_speeds[i])
            wpnt.s_m         = float(s_array[i])
            wpnt.d_m         = float(resampled_d[i])
            self.opponent_wpnts.oppwpnts.append(wpnt)

        self.opponent_s_array = np.array([w.s_m for w in self.opponent_wpnts.oppwpnts])

        # 7) Set starting position
        self.dynamic_obstacle.s_center = self.starting_s
        self.looptime = 1.0 / 50.0
        self._start_time = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            f'[obstacle_publisher] Ready! '
            f'speed_type={self.speed_type}, trajectory={self.waypoints_type}, '
            f'start_s={self.starting_s}, opp_type={self.opp_type}'
        )

        self.timer = self.create_timer(self.looptime, self._timer_cb)

    # ====================================================================== #
    # Main loop                                                               #
    # ====================================================================== #
    def _timer_cb(self):
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._start_time

        # --- Find nearest waypoint index ---
        s = self.dynamic_obstacle.s_center
        approx_idx = int(np.abs(self.opponent_s_array - s).argmin())

        # --- Compute speed using selected profile ---
        base_raceline_speed = self.opponent_wpnts.oppwpnts[approx_idx].proj_vs_mps
        speed = self._compute_speed(base_raceline_speed, elapsed)

        # --- Advance obstacle position ---
        self.dynamic_obstacle.s_center = (s + speed * self.looptime) % self.max_s
        self.dynamic_obstacle.s_start  = (self.dynamic_obstacle.s_center - self.obj_len / 2) % self.max_s
        self.dynamic_obstacle.s_end    = (self.dynamic_obstacle.s_center + self.obj_len / 2) % self.max_s
        self.dynamic_obstacle.d_center = self.opponent_wpnts.oppwpnts[approx_idx].d_m
        self.dynamic_obstacle.d_right  = self.dynamic_obstacle.d_center - 0.1
        self.dynamic_obstacle.d_left   = self.dynamic_obstacle.d_center + 0.1
        self.dynamic_obstacle.vs       = speed

        # --- Build message ---
        obstacle_msg = ObstacleArray()
        obstacle_msg.header.stamp    = self.get_clock().now().to_msg()
        obstacle_msg.header.frame_id = 'frenet'
        obstacle_msg.obstacles.append(self.dynamic_obstacle)

        self.obstacle_pub.publish(obstacle_msg)

        # HUD LOGGING: Detailed status of virtual opponent
        self.get_logger().info(
            f"VIRTUAL OPPONENT: S: {self.dynamic_obstacle.s_center:.1f}m | D: {self.dynamic_obstacle.d_center:.1f}m | Speed: {self.dynamic_obstacle.vs:.2f}m/s | Profile: {self.speed_type}",
            throttle_duration_sec=1.0
        )

        # --- ALWAYS publish RVIZ markers ---
        self._publish_markers(obstacle_msg.obstacles)

        # --- Optional: paint onto occupancy map (lidar mode) ---
        if self.opp_type == 'lidar' and self.original_map is not None:
            xy      = self.converter.get_cartesian(self.dynamic_obstacle.s_center,
                                                   self.dynamic_obstacle.d_center)
            rc_add  = self._coord_2_cell_rc(float(xy[0]), float(xy[1]))
            ind_add = self._rc_2_ind(rc_add[0], rc_add[1])
            if self.last_ind is not None:
                self._clear_lidar_obs(self.last_ind)
            self.last_ind = ind_add
            self._add_lidar_obs(ind_add)

        # --- Periodically publish opponent trajectory (for Spliner) ---
        self.counter += 1
        if self.counter > 25:
            traj_msg = OpponentTrajectory()
            traj_msg.header.stamp    = self.get_clock().now().to_msg()
            traj_msg.header.frame_id = 'frenet'
            traj_msg.lap_count       = 2
            traj_msg.oppwpnts        = self.opponent_wpnts.oppwpnts
            self.opponent_traj_pub.publish(traj_msg)
            self.counter = 0


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
