import rclpy
import rclpy.time
import rclpy.duration
import rclpy.clock
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult

import numpy as np
import range_libc
from threading import Lock
from collections import deque
from time import time, sleep
from enum import Enum
from typing import Optional
import cProfile
import pstats

from geometry_msgs.msg import (PoseStamped, PoseArray, PoseWithCovarianceStamped,
                                PointStamped, TransformStamped, Quaternion)
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import tf2_ros
from tf2_ros import TransformBroadcaster

from state_estimation import synpf_utils as Utils


class RangeLibVariant(Enum):
    VAR_NO_EVAL_SENSOR_MODEL = 0
    VAR_CALC_RANGE_MANY_EVAL_SENSOR = 1
    VAR_REPEAT_ANGLES_EVAL_SENSOR = 2
    VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT = 3
    VAR_RADIAL_CDDT_OPTIMIZATIONS = 4


class ParticleFilter(Node):

    def __init__(self):
        super().__init__('synpf_node')
        self._declare_params()
        self._read_params()

        # State flags (same as ROS1)
        self.state_lock = Lock()
        self.map_initialized = False
        self.lidar_initialized = False
        self.odom_initialized = False
        self.first_sensor_update = True

        # Odom deque — maxlen=5 matches ROS1
        self.odom_msgs: deque = deque([], maxlen=5)
        self.last_odom_msg: Optional[Odometry] = None
        self.odometry_data = np.array([0.0, 0.0, 0.0])
        self.last_pose: Optional[np.ndarray] = None
        self.curr_pose: Optional[np.ndarray] = None
        self.last_stamp: Optional[rclpy.time.Time] = None
        self.last_pub_stamp: rclpy.time.Time = rclpy.time.Time()

        # Particles + weights
        self.particles = np.zeros((self.MAX_PARTICLES, 3))
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)
        self.particle_indices = np.arange(self.MAX_PARTICLES)
        self.inferred_pose: Optional[np.ndarray] = None
        self.local_deltas = np.zeros((self.MAX_PARTICLES, 3))
        if self.PUB_COVARIANCE:
            self.cov = np.zeros((3, 3))

        # Sensor model caches (allocated on first sensor_model() call)
        self.queries: Optional[np.ndarray] = None
        self.ranges: Optional[np.ndarray] = None
        self.tiled_angles: Optional[np.ndarray] = None
        self.sensor_model_table: Optional[np.ndarray] = None

        # LiDAR metadata — populated by _wait_for_scan()
        self.NUM_LIDAR_BEAMS: Optional[int] = None
        self.START_THETA: Optional[float] = None
        self.END_THETA: Optional[float] = None
        self.downsampled_ranges: Optional[np.ndarray] = None

        # Initialise in same order as ROS1
        self.get_omap()
        self.precompute_sensor_model()

        # Block until first /scan arrives (mirrors rospy.wait_for_message)
        # This MUST happen after get_omap so NUM_LIDAR_BEAMS can be checked if needed
        self._wait_for_scan()
        self.get_boxed_indices()

        # Initial pose
        has_init = (self.has_parameter('initial_pose_x') and
                    self.has_parameter('initial_pose_y') and
                    self.has_parameter('initial_pose_theta'))
        if has_init:
            self.initialize_particles_pose(
                self.get_parameter('initial_pose_x').value,
                self.get_parameter('initial_pose_y').value,
                posetheta=self.get_parameter('initial_pose_theta').value)
        else:
            self.initialize_global()

        # Publishers
        self.particle_pub = self.create_publisher(PoseArray, '/pf/viz/particles', 1)
        self.pose_pub = self.create_publisher(PoseStamped, self.POSE_PUB_TOPIC, 1)
        if self.PUB_COVARIANCE:
            self.pose_cov_pub = self.create_publisher(
                PoseWithCovarianceStamped,
                self.POSE_PUB_TOPIC + '/with_covariance', 1)

        # TF broadcaster + listener
        self.pub_tf = TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Wait for base_frame_id → laser_frame_id TF (mirrors ROS1 while loop)
        self._wait_for_tf(self.BASE_FRAME_ID, self.LASER_FRAME_ID)
        trans: TransformStamped = self.tf_buffer.lookup_transform(
            self.BASE_FRAME_ID, self.LASER_FRAME_ID, rclpy.time.Time())
        self.laser_base_link_offset = np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            0.0])
        self.particle_utils = Utils.ParticleUtils(self.laser_base_link_offset)

        # QoS profiles
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=2)
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=2)
        init_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # Separate callback groups to avoid deadlock with the loop thread
        self.scan_cbg  = MutuallyExclusiveCallbackGroup()
        self.odom_cbg  = MutuallyExclusiveCallbackGroup()
        self.init_cbg  = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            LaserScan, self.ODOM_SCAN_TOPIC, self.lidarCB,
            scan_qos, callback_group=self.scan_cbg)
        self.create_subscription(
            Odometry, self.ODOM_TOPIC, self.odomCB,
            odom_qos, callback_group=self.odom_cbg)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose',
            self.clicked_poseCB, init_qos, callback_group=self.init_cbg)
        self.create_subscription(
            PointStamped, '/clicked_point',
            self.clicked_pointCB, 1, callback_group=self.init_cbg)

        # Live-tunable params (replaces dynamic_reconfigure)
        self.add_on_set_parameters_callback(self.dyn_param_cb)

        if self.DEBUG:
            self.sensor_model_time_ms: deque = deque(maxlen=150)
            self.motion_model_time_ms: deque = deque(maxlen=150)
            self.overall_time_ms: deque = deque(maxlen=150)
            self.itr = 0
            self.profiler = cProfile.Profile()

        self.get_logger().info('SynPF finished initializing.')

    # -----------------------------------------------------------------------
    # Parameter declaration + reading
    # -----------------------------------------------------------------------

    def _declare_params(self):
        self.declare_parameter('map_yaml_path', '')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odometry_topic', '/odom')
        self.declare_parameter('pose_pub_topic', '/tracked_pose')
        self.declare_parameter('viz', False)
        self.declare_parameter('max_viz_particles', 50)
        self.declare_parameter('max_particles', 3000)
        self.declare_parameter('squash_factor', 2.2)
        self.declare_parameter('max_range', 20.0)
        self.declare_parameter('des_lidar_beams', 21)
        self.declare_parameter('lidar_aspect_ratio', 3.5)
        self.declare_parameter('range_method', 'glt')
        self.declare_parameter('rangelib_variant', 3)
        self.declare_parameter('theta_discretization', 150)
        self.declare_parameter('z_hit', 0.85)
        self.declare_parameter('z_short', 0.10)
        self.declare_parameter('z_max', 0.025)
        self.declare_parameter('z_rand', 0.025)
        self.declare_parameter('sigma_hit', 0.1)
        self.declare_parameter('lambda_short', 0.25)
        self.declare_parameter('motion_model', 'tum')
        self.declare_parameter('alpha_1_tum', 0.5)
        self.declare_parameter('alpha_2_tum', 0.015)
        self.declare_parameter('alpha_3_tum', 1.0)
        self.declare_parameter('alpha_4_tum', 0.1)
        self.declare_parameter('lam_thresh', 0.1)
        self.declare_parameter('alpha_1_amcl', 0.5)
        self.declare_parameter('alpha_2_amcl', 0.5)
        self.declare_parameter('alpha_3_amcl', 1.0)
        self.declare_parameter('alpha_4_amcl', 0.1)
        self.declare_parameter('motion_dispersion_arc_x', 0.8)
        self.declare_parameter('motion_dispersion_arc_y', 1.0)
        self.declare_parameter('motion_dispersion_arc_theta', 0.75)
        self.declare_parameter('motion_dispersion_arc_xy', 1.0)
        self.declare_parameter('motion_dispersion_arc_x_min', 0.01)
        self.declare_parameter('motion_dispersion_arc_y_min', 0.03)
        self.declare_parameter('motion_dispersion_arc_y_max', 1.0)
        self.declare_parameter('motion_dispersion_arc_theta_min', 0.01)
        self.declare_parameter('motion_dispersion_arc_xy_min_x', 0.10)
        self.declare_parameter('motion_dispersion_x', 0.05)
        self.declare_parameter('motion_dispersion_y', 0.025)
        self.declare_parameter('motion_dispersion_theta', 0.25)
        self.declare_parameter('initial_var_x', 0.5)
        self.declare_parameter('initial_var_y', 0.5)
        self.declare_parameter('initial_var_theta', 0.4)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('pub_covariance', False)
        self.declare_parameter('debug', False)
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('laser_frame_id', 'laser')

    def _read_params(self):
        g = self.get_parameter
        self.MAP_YAML_PATH       = str(g('map_yaml_path').value)
        self.get_logger().info(f"DEBUG: Read map_yaml_path as: '{self.MAP_YAML_PATH}'")
        self.MAX_PARTICLES       = int(g('max_particles').value)
        self.MAX_VIZ_PARTICLES   = int(g('max_viz_particles').value)
        self.INV_SQUASH_FACTOR   = 1.0 / float(g('squash_factor').value)
        self.MAX_RANGE_METERS    = float(g('max_range').value)
        self.WHICH_RM            = str(g('range_method').value).lower()
        self.RANGELIB_VAR        = RangeLibVariant(int(g('rangelib_variant').value))
        self.POSE_PUB_TOPIC      = str(g('pose_pub_topic').value)
        self.DO_VIZ              = bool(g('viz').value)
        self.PUBLISH_TF          = bool(g('publish_tf').value)
        self.ODOM_TOPIC          = str(g('odometry_topic').value)
        self.ODOM_SCAN_TOPIC     = str(g('scan_topic').value)
        self.THETA_DISCRETIZATION = int(g('theta_discretization').value)
        self.PUB_COVARIANCE      = bool(g('pub_covariance').value)
        self.DEBUG               = bool(g('debug').value)
        self.BASE_FRAME_ID       = str(g('base_frame_id').value)
        self.LASER_FRAME_ID      = str(g('laser_frame_id').value)
        self.DES_LIDAR_BEAMS     = int(g('des_lidar_beams').value)
        self.LIDAR_ASPECT_RATIO  = float(g('lidar_aspect_ratio').value)

        self.INIT_VAR_X  = float(g('initial_var_x').value)
        self.INIT_VAR_Y  = float(g('initial_var_y').value)
        self.INIT_VAR_TH = float(g('initial_var_theta').value)

        self.Z_HIT    = float(g('z_hit').value)
        self.Z_SHORT  = float(g('z_short').value)
        self.Z_MAX    = float(g('z_max').value)
        self.Z_RAND   = float(g('z_rand').value)
        self.SIGMA_HIT  = float(g('sigma_hit').value)
        self.LAM_SHORT  = float(g('lambda_short').value)

        self.MOTION_MODEL = str(g('motion_model').value).lower()
        if self.MOTION_MODEL == 'tum':
            self.ALPHA_1   = float(g('alpha_1_tum').value)
            self.ALPHA_2   = float(g('alpha_2_tum').value)
            self.ALPHA_3   = float(g('alpha_3_tum').value)
            self.ALPHA_4   = float(g('alpha_4_tum').value)
            self.LAM_THRESH = float(g('lam_thresh').value)
            self.get_logger().info(
                f'PF2 TUM params: a1={self.ALPHA_1} a2={self.ALPHA_2} '
                f'a3={self.ALPHA_3} a4={self.ALPHA_4} lam={self.LAM_THRESH}')
        elif self.MOTION_MODEL == 'amcl':
            self.ALPHA_1 = float(g('alpha_1_amcl').value)
            self.ALPHA_2 = float(g('alpha_2_amcl').value)
            self.ALPHA_3 = float(g('alpha_3_amcl').value)
            self.ALPHA_4 = float(g('alpha_4_amcl').value)
        elif self.MOTION_MODEL == 'arc':
            self.get_logger().warn('Using arc motion model - may not be fully tested.')
            self.MOTION_DISPERSION_ARC_X     = float(g('motion_dispersion_arc_x').value)
            self.MOTION_DISPERSION_ARC_Y     = float(g('motion_dispersion_arc_y').value)
            self.MOTION_DISPERSION_ARC_THETA = float(g('motion_dispersion_arc_theta').value)
            self.MOTION_DISPERSION_ARC_XY    = float(g('motion_dispersion_arc_xy').value)
            self.MOTION_DISPERSION_ARC_X_MIN = float(g('motion_dispersion_arc_x_min').value)
            self.MOTION_DISPERSION_ARC_Y_MIN = float(g('motion_dispersion_arc_y_min').value)
            self.MOTION_DISPERSION_ARC_Y_MAX = float(g('motion_dispersion_arc_y_max').value)
            self.MOTION_DISPERSION_ARC_THETA_MIN = float(g('motion_dispersion_arc_theta_min').value)
            self.MOTION_DISPERSION_ARC_XY_MIN_X  = float(g('motion_dispersion_arc_xy_min_x').value)
        else:
            self.get_logger().warn('Using default MIT PF motion model - may not be fully tested.')
            self.MOTION_DISPERSION_X     = float(g('motion_dispersion_x').value)
            self.MOTION_DISPERSION_Y     = float(g('motion_dispersion_y').value)
            self.MOTION_DISPERSION_THETA = float(g('motion_dispersion_theta').value)

    # -----------------------------------------------------------------------
    # Startup helpers
    # -----------------------------------------------------------------------

    def _wait_for_scan(self):
        """Block until first /scan message arrives (replaces rospy.wait_for_message)."""
        import threading
        got_scan = threading.Event()

        def _cb(msg: LaserScan):
            if not got_scan.is_set():
                self.NUM_LIDAR_BEAMS = len(msg.ranges)
                self.START_THETA = msg.angle_min
                self.END_THETA = msg.angle_max
                got_scan.set()

        tmp_sub = self.create_subscription(
            LaserScan, self.ODOM_SCAN_TOPIC, _cb, 1)
        self.get_logger().info('Waiting for first /scan message...')
        while not got_scan.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(tmp_sub)
        self.get_logger().info(
            f'Got scan: {self.NUM_LIDAR_BEAMS} beams, '
            f'[{self.START_THETA:.2f}, {self.END_THETA:.2f}] rad')

    def _wait_for_tf(self, target: str, source: str):
        """Block until TF is available (mirrors ROS1 while not can_transform loop)."""
        self.get_logger().info(f'Waiting for {target} → {source} TF...')
        while not self.tf_buffer.can_transform(
                target, source, rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1)):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'{target} → {source} TF OK')

    # -----------------------------------------------------------------------
    # Map loading  (replaces rospy.wait_for_service("static_map"))
    # -----------------------------------------------------------------------

    def get_omap(self):
        map_yaml = self.MAP_YAML_PATH
        if not map_yaml:
            self.get_logger().error('map_yaml_path parameter is empty!')
            raise RuntimeError('map_yaml_path must be set')

        map_msg = Utils.load_map_as_occupancy_grid(map_yaml)
        self.map_info = map_msg.info
        oMap = range_libc.PyOMap(map_msg)
        self.MAX_RANGE_PX = int(self.MAX_RANGE_METERS / self.map_info.resolution)

        self.get_logger().info(f'Initializing range method: {self.WHICH_RM}')
        if self.WHICH_RM == 'bl':
            self.range_method = range_libc.PyBresenhamsLine(oMap, self.MAX_RANGE_PX)
        elif 'cddt' in self.WHICH_RM:
            self.range_method = range_libc.PyCDDTCast(
                oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
            if self.WHICH_RM == 'pcddt':
                self.get_logger().info('Pruning CDDT...')
                self.range_method.prune()
        elif self.WHICH_RM == 'rm':
            self.range_method = range_libc.PyRayMarching(oMap, self.MAX_RANGE_PX)
        elif self.WHICH_RM == 'rmgpu':
            self.range_method = range_libc.PyRayMarchingGPU(oMap, self.MAX_RANGE_PX)
        elif self.WHICH_RM == 'glt':
            self.range_method = range_libc.PyGiantLUTCast(
                oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
        else:
            raise ValueError(f'Unknown range_method: {self.WHICH_RM}')
        self.get_logger().info('Done loading map')

        array_255 = np.array(map_msg.data).reshape(
            (map_msg.info.height, map_msg.info.width))
        self.permissible_region = np.zeros_like(array_255, dtype=bool)
        self.permissible_region[array_255 == 0] = 1
        self.map_initialized = True

    # -----------------------------------------------------------------------
    # Sensor model precomputation  (verbatim from ROS1, log calls only change)
    # -----------------------------------------------------------------------

    def precompute_sensor_model(self):
        self.get_logger().info('Precomputing sensor model')
        z_short = self.Z_SHORT
        z_max   = self.Z_MAX
        z_rand  = self.Z_RAND
        z_hit   = self.Z_HIT
        # sigma and lambda converted from metres to pixel space
        sigma_hit = self.SIGMA_HIT / self.map_info.resolution
        lam_short = self.LAM_SHORT / self.map_info.resolution

        table_width = int(self.MAX_RANGE_PX) + 1
        self.sensor_model_table = np.zeros((table_width, table_width))

        norm_gau = np.zeros((table_width,))
        norm_exp = np.zeros((table_width,))
        for d in range(table_width):
            sum_gau = 0.0
            sum_exp = 0.0
            for r in range(table_width):
                z = float(d - r)
                sum_gau += (np.exp(-(z * z) / (2.0 * sigma_hit * sigma_hit)) /
                            (sigma_hit * np.sqrt(2.0 * np.pi)))
                if r <= d:
                    sum_exp += lam_short * np.exp(-lam_short * r)
            norm_gau[d] = 1.0 / sum_gau if sum_gau > 0 else 0.0
            norm_exp[d] = 1.0 / sum_exp if sum_exp > 0 else 0.0

        for d in range(table_width):
            norm = 0.0
            for r in range(table_width):
                prob = 0.0
                z = float(d - r)
                prob += z_hit * (
                    np.exp(-(z * z) / (2.0 * sigma_hit * sigma_hit)) /
                    (sigma_hit * np.sqrt(2.0 * np.pi))) * norm_gau[d]
                if r <= d:
                    prob += z_short * (lam_short * np.exp(-lam_short * r)) * norm_exp[d]
                if r == int(self.MAX_RANGE_PX):
                    prob += z_max
                if r < self.MAX_RANGE_PX:
                    prob += z_rand * 1.0 / self.MAX_RANGE_PX
                norm += prob
                self.sensor_model_table[r, d] = prob
            self.sensor_model_table[:, d] /= norm

        if self.RANGELIB_VAR.value > 0:
            self.range_method.set_sensor_model(self.sensor_model_table)

    # -----------------------------------------------------------------------
    # Boxed LiDAR beam selection  (verbatim from ROS1 — TUM paper method)
    # -----------------------------------------------------------------------

    def get_boxed_indices(self):
        beam_angles = np.linspace(
            self.START_THETA, self.END_THETA, self.NUM_LIDAR_BEAMS)

        MID_IDX = self.NUM_LIDAR_BEAMS // 2
        sparse_idxs = [MID_IDX]

        a = self.LIDAR_ASPECT_RATIO
        beam_proj = 2 * a * np.array([np.cos(beam_angles), np.sin(beam_angles)])
        beam_intersections = np.zeros((2, self.NUM_LIDAR_BEAMS))

        box_corners = [(a, 1), (a, -1), (-a, -1), (-a, 1)]
        for idx in range(len(box_corners)):
            x1, y1 = box_corners[idx]
            x2, y2 = box_corners[0] if idx == 3 else box_corners[idx + 1]
            for i in range(self.NUM_LIDAR_BEAMS):
                x4 = beam_proj[0, i]
                y4 = beam_proj[1, i]
                den = (x1 - x2) * (-y4) - (y1 - y2) * (-x4)
                if den == 0:
                    continue
                t = ((x1) * (-y4) - (y1) * (-x4)) / den
                u = ((x1) * (y1 - y2) - (y1) * (x1 - x2)) / den
                px = u * x4
                py = u * y4
                if 0 <= t <= 1.0 and 0 <= u <= 1.0:
                    beam_intersections[0, i] = px
                    beam_intersections[1, i] = py

        dx = np.diff(beam_intersections[0, :])
        dy = np.diff(beam_intersections[1, :])
        dist = np.sqrt(dx**2 + dy**2)
        total_dist = np.sum(dist)
        dist_amt = total_dist / (self.DES_LIDAR_BEAMS - 1)

        idx = MID_IDX + 1
        DES_BEAMS2 = self.DES_LIDAR_BEAMS // 2 + 1
        acc = 0
        while len(sparse_idxs) <= DES_BEAMS2:
            acc += dist[idx]
            if acc >= dist_amt:
                acc = 0
                sparse_idxs.append(idx - 1)
            idx += 1
            if idx == self.NUM_LIDAR_BEAMS - 1:
                sparse_idxs.append(self.NUM_LIDAR_BEAMS - 1)
                break

        mirrored_half = []
        for idx in sparse_idxs[1:]:
            new_idx = 2 * sparse_idxs[0] - idx
            mirrored_half.insert(0, new_idx)
        sparse_idxs = mirrored_half + sparse_idxs

        self.LIDAR_SAMPLE_IDXS = np.array(sparse_idxs)
        self.LIDAR_THETA_LUT = beam_angles[self.LIDAR_SAMPLE_IDXS].astype(np.single)

    # -----------------------------------------------------------------------
    # Particle initialization
    # -----------------------------------------------------------------------

    def initialize_global(self):
        """Spread particles over the permissible region (lost-robot mode)."""
        while self.state_lock.locked():
            self.get_logger().info(
                'PF2 Global Initialization: Waiting for state lock', once=True)
            sleep(0.1)
        self.state_lock.acquire()
        self.get_logger().info('Lost Robot Initialization')

        permissible_x, permissible_y = np.where(self.permissible_region == 1)
        indices = np.random.randint(0, len(permissible_x), size=self.MAX_PARTICLES)

        permissible_states = np.zeros((self.MAX_PARTICLES, 3))
        permissible_states[:, 0] = permissible_y[indices]
        permissible_states[:, 1] = permissible_x[indices]
        permissible_states[:, 2] = np.random.random(self.MAX_PARTICLES) * np.pi * 2.0

        Utils.map_to_world(permissible_states, self.map_info)
        self.particles = permissible_states
        self.weights[:] = 1.0 / self.MAX_PARTICLES
        self.state_lock.release()

    def initialize_particles_pose(self, posex: float, posey: float,
                                  posetheta: Optional[float] = None,
                                  poseo=None):
        """Initialize particles around a given pose."""
        assert not (posetheta is None and poseo is None)
        assert not (posetheta is not None and poseo is not None)

        while self.state_lock.locked():
            self.get_logger().info(
                'PF2 Pose Initialization: Waiting for state lock', once=True)
            sleep(0.1)
        self.state_lock.acquire()

        if poseo is not None:
            posetheta = Utils.quaternion_to_angle(poseo)

        self.get_logger().info(
            f'Setting initial pose: x={posex:.2f} y={posey:.2f} '
            f'theta={np.degrees(posetheta):.2f}deg')
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)
        self.particles[:, 0] = posex + np.random.normal(
            scale=self.INIT_VAR_X, size=self.MAX_PARTICLES)
        self.particles[:, 1] = posey + np.random.normal(
            scale=self.INIT_VAR_Y, size=self.MAX_PARTICLES)
        self.particles[:, 2] = posetheta + np.random.normal(
            scale=self.INIT_VAR_TH, size=self.MAX_PARTICLES)
        self.state_lock.release()

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def dyn_param_cb(self, params):
        """Replace dynamic_reconfigure: only alpha_1-4 and lam_thresh are live-tunable."""
        if self.MOTION_MODEL in ('tum', 'amcl'):
            for p in params:
                if p.name == 'alpha_1':
                    self.ALPHA_1 = float(p.value)
                elif p.name == 'alpha_2':
                    self.ALPHA_2 = float(p.value)
                elif p.name == 'alpha_3':
                    self.ALPHA_3 = float(p.value)
                elif p.name == 'alpha_4':
                    self.ALPHA_4 = float(p.value)
                elif p.name == 'lam_thresh' and self.MOTION_MODEL == 'tum':
                    self.LAM_THRESH = float(p.value)
            self.get_logger().info(
                f'PF2 param update: a1={self.ALPHA_1} a2={self.ALPHA_2} '
                f'a3={self.ALPHA_3} a4={self.ALPHA_4}')
        return SetParametersResult(successful=True)

    def lidarCB(self, msg: LaserScan):
        self.downsampled_ranges = np.array(msg.ranges)[self.LIDAR_SAMPLE_IDXS]
        self.lidar_initialized = True

    def odomCB(self, msg: Odometry):
        self.odom_msgs.append(msg)
        self.last_odom_msg = msg

    def clicked_poseCB(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        self.initialize_particles_pose(x, y, poseo=o)

    def clicked_pointCB(self, msg: PointStamped):
        self.get_logger().info('Received PointStamped — re-initializing globally')
        self.initialize_global()

    # -----------------------------------------------------------------------
    # Visualization + publishing
    # -----------------------------------------------------------------------

    def publish_tf(self, pose: np.ndarray, stamp: rclpy.time.Time):
        """Publish TF map→base_link and /tracked_pose. Mirrors ROS1 publish_tf."""
        if stamp.nanoseconds <= self.last_pub_stamp.nanoseconds:
            return

        map_base_link_pos = pose[0:2]
        yaw = pose[2]
        # Inline quaternion from yaw (replaces tft.quaternion_from_euler)
        qx, qy, qz, qw = 0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)

        stamp_msg = stamp.to_msg()
        header = Utils.make_header('map', stamp_msg)

        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.pose.position.x = float(map_base_link_pos[0])
        pose_msg.pose.position.y = float(map_base_link_pos[1])
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)

        if self.PUB_COVARIANCE:
            pose_cov_msg = PoseWithCovarianceStamped()
            pose_cov_msg.pose.pose = pose_msg.pose
            pose_cov_msg.header = header
            covariance = np.zeros((6, 6))
            covariance[0:2, 0:2] = self.cov[0:2, 0:2]
            covariance[5, 5]     = self.cov[2, 2]
            covariance[0:2, 5]   = self.cov[0:2, 2]
            covariance[5, 0:2]   = self.cov[0:2, 2]
            pose_cov_msg.pose.covariance = covariance.flatten().tolist()
            self.pose_cov_pub.publish(pose_cov_msg)

        if self.PUBLISH_TF:
            t = TransformStamped()
            t.header = header
            t.child_frame_id = self.BASE_FRAME_ID
            t.transform.translation.x = float(map_base_link_pos[0])
            t.transform.translation.y = float(map_base_link_pos[1])
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.pub_tf.sendTransform(t)

        self.last_pub_stamp = stamp

    def visualise(self):
        if not self.DO_VIZ:
            return

        def _publish_particles(particles):
            pa = PoseArray()
            pa.header = Utils.make_header('map', self.get_clock().now().to_msg())
            pa.poses = self.particle_utils.particles_to_poses(particles)
            self.particle_pub.publish(pa)

        # get_subscription_count() replaces ROS1 get_num_connections()
        if self.particle_pub.get_subscription_count() > 0:
            if self.MAX_PARTICLES > self.MAX_VIZ_PARTICLES:
                proposal_indices = np.random.choice(
                    self.particle_indices, self.MAX_VIZ_PARTICLES, p=self.weights)
                _publish_particles(self.particles[proposal_indices, :])
            else:
                _publish_particles(self.particles)

    # -----------------------------------------------------------------------
    # Motion model  (verbatim from ROS1 — pure numpy, no ROS API)
    # -----------------------------------------------------------------------

    def motion_model(self, proposal_dist: np.ndarray, action: np.ndarray):
        if self.DEBUG:
            tic = time()

        if self.MOTION_MODEL in ('amcl', 'tum'):
            a1 = self.ALPHA_1
            a2 = self.ALPHA_2
            a3 = self.ALPHA_3
            a4 = self.ALPHA_4

            dx     = self.curr_pose[0] - self.last_pose[0]
            dy     = self.curr_pose[1] - self.last_pose[1]
            dtheta = Utils.angle_diff(self.curr_pose[2], self.last_pose[2])
            d_trans = np.sqrt(dx**2 + dy**2)

            if d_trans < 0.01:
                return

            d_rot1 = (Utils.angle_diff(np.arctan2(dy, dx), self.last_pose[2])
                      if d_trans > 0.01 else 0.0)

            reverse_offset = 0.0
            reverse_spread = 1.0
            if len(self.odom_msgs) and self.odom_msgs[-1].twist.twist.linear.x < -0.05:
                reverse_offset = np.pi
                reverse_spread = 1.1
                d_rot1 += np.pi if d_rot1 < -np.pi / 2 else -np.pi

            d_rot2 = Utils.angle_diff(dtheta, d_rot1)

            d_rot1 = min(abs(Utils.angle_diff(d_rot1, 0.0)),
                         abs(Utils.angle_diff(d_rot1, np.pi)))
            d_rot2 = min(abs(Utils.angle_diff(d_rot2, 0.0)),
                         abs(Utils.angle_diff(d_rot2, np.pi)))

            if self.MOTION_MODEL == 'amcl':
                scale_rot1 = a1 * d_rot1 + a2 * d_trans
                scale_rot2 = a1 * d_rot2 + a2 * d_trans
            else:  # tum
                scale_rot1 = a1 * d_rot1 + a2 / max(d_trans, self.LAM_THRESH)
                scale_rot2 = a1 * d_rot2 + a2 / max(d_trans, self.LAM_THRESH)

            scale_trans = a3 * d_trans + a4 * (d_rot1 + d_rot2)

            scale_rot1 *= reverse_spread
            scale_rot2 *= reverse_spread
            scale_trans *= reverse_spread

            d_rot1  += np.random.normal(scale=scale_rot1, size=self.MAX_PARTICLES)
            d_trans += np.random.normal(loc=scale_trans / 2, scale=scale_trans,
                                        size=self.MAX_PARTICLES)
            d_rot2  += np.random.normal(scale=scale_rot2, size=self.MAX_PARTICLES)

            eff_hdg = proposal_dist[:, 2] + d_rot1 + reverse_offset
            proposal_dist[:, 0] += d_trans * np.cos(eff_hdg)
            proposal_dist[:, 1] += d_trans * np.sin(eff_hdg)
            proposal_dist[:, 2] += d_rot1 + d_rot2

        elif self.MOTION_MODEL == 'arc':
            dx_, dy_, dtheta_ = action
            scale_x = max(self.MOTION_DISPERSION_ARC_X_MIN,
                          abs(dx_) * self.MOTION_DISPERSION_ARC_X)
            scale_y = abs(dy_) * self.MOTION_DISPERSION_ARC_Y
            if abs(dx_) > self.MOTION_DISPERSION_ARC_XY_MIN_X:
                scale_y += (abs(dx_) - self.MOTION_DISPERSION_ARC_XY_MIN_X) * \
                            self.MOTION_DISPERSION_ARC_XY
            scale_y = min(self.MOTION_DISPERSION_ARC_Y_MAX,
                          max(self.MOTION_DISPERSION_ARC_Y_MIN, scale_y))
            scale_th = max(self.MOTION_DISPERSION_ARC_THETA_MIN,
                           abs(dtheta_) * self.MOTION_DISPERSION_ARC_THETA)

            dx     = np.random.normal(loc=dx_,     scale=scale_x,  size=self.MAX_PARTICLES)
            dy     = np.random.normal(loc=dy_,     scale=scale_y,  size=self.MAX_PARTICLES)
            dtheta = np.random.normal(loc=dtheta_, scale=scale_th, size=self.MAX_PARTICLES)

            r = dx / dtheta
            angle = (np.pi / 2) * (np.sign(dtheta) + (dtheta == 0))
            cx = proposal_dist[:, 0] + r * np.cos(proposal_dist[:, 2] + angle)
            cy = proposal_dist[:, 1] + r * np.sin(proposal_dist[:, 2] + angle)
            psi = np.arctan2(proposal_dist[:, 1] - cy, proposal_dist[:, 0] - cx)
            x_ = cx + r * np.cos(psi + dtheta)
            y_ = cy + r * np.sin(psi + dtheta)
            theta_ = proposal_dist[:, 2] + dtheta
            x_ += dy * np.cos(theta_ + np.pi / 2)
            y_ += dy * np.sin(theta_ + np.pi / 2)
            proposal_dist[:, 0] = x_
            proposal_dist[:, 1] = y_
            proposal_dist[:, 2] = theta_

        else:  # default (MIT)
            dx, dy, dtheta = action
            cosines = np.cos(proposal_dist[:, 2])
            sines   = np.sin(proposal_dist[:, 2])
            self.local_deltas[:, 0] = cosines * dx - sines * dy
            self.local_deltas[:, 1] = sines   * dx + cosines * dy
            self.local_deltas[:, 2] = dtheta
            proposal_dist[:, :] += self.local_deltas
            proposal_dist[:, 0] += np.random.normal(
                loc=0.0, scale=self.MOTION_DISPERSION_X, size=self.MAX_PARTICLES)
            proposal_dist[:, 1] += np.random.normal(
                loc=0.0, scale=self.MOTION_DISPERSION_Y, size=self.MAX_PARTICLES)
            proposal_dist[:, 2] += np.random.normal(
                loc=0.0, scale=self.MOTION_DISPERSION_THETA, size=self.MAX_PARTICLES)

        # Clamp angles to (-pi, pi)
        proposal_s = np.sin(proposal_dist[:, 2])
        proposal_c = np.cos(proposal_dist[:, 2])
        proposal_dist[:, 2] = np.arctan2(proposal_s, proposal_c)

        if self.DEBUG:
            self.motion_model_time_ms.append((time() - tic) * 1000)

    # -----------------------------------------------------------------------
    # Sensor model  (verbatim from ROS1 — range_libc calls unchanged)
    # -----------------------------------------------------------------------

    def sensor_model(self, proposal_dist: np.ndarray,
                     obs: np.ndarray, weights: np.ndarray):
        if self.DEBUG:
            tic = time()

        num_rays = self.DES_LIDAR_BEAMS

        if self.first_sensor_update:
            if self.RANGELIB_VAR in (RangeLibVariant.VAR_NO_EVAL_SENSOR_MODEL,
                                     RangeLibVariant.VAR_CALC_RANGE_MANY_EVAL_SENSOR):
                self.get_logger().warn(
                    'VAR_NO_EVAL / VAR_CALC_RANGE_MANY: proposal_dist not '
                    'laser-frame transformed. Performance will be worse.')
                self.queries = np.zeros(
                    (num_rays * self.MAX_PARTICLES, 3), dtype=np.float32)
            else:
                self.queries = np.zeros(
                    (self.MAX_PARTICLES, 3), dtype=np.float32)

            self.ranges = np.zeros(
                num_rays * self.MAX_PARTICLES, dtype=np.float32)
            self.tiled_angles = np.tile(self.LIDAR_THETA_LUT, self.MAX_PARTICLES)
            self.first_sensor_update = False

        # Transform particles from base_link frame to laser frame
        proposal_s = np.sin(proposal_dist[:, 2])
        proposal_c = np.cos(proposal_dist[:, 2])
        rot = np.array([[proposal_c, -proposal_s],
                        [proposal_s,  proposal_c]]).transpose(2, 0, 1)  # (N,2,2)
        laser_offset_2d = self.laser_base_link_offset[:2]
        res = (rot @ laser_offset_2d[np.newaxis, :, np.newaxis]).reshape(
            self.MAX_PARTICLES, 2)

        self.queries[:, :] = proposal_dist
        self.queries[:, :2] += res

        if self.RANGELIB_VAR == RangeLibVariant.VAR_RADIAL_CDDT_OPTIMIZATIONS:
            if 'cddt' in self.WHICH_RM:
                self.range_method.calc_range_many_radial_optimized(
                    num_rays, self.LIDAR_THETA_LUT[0], self.LIDAR_THETA_LUT[-1],
                    self.queries, self.ranges)
                self.range_method.eval_sensor_model(
                    obs, self.ranges, self.weights, num_rays, self.MAX_PARTICLES)
                self.weights = np.power(self.weights, self.INV_SQUASH_FACTOR)
            else:
                raise ValueError(
                    'Radial CDDT optimizations require cddt or pcddt range method')

        elif self.RANGELIB_VAR == RangeLibVariant.VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT:
            self.range_method.calc_range_repeat_angles_eval_sensor_model(
                self.queries, self.LIDAR_THETA_LUT, obs, self.weights)
            np.power(self.weights, self.INV_SQUASH_FACTOR, self.weights)

        elif self.RANGELIB_VAR == RangeLibVariant.VAR_REPEAT_ANGLES_EVAL_SENSOR:
            self.range_method.calc_range_repeat_angles(
                self.queries, self.LIDAR_THETA_LUT, self.ranges)
            self.range_method.eval_sensor_model(
                obs, self.ranges, self.weights, num_rays, self.MAX_PARTICLES)
            np.power(self.weights, self.INV_SQUASH_FACTOR, self.weights)

        elif self.RANGELIB_VAR == RangeLibVariant.VAR_CALC_RANGE_MANY_EVAL_SENSOR:
            self.queries[:, 0] = np.repeat(proposal_dist[:, 0], num_rays)
            self.queries[:, 1] = np.repeat(proposal_dist[:, 1], num_rays)
            self.queries[:, 2] = np.repeat(proposal_dist[:, 2], num_rays)
            self.queries[:, 2] += self.tiled_angles
            self.range_method.calc_range_many(self.queries, self.ranges)
            self.range_method.eval_sensor_model(
                obs, self.ranges, self.weights, num_rays, self.MAX_PARTICLES)
            np.power(self.weights, self.INV_SQUASH_FACTOR, self.weights)

        elif self.RANGELIB_VAR == RangeLibVariant.VAR_NO_EVAL_SENSOR_MODEL:
            self.queries[:, 0] = np.repeat(proposal_dist[:, 0], num_rays)
            self.queries[:, 1] = np.repeat(proposal_dist[:, 1], num_rays)
            self.queries[:, 2] = np.repeat(proposal_dist[:, 2], num_rays)
            self.queries[:, 2] += self.tiled_angles
            self.range_method.calc_range_many(self.queries, self.ranges)

            obs_px    = obs / float(self.map_info.resolution)
            ranges_px = self.ranges / float(self.map_info.resolution)
            obs_px[obs_px > self.MAX_RANGE_PX] = self.MAX_RANGE_PX
            ranges_px[ranges_px > self.MAX_RANGE_PX] = self.MAX_RANGE_PX

            intobs = np.rint(obs_px).astype(np.uint16)
            intrng = np.rint(ranges_px).astype(np.uint16)

            for i in range(self.MAX_PARTICLES):
                weight = np.product(
                    self.sensor_model_table[
                        intobs, intrng[i * num_rays:(i + 1) * num_rays]])
                weights[i] = np.power(weight, self.INV_SQUASH_FACTOR)
        else:
            raise ValueError(
                f'Invalid rangelib_variant: {self.RANGELIB_VAR}')

        if self.DEBUG:
            self.sensor_model_time_ms.append((time() - tic) * 1000)

    # -----------------------------------------------------------------------
    # MCL step  (verbatim from ROS1)
    # -----------------------------------------------------------------------

    def MCL(self, odom_data: np.ndarray, observations: np.ndarray):
        proposal_indices = np.random.choice(
            self.particle_indices, self.MAX_PARTICLES, p=self.weights)
        proposal_distribution = self.particles[proposal_indices, :]

        self.motion_model(proposal_distribution, odom_data)
        self.sensor_model(proposal_distribution, observations, self.weights)

        # Out-of-bounds particles: weight *= 0.01 (not zeroed — matches ROS1)
        particles_in_map = np.copy(proposal_distribution)
        Utils.world_to_map(particles_in_map, self.map_info)
        limit = self.permissible_region.shape
        particles_in_map = np.clip(
            particles_in_map[:, 0:2].astype('int'),
            [0, 0], [limit[1] - 1, limit[0] - 1])
        valid_particles = self.permissible_region[
            particles_in_map[:, 1], particles_in_map[:, 0]]
        self.weights = np.where(valid_particles, self.weights, 0.01 * self.weights)

        weight_sum = np.sum(self.weights)
        self.weights /= weight_sum

        self.particles = proposal_distribution

        # Covariance computation
        if self.inferred_pose is not None and self.PUB_COVARIANCE:
            spread = self.particles - self.inferred_pose
            spread[spread[:, 2] >  np.pi, 2] -= 2 * np.pi
            spread[spread[:, 2] < -np.pi, 2] += 2 * np.pi
            spread = spread[:, :, np.newaxis]
            inner_prod = spread @ spread.transpose(0, 2, 1)
            res = self.weights[:, np.newaxis, np.newaxis] * inner_prod
            self.cov = np.sum(res, axis=0)

    # -----------------------------------------------------------------------
    # Update step
    # -----------------------------------------------------------------------

    def update(self):
        if not (self.lidar_initialized and self.odom_initialized and
                self.map_initialized):
            return

        if self.state_lock.locked():
            self.get_logger().info('Concurrency error avoided')
            return

        self.state_lock.acquire()
        observation = np.copy(self.downsampled_ranges).astype(np.float32)
        self.MCL(self.odometry_data, observation)
        self.odometry_data = np.zeros(3)

        inferred_x = np.sum(self.particles[:, 0] * self.weights)
        inferred_y = np.sum(self.particles[:, 1] * self.weights)
        inferred_s = np.sum(np.sin(self.particles[:, 2]) * self.weights)
        inferred_c = np.sum(np.cos(self.particles[:, 2]) * self.weights)
        self.inferred_pose = np.array(
            (inferred_x, inferred_y, np.arctan2(inferred_s, inferred_c)))
        self.state_lock.release()

        self.publish_tf(self.inferred_pose, self.last_stamp)
        self.visualise()

    # -----------------------------------------------------------------------
    # Main loop  (replaces rospy.Rate + while not rospy.is_shutdown())
    # -----------------------------------------------------------------------

    def loop(self):
        """200 Hz processing loop — runs in a dedicated thread."""
        rate_sec = 1.0 / 200.0

        while rclpy.ok():
            if self.DEBUG:
                self.profiler.enable()
                tic = time()

            if len(self.odom_msgs) == 0:
                sleep(rate_sec)
                continue

            # Drain queue and use only the newest message to avoid lag when
            # sim publishes faster than realtime (queue fills, oldest processed).
            while len(self.odom_msgs) > 1:
                self.odom_msgs.popleft()
            msg: Odometry = self.odom_msgs[0]
            position = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y])
            orientation = Utils.quaternion_to_angle(msg.pose.pose.orientation)
            self.curr_pose = np.array([position[0], position[1], orientation])

            self.update()

            if isinstance(self.last_pose, np.ndarray):
                rot = Utils.rotation_matrix(-self.last_pose[2])
                delta = np.array([position - self.last_pose[0:2]]).transpose()
                local_delta = (rot * delta).transpose()
                self.odometry_data = np.array(
                    [local_delta[0, 0], local_delta[0, 1],
                     orientation - self.last_pose[2]])
                self.odom_initialized = True
            else:
                self.get_logger().info('PF2...Received first Odometry message')

            self.last_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            self.last_pose = self.curr_pose
            self.odom_msgs.popleft()

            sleep(rate_sec)

            if self.DEBUG:
                self.profiler.disable()
                self.overall_time_ms.append((time() - tic) * 1000)

                if (self.itr % 30) == 0 and len(self.sensor_model_time_ms) > 0:
                    s_mean = np.mean(self.sensor_model_time_ms)
                    s_std  = np.std(self.sensor_model_time_ms)
                    m_mean = np.mean(self.motion_model_time_ms)
                    m_std  = np.std(self.motion_model_time_ms)
                    o_mean = np.mean(self.overall_time_ms)
                    o_std  = np.std(self.overall_time_ms)
                    self.get_logger().info(
                        f'Sensor: {s_mean:.2f}ms±{s_std:.2f} | '
                        f'Motion: {m_mean:.2f}ms±{m_std:.2f} | '
                        f'Overall: {o_mean:.2f}ms±{o_std:.2f}')

                if (self.itr % 500) == 0:
                    stats = pstats.Stats(self.profiler)
                    stats.sort_stats(pstats.SortKey.TIME)
                    stats.dump_stats(filename='synpf_stats.prof')
                    self.get_logger().warn('SynPF: dumped profiling stats to synpf_stats.prof')

                self.itr += 1


def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilter()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(pf)

    import threading
    loop_thread = threading.Thread(target=pf.loop, daemon=True)
    loop_thread.start()

    try:
        executor.spin()
    finally:
        executor.shutdown()
        pf.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
