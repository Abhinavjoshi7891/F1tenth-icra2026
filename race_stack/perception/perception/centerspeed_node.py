#!/usr/bin/env python3
"""
CenterSpeed ROS 2 Node — Phase 2 Port
======================================
A faithful port of the ROS 1 CenterSpeed_inference.py to ROS 2 rclpy.

Key design decisions:
  - State-gate pattern (self.converter = None) mirrors detect.py exactly.
  - TF2 double-lookup (laser → base_link → map) matches detect.py.
  - pycoral / tflite_runtime wrapped in try/except so the node can run
    on a standard PC (CPU PyTorch) without the Coral Edge TPU hardware.
  - No dynamic_reconfigure: parameters declared via rclpy parameters at startup.
  - Model path resolved relative to the 'resource/' directory of this package.
"""

import os
import sys
import time
import numpy as np
from bisect import bisect_left
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

import torch
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, HistoryPolicy

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

from frenet_conversion.frenet_converter import FrenetConverter
from f110_msgs.msg import WpntArray, ObstacleArray
from f110_msgs.msg import Obstacle as ObstacleMessage
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

# -----------------------------------------------------------------------
# Hardware acceleration imports (Coral Edge TPU / TFLite).
# Wrapped in try/except so the node compiles and runs without the TPU.
# -----------------------------------------------------------------------
try:
    from pycoral.utils.edgetpu import make_interpreter, list_edge_tpus
    from pycoral.adapters.common import input_size
    CORAL_AVAILABLE = True
except ImportError:
    CORAL_AVAILABLE = False

try:
    from tflite_runtime.interpreter import Interpreter, load_delegate
    TFLITE_AVAILABLE = True
except ImportError:
    TFLITE_AVAILABLE = False

# -----------------------------------------------------------------------
# Model definitions (copied from ROS 1 TinyCenterSpeed/src/models/)
# -----------------------------------------------------------------------
from perception.centerspeed_models.CenterSpeed import (
    CenterSpeedModular,
    CenterSpeedDenseResidual,
)


# -----------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------
def normalize_s(x, track_length):
    x = x % track_length
    if x > track_length / 2:
        x -= track_length
    return x


class Obstacle:
    """Internal obstacle data structure (same as ROS 1)."""
    def __init__(self, x, y, vx, vy, size, theta, id=None):
        self.center_x = x
        self.center_y = y
        self.vx = vx
        self.vy = vy
        self.size = size
        self.id = id
        self.theta = theta

    def squaredDist(self, other):
        return (self.center_x - other.center_x) ** 2 + \
               (self.center_y - other.center_y) ** 2


# -----------------------------------------------------------------------
# ROS 2 Node
# -----------------------------------------------------------------------
class CenterSpeedNode(Node):
    """
    ROS 2 port of the TinyCenterSpeed inference node.

    Subscribes:
        /scan                (sensor_msgs/LaserScan)
        /global_waypoints    (f110_msgs/WpntArray)
        /car_state/odom_frenet (nav_msgs/Odometry)

    Publishes:
        /perception/detection/raw_obstacles   (f110_msgs/ObstacleArray)  — if tracking_enabled
        /perception/obstacles                 (f110_msgs/ObstacleArray)  — if tracking disabled
        /perception/obstacles_markers_cs      (visualization_msgs/MarkerArray)
        /perception/centerspeed_image         (sensor_msgs/Image)        — if publish_image
        /perception/detection/cs_latency      (std_msgs/Float32)         — if measuring
    """

    def __init__(self):
        super().__init__('centerspeed')

        # ---- Declare ROS 2 parameters (replaces rospy.get_param) ----
        self.declare_parameter('measuring', False)
        self.declare_parameter('tracking_enabled', False)
        self.declare_parameter('using_raceline', True)
        self.declare_parameter('visualize', False)
        self.declare_parameter('rate', 40.0)
        self.declare_parameter('pixel_size', 0.12)
        self.declare_parameter('image_size', 64)
        self.declare_parameter('feature_size', 3)
        self.declare_parameter('num_opponents', 1)
        self.declare_parameter('boundary_inflation', 0.3)
        self.declare_parameter('using_coral', False)
        self.declare_parameter('dense', True)
        self.declare_parameter('quantize', False)
        self.declare_parameter('publish_cartesian', False)
        self.declare_parameter('publish_foxglove', False)
        self.declare_parameter('detection_threshold', 0.2)
        self.declare_parameter('max_viewing_distance', 9.0)

        # ---- Read parameters ----
        self.measuring           = self.get_parameter('measuring').value
        self.tracking_enabled    = self.get_parameter('tracking_enabled').value
        self.using_raceline      = self.get_parameter('using_raceline').value
        self.visualize           = self.get_parameter('visualize').value
        self.rate_hz             = self.get_parameter('rate').value
        self.pixelsize           = self.get_parameter('pixel_size').value
        self.image_size          = self.get_parameter('image_size').value
        self.feature_size        = self.get_parameter('feature_size').value
        self.num_opponents       = self.get_parameter('num_opponents').value
        self.boundaries_inflation = self.get_parameter('boundary_inflation').value
        self.using_coral         = self.get_parameter('using_coral').value
        self.dense               = self.get_parameter('dense').value
        self.quantize            = self.get_parameter('quantize').value
        self.use_cartesian       = self.get_parameter('publish_cartesian').value
        self.publish_image_flag  = self.get_parameter('publish_foxglove').value
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.max_viewing_distance = self.get_parameter('max_viewing_distance').value

        # ---- Coral availability check ----
        if self.using_coral and not CORAL_AVAILABLE:
            self.get_logger().warn(
                '[CenterSpeed]: using_coral=True but pycoral is NOT installed. '
                'Falling back to CPU PyTorch.')
            self.using_coral = False

        # ---- Derived constants ----
        self.origin_offset      = (self.image_size // 2) * self.pixelsize
        self.distance_threshold = self.pixelsize * self.image_size // 2 + 0.5

        # ---- State gates (mirrors detect.py) ----
        self.converter      = None   # Set once in pathCb
        self.waypoints      = None
        self.psi_array      = None
        self.track_length   = None
        self.s_array        = None
        self.d_right_array  = None
        self.d_left_array   = None
        self.biggest_d      = None
        self.smallest_d     = None
        self.path_needs_update = False

        # ---- Ego state ----
        self.ego_s   = None
        self.ego_yaw = None

        # ---- LiDAR buffers ----
        self.scan1       = None
        self.scan2       = None
        self.angles      = None
        self.cos_angles  = None
        self.sin_angles  = None
        self.frame1      = None
        self.frame2      = None

        # ---- Obstacle tracking buffer ----
        self.tracked_obstacles = []
        self.current_stamp     = None

        # ---- TF2 (mirrors exact detect.py pattern) ----
        self.tf_buffer   = Buffer(rclpy.duration.Duration(seconds=5))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # ---- Publishers ----
        if self.tracking_enabled:
            self.obs_pub = self.create_publisher(
                ObstacleArray, '/perception/detection/raw_obstacles', 5)
        else:
            self.obs_pub = self.create_publisher(
                ObstacleArray, '/perception/obstacles', 5)

        self.marker_pub = self.create_publisher(
            MarkerArray, '/perception/obstacles_markers_cs', 5)
        self.vel_marker_pub = self.create_publisher(
            MarkerArray, '/perception/velocity_marker_cs', 5)

        if self.measuring:
            self.latency_pub   = self.create_publisher(
                Float32, '/perception/detection/cs_latency', 5)
            self.v_cart_pub    = self.create_publisher(
                Float32MultiArray, '/perception/obstacle_v_cartesian', 5)

        if self.publish_image_flag:
            self.image_pub = self.create_publisher(
                Image, '/perception/centerspeed_image', 5)

        # ---- Subscribers ----
        self.create_subscription(LaserScan,  '/scan',                  self.laser_cb,     10)
        self.create_subscription(WpntArray,  '/global_waypoints',      self.path_cb,      10)
        self.create_subscription(Odometry, '/car_state/odom_frenet',   self.car_state_cb, 10)

        # ---- Main inference timer ----
        # Timer is created but will exit early via state gates until ready.
        self._model_loaded = False
        self._initial_frame = True
        self.timer = self.create_timer(1.0 / self.rate_hz, self._pipeline)

        self.get_logger().info('[CenterSpeed]: Node initialized. Waiting for waypoints and scans...')

    # ================================================================== #
    # Callbacks                                                           #
    # ================================================================== #

    def laser_cb(self, msg: LaserScan):
        """Store last two scans and precompute angle trig (once)."""
        if self.angles is None:
            self.angles     = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            self.cos_angles = np.cos(self.angles)
            self.sin_angles = np.sin(self.angles)

        if self.scan1 is None:
            self.scan1 = msg
        else:
            self.scan2 = self.scan1
            self.scan1 = msg

    def path_cb(self, data: WpntArray):
        """State-gate: initialize FrenetConverter once, then update boundary arrays."""
        # --- One-time converter init (mirrors detect.py pathCb) ---
        if self.converter is None:
            self.waypoints = np.array(
                [[wpnt.x_m, wpnt.y_m] for wpnt in data.wpnts])
            self.psi_array = np.array([wpnt.psi_rad for wpnt in data.wpnts])
            self.converter = FrenetConverter(
                self.waypoints[:, 0], self.waypoints[:, 1], self.psi_array)
            self.get_logger().info('[CenterSpeed]: FrenetConverter initialized.')

        # --- Boundary arrays (recalculated if track updates) ---
        if (self.s_array is None or self.path_needs_update) and self.converter is not None:
            self.s_array       = []
            self.d_right_array = []
            self.d_left_array  = []
            for wpnt in data.wpnts:
                self.s_array.append(wpnt.s_m)
                self.d_right_array.append(wpnt.d_right - self.boundaries_inflation)
                self.d_left_array.append(wpnt.d_left  - self.boundaries_inflation)

            self.smallest_d  = min(self.d_right_array + self.d_left_array)
            self.biggest_d   = max(self.d_right_array + self.d_left_array)
            self.track_length = data.wpnts[-1].s_m
            self.path_needs_update = False
            self.get_logger().info(
                f'[CenterSpeed]: Track boundaries ready. '
                f'Length={self.track_length:.1f}m')

    def car_state_cb(self, data: Odometry):
        """Store ego s-position and yaw from Frenet odometry."""
        self.ego_s = data.pose.pose.position.x
        q = data.pose.pose.orientation
        self.ego_yaw = Rotation.from_quat(
            [q.x, q.y, q.z, q.w]).as_euler('xyz')[2]

    # ================================================================== #
    # Core pipeline                                                       #
    # ================================================================== #

    def _pipeline(self):
        """Main timer callback. Exits early until all state gates are satisfied."""
        # State gate: wait for converter and two scans
        if self.converter is None or self.track_length is None:
            self.get_logger().info(
                '[CenterSpeed]: Waiting for waypoints...', throttle_duration_sec=2.0)
            return
        if self.scan1 is None or self.scan2 is None:
            self.get_logger().info(
                '[CenterSpeed]: Waiting for two consecutive LiDAR scans...',
                throttle_duration_sec=2.0)
            return

        # Load model once (deferred to ensure all ROS setup is complete)
        if not self._model_loaded:
            version = 2 if self.dense else 1
            self._load_model(version=version)
            self._model_loaded = True
            self.get_logger().info('[CenterSpeed]: Model loaded. Starting inference.')

        if self.measuring:
            t0 = time.perf_counter()

        self.current_stamp = self.get_clock().now().to_msg()

        # Build the two-frame input tensor
        if self._initial_frame:
            self.frame1 = self._preprocess(self.scan1)
            self.frame2 = self._preprocess(self.scan2)
            self._initial_frame = False
        else:
            self.frame1 = self.frame2
            self.frame2 = self._preprocess(self.scan1)

        # Run the correct version
        version = 2 if (self.dense or self.using_coral) else 1
        with torch.no_grad():
            if version == 2:
                self._main_v2()
            else:
                self._main_v1()

        if self.measuring:
            latency = time.perf_counter() - t0
            msg = Float32()
            msg.data = float(latency)
            self.latency_pub.publish(msg)

    # ================================================================== #
    # Inference pipelines (v1 / v2)                                      #
    # ================================================================== #

    def _main_v1(self):
        """CenterSpeed 1.0 pipeline."""
        preprocessed = torch.cat([self.frame1, self.frame2], dim=0)
        output_hm, output_data = self._inference(preprocessed, version=1)
        output_data = output_data.squeeze(0)
        output_hm   = output_hm.squeeze(0).squeeze(0)

        x, y = self._find_k_peaks(output_hm, self.num_opponents, 0.8)
        if x is not None:
            for xi, yi in zip(x, y):
                if np.sqrt(xi**2 + yi**2) > self.distance_threshold:
                    continue
                x_g, y_g = self._lidar_to_global(xi, yi)
                if x_g is None:
                    continue
                self.tracked_obstacles.append(
                    Obstacle(x=x_g, y=y_g,
                             vx=float(output_data[0]),
                             vy=float(-output_data[1]),
                             size=0.5,
                             theta=float(output_data[2]),
                             id=0))

        self._publish_obstacles()
        self._publish_markers()
        self.tracked_obstacles.clear()

    def _main_v2(self):
        """CenterSpeed 2.0 (Dense Residual) pipeline."""
        preprocessed = torch.cat([self.frame1, self.frame2], dim=0)
        output = self._inference(preprocessed, version=2)

        if not self.using_coral:
            output = output.squeeze(0).squeeze(0) if output.dim() == 4 else output.squeeze(0)
        else:
            output = output.squeeze(0)
            output = torch.permute(output, (2, 0, 1))

        y_ind, x_ind = self._find_k_peaks(output[0], self.num_opponents, 0.5,
                                            return_cartesian=False)
        if x_ind is not None:
            x_cart, y_cart = self._index_to_cartesian(y_ind, x_ind)
            for i, (xi, yi) in enumerate(zip(x_cart, y_cart)):
                if np.sqrt(xi**2 + yi**2) > self.distance_threshold:
                    continue
                x_g, y_g = self._lidar_to_global(xi, yi)
                if x_g is None:
                    continue
                vx    = float(output[1, int(x_ind[i]), int(y_ind[i])])
                vy    = float(-output[2, int(x_ind[i]), int(y_ind[i])])
                theta = float(output[3, int(x_ind[i]), int(y_ind[i])])
                self.tracked_obstacles.append(
                    Obstacle(x=x_g, y=y_g, vx=vx, vy=vy, size=0.5, theta=theta, id=0))

        self._publish_obstacles()
        self._publish_markers()

        if self.publish_image_flag:
            self._publish_image(output[0])

        self.tracked_obstacles.clear()

    # ================================================================== #
    # Model management                                                    #
    # ================================================================== #

    def _load_model(self, version: int = 2):
        """Load the PyTorch or Coral model."""
        pkg_share = get_package_share_directory('perception')
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        if not self.using_coral:
            if version == 1:
                model_path = os.path.join(pkg_share, 'resource', 'TinyCenterSpeed.pt')
                self.net = CenterSpeedModular(image_size=self.image_size)
            else:
                model_path = os.path.join(pkg_share, 'resource', 'TinyCenterSpeed.pt')
                self.net = CenterSpeedDenseResidual(image_size=self.image_size)

            self.get_logger().info(f'[CenterSpeed]: Loading model from {model_path}')
            self.net.load_state_dict(
                torch.load(model_path, map_location=self.device, weights_only=True))

            if self.quantize:
                self.net = torch.quantization.quantize_dynamic(
                    self.net, {torch.nn.Conv2d, torch.nn.Linear}, dtype=torch.qint8)
                self.get_logger().info('[CenterSpeed]: Model quantized (INT8).')

            self.net.eval()
            self.net.to(self.device)
            self.get_logger().info(
                f'[CenterSpeed]: PyTorch model ready on {self.device}.')
        else:
            # Coral Edge TPU path
            coral_path = os.path.join(base_path, 'tinycs_float32_edgetpu.tflite')
            self.get_logger().info(f'[CenterSpeed]: Loading Coral model from {coral_path}')
            self.interpreter = make_interpreter(coral_path)
            self.interpreter.allocate_tensors()
            self.get_logger().info('[CenterSpeed]: Coral model ready.')

    def _inference(self, data: torch.Tensor, version: int = 2):
        """Run inference on the given preprocessed tensor."""
        if not self.using_coral:
            data = data.unsqueeze(0).to(self.device)
            if version == 1:
                output_hm, output_data = self.net(data)
                return output_hm, output_data
            else:
                return self.net(data)
        else:
            # Coral: reformat tensor to NHWC
            input_details = self.interpreter.get_input_details()
            self.interpreter.set_tensor(
                input_details[0]['index'],
                data.unsqueeze(0).permute(0, 2, 3, 1).numpy())
            self.interpreter.invoke()
            output_details = self.interpreter.get_output_details()
            output_hm = self.interpreter.get_tensor(output_details[0]['index'])
            return torch.tensor(output_hm)

    # ================================================================== #
    # LiDAR preprocessing                                                #
    # ================================================================== #

    def _preprocess(self, scan: LaserScan) -> torch.Tensor:
        """Convert a LaserScan to a (feature_size, image_size, image_size) tensor."""
        lidar_data  = torch.tensor(scan.ranges,      dtype=torch.float32)
        intensities = torch.tensor(scan.intensities, dtype=torch.float32)

        # Normalize intensities (guard against empty or zero-range)
        if intensities.numel() > 0:
            i_min, i_max = intensities.min(), intensities.max()
            if i_max > i_min:
                intensities = (intensities - i_min) / (i_max - i_min)
            else:
                intensities = torch.zeros_like(intensities)
        else:
            # Fallback if intensities are missing from the LiDAR scan
            intensities = torch.zeros_like(lidar_data)

        grid = torch.zeros(
            (self.feature_size, self.image_size, self.image_size), dtype=torch.float32)

        cos_t = torch.tensor(self.cos_angles, dtype=torch.float32)
        sin_t = torch.tensor(self.sin_angles, dtype=torch.float32)

        x       = lidar_data * cos_t
        y       = lidar_data * sin_t
        x_coord = ((x + self.origin_offset) / self.pixelsize).to(torch.int)
        y_coord = ((y + self.origin_offset) / self.pixelsize).to(torch.int)

        valid = ((x_coord >= 0) & (x_coord < self.image_size) &
                 (y_coord >= 0) & (y_coord < self.image_size))
        xv, yv = x_coord[valid], y_coord[valid]

        grid[0, yv, xv]  = 1
        grid[1, yv, xv]  = torch.maximum(grid[1, yv, xv], intensities[valid])
        grid[2, yv, xv] += 1
        return grid

    # ================================================================== #
    # Peak detection and coordinate conversion                           #
    # ================================================================== #

    def _find_k_peaks(self, image: torch.Tensor, k: int, radius: float,
                      return_cartesian: bool = True):
        """Find k local maxima in the heatmap, suppressing neighbourhood."""
        radius_px = int(radius // self.pixelsize)
        image = image.clone()
        xs, ys = [], []

        for _ in range(k):
            max_val, max_idx = torch.max(image.view(-1), 0)
            if max_val < self.detection_threshold:
                break
            coords = np.unravel_index(max_idx.item(), image.shape)
            xs.append(coords[0])
            ys.append(coords[1])
            # Zero out the neighbourhood
            r0 = max(0, coords[0] - radius_px)
            r1 = min(image.shape[0], coords[0] + radius_px)
            c0 = max(0, coords[1] - radius_px)
            c1 = min(image.shape[1], coords[1] + radius_px)
            image[r0:r1, c0:c1] = 0

        if not xs:
            return None, None

        xs, ys = np.array(xs), np.array(ys)
        if return_cartesian:
            return self._index_to_cartesian(ys, xs)
        return ys, xs

    def _index_to_cartesian(self, x_img, y_img):
        """Image-space index → LiDAR-frame Cartesian (metres)."""
        x = x_img * self.pixelsize - self.origin_offset
        y = y_img * self.pixelsize - self.origin_offset
        return x, y

    # ================================================================== #
    # Coordinate transform (LiDAR → map)                                 #
    # ================================================================== #

    def _lidar_to_global(self, x: float, y: float):
        """
        Transform a point from the LiDAR frame to the map frame.
        Mirrors the exact detect.py pattern: try/except with short timeout,
        return None on failure so the pipeline continues gracefully.

        Chain: laser → base_link → map
        """
        point_stamped = PointStamped()
        point_stamped.header.frame_id = 'laser'
        point_stamped.header.stamp    = self.get_clock().now().to_msg()
        point_stamped.point           = Point(x=float(x), y=float(y), z=0.0)

        try:
            t_bl = self.tf_buffer.lookup_transform(
                target_frame='base_link',
                source_frame='laser',
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.03))
            t_map = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame='base_link',
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.03))
        except Exception as e:
            self.get_logger().warn(
                f'[CenterSpeed]: TF lookup failed: {e}', throttle_duration_sec=2.0)
            return None, None

        p_bl  = tf2_geometry_msgs.do_transform_point(point_stamped, t_bl)
        p_map = tf2_geometry_msgs.do_transform_point(p_bl, t_map)
        return p_map.point.x, p_map.point.y

    # ================================================================== #
    # Raceline-aware filtering and publishing                            #
    # ================================================================== #

    def _laser_point_on_track(self, s: float, d: float) -> bool:
        """Return True if the Frenet point (s, d) lies within the drivable track."""
        if self.ego_s is not None and \
                normalize_s(s - self.ego_s, self.track_length) > self.max_viewing_distance:
            return False
        if abs(d) >= self.biggest_d:
            return False
        if abs(d) <= abs(self.smallest_d):
            return True
        idx = bisect_left(self.s_array, s)
        if idx:
            idx -= 1
        if d <= -self.d_right_array[idx] or d >= self.d_left_array[idx]:
            return False
        return True

    def _publish_obstacles(self):
        """Build and publish the ObstacleArray. Applies raceline filter when enabled."""
        msg = ObstacleArray()
        msg.header.stamp    = self.current_stamp
        msg.header.frame_id = 'map'

        if not self.tracked_obstacles:
            self.obs_pub.publish(msg)
            return

        x_centers = [o.center_x for o in self.tracked_obstacles]
        y_centers = [o.center_y for o in self.tracked_obstacles]
        s_pts, d_pts = self.converter.get_frenet(
            np.array(x_centers), np.array(y_centers))

        # Sort by |d| ascending (closest to centre-line first)
        sorted_idx = sorted(range(len(d_pts)), key=lambda i: abs(d_pts[i]))
        remove = []

        for idx in sorted_idx:
            obs = self.tracked_obstacles[idx]
            s, d = s_pts[idx], d_pts[idx]

            if self.using_raceline:
                if not self._laser_point_on_track(s, d):
                    remove.append(obs)
                    continue
            else:
                # Testing mode: use raw Cartesian
                s, d = x_centers[idx], y_centers[idx]

            # Build velocity in Frenet frame
            yaw_diff = obs.theta % (2 * np.pi)
            if yaw_diff > np.pi:
                yaw_diff -= 2 * np.pi
            yaw_diff = -yaw_diff
            ROT  = np.array([[np.cos(yaw_diff), -np.sin(yaw_diff)],
                              [np.sin(yaw_diff),  np.cos(yaw_diff)]])
            vx, vy = np.dot(ROT, [obs.vx, obs.vy])

            if self.measuring and hasattr(self, 'v_cart_pub') and self.ego_yaw is not None:
                ego_yaw_diff = (self.ego_yaw + obs.theta) % (2 * np.pi)
                if ego_yaw_diff > np.pi:
                    ego_yaw_diff -= 2 * np.pi
                ROT_g = np.array([[np.cos(ego_yaw_diff), -np.sin(ego_yaw_diff)],
                                   [np.sin(ego_yaw_diff),  np.cos(ego_yaw_diff)]])
                vx_g, vy_g = np.dot(ROT_g, [obs.vx, obs.vy])
                self.v_cart_pub.publish(Float32MultiArray(data=[float(vx_g), float(vy_g)]))

            obstacle_orientation_global = 0.0
            if self.ego_yaw is not None:
                obstacle_orientation_global = (obs.theta + self.ego_yaw) % (2 * np.pi)
                if obstacle_orientation_global > np.pi:
                    obstacle_orientation_global -= 2 * np.pi

            if not self.use_cartesian:
                vs, vd = self.converter.get_frenet_velocities(
                    vx, vy, obstacle_orientation_global, s)
            else:
                vs, vd = vx, vy

            obs_msg = ObstacleMessage()
            obs_msg.id        = int(obs.id) if obs.id is not None else 0
            obs_msg.s_start   = float(s - obs.size / 2)
            obs_msg.s_end     = float(s + obs.size / 2)
            obs_msg.d_left    = float(d + obs.size / 2)
            obs_msg.d_right   = float(d - obs.size / 2)
            obs_msg.s_center  = float(s)
            obs_msg.d_center  = float(d)
            obs_msg.size      = float(obs.size)
            obs_msg.vs        = float(vs)
            obs_msg.vd        = float(vd)
            obs_msg.is_static = bool(abs(vs) < 0.1 and abs(vd) < 0.1)
            msg.obstacles.append(obs_msg)

        self.obs_pub.publish(msg)
        self.get_logger().info(
            f'[CenterSpeed]: Publishing {len(msg.obstacles)} obstacle(s).',
            throttle_duration_sec=1.0)

        for obs in remove:
            if obs in self.tracked_obstacles:
                self.tracked_obstacles.remove(obs)

    # ================================================================== #
    # Visualization                                                       #
    # ================================================================== #

    def _publish_markers(self):
        """Publish RViz sphere markers for each tracked obstacle."""
        marker_array = MarkerArray()
        # Clear stale markers first
        clear_m = Marker()
        clear_m.action = Marker.DELETEALL
        marker_array.markers.append(clear_m)

        for i, obs in enumerate(self.tracked_obstacles):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = self.current_stamp
            m.id              = i
            m.type            = Marker.SPHERE
            m.scale.x         = obs.size * 1.5
            m.scale.y         = obs.size
            m.scale.z         = obs.size
            m.color.a         = 0.5
            m.color.r         = 0.0
            m.color.g         = 1.0
            m.color.b         = float(i)
            m.pose.position.x = float(obs.center_x)
            m.pose.position.y = float(obs.center_y)
            m.pose.position.z = 0.0

            # Orientation
            orientation = 0.0
            if self.ego_yaw is not None:
                orientation = (obs.theta + self.ego_yaw) % (2 * np.pi)
                if orientation > np.pi:
                    orientation -= 2 * np.pi
            from scipy.spatial.transform import Rotation as R
            q = R.from_euler('z', orientation).as_quat()
            m.pose.orientation.x = float(q[0])
            m.pose.orientation.y = float(q[1])
            m.pose.orientation.z = float(q[2])
            m.pose.orientation.w = float(q[3])
            m.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
            marker_array.markers.append(m)

        self.marker_pub.publish(marker_array)

    def _publish_image(self, tensor: torch.Tensor):
        """Publish heatmap tensor as a ROS Image for Foxglove inspection."""
        try:
            from cv_bridge import CvBridge
        except ImportError:
            return

        if tensor.requires_grad:
            tensor = tensor.detach()
        arr = tensor.cpu().numpy()
        if arr.dtype != np.uint8:
            arr = np.clip(arr * 255, 0, 255).astype(np.uint8)

        bridge = CvBridge()
        ros_img = bridge.cv2_to_imgmsg(arr, encoding='mono8')
        self.image_pub.publish(ros_img)


# ======================================================================= #
# Entry point                                                              #
# ======================================================================= #
def main(args=None):
    rclpy.init(args=args)
    node = CenterSpeedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
