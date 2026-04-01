#!/usr/bin/env python3
"""
AutoDRIVE → ForzaETH adapter node.

Bridges AutoDRIVE RoboRacer topics to ForzaETH race stack interface.

Responsibilities:
  1. Encoder angles → /odom  (Ackermann dead-reckoning + tf odom→base_link)
  2. /autodrive/roboracer_1/lidar → /scan
  3. /autodrive/roboracer_1/imu  → /sensors/imu/raw
  4. /drive (AckermannDriveStamped) → throttle_command + steering_command

AutoDRIVE bridge publishes encoder angles in RADIANS (not ticks).
encoder_angle [rad] × wheel_radius [m] = arc_length [m]

Vehicle constants (AutoDRIVE technical spec):
  Wheel radius  : 0.0590 m
  Wheelbase     : 0.3240 m
  Steer limit   : ±0.5236 rad ↔ ±1.0 normalised
  Top speed     : 22.88 m/s ↔ throttle=1.0

QoS: AutoDRIVE bridge uses RELIABLE — all subscriptions must match.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu, JointState
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped

# ── Vehicle constants ─────────────────────────────────────────────────────────
WHEEL_RADIUS        = 0.0590   # m
WHEELBASE           = 0.3240   # m
STEER_NORM_TO_RAD   = 0.5236   # |max steering angle| rad; normalised ∈ [-1,1]
THROTTLE_NORM_TO_MS = 22.88    # top speed m/s; normalised ∈ [-1,1]

# ── QoS ───────────────────────────────────────────────────────────────────────
# AutoDRIVE bridge publishes with RELIABLE — subscribers must match
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)
CTRL_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


class AutoDRIVEAdapter(Node):
    def __init__(self):
        super().__init__('autodrive_adapter')

        # Odometry state
        self._x:     float = 0.0
        self._y:     float = 0.0
        self._theta: float = 0.0

        # Encoder angle state (radians, cumulative from bridge)
        self._left_angle_prev:  float = None
        self._right_angle_prev: float = None
        self._steering_rad:     float = 0.0

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Static tf: base_link → ego_racecar/laser (LiDAR position from AutoDRIVE spec)
        self._static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'base_link'
        static_tf.child_frame_id = 'ego_racecar/laser'
        static_tf.transform.translation.x = 0.2733
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.096
        static_tf.transform.rotation.w = 1.0
        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = 0.0
        self._static_tf_broadcaster.sendTransform(static_tf)



        # ── Publishers ────────────────────────────────────────────────────────
        self._odom_pub     = self.create_publisher(Odometry,              '/odom',                                          CTRL_QOS)
        self._scan_pub     = self.create_publisher(LaserScan,             '/scan',                                          RELIABLE_QOS)
        self._imu_pub      = self.create_publisher(Imu,                   '/sensors/imu/raw',                               RELIABLE_QOS)
        self._throttle_pub = self.create_publisher(Float32,               '/autodrive/roboracer_1/throttle_command',        CTRL_QOS)
        self._steering_pub = self.create_publisher(Float32,               '/autodrive/roboracer_1/steering_command',        CTRL_QOS)

        # ── Subscribers — all RELIABLE to match AutoDRIVE bridge ─────────────
        self.create_subscription(JointState,           '/autodrive/roboracer_1/left_encoder',  self._left_enc_cb,  RELIABLE_QOS)
        self.create_subscription(JointState,           '/autodrive/roboracer_1/right_encoder', self._right_enc_cb, RELIABLE_QOS)
        self.create_subscription(Float32,              '/autodrive/roboracer_1/steering',       self._steering_cb,  RELIABLE_QOS)
        self.create_subscription(LaserScan,            '/autodrive/roboracer_1/lidar',          self._lidar_cb,     RELIABLE_QOS)
        self.create_subscription(Imu,                  '/autodrive/roboracer_1/imu',            self._imu_cb,       RELIABLE_QOS)
        self.create_subscription(AckermannDriveStamped, '/drive',                               self._drive_cb,     CTRL_QOS)

        # Teleop relay for manual mapping
        self._teleop_active = False
        self._teleop_timer = None
        self.create_subscription(Twist, '/key_vel', self._teleop_cb, CTRL_QOS)

        self.get_logger().info('[autodrive_adapter] Node started. QoS: RELIABLE on all AutoDRIVE topics.')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _steering_cb(self, msg: Float32) -> None:
        """Cache steering angle. Bridge publishes normalised [-1,1] → convert to rad."""
        self._steering_rad = float(msg.data) * STEER_NORM_TO_RAD

    def _left_enc_cb(self, msg: JointState) -> None:
        """Cache left encoder angle (radians, cumulative)."""
        if len(msg.position) == 0:
            return
        angle = float(msg.position[0])
        if self._left_angle_prev is None:
            self._left_angle_prev = angle
            return
        # Store previous for delta computation in right_enc_cb
        self._left_angle_prev = angle

    def _right_enc_cb(self, msg: JointState) -> None:
        """
        Compute odometry from right encoder angle delta.

        AutoDRIVE bridge gives cumulative encoder shaft angle in radians.
        Arc length = delta_angle * wheel_radius
        """
        if len(msg.position) == 0:
            return
        angle = float(msg.position[0])

        if self._right_angle_prev is None or self._left_angle_prev is None:
            self._right_angle_prev = angle
            return

        # Arc length from right wheel
        d_s = (angle - self._right_angle_prev) * WHEEL_RADIUS
        self._right_angle_prev = angle

        # Ackermann bicycle model update
        delta = self._steering_rad
        if abs(delta) < 1e-6:
            self._x     += d_s * math.cos(self._theta)
            self._y     += d_s * math.sin(self._theta)
        else:
            R        = WHEELBASE / math.tan(delta)
            d_theta  = d_s / R
            self._x     += R * (math.sin(self._theta + d_theta) - math.sin(self._theta))
            self._y     += R * (math.cos(self._theta)           - math.cos(self._theta + d_theta))
            self._theta += d_theta
            self._theta  = math.atan2(math.sin(self._theta), math.cos(self._theta))

        stamp = msg.header.stamp

        # Publish /odom
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x  = self._x
        odom.pose.pose.position.y  = self._y
        odom.pose.pose.position.z  = 0.0
        odom.pose.pose.orientation = _yaw_to_quat(self._theta)
        self._odom_pub.publish(odom)

        # Broadcast tf odom → base_link
        t = TransformStamped()
        t.header.stamp            = stamp
        t.header.frame_id         = 'odom'
        t.child_frame_id          = 'base_link'
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        t.transform.rotation      = _yaw_to_quat(self._theta)
        self._tf_broadcaster.sendTransform(t)

    def _lidar_cb(self, msg: LaserScan) -> None:
        """Relay LiDAR with corrected frame_id for ForzaETH."""
        msg.header.frame_id = 'ego_racecar/laser'
        self._scan_pub.publish(msg)

    def _imu_cb(self, msg: Imu) -> None:
        """Relay IMU to ForzaETH EKF topic."""
        msg.header.frame_id = 'base_link'
        self._imu_pub.publish(msg)

    def _drive_cb(self, msg: AckermannDriveStamped) -> None:
        """
        Convert ForzaETH /drive → AutoDRIVE throttle + steering commands.

        ForzaETH: speed [m/s], steering_angle [rad]
        AutoDRIVE: normalised Float32 in [-1, 1]
        """
        steer_norm = float(msg.drive.steering_angle) / STEER_NORM_TO_RAD
        steer_norm = max(-1.0, min(1.0, steer_norm))

        throttle = float(msg.drive.speed) / THROTTLE_NORM_TO_MS
        throttle = max(-1.0, min(1.0, throttle))

        s = Float32();  s.data  = steer_norm
        th = Float32(); th.data = throttle
        self._steering_pub.publish(s)
        self._throttle_pub.publish(th)

    def _teleop_cb(self, msg: Twist) -> None:
        """Manual teleop relay for mapping. Overrides /drive when active."""
        throttle = float(msg.linear.x) / THROTTLE_NORM_TO_MS
        throttle = max(-1.0, min(1.0, throttle))
        # Convert angular.z to steering: assume max angular = 1.0 rad/s → max steer
        steer_norm = -float(msg.angular.z) * 0.4
        steer_norm = max(-1.0, min(1.0, steer_norm))
        s = Float32(); s.data = steer_norm
        th = Float32(); th.data = throttle
        self._steering_pub.publish(s)
        self._throttle_pub.publish(th)


def main(args=None):
    rclpy.init(args=args)
    node = AutoDRIVEAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()