#!/usr/bin/env python3
"""
AutoDRIVE → ForzaETH adapter node.

Bridges AutoDRIVE RoboRacer topics to ForzaETH race stack interface.

Responsibilities:
  1. Encoder angles → /odom  (Ackermann dead-reckoning + tf odom→base_link)
  2. /autodrive/roboracer_1/lidar → /scan
  3. /autodrive/roboracer_1/imu  → /sensors/imu/raw
  4. /drive (AckermannDriveStamped) → throttle_command + steering_command
     with CLOSED-LOOP PID speed control (AutoDRIVE has no internal speed controller)

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

# ── Speed controller constants (calibrated from LocalMapPP) ──────────────────
# AutoDRIVE has NO internal speed controller — throttle is raw motor duty cycle.
# We need closed-loop control: feedforward + PID feedback.
THROTTLE_FF_GAIN    = 0.05     # feedforward: throttle = speed_mps * gain
                               # (from LocalMapPP empirical calibration)
SPEED_KP            = 0.08     # proportional gain for speed error
SPEED_KI            = 0.01     # integral gain (prevents steady-state error)
SPEED_KD            = 0.005    # derivative gain (damps oscillation)
SPEED_I_MAX         = 0.3      # integral windup clamp
MAX_THROTTLE        = 1.0      # absolute throttle clamp

# ── QoS ───────────────────────────────────────────────────────────────────────
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

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5
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
        self._last_enc_time: float = None
        self._steering_rad:     float = 0.0

        # Velocity state for twist field (computed each encoder cycle)
        self._vx:    float = 0.0
        self._omega: float = 0.0

        # ── Speed controller state ────────────────────────────────────────────
        self._actual_speed: float = 0.0    # from /autodrive/roboracer_1/speed
        self._speed_error_integral: float = 0.0
        self._speed_error_prev: float = 0.0
        self._last_drive_time: float = None

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Static tf: base_link → ego_racecar/laser
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
        self._odom_pub     = self.create_publisher(Odometry,   '/odom',                                   CTRL_QOS)
        self._scan_pub = self.create_publisher(LaserScan, '/scan', SENSOR_QOS)
        self._imu_pub      = self.create_publisher(Imu,        '/sensors/imu/raw',                        RELIABLE_QOS)
        self._throttle_pub = self.create_publisher(Float32,    '/autodrive/roboracer_1/throttle_command',  CTRL_QOS)
        self._steering_pub = self.create_publisher(Float32,    '/autodrive/roboracer_1/steering_command',  CTRL_QOS)
        self._speed_pub = self.create_publisher(Float32, '/autodrive/roboracer_1/speed', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(JointState,            '/autodrive/roboracer_1/left_encoder',  self._left_enc_cb,  RELIABLE_QOS)
        self.create_subscription(JointState,            '/autodrive/roboracer_1/right_encoder', self._right_enc_cb, RELIABLE_QOS)
        self.create_subscription(Float32,               '/autodrive/roboracer_1/steering',      self._steering_cb,  RELIABLE_QOS)
        self.create_subscription(LaserScan,             '/autodrive/roboracer_1/lidar',         self._lidar_cb,     RELIABLE_QOS)
        self.create_subscription(Imu,                   '/autodrive/roboracer_1/imu',           self._imu_cb,       RELIABLE_QOS)
        self.create_subscription(Float32,               '/autodrive/roboracer_1/speed',         self._speed_cb,     RELIABLE_QOS)
        self.create_subscription(AckermannDriveStamped, '/drive',                               self._drive_cb,     CTRL_QOS)
        self.create_subscription(Twist,                 '/key_vel',                             self._teleop_cb,    CTRL_QOS)

        # Teleop state
        self._teleop_active = False

        self.get_logger().info(
            '[autodrive_adapter] Node started. '
            'PID speed controller enabled (FF=%.3f, Kp=%.3f, Ki=%.3f, Kd=%.3f)' %
            (THROTTLE_FF_GAIN, SPEED_KP, SPEED_KI, SPEED_KD))

    # ── Sensor callbacks ──────────────────────────────────────────────────────

    def _steering_cb(self, msg: Float32) -> None:
        self._steering_rad = float(msg.data) * STEER_NORM_TO_RAD

    def _speed_cb(self, msg: Float32) -> None:
        """Cache actual vehicle speed from AutoDRIVE bridge."""
        self._actual_speed = float(msg.data)

    def _left_enc_cb(self, msg: JointState) -> None:
        if len(msg.position) == 0:
            return
        angle = float(msg.position[0])
        if self._left_angle_prev is None:
            self._left_angle_prev = angle
            return
        self._left_angle_prev = angle

    def _right_enc_cb(self, msg: JointState) -> None:
        if len(msg.position) == 0:
            return
        angle = float(msg.position[0])

        if self._right_angle_prev is None or self._left_angle_prev is None:
            self._right_angle_prev = angle
            return

        d_s = (angle - self._right_angle_prev) * WHEEL_RADIUS

        # ── Compute velocity BEFORE updating _last_enc_time ──────────────
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self._last_enc_time is not None:
            dt_enc = now_sec - self._last_enc_time
            if dt_enc > 1e-4:
                self._vx = d_s / dt_enc
                self._omega = self._vx * math.tan(self._steering_rad) / WHEELBASE

                # Also publish speed topic + update PID state
                speed_msg = Float32()
                speed_msg.data = float(abs(self._vx))
                self._speed_pub.publish(speed_msg)
                self._actual_speed = float(abs(self._vx))
            else:
                # dt too small — keep previous velocity (don't zero it out)
                pass
        else:
            # First encoder pair — no velocity yet
            self._vx = 0.0
            self._omega = 0.0

        # Update time AFTER using it for dt
        self._last_enc_time = now_sec
        self._right_angle_prev = angle

        # ── Dead-reckoning integration (unchanged) ───────────────────────
        delta = self._steering_rad
        if abs(delta) < 1e-6:
            self._x += d_s * math.cos(self._theta)
            self._y += d_s * math.sin(self._theta)
        else:
            R = WHEELBASE / math.tan(delta)
            d_theta = d_s / R
            self._x += R * (math.sin(self._theta + d_theta) - math.sin(self._theta))
            self._y += R * (math.cos(self._theta) - math.cos(self._theta + d_theta))
            self._theta += d_theta
            self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))

        # ── Publish /odom WITH twist ─────────────────────────────────────
        stamp = msg.header.stamp

        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x  = self._x
        odom.pose.pose.position.y  = self._y
        odom.pose.pose.position.z  = 0.0
        odom.pose.pose.orientation = _yaw_to_quat(self._theta)

        # Twist — THIS is what the EKF actually fuses
        odom.twist.twist.linear.x  = self._vx
        odom.twist.twist.angular.z = self._omega

        # Non-zero covariances so EKF treats these as real measurements
        # (36-element row-major 6×6: indices [0]=var(vx), [35]=var(yaw_rate))
        odom.twist.covariance[0]  = 0.001    # var(vx)  — encoder is decent
        odom.twist.covariance[7]  = 0.001   # var(vy) ← ADD THIS
        odom.twist.covariance[35] = 0.003    # var(wz)  — derived from steering model
        self._odom_pub.publish(odom)

        # ── Publish tf odom → base_link ──────────────────────────────────
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
        msg.header.frame_id = 'ego_racecar/laser'
        self._scan_pub.publish(msg)

    def _imu_cb(self, msg: Imu) -> None:
        msg.header.frame_id = 'base_link'
        self._imu_pub.publish(msg)

    # ── Drive command with PID speed controller ───────────────────────────────

    def _drive_cb(self, msg: AckermannDriveStamped) -> None:
        """
        Convert ForzaETH /drive → AutoDRIVE throttle + steering commands.

        Uses closed-loop PID speed control because AutoDRIVE sim has no
        internal speed controller (throttle is raw motor duty cycle).

        Steering: direct conversion rad → normalised [-1,1]
        Throttle: feedforward (from LocalMapPP calibration) + PID feedback
        """
        if self._teleop_active:
            return

        # ── Steering: same conversion as before ──────────────────────────────
        steer_norm = float(msg.drive.steering_angle) / STEER_NORM_TO_RAD
        steer_norm = max(-1.0, min(1.0, steer_norm))

        # ── Throttle: feedforward + PID ──────────────────────────────────────
        desired_speed = float(msg.drive.speed)

        # Time delta for PID
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_drive_time is None:
            dt = 0.02  # assume ~50Hz on first call
        else:
            dt = now - self._last_drive_time
            dt = max(dt, 0.001)  # safety floor
            dt = min(dt, 0.5)    # safety ceiling (e.g. after pause)
        self._last_drive_time = now

        # Speed error
        speed_error = desired_speed - self._actual_speed

        # PID terms
        # Proportional
        p_term = SPEED_KP * speed_error

        # Integral with anti-windup
        self._speed_error_integral += speed_error * dt
        self._speed_error_integral = max(-SPEED_I_MAX,
                                          min(SPEED_I_MAX, self._speed_error_integral))
        i_term = SPEED_KI * self._speed_error_integral

        # Derivative (on error, not on measurement — acceptable for this use case)
        d_term = SPEED_KD * (speed_error - self._speed_error_prev) / dt
        self._speed_error_prev = speed_error

        # Feedforward from LocalMapPP calibration
        ff_term = THROTTLE_FF_GAIN * desired_speed

        # Total throttle
        throttle = ff_term + p_term + i_term + d_term

        # Clamp
        if desired_speed <= 0.01:
            # Explicit stop command
            throttle = 0.0
            self._speed_error_integral = 0.0
        else:
            throttle = max(0.0, min(MAX_THROTTLE, throttle))

        # Publish
        s = Float32();  s.data = steer_norm
        th = Float32(); th.data = throttle
        self._steering_pub.publish(s)
        self._throttle_pub.publish(th)

    # ── Teleop (manual mapping) ───────────────────────────────────────────────

    def _teleop_cb(self, msg: Twist) -> None:
        """Manual teleop relay for mapping. Prioritized override of /drive."""
        self._teleop_active = True

        if abs(msg.linear.x) < 0.001 and abs(msg.angular.z) < 0.001:
            self._teleop_active = False

        # Teleop uses the simple gain — no PID needed at low mapping speeds
        throttle = float(msg.linear.x) * THROTTLE_FF_GAIN
        throttle = max(-MAX_THROTTLE, min(MAX_THROTTLE, throttle))

        steer_norm = float(msg.angular.z)
        steer_norm = max(-1.0, min(1.0, steer_norm))

        s = Float32();  s.data = steer_norm
        th = Float32(); th.data = throttle
        self._steering_pub.publish(s)
        self._throttle_pub.publish(th)

        # Reset PID state when switching back from teleop
        self._speed_error_integral = 0.0
        self._speed_error_prev = 0.0


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