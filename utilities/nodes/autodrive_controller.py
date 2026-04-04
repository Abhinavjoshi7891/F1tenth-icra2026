#!/usr/bin/env python3
"""
AutoDRIVE Racing Controller v2 — with diagnostic logging.

Usage (ForzaETH container, sec terminal):
  source ~/ws/install/setup.bash
  python3 ~/ws/src/race_stack/utilities/nodes/autodrive_controller.py
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from f110_msgs.msg import WpntArray

# ── AutoDRIVE vehicle constants ───────────────────────────────────────────────
WHEELBASE               = 0.33
MAX_STEERING_ANGLE_RAD  = 0.5236
MAX_STEER_CMD           = 1.0

# ── Throttle calibration (from LocalMapPP) ────────────────────────────────────
THROTTLE_SPEED_GAIN     = 0.05
MAX_THROTTLE            = 0.3      # start conservative

# ── Pure Pursuit ──────────────────────────────────────────────────────────────
BASE_LOOKAHEAD          = 1.2
SPEED_LOOKAHEAD_GAIN    = 0.4
MAX_LOOKAHEAD           = 3.0

# ── Speed lookahead ───────────────────────────────────────────────────────────
SPEED_LOOKAHEAD_DIST    = 2.5

# ── Steering rate limiter ─────────────────────────────────────────────────────
MAX_STEER_DELTA         = 0.75

# ── Logging ───────────────────────────────────────────────────────────────────
LOG_EVERY_N             = 10       # log every N control cycles (~4Hz at 40Hz loop)

# ── QoS ───────────────────────────────────────────────────────────────────────
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)
DEFAULT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=10)


class AutoDRIVEController(Node):
    def __init__(self):
        super().__init__('autodrive_controller')

        # State
        self._car_x = 0.0
        self._car_y = 0.0
        self._car_yaw = 0.0
        self._actual_speed = 0.0
        self._pose_received = False
        self._speed_received = False

        # Waypoints
        self._wp_x = None
        self._wp_y = None
        self._wp_v = None
        self._wp_s = None
        self._wp_ready = False
        self._n_wp = 0

        # Rate limiter
        self._prev_steer_cmd = 0.0

        # Logging counter
        self._loop_count = 0

        # Publishers
        self._throttle_pub = self.create_publisher(
            Float32, '/autodrive/roboracer_1/throttle_command', DEFAULT_QOS)
        self._steering_pub = self.create_publisher(
            Float32, '/autodrive/roboracer_1/steering_command', DEFAULT_QOS)

        # Subscribers
        self.create_subscription(
            WpntArray, '/global_waypoints', self._waypoints_cb, DEFAULT_QOS)
        self.create_subscription(
            PoseStamped, '/car_state/pose', self._pose_cb, DEFAULT_QOS)
        self.create_subscription(
            Float32, '/autodrive/roboracer_1/speed', self._speed_cb, RELIABLE_QOS)

        # 40 Hz control loop
        self._control_timer = self.create_timer(1.0 / 40.0, self._control_loop)

        self.get_logger().info(
            f'[autodrive_controller] Started. '
            f'Throttle gain={THROTTLE_SPEED_GAIN}, '
            f'Max throttle={MAX_THROTTLE}, '
            f'Base lookahead={BASE_LOOKAHEAD}m, '
            f'Max steer={MAX_STEER_CMD}, '
            f'Steer rate limit={MAX_STEER_DELTA}/cycle')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _waypoints_cb(self, msg: WpntArray) -> None:
        n = len(msg.wpnts)
        if n < 2:
            return
        self._wp_x = np.array([w.x_m for w in msg.wpnts])
        self._wp_y = np.array([w.y_m for w in msg.wpnts])
        self._wp_v = np.array([w.vx_mps for w in msg.wpnts])
        self._wp_s = np.array([w.s_m for w in msg.wpnts])
        self._n_wp = n
        self._wp_ready = True
        if not hasattr(self, '_wp_logged'):
            self._wp_logged = True
            self.get_logger().info(
                f'[WAYPOINTS] Received {n} waypoints. '
                f'Track={self._wp_s[-1]:.1f}m, '
                f'Speed=[{np.min(self._wp_v):.1f}, {np.max(self._wp_v):.1f}] m/s, '
                f'X=[{np.min(self._wp_x):.1f}, {np.max(self._wp_x):.1f}], '
                f'Y=[{np.min(self._wp_y):.1f}, {np.max(self._wp_y):.1f}]')

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._car_x = msg.pose.position.x
        self._car_y = msg.pose.position.y
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._car_yaw = math.atan2(siny, cosy)
        self._pose_received = True

    def _speed_cb(self, msg: Float32) -> None:
        self._actual_speed = float(msg.data)
        self._speed_received = True

    # ── Control Loop ──────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        self._loop_count += 1
        should_log = (self._loop_count % LOG_EVERY_N == 0)

        if not self._wp_ready or not self._pose_received:
            if should_log:
                self.get_logger().warn(
                    f'[WAITING] wp_ready={self._wp_ready}, '
                    f'pose_received={self._pose_received}, '
                    f'speed_received={self._speed_received}')
            self._publish(0.0, 0.0)
            return

        # 1. Find nearest waypoint
        dx = self._wp_x - self._car_x
        dy = self._wp_y - self._car_y
        dists_sq = dx * dx + dy * dy
        nearest_idx = int(np.argmin(dists_sq))
        nearest_dist = math.sqrt(dists_sq[nearest_idx])

        # 2. Lookahead distance
        lookahead = BASE_LOOKAHEAD + SPEED_LOOKAHEAD_GAIN * self._actual_speed
        lookahead = min(lookahead, MAX_LOOKAHEAD)

        # 3. Find lookahead waypoint
        current_s = self._wp_s[nearest_idx]
        target_s = current_s + lookahead
        total_s = self._wp_s[-1]
        wrapped = False
        if target_s > total_s:
            target_s -= total_s
            wrapped = True

        la_idx = int(np.searchsorted(self._wp_s, target_s) % self._n_wp)

        la_x = self._wp_x[la_idx]
        la_y = self._wp_y[la_idx]

        # 4. Transform to car local frame
        dx_la = la_x - self._car_x
        dy_la = la_y - self._car_y
        cos_yaw = math.cos(self._car_yaw)
        sin_yaw = math.sin(self._car_yaw)
        local_x = cos_yaw * dx_la + sin_yaw * dy_la
        local_y = -sin_yaw * dx_la + cos_yaw * dy_la

        # 5. Pure Pursuit steering
        true_lookahead = math.sqrt(local_x ** 2 + local_y ** 2)
        if true_lookahead < 0.01:
            steering_angle_rad = 0.0
            curvature = 0.0
        else:
            curvature = 2.0 * local_y / (true_lookahead ** 2)
            steering_angle_rad = math.atan(WHEELBASE * curvature)

        # Normalize and clamp
        steer_cmd_raw = steering_angle_rad / MAX_STEERING_ANGLE_RAD
        steer_cmd_raw = max(-1.0, min(1.0, steer_cmd_raw))
        steer_cmd_clamped = max(-MAX_STEER_CMD, min(MAX_STEER_CMD, steer_cmd_raw))

        # Rate limiter
        delta = steer_cmd_clamped - self._prev_steer_cmd
        if abs(delta) > MAX_STEER_DELTA:
            steer_cmd_final = self._prev_steer_cmd + math.copysign(MAX_STEER_DELTA, delta)
        else:
            steer_cmd_final = steer_cmd_clamped
        steer_was_limited = (abs(delta) > MAX_STEER_DELTA)
        self._prev_steer_cmd = steer_cmd_final

        # 6. Speed from velocity profile
        target_speed = self._get_lookahead_speed(nearest_idx)

        # 7. Throttle
        throttle = target_speed * THROTTLE_SPEED_GAIN
        throttle = max(0.0, min(MAX_THROTTLE, throttle))

        # 8. Publish
        self._publish(throttle, steer_cmd_final)

        # ── DIAGNOSTIC LOG ────────────────────────────────────────────────────
        if should_log:
            self.get_logger().info(
                f'[CTRL] '
                f'pose=({self._car_x:.2f},{self._car_y:.2f}) '
                f'yaw={math.degrees(self._car_yaw):.1f}° | '
                f'nearest_wp={nearest_idx} dist={nearest_dist:.3f}m | '
                f'la_idx={la_idx} la_world=({la_x:.2f},{la_y:.2f}) '
                f'la_local=({local_x:.2f},{local_y:.2f}) '
                f'la_dist={true_lookahead:.2f}m wrap={wrapped} | '
                f'curv={curvature:.3f} steer_rad={steering_angle_rad:.3f} '
                f'steer_raw={steer_cmd_raw:.3f} '
                f'steer_final={steer_cmd_final:.3f} '
                f'rate_limited={steer_was_limited} | '
                f'target_v={target_speed:.2f} actual_v={self._actual_speed:.2f} '
                f'throttle={throttle:.4f}')

    def _get_lookahead_speed(self, nearest_idx: int) -> float:
        current_s = self._wp_s[nearest_idx]
        target_s = current_s + SPEED_LOOKAHEAD_DIST
        total_s = self._wp_s[-1]
        if target_s > total_s:
            mask = (self._wp_s >= current_s) | (self._wp_s <= target_s - total_s)
        else:
            mask = (self._wp_s >= current_s) & (self._wp_s <= target_s)
        if np.sum(mask) == 0:
            return float(self._wp_v[nearest_idx])
        return float(np.min(self._wp_v[mask]))

    def _publish(self, throttle: float, steering: float) -> None:
        th = Float32(); th.data = float(throttle)
        st = Float32(); st.data = float(steering)
        self._throttle_pub.publish(th)
        self._steering_pub.publish(st)


def main(args=None):
    rclpy.init(args=args)
    node = AutoDRIVEController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        th = Float32(); th.data = 0.0
        st = Float32(); st.data = 0.0
        node._throttle_pub.publish(th)
        node._steering_pub.publish(st)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()