#!/usr/bin/env python3
"""
AutoDRIVE Racing Controller v6.

Root cause of v4 failure at higher throttle:
  The velocity profile (ForzaETH, up to 9.1 m/s) was built for f1tenth_gym
  which has instantaneous speed response. In AutoDRIVE, the actual speed
  does not equal commanded speed — it depends on physics. At MAX_THROTTLE=0.020
  the car crawls and turns are fine. At 0.08+ the car reaches corners before
  Pure Pursuit can respond with sufficient steering.

Fixes in v5:
  1. Lookahead reduced: BASE=1.0m (was 1.5m) — controller sees corners sooner.
  2. Velocity profile scaling: instead of using ForzaETH's absolute speeds
     (which assume VESC hardware), we scale them to AutoDRIVE's throttle range
     using a configurable MAX_RACE_THROTTLE. The profile shape is preserved
     (corners slow, straights fast) but absolute values are rescaled.
  3. Respawn detection: if position jumps > RESPAWN_JUMP_M in one cycle,
     the event is logged and odom is NOT affected (odom is adapter's problem,
     not ours). Controller resets steering state to avoid post-respawn oscillation.
  4. Steering gain boost: MAX_STEER_DELTA increased and rate limiter relaxed
     at high CTE to allow faster recovery.
  5. Emergency brake now correctly compares actual LiDAR front distance
     to a reasonable threshold (not disabled).

Tuning workflow:
  Start at MAX_RACE_THROTTLE = 0.05 (known safe)
  Raise by 0.01 per run until collisions appear, then back off 0.01.
  Target: MAX_RACE_THROTTLE = 0.15-0.25 for competitive lap times.

Usage:
  source ~/ws/install/setup.bash
  python3 ~/ws/src/race_stack/utilities/nodes/autodrive_controller.py
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from f110_msgs.msg import WpntArray

# ── AutoDRIVE vehicle constants ───────────────────────────────────────────────
WHEELBASE               = 0.33
MAX_STEERING_ANGLE_RAD  = 0.5236     # ±30° → steering_command ±1.0

# ── Throttle ──────────────────────────────────────────────────────────────────
# The ForzaETH velocity profile has speeds up to 9.1 m/s (built for VESC hardware).
# We rescale: AutoDRIVE throttle = (wp_speed / wp_speed_max) * MAX_RACE_THROTTLE
# This preserves the profile SHAPE (brake at corners, full throttle on straights)
# while keeping absolute throttle within AutoDRIVE's safe range.
THROTTLE_SPEED_GAIN     = 0.05       # throttle = sector_scaled_speed_mps * gain (LocalMapPP calibration)
MAX_THROTTLE            = 0.35       # safety ceiling — the sector profile controls corner speeds
MIN_THROTTLE            = 0.00       # floor — 0 unless motor deadband confirmed

# ── Pure Pursuit ──────────────────────────────────────────────────────────────
BASE_LOOKAHEAD          = 1.0        # reduced from 1.5m — sees corners earlier
MIN_LOOKAHEAD           = 0.6        # at max CTE
MAX_LOOKAHEAD           = 3.0

# ── CTE-adaptive lookahead ────────────────────────────────────────────────────
CTE_REDUCE_START        = 0.10       # start reducing lookahead above this [m]
CTE_REDUCE_FULL         = 0.40       # use MIN_LOOKAHEAD at this CTE [m]

# ── Speed profile lookahead (braking preview) ─────────────────────────────────
SPEED_LOOKAHEAD_DIST    = 1.5        # look ahead this far for min speed [m]

# ── Steering rate limiter ─────────────────────────────────────────────────────
MAX_STEER_DELTA_NORMAL  = 0.30       # max steering change per cycle on-line
MAX_STEER_DELTA_RECOVER = 0.60       # relaxed limit during CTE recovery (CTE > 0.15m)

# ── LiDAR forward collision guard ─────────────────────────────────────────────
FRONT_HALF_ANGLE_DEG    = 25.0       # half-width of forward scan arc [deg]
FRONT_CLEAR_M           = 0.35       # brake to MIN_THROTTLE if front < this [m]

# ── Respawn detection ─────────────────────────────────────────────────────────
RESPAWN_JUMP_M          = 2.0        # if position jumps > this in one cycle → respawn

# ── Yaw validation ────────────────────────────────────────────────────────────
YAW_REJECT_THRESHOLD_DEG = 30.0

# ── Logging ───────────────────────────────────────────────────────────────────
LOG_EVERY_N             = 20         # ~2Hz at 40Hz

# ── QoS ───────────────────────────────────────────────────────────────────────
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)
DEFAULT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=10)


def _quat_to_yaw(x, y, z, w) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


class AutoDRIVEController(Node):
    def __init__(self):
        super().__init__('autodrive_controller')

        # ── Localisation ──────────────────────────────────────────────────────
        self._car_x         = 0.0
        self._car_y         = 0.0
        self._car_yaw       = 0.0
        self._prev_car_yaw  = None
        self._prev_car_x    = None
        self._prev_car_y    = None
        self._pose_received = False
        self._pose_yaw_src  = 'none'
        self._carstate_yaw  = None
        self._respawn_count = 0

        # ── Speed ─────────────────────────────────────────────────────────────
        self._actual_speed  = 0.0
        self._speed_received = False

        # ── LiDAR ─────────────────────────────────────────────────────────────
        self._front_clear_m = 999.0
        self._scan_received = False

        # ── Waypoints ─────────────────────────────────────────────────────────
        self._wp_x    = None
        self._wp_y    = None
        self._wp_v    = None   # ForzaETH speeds (up to 9.1 m/s)
        self._wp_s    = None
        self._wp_ready = False
        self._n_wp    = 0

        # ── Control state ─────────────────────────────────────────────────────
        self._prev_steer_cmd = 0.0
        self._loop_count     = 0

        # ── Publishers ────────────────────────────────────────────────────────
        self._throttle_pub = self.create_publisher(
            Float32, '/autodrive/roboracer_1/throttle_command', DEFAULT_QOS)
        self._steering_pub = self.create_publisher(
            Float32, '/autodrive/roboracer_1/steering_command', DEFAULT_QOS)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(WpntArray,   '/global_waypoints_scaled',            self._wp_cb,       DEFAULT_QOS)
        self.create_subscription(PoseStamped, '/tracked_pose',                self._tracked_cb,  DEFAULT_QOS)
        self.create_subscription(PoseStamped, '/car_state/pose',              self._carstate_cb, DEFAULT_QOS)
        self.create_subscription(Float32,     '/autodrive/roboracer_1/speed', self._speed_cb,    RELIABLE_QOS)
        self.create_subscription(LaserScan,   '/scan',                        self._scan_cb,     RELIABLE_QOS)

        self._control_timer = self.create_timer(1.0 / 40.0, self._control_loop)

        self.get_logger().info(
            f'[autodrive_controller v6]\n'
            f'  Throttle      : speed*{THROTTLE_SPEED_GAIN} (sector-scaled), ceiling={MAX_THROTTLE}\n'
            f'  Lookahead     : base={BASE_LOOKAHEAD}m  min={MIN_LOOKAHEAD}m  max={MAX_LOOKAHEAD}m\n'
            f'  CTE adapt     : {CTE_REDUCE_START}m→{CTE_REDUCE_FULL}m\n'
            f'  Steer rate    : normal=±{MAX_STEER_DELTA_NORMAL}  recovery=±{MAX_STEER_DELTA_RECOVER}\n'
            f'  Collision guard: ±{FRONT_HALF_ANGLE_DEG}° < {FRONT_CLEAR_M}m → brake\n'
            f'  Respawn detect: jump > {RESPAWN_JUMP_M}m → reset steering state')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _wp_cb(self, msg: WpntArray) -> None:
        n = len(msg.wpnts)
        if n < 2:
            return
        self._wp_x = np.array([w.x_m    for w in msg.wpnts])
        self._wp_y = np.array([w.y_m    for w in msg.wpnts])
        self._wp_v = np.array([w.vx_mps for w in msg.wpnts])
        self._wp_s = np.array([w.s_m    for w in msg.wpnts])
        self._n_wp = n
        self._wp_ready = True
        if not hasattr(self, '_wp_logged'):
            self._wp_logged = True
            self.get_logger().info(
                f'[WP] {n} waypoints. Track={self._wp_s[-1]:.1f}m  '
                f'Speed=[{self._wp_v.min():.2f},{self._wp_v.max():.2f}]m/s  '
                f'WP0=({self._wp_x[0]:.2f},{self._wp_y[0]:.2f})  '
                f'Subscribing to /global_waypoints_scaled (sector limits pre-applied by ForzaETH)')

    def _tracked_cb(self, msg: PoseStamped) -> None:
        new_x = msg.pose.position.x
        new_y = msg.pose.position.y
        q = msg.pose.orientation
        quat_yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)

        # Respawn detection
        if self._prev_car_x is not None:
            jump = math.sqrt((new_x - self._prev_car_x)**2 + (new_y - self._prev_car_y)**2)
            if jump > RESPAWN_JUMP_M:
                self._respawn_count += 1
                self._prev_steer_cmd = 0.0   # reset steering state
                self.get_logger().warn(
                    f'[RESPAWN #{self._respawn_count}] Position jumped {jump:.2f}m: '
                    f'({self._prev_car_x:.2f},{self._prev_car_y:.2f}) → ({new_x:.2f},{new_y:.2f}). '
                    f'Steering state reset.')

        # Yaw validation
        if self._prev_car_yaw is not None:
            delta_deg = abs(math.degrees(
                math.atan2(math.sin(quat_yaw - self._prev_car_yaw),
                            math.cos(quat_yaw - self._prev_car_yaw))))
            if delta_deg > YAW_REJECT_THRESHOLD_DEG:
                if self._prev_car_x is not None:
                    dx = new_x - self._prev_car_x
                    dy = new_y - self._prev_car_y
                    if math.sqrt(dx*dx + dy*dy) > 0.01:
                        self._car_yaw = math.atan2(dy, dx)
                        self._pose_yaw_src = 'pos_diff'
            else:
                self._car_yaw = quat_yaw
                self._pose_yaw_src = 'quat'
        else:
            self._car_yaw = quat_yaw
            self._pose_yaw_src = 'quat'

        self._car_x = new_x
        self._car_y = new_y
        self._prev_car_yaw = self._car_yaw
        self._prev_car_x   = new_x
        self._prev_car_y   = new_y
        self._pose_received = True

    def _carstate_cb(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        self._carstate_yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)

    def _speed_cb(self, msg: Float32) -> None:
        self._actual_speed   = float(msg.data)
        self._speed_received = True

    def _scan_cb(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        if n == 0:
            return
        self._scan_received = True
        half_rad   = math.radians(FRONT_HALF_ANGLE_DEG)
        centre_idx = int(round(-msg.angle_min / msg.angle_increment))
        half_idx   = int(round(half_rad / msg.angle_increment))
        lo = max(0, centre_idx - half_idx)
        hi = min(n - 1, centre_idx + half_idx)
        front = [r for r in msg.ranges[lo:hi+1] if math.isfinite(r) and r > 0.06]
        self._front_clear_m = min(front) if front else 999.0

    # ── Control Loop ──────────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        self._loop_count += 1
        should_log = (self._loop_count % LOG_EVERY_N == 0)

        if self._loop_count == 80 and not self._speed_received:
            self.get_logger().warn('[SPEED] 0 publishers on /autodrive/roboracer_1/speed. Feedforward only.')

        if not self._wp_ready or not self._pose_received:
            if should_log:
                self.get_logger().warn(f'[WAIT] wp={self._wp_ready} pose={self._pose_received}')
            self._publish(0.0, 0.0)
            return

        # 1. Nearest waypoint (CTE)
        dx       = self._wp_x - self._car_x
        dy       = self._wp_y - self._car_y
        dists_sq = dx*dx + dy*dy
        nearest_idx  = int(np.argmin(dists_sq))
        cte          = math.sqrt(float(dists_sq[nearest_idx]))

        # 2. Adaptive lookahead — shorter when off-line for faster correction
        if cte > CTE_REDUCE_FULL:
            lookahead = MIN_LOOKAHEAD
        elif cte > CTE_REDUCE_START:
            frac = (cte - CTE_REDUCE_START) / (CTE_REDUCE_FULL - CTE_REDUCE_START)
            lookahead = BASE_LOOKAHEAD - frac * (BASE_LOOKAHEAD - MIN_LOOKAHEAD)
        else:
            lookahead = BASE_LOOKAHEAD
        lookahead = min(lookahead, MAX_LOOKAHEAD)

        # 3. Lookahead waypoint
        total_s   = float(self._wp_s[-1])
        current_s = float(self._wp_s[nearest_idx])
        target_s  = current_s + lookahead
        wrapped   = False
        if target_s > total_s:
            target_s -= total_s
            wrapped   = True
        la_idx = int(np.searchsorted(self._wp_s, target_s) % self._n_wp)
        la_x   = float(self._wp_x[la_idx])
        la_y   = float(self._wp_y[la_idx])

        # 4. Car-local frame transform
        dx_la   = la_x - self._car_x
        dy_la   = la_y - self._car_y
        cos_yaw = math.cos(self._car_yaw)
        sin_yaw = math.sin(self._car_yaw)
        local_x =  cos_yaw * dx_la + sin_yaw * dy_la
        local_y = -sin_yaw * dx_la + cos_yaw * dy_la

        # 5. Pure Pursuit steering
        true_la = math.sqrt(local_x**2 + local_y**2)
        if true_la < 0.01:
            steering_rad = 0.0
            curvature    = 0.0
        else:
            curvature    = 2.0 * local_y / (true_la**2)
            steering_rad = math.atan(WHEELBASE * curvature)

        steer_raw = max(-1.0, min(1.0, steering_rad / MAX_STEERING_ANGLE_RAD))

        # 6. Steering rate limiter — relaxed during CTE recovery
        max_delta = MAX_STEER_DELTA_RECOVER if cte > CTE_REDUCE_START else MAX_STEER_DELTA_NORMAL
        delta = steer_raw - self._prev_steer_cmd
        if abs(delta) > max_delta:
            steer_final  = self._prev_steer_cmd + math.copysign(max_delta, delta)
            rate_limited = True
        else:
            steer_final  = steer_raw
            rate_limited = False
        self._prev_steer_cmd = steer_final

        # 7. Throttle — /global_waypoints_scaled already has sector speed limits applied.
        # Simple feedforward: sector_speed * calibrated_gain (from LocalMapPP).
        profile_speed = self._get_lookahead_speed(nearest_idx)
        throttle = profile_speed * THROTTLE_SPEED_GAIN
        throttle = max(MIN_THROTTLE, min(MAX_THROTTLE, throttle))

        # 8. Forward collision guard
        collision_brake = False
        if self._scan_received and self._front_clear_m < FRONT_CLEAR_M:
            throttle        = MIN_THROTTLE
            collision_brake = True

        # 9. Publish
        self._publish(throttle, steer_final)

        # ── Diagnostic log ────────────────────────────────────────────────────
        if should_log:
            yaw_d = ''
            if self._carstate_yaw is not None:
                yd = abs(math.degrees(math.atan2(
                    math.sin(self._car_yaw - self._carstate_yaw),
                    math.cos(self._car_yaw - self._carstate_yaw))))
                yaw_d = f'  yaw_Δ={yd:.1f}°'
            spd = f'{self._actual_speed:.2f}m/s' if self._speed_received else 'NO-FEED'
            self.get_logger().info(
                f'[CTRL#{self._loop_count}]\n'
                f'  LOC  : pos=({self._car_x:.2f},{self._car_y:.2f})  '
                f'yaw={math.degrees(self._car_yaw):.1f}°  src={self._pose_yaw_src}{yaw_d}  '
                f'respawns={self._respawn_count}\n'
                f'  WP   : nearest={nearest_idx}  CTE={cte:.3f}m  la={lookahead:.2f}m  '
                f'la_idx={la_idx}  la_local=({local_x:.2f},{local_y:.2f})  wrap={wrapped}\n'
                f'  STEER: curv={curvature:.3f}  rad={steering_rad:.3f}  '
                f'raw={steer_raw:.3f}  final={steer_final:.3f}  rate_lim={rate_limited}  max_Δ={max_delta:.2f}\n'
                f'  SPEED: profile={profile_speed:.2f}m/s  '
                f'actual={spd}  throttle={throttle:.4f}  '
                f'front={self._front_clear_m:.2f}m  brake={collision_brake}')

    def _get_lookahead_speed(self, nearest_idx: int) -> float:
        current_s = float(self._wp_s[nearest_idx])
        target_s  = current_s + SPEED_LOOKAHEAD_DIST
        total_s   = float(self._wp_s[-1])
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