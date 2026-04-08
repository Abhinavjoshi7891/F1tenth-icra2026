#!/usr/bin/env python3
"""
local_map_pp_cartographer.py

A surgical modification of local_map_pp_v2.py for the ICRA 2026
RoboRacer Sim Racing League competition.

KEY CHANGE — IPS is RESTRICTED in competition.
This node replaces the IPS-based localisation pipeline with
Cartographer's /tracked_pose, which:
  - Provides (x, y) in the Cartographer map frame
  - Provides heading directly from the orientation quaternion
  - Runs at 200 Hz with <1 ms delay (confirmed)

WHAT CHANGED vs local_map_pp_v2:
  REMOVED:  _ips_callback         — IPS subscription gone
  REMOVED:  _lap_count_callback   — no lap-0 mapping needed
  REMOVED:  _build_track_map()    — no lap-0 mapping needed
  REMOVED:  _compute_velocity_profile() — done offline by generate_porto_npz.py
  REMOVED:  LocalMapGenerator     — no local map needed, global map only
  REMOVED:  _pure_pursuit_center_line() — fallback removed, dead without LocalMapGenerator
  ADDED:    _tracked_pose_cb()    — /tracked_pose → (x, y, heading)
  CHANGED:  __init__              — loads porto_raceline.npz at startup,
                                    sets mapping_complete=True immediately
  UNCHANGED: _pure_pursuit_global_map()   — identical algorithm
  UNCHANGED: _publish_commands()          — same AutoDRIVE interface
  UNCHANGED: all throttle/steering constants

WHAT TO MOUNT IN DOCKER:
  -v /path/to/porto_raceline.npz:/tmp/porto_raceline.npz

NPZ file is generated once offline by generate_porto_npz.py.

COMPETITION NOTE:
  /autodrive/roboracer_1/ips    — RESTRICTED (not available)
  /autodrive/roboracer_1/speed  — no publisher in current devkit build
  /tracked_pose                 — from Cartographer running in ForzaETH container
                                  on ROS_DOMAIN_ID=48, same network
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import csv
import time
import os

# ── Path to pre-generated raceline ───────────────────────────────────────────
NPZ_PATH = '/tmp/porto_raceline.npz'

# ── Vehicle constants (AutoDRIVE RoboRacer spec) ──────────────────────────────
WHEELBASE              = 0.33          # m  (spec says 0.324, use 0.33 for controller)
MAX_SPEED              = 8.0           # m/s

# ── Pure Pursuit parameters (from local_map_pp_v2, proven working) ────────────
CONSTANT_LOOKAHEAD     = 0.5           # m  — base lookahead for raceline PP
VARIABLE_LOOKAHEAD     = 2.0           # m  — speed-dependent additional lookahead
MAX_STEER_RAD          = 0.4           # rad — steering clip

# ── AutoDRIVE actuator conversion ─────────────────────────────────────────────
MAX_STEERING_ANGLE_RAD = 0.5236        # rad → normalised ±1.0
THROTTLE_SPEED_GAIN    = 0.040          # throttle = speed_mps * gain
MAX_THROTTLE           = 1.0           # hard clamp

# ── Speed lookahead for velocity profile lookup ───────────────────────────────
CURVATURE_LOOKAHEAD_M  = 2.5           # m ahead to find minimum speed

# ── Yaw validation (guard against Cartographer jitter) ───────────────────────
YAW_REJECT_DEG         = 60.0          # reject quaternion if yaw jumps > this

# ── QoS ───────────────────────────────────────────────────────────────────────
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1)
DEFAULT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10)


def _quat_to_yaw(x, y, z, w) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _get_steering_actuation(lookahead_point, lookahead_distance, wheelbase) -> float:
    """Pure Pursuit formula — identical to local_map_pp_v2.get_local_steering_actuation()."""
    waypoint_y = lookahead_point[1]
    if abs(waypoint_y) < 1e-6:
        return 0.0
    radius = 1.0 / (2.0 * waypoint_y / lookahead_distance ** 2)
    return float(np.arctan(wheelbase / radius))


class LocalMapPPCartographer(Node):
    def __init__(self):
        super().__init__('local_map_pp_cartographer')

        # ── Load pre-computed raceline ────────────────────────────────────────
        self.get_logger().info(f'Loading raceline from {NPZ_PATH} ...')
        data = np.load(NPZ_PATH)
        self.track_map_np     = data['points']            # (N,2) x,y in map frame
        self.track_curvatures = data['curvatures']        # (N,)  |kappa| [rad/m]
        self.track_map_s      = data['arc_length']        # (N,)  cumulative s [m]
        self.velocity_profile = data['velocity_profile']  # (N,)  [m/s]

        n = len(self.track_map_np)
        self.get_logger().info(
            f'[NPZ] {n} waypoints loaded. '
            f'Track={self.track_map_s[-1]:.1f}m  '
            f'Speed=[{self.velocity_profile.min():.2f}, {self.velocity_profile.max():.2f}] m/s')

        # ── Localisation state ────────────────────────────────────────────────
        self.global_x     = 0.0
        self.global_y     = 0.0
        self.heading      = 0.0    # rad, in Cartographer map frame
        self._prev_yaw    = None
        self._prev_pose_x = None
        self._prev_pose_y = None
        self._pose_received = False

        # ── Speed state ───────────────────────────────────────────────────────
        self.current_speed = 0.0   # m/s

        # ── Frame counter ─────────────────────────────────────────────────────
        self.frame_count = 0

        # ── Telemetry ─────────────────────────────────────────────────────────
        tel_path = '/tmp/telemetry/race_telemetry_cartographer.csv'
        os.makedirs(os.path.dirname(tel_path), exist_ok=True)
        self._tel_file   = open(tel_path, 'w', newline='')
        self._tel_writer = csv.writer(self._tel_file)
        self._tel_writer.writerow([
            'timestamp', 'frame', 'speed', 'steering_cmd', 'throttle_cmd',
            'steering_angle_deg', 'cross_track_error', 'target_speed',
            'car_x', 'car_y', 'heading_deg'])
        self._start_time = time.time()

        # ── Publishers ────────────────────────────────────────────────────────
        self._throttle_pub = self.create_publisher(
            Float32, '/autodrive/roboracer_1/throttle_command', DEFAULT_QOS)
        self._steering_pub = self.create_publisher(
            Float32, '/autodrive/roboracer_1/steering_command', DEFAULT_QOS)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            LaserScan, '/autodrive/roboracer_1/lidar',
            self._lidar_cb, RELIABLE_QOS)

        self.create_subscription(
            PoseStamped, '/tracked_pose',
            self._tracked_pose_cb, DEFAULT_QOS)

        self.create_subscription(
            Float32, '/autodrive/roboracer_1/speed',
            self._speed_cb, RELIABLE_QOS)

        self.get_logger().info(
            '[local_map_pp_cartographer] Ready.\n'
            '  Localisation : /tracked_pose (Cartographer map frame)\n'
            f'  Raceline     : {NPZ_PATH}\n'
            '  Control loop : LiDAR callback @ 40Hz\n'
            '  IPS          : NOT USED (restricted in competition)')

    # ── Localisation callback ─────────────────────────────────────────────────

    def _tracked_pose_cb(self, msg: PoseStamped) -> None:
        new_x = msg.pose.position.x
        new_y = msg.pose.position.y
        q     = msg.pose.orientation
        quat_yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)

        if self._prev_yaw is not None:
            delta_deg = abs(math.degrees(
                math.atan2(math.sin(quat_yaw - self._prev_yaw),
                           math.cos(quat_yaw - self._prev_yaw))))
            if delta_deg > YAW_REJECT_DEG:
                # Fallback: derive heading from position displacement
                if self._prev_pose_x is not None:
                    dx = new_x - self._prev_pose_x
                    dy = new_y - self._prev_pose_y
                    dist = math.sqrt(dx * dx + dy * dy)
                    if dist > 0.05:
                        self.heading = math.atan2(dy, dx)
            else:
                self.heading = quat_yaw
        else:
            self.heading = quat_yaw

        self.global_x     = new_x
        self.global_y     = new_y
        self._prev_yaw    = self.heading
        self._prev_pose_x = new_x
        self._prev_pose_y = new_y
        self._pose_received = True

    def _speed_cb(self, msg: Float32) -> None:
        self.current_speed = float(msg.data)

    # ── Control loop ──────────────────────────────────────────────────────────

    def _lidar_cb(self, msg: LaserScan) -> None:
        self.frame_count += 1

        if self.frame_count < 10:
            self._publish_commands(0.0, 0.0)
            return

        if not self._pose_received:
            if self.frame_count % 40 == 0:
                self.get_logger().warn('[WAIT] No /tracked_pose received yet...')
            self._publish_commands(0.0, 0.0)
            return

        steering_angle, target_speed = self._pure_pursuit_global_map()

        steering_cmd = float(np.clip(steering_angle / MAX_STEERING_ANGLE_RAD, -1.0, 1.0))
        throttle_cmd = float(np.clip(target_speed * THROTTLE_SPEED_GAIN, 0.0, MAX_THROTTLE))

        self._publish_commands(throttle_cmd, steering_cmd)

        if self.frame_count % 5 == 0:
            cte = getattr(self, '_last_cte', -1.0)
            ts  = getattr(self, '_last_target_speed', 0.0)
            self._tel_writer.writerow([
                f'{time.time() - self._start_time:.3f}',
                self.frame_count,
                f'{self.current_speed:.3f}',
                f'{steering_cmd:.4f}',
                f'{throttle_cmd:.4f}',
                f'{math.degrees(steering_angle):.2f}',
                f'{cte:.4f}',
                f'{ts:.2f}',
                f'{self.global_x:.3f}',
                f'{self.global_y:.3f}',
                f'{math.degrees(self.heading):.1f}',
            ])
            self._tel_file.flush()

        if self.frame_count % 80 == 0:
            cte = getattr(self, '_last_cte', -1.0)
            ts  = getattr(self, '_last_target_speed', 0.0)
            self.get_logger().info(
                f'[RACE] pos=({self.global_x:.2f},{self.global_y:.2f}) '
                f'yaw={math.degrees(self.heading):.1f}° | '
                f'CTE={cte:.3f}m | '
                f'target={ts:.2f}m/s actual={self.current_speed:.2f}m/s | '
                f'steer={steering_cmd:.3f} throttle={throttle_cmd:.4f}')

    # ── Pure Pursuit ──────────────────────────────────────────────────────────

    def _pure_pursuit_global_map(self):
        lookahead_dist = CONSTANT_LOOKAHEAD + (self.current_speed / MAX_SPEED) * VARIABLE_LOOKAHEAD

        dists = np.sqrt(
            (self.track_map_np[:, 0] - self.global_x) ** 2 +
            (self.track_map_np[:, 1] - self.global_y) ** 2)
        nearest_idx = int(np.argmin(dists))
        self._last_cte = float(dists[nearest_idx])
        current_s = float(self.track_map_s[nearest_idx])

        total_s  = float(self.track_map_s[-1])
        target_s = current_s + lookahead_dist
        if target_s > total_s:
            target_s -= total_s

        n_pts = len(self.track_map_np)
        la_idx = int(np.searchsorted(self.track_map_s, target_s)) - 1
        if la_idx < 0:
            la_idx = 0
        la_idx_next = (la_idx + 1) % n_pts

        s_la   = float(self.track_map_s[la_idx])
        s_next = float(self.track_map_s[la_idx_next])
        if la_idx_next == 0:
            s_next = total_s

        seg_len = s_next - s_la
        frac    = float(np.clip((target_s - s_la) / seg_len, 0.0, 1.0)) \
                  if seg_len > 0.01 else 0.0

        la_world = ((1.0 - frac) * self.track_map_np[la_idx] +
                    frac          * self.track_map_np[la_idx_next])

        dx    = la_world[0] - self.global_x
        dy    = la_world[1] - self.global_y
        cos_h = math.cos(self.heading)
        sin_h = math.sin(self.heading)
        local_x =  cos_h * dx + sin_h * dy
        local_y = -sin_h * dx + cos_h * dy

        true_la = math.sqrt(local_x ** 2 + local_y ** 2)
        if true_la < 0.01:
            steering_angle = 0.0
        else:
            steering_angle = _get_steering_actuation(
                np.array([local_x, local_y]), true_la, WHEELBASE)
        steering_angle = float(np.clip(steering_angle, -MAX_STEER_RAD, MAX_STEER_RAD))

        target_speed = self._get_profiled_speed(nearest_idx)
        self._last_target_speed = target_speed

        return steering_angle, target_speed

    def _get_profiled_speed(self, nearest_idx: int) -> float:
        current_s   = float(self.track_map_s[nearest_idx])
        lookahead_s = current_s + CURVATURE_LOOKAHEAD_M
        total_s     = float(self.track_map_s[-1])

        if lookahead_s > total_s:
            mask = (self.track_map_s >= current_s) | (self.track_map_s <= lookahead_s - total_s)
        else:
            mask = (self.track_map_s >= current_s) & (self.track_map_s <= lookahead_s)

        if np.sum(mask) == 0:
            return float(self.velocity_profile[nearest_idx])
        return float(np.min(self.velocity_profile[mask]))

    # ── Publish ───────────────────────────────────────────────────────────────

    def _publish_commands(self, throttle: float, steering: float) -> None:
        th = Float32(); th.data = float(throttle)
        st = Float32(); st.data = float(steering)
        self._throttle_pub.publish(th)
        self._steering_pub.publish(st)

    def destroy_node(self):
        self._publish_commands(0.0, 0.0)
        self._tel_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LocalMapPPCartographer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()