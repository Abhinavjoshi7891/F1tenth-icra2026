#!/usr/bin/env python3
"""
AutoDRIVE Pipeline Logger — full end-to-end transparency.

Shows every layer of the data pipeline in human-readable form:
  Layer 1: Raw AutoDRIVE sim data (what the devkit bridge receives from sim)
  Layer 2: Translated ROS topics (what the devkit bridge publishes)
  Layer 3: ForzaETH adapter translations (what the adapter produces)
  Layer 4: ForzaETH stack outputs (localisation, waypoints)
  Layer 5: Controller decisions (what control is being sent)
  Layer 6: Commands going back to sim

Run in ForzaETH container (sec terminal):
  source ~/ws/install/setup.bash
  python3 ~/ws/src/race_stack/utilities/nodes/pipeline_logger.py

Prints a formatted status block every ~1 second.
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu, JointState
from std_msgs.msg import Float32, Int32
from ackermann_msgs.msg import AckermannDriveStamped

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)
DEFAULT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=10)

def quat_to_yaw_deg(x, y, z, w):
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny, cosy))

def age(t):
    """Return age of timestamp in ms. t is float seconds since node start."""
    if t is None:
        return 'NEVER'
    return f'{(time.monotonic() - t)*1000:.0f}ms ago'

class PipelineLogger(Node):
    def __init__(self):
        super().__init__('pipeline_logger')

        # ── Layer 1+2: Raw AutoDRIVE sim → devkit bridge → ROS topics ─────────
        # These come from the devkit bridge (autodrive_roboracer container)
        self.sim_speed         = None   # /autodrive/roboracer_1/speed (m/s)
        self.sim_speed_t       = None
        self.sim_steer_fb      = None   # /autodrive/roboracer_1/steering (normalised, feedback)
        self.sim_steer_fb_t    = None
        self.sim_throttle_fb   = None   # /autodrive/roboracer_1/throttle (normalised, feedback)
        self.sim_throttle_fb_t = None
        self.sim_ips_x         = None   # /autodrive/roboracer_1/ips (GPS-like position)
        self.sim_ips_y         = None
        self.sim_ips_t         = None
        self.sim_lidar_min     = None   # /autodrive/roboracer_1/lidar min range
        self.sim_lidar_front   = None   # forward-facing beam
        self.sim_lidar_t       = None
        self.sim_imu_yaw_rate  = None   # /autodrive/roboracer_1/imu angular_velocity.z
        self.sim_imu_ax        = None   # linear_acceleration.x
        self.sim_imu_t         = None
        self.sim_left_enc      = None   # /autodrive/roboracer_1/left_encoder (rad, cumulative)
        self.sim_right_enc     = None
        self.sim_enc_t         = None
        self.sim_lap_count     = None
        self.sim_lap_t         = None

        # ── Layer 3: Adapter → ForzaETH translation ───────────────────────────
        self.odom_x            = None   # /odom position
        self.odom_y            = None
        self.odom_yaw_deg      = None
        self.odom_t            = None
        self.scan_min          = None   # /scan min range (relayed lidar)
        self.scan_t            = None

        # ── Layer 4: ForzaETH stack (localisation + planning) ─────────────────
        self.carstate_x        = None   # /car_state/pose
        self.carstate_y        = None
        self.carstate_yaw_deg  = None
        self.carstate_t        = None
        self.tracked_x         = None   # /tracked_pose (direct Cartographer)
        self.tracked_y         = None
        self.tracked_yaw_deg   = None
        self.tracked_t         = None

        # ── Layer 5: Controller command (/drive from ForzaETH controller) ─────
        self.drive_speed       = None   # m/s commanded
        self.drive_steer_rad   = None   # rad commanded
        self.drive_t           = None

        # ── Layer 6: Commands going back to sim ───────────────────────────────
        self.cmd_throttle      = None   # /autodrive/roboracer_1/throttle_command
        self.cmd_throttle_t    = None
        self.cmd_steer         = None   # /autodrive/roboracer_1/steering_command
        self.cmd_steer_t       = None

        # ── Subscriptions ─────────────────────────────────────────────────────
        # Layer 1+2: Raw sim data from devkit bridge
        self.create_subscription(Float32,    '/autodrive/roboracer_1/speed',
                                 self._speed_cb,       RELIABLE_QOS)
        self.create_subscription(Float32,    '/autodrive/roboracer_1/steering',
                                 self._steer_fb_cb,    RELIABLE_QOS)
        self.create_subscription(Float32,    '/autodrive/roboracer_1/throttle',
                                 self._throttle_fb_cb, RELIABLE_QOS)
        self.create_subscription(LaserScan,  '/autodrive/roboracer_1/lidar',
                                 self._sim_lidar_cb,   RELIABLE_QOS)
        self.create_subscription(Imu,        '/autodrive/roboracer_1/imu',
                                 self._sim_imu_cb,     RELIABLE_QOS)
        self.create_subscription(JointState, '/autodrive/roboracer_1/left_encoder',
                                 self._left_enc_cb,    RELIABLE_QOS)
        self.create_subscription(JointState, '/autodrive/roboracer_1/right_encoder',
                                 self._right_enc_cb,   RELIABLE_QOS)
        self.create_subscription(Int32,      '/autodrive/roboracer_1/lap_count',
                                 self._lap_cb,         RELIABLE_QOS)

        # IPS — uses Point, but bridge publishes as geometry_msgs/Point
        from geometry_msgs.msg import Point
        self.create_subscription(Point, '/autodrive/roboracer_1/ips',
                                 self._ips_cb, RELIABLE_QOS)

        # Layer 3: Adapter outputs
        self.create_subscription(Odometry,  '/odom',  self._odom_cb,  DEFAULT_QOS)
        self.create_subscription(LaserScan, '/scan',  self._scan_cb,  RELIABLE_QOS)

        # Layer 4: ForzaETH stack
        self.create_subscription(PoseStamped, '/car_state/pose',
                                 self._carstate_cb, DEFAULT_QOS)
        self.create_subscription(PoseStamped, '/tracked_pose',
                                 self._tracked_cb,  DEFAULT_QOS)

        # Layer 5: /drive command
        self.create_subscription(AckermannDriveStamped, '/drive',
                                 self._drive_cb, DEFAULT_QOS)

        # Layer 6: Commands to sim
        self.create_subscription(Float32, '/autodrive/roboracer_1/throttle_command',
                                 self._cmd_throttle_cb, DEFAULT_QOS)
        self.create_subscription(Float32, '/autodrive/roboracer_1/steering_command',
                                 self._cmd_steer_cb, DEFAULT_QOS)

        # Print timer: once per second
        self.create_timer(1.0, self._print_status)
        self._start = time.monotonic()
        self.get_logger().info('[pipeline_logger] Started. Status will print every 1s.')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _speed_cb(self, msg):
        self.sim_speed   = msg.data
        self.sim_speed_t = time.monotonic()

    def _steer_fb_cb(self, msg):
        self.sim_steer_fb   = msg.data
        self.sim_steer_fb_t = time.monotonic()

    def _throttle_fb_cb(self, msg):
        self.sim_throttle_fb   = msg.data
        self.sim_throttle_fb_t = time.monotonic()

    def _sim_lidar_cb(self, msg):
        import numpy as np
        ranges = np.array(msg.ranges)
        valid  = ranges[ranges > 0.06]
        self.sim_lidar_min   = float(valid.min()) if len(valid) else 0.0
        n = len(msg.ranges)
        center = msg.ranges[n // 2]
        self.sim_lidar_front = center if (center > 0.06 and center < 30.0) else 0.0
        self.sim_lidar_t     = time.monotonic()

    def _sim_imu_cb(self, msg):
        self.sim_imu_yaw_rate = msg.angular_velocity.z
        self.sim_imu_ax       = msg.linear_acceleration.x
        self.sim_imu_t        = time.monotonic()

    def _left_enc_cb(self, msg):
        if msg.position:
            self.sim_left_enc = msg.position[0]

    def _right_enc_cb(self, msg):
        if msg.position:
            self.sim_right_enc = msg.position[0]
            self.sim_enc_t     = time.monotonic()

    def _lap_cb(self, msg):
        self.sim_lap_count = msg.data
        self.sim_lap_t     = time.monotonic()

    def _ips_cb(self, msg):
        self.sim_ips_x = msg.x
        self.sim_ips_y = msg.y
        self.sim_ips_t = time.monotonic()

    def _odom_cb(self, msg):
        self.odom_x   = msg.pose.pose.position.x
        self.odom_y   = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.odom_yaw_deg = quat_to_yaw_deg(q.x, q.y, q.z, q.w)
        self.odom_t   = time.monotonic()

    def _scan_cb(self, msg):
        import numpy as np
        ranges = np.array(msg.ranges)
        valid  = ranges[ranges > 0.06]
        self.scan_min = float(valid.min()) if len(valid) else 0.0
        self.scan_t   = time.monotonic()

    def _carstate_cb(self, msg):
        self.carstate_x   = msg.pose.position.x
        self.carstate_y   = msg.pose.position.y
        q = msg.pose.orientation
        self.carstate_yaw_deg = quat_to_yaw_deg(q.x, q.y, q.z, q.w)
        self.carstate_t   = time.monotonic()

    def _tracked_cb(self, msg):
        self.tracked_x   = msg.pose.position.x
        self.tracked_y   = msg.pose.position.y
        q = msg.pose.orientation
        self.tracked_yaw_deg = quat_to_yaw_deg(q.x, q.y, q.z, q.w)
        self.tracked_t   = time.monotonic()

    def _drive_cb(self, msg):
        self.drive_speed     = msg.drive.speed
        self.drive_steer_rad = msg.drive.steering_angle
        self.drive_t         = time.monotonic()

    def _cmd_throttle_cb(self, msg):
        self.cmd_throttle   = msg.data
        self.cmd_throttle_t = time.monotonic()

    def _cmd_steer_cb(self, msg):
        self.cmd_steer   = msg.data
        self.cmd_steer_t = time.monotonic()

    # ── Printer ───────────────────────────────────────────────────────────────

    def f(self, val, fmt='.3f', unit='', na='---'):
        return (f'{val:{fmt}}{unit}') if val is not None else na

    def _print_status(self):
        sep = '─' * 70
        uptime = time.monotonic() - self._start

        # Yaw agreement check
        yaw_ok = '✓'
        if self.carstate_yaw_deg is not None and self.tracked_yaw_deg is not None:
            delta = abs(self.carstate_yaw_deg - self.tracked_yaw_deg)
            delta = min(delta, 360 - delta)
            if delta > 10.0:
                yaw_ok = f'⚠ MISMATCH {delta:.1f}°'

        print(f'\n{sep}')
        print(f'  PIPELINE STATUS  |  uptime={uptime:.0f}s')
        print(sep)

        print('  LAYER 1+2 │ AutoDRIVE Sim → Devkit Bridge → ROS Topics')
        print(f'    Speed        : {self.f(self.sim_speed, ".2f", " m/s")}  [{age(self.sim_speed_t)}]')
        print(f'    Steering fb  : {self.f(self.sim_steer_fb, ".3f", " norm")}  [{age(self.sim_steer_fb_t)}]')
        print(f'    Throttle fb  : {self.f(self.sim_throttle_fb, ".3f", " norm")}  [{age(self.sim_throttle_fb_t)}]')
        print(f'    IPS position : ({self.f(self.sim_ips_x, ".2f")}, {self.f(self.sim_ips_y, ".2f")}) m  [{age(self.sim_ips_t)}]')
        print(f'    LiDAR        : min={self.f(self.sim_lidar_min, ".2f", "m")}  front={self.f(self.sim_lidar_front, ".2f", "m")}  [{age(self.sim_lidar_t)}]')
        print(f'    IMU          : yaw_rate={self.f(self.sim_imu_yaw_rate, ".3f", " rad/s")}  ax={self.f(self.sim_imu_ax, ".2f", " m/s²")}  [{age(self.sim_imu_t)}]')
        enc_l = f'{self.sim_left_enc:.4f} rad' if self.sim_left_enc is not None else '---'
        enc_r = f'{self.sim_right_enc:.4f} rad' if self.sim_right_enc is not None else '---'
        print(f'    Encoders     : L={enc_l}  R={enc_r}  [{age(self.sim_enc_t)}]')
        print(f'    Lap count    : {self.f(self.sim_lap_count, ".0f")}  [{age(self.sim_lap_t)}]')

        print(f'  LAYER 3   │ Adapter → ForzaETH Translation')
        print(f'    /odom        : pos=({self.f(self.odom_x, ".2f")}, {self.f(self.odom_y, ".2f")}) m  yaw={self.f(self.odom_yaw_deg, ".1f", "°")}  [{age(self.odom_t)}]')
        print(f'    /scan        : min_range={self.f(self.scan_min, ".2f", "m")}  [{age(self.scan_t)}]')

        print(f'  LAYER 4   │ ForzaETH Stack (Localisation)')
        print(f'    /car_state/pose : pos=({self.f(self.carstate_x, ".2f")}, {self.f(self.carstate_y, ".2f")}) m  yaw={self.f(self.carstate_yaw_deg, ".1f", "°")}  [{age(self.carstate_t)}]')
        print(f'    /tracked_pose   : pos=({self.f(self.tracked_x, ".2f")}, {self.f(self.tracked_y, ".2f")}) m  yaw={self.f(self.tracked_yaw_deg, ".1f", "°")}  [{age(self.tracked_t)}]')
        print(f'    Yaw agreement   : {yaw_ok}')

        print(f'  LAYER 5   │ Controller Decision (/drive from ForzaETH controller node)')
        if self.drive_t is not None:
            steer_deg = math.degrees(self.drive_steer_rad) if self.drive_steer_rad is not None else 0
            steer_norm = self.drive_steer_rad / 0.5236 if self.drive_steer_rad is not None else 0
            print(f'    Commanded speed  : {self.f(self.drive_speed, ".2f", " m/s")}')
            print(f'    Commanded steer  : {self.f(self.drive_steer_rad, ".3f", " rad")} = {steer_deg:.1f}° = {steer_norm:.3f} norm')
            print(f'    Last update      : {age(self.drive_t)}')
        else:
            print('    /drive           : NOT RECEIVING — controller not running?')

        print(f'  LAYER 6   │ Commands → Sim (via Adapter/Controller)')
        throttle_ms = self.cmd_throttle * 22.88 if self.cmd_throttle is not None else None
        print(f'    throttle_command : {self.f(self.cmd_throttle, ".4f", " norm")} ≈ {self.f(throttle_ms, ".2f", " m/s equiv")}  [{age(self.cmd_throttle_t)}]')
        print(f'    steering_command : {self.f(self.cmd_steer, ".4f", " norm")} = {self.f((self.cmd_steer or 0)*0.5236, ".3f", " rad")}  [{age(self.cmd_steer_t)}]')

        print(sep)

        # ISSUE FLAGS
        issues = []
        if self.sim_speed_t is None:
            issues.append('⚠  /autodrive/roboracer_1/speed has NO PUBLISHER — speed feedback unavailable')
        if self.sim_ips_t is None:
            issues.append('⚠  /autodrive/roboracer_1/ips not receiving — IPS unavailable')
        if self.carstate_t and self.tracked_t:
            delta = abs((self.carstate_yaw_deg or 0) - (self.tracked_yaw_deg or 0))
            delta = min(delta, 360 - delta)
            if delta > 10:
                issues.append(f'⚠  YAW MISMATCH: car_state={self.carstate_yaw_deg:.1f}° vs tracked={self.tracked_yaw_deg:.1f}° (delta={delta:.1f}°) — EKF is corrupting orientation')
        if self.odom_x is not None and abs(self.odom_x) > 50:
            issues.append(f'⚠  ODOM DRIFT: odom_x={self.odom_x:.1f}m — encoder dead-reckoning has drifted far')
        if self.cmd_throttle_t is None:
            issues.append('⚠  throttle_command not being sent — controller not running?')
        if issues:
            print('  ISSUES:')
            for i in issues:
                print(f'    {i}')
            print(sep)


def main(args=None):
    rclpy.init(args=args)
    node = PipelineLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()