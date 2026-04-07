#!/usr/bin/env python3
"""
cartographer_pp_monitor.py

Diagnostic monitor for the local_map_pp_cartographer setup.
Prints a status block every 1s showing exactly what each layer
is doing and flagging any known failure modes.

Run in ANY terminal on ROS_DOMAIN_ID=48 (host or any container):
  source /opt/ros/humble/setup.bash
  python3 cartographer_pp_monitor.py

Layers monitored:
  SIM      : AutoDRIVE sim outputs (speed, lidar, encoders)
  ADAPTER  : adapter_node outputs (/scan, /odom)
  LOCALISE : Cartographer /tracked_pose
  CONTROL  : throttle_command, steering_command going to sim
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
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
    if t is None:
        return 'NEVER'
    dt = (time.monotonic() - t) * 1000
    if dt > 2000:
        return f'STALE {dt/1000:.1f}s'
    return f'{dt:.0f}ms'


def fmt(val, spec='.3f', unit='', na='---'):
    return f'{val:{spec}}{unit}' if val is not None else na


class CartographerPPMonitor(Node):
    def __init__(self):
        super().__init__('cartographer_pp_monitor')

        # SIM layer
        self.sim_speed        = None; self.sim_speed_t    = None
        self.sim_lidar_front  = None; self.sim_lidar_min  = None
        self.sim_lidar_t      = None

        # ADAPTER layer
        self.odom_x = None; self.odom_y = None
        self.odom_vx = None; self.odom_t = None
        self.scan_min = None; self.scan_t = None

        # LOCALISATION layer
        self.pose_x   = None; self.pose_y   = None
        self.pose_yaw = None; self.pose_t   = None
        self._pose_history = []   # last 5 yaw values for jitter measurement

        # CONTROLLER layer (ForzaETH /drive — if running full stack)
        self.drive_speed    = None; self.drive_steer = None; self.drive_t = None

        # COMMANDS layer (what actually goes to sim)
        self.cmd_throttle   = None; self.cmd_throttle_t = None
        self.cmd_steer      = None; self.cmd_steer_t    = None

        # Derived stats
        self._last_pose_x   = None; self._last_pose_y = None
        self._last_pose_t   = None
        self.estimated_speed = None   # from position diff

        # Subscriptions
        self.create_subscription(Float32,   '/autodrive/roboracer_1/speed',
                                 self._sim_speed_cb,   RELIABLE_QOS)
        self.create_subscription(LaserScan, '/autodrive/roboracer_1/lidar',
                                 self._sim_lidar_cb,   RELIABLE_QOS)
        self.create_subscription(Odometry,  '/odom',
                                 self._odom_cb,         DEFAULT_QOS)
        self.create_subscription(LaserScan, '/scan',
                                 self._scan_cb,         RELIABLE_QOS)
        self.create_subscription(PoseStamped, '/tracked_pose',
                                 self._pose_cb,         DEFAULT_QOS)
        self.create_subscription(AckermannDriveStamped, '/drive',
                                 self._drive_cb,        DEFAULT_QOS)
        self.create_subscription(Float32, '/autodrive/roboracer_1/throttle_command',
                                 self._cmd_throttle_cb, DEFAULT_QOS)
        self.create_subscription(Float32, '/autodrive/roboracer_1/steering_command',
                                 self._cmd_steer_cb,    DEFAULT_QOS)

        self.create_timer(1.0, self._print)
        self._start = time.monotonic()
        self.get_logger().info('[cartographer_pp_monitor] Running. Printing every 1s.')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _sim_speed_cb(self, msg):
        self.sim_speed   = float(msg.data)
        self.sim_speed_t = time.monotonic()

    def _sim_lidar_cb(self, msg):
        import numpy as np
        r = np.array(msg.ranges)
        valid = r[(r > 0.06) & np.isfinite(r)]
        self.sim_lidar_min   = float(valid.min()) if len(valid) else None
        n = len(msg.ranges)
        front = msg.ranges[n // 2]
        self.sim_lidar_front = float(front) if (0.06 < front < 30.0) else None
        self.sim_lidar_t     = time.monotonic()

    def _odom_cb(self, msg):
        self.odom_x  = msg.pose.pose.position.x
        self.odom_y  = msg.pose.pose.position.y
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_t  = time.monotonic()

    def _scan_cb(self, msg):
        import numpy as np
        r = np.array(msg.ranges)
        valid = r[(r > 0.06) & np.isfinite(r)]
        self.scan_min = float(valid.min()) if len(valid) else None
        self.scan_t   = time.monotonic()

    def _pose_cb(self, msg):
        now = time.monotonic()
        self.pose_x   = msg.pose.position.x
        self.pose_y   = msg.pose.position.y
        q = msg.pose.orientation
        self.pose_yaw = quat_to_yaw_deg(q.x, q.y, q.z, q.w)
        self.pose_t   = now

        # Track yaw history for jitter measurement
        self._pose_history.append(self.pose_yaw)
        if len(self._pose_history) > 10:
            self._pose_history.pop(0)

        # Estimate speed from position diff
        if self._last_pose_x is not None and self._last_pose_t is not None:
            dt = now - self._last_pose_t
            if dt > 0.01:
                dx = self.pose_x - self._last_pose_x
                dy = self.pose_y - self._last_pose_y
                self.estimated_speed = math.sqrt(dx*dx + dy*dy) / dt
        self._last_pose_x = self.pose_x
        self._last_pose_y = self.pose_y
        self._last_pose_t = now

    def _drive_cb(self, msg):
        self.drive_speed = msg.drive.speed
        self.drive_steer = msg.drive.steering_angle
        self.drive_t     = time.monotonic()

    def _cmd_throttle_cb(self, msg):
        self.cmd_throttle   = float(msg.data)
        self.cmd_throttle_t = time.monotonic()

    def _cmd_steer_cb(self, msg):
        self.cmd_steer   = float(msg.data)
        self.cmd_steer_t = time.monotonic()

    # ── Printer ───────────────────────────────────────────────────────────────

    def _print(self):
        sep = '─' * 68
        uptime = time.monotonic() - self._start
        print(f'\n{sep}')
        print(f'  CARTOGRAPHER PP MONITOR  |  uptime={uptime:.0f}s')
        print(sep)

        # SIM
        print('  [SIM]      AutoDRIVE outputs')
        print(f'    /speed       : {fmt(self.sim_speed, ".3f", " m/s")}  [{age(self.sim_speed_t)}]')
        print(f'    /lidar       : front={fmt(self.sim_lidar_front, ".2f", "m")}  min={fmt(self.sim_lidar_min, ".2f", "m")}  [{age(self.sim_lidar_t)}]')

        # ADAPTER
        print('  [ADAPTER]  adapter_node outputs')
        print(f'    /odom        : pos=({fmt(self.odom_x, ".2f")}, {fmt(self.odom_y, ".2f")}) m  vx={fmt(self.odom_vx, ".2f", " m/s")}  [{age(self.odom_t)}]')
        print(f'    /scan        : min={fmt(self.scan_min, ".2f", "m")}  [{age(self.scan_t)}]')

        # LOCALISATION
        print('  [LOCALISE] /tracked_pose (Cartographer)')
        print(f'    pose         : ({fmt(self.pose_x, ".3f")}, {fmt(self.pose_y, ".3f")}) m  yaw={fmt(self.pose_yaw, ".1f", "°")}  [{age(self.pose_t)}]')
        print(f'    speed est.   : {fmt(self.estimated_speed, ".2f", " m/s")} (from pos diff)')

        # Yaw jitter
        if len(self._pose_history) >= 3:
            yaw_range = max(self._pose_history) - min(self._pose_history)
            jitter_flag = '  ⚠ HIGH JITTER' if yaw_range > 30 else '  ✓'
            print(f'    yaw jitter   : {yaw_range:.1f}° over last {len(self._pose_history)} samples{jitter_flag}')

        # CONTROLLER /drive (ForzaETH full stack only — may be absent)
        print('  [/drive]   ForzaETH controller output (absent if using cartographer PP directly)')
        if self.drive_t is not None:
            print(f'    speed cmd    : {fmt(self.drive_speed, ".2f", " m/s")}  steer={fmt(self.drive_steer, ".3f", " rad")}  [{age(self.drive_t)}]')
        else:
            print('    /drive       : NOT RECEIVING (expected if cartographer PP is the controller)')

        # COMMANDS
        print('  [COMMANDS] Going to sim')
        steer_rad = (self.cmd_steer or 0.0) * 0.5236
        throttle_equiv = (self.cmd_throttle or 0.0) / 0.05  # reverse of THROTTLE_SPEED_GAIN
        at_limit = '  ⚠ AT STEERING LIMIT' if self.cmd_steer is not None and abs(self.cmd_steer) > 0.75 else ''
        print(f'    throttle_cmd : {fmt(self.cmd_throttle, ".4f", " norm")} ≈ {throttle_equiv:.1f} m/s target  [{age(self.cmd_throttle_t)}]')
        print(f'    steering_cmd : {fmt(self.cmd_steer, ".4f", " norm")} = {steer_rad:.3f} rad{at_limit}  [{age(self.cmd_steer_t)}]')

        print(sep)

        # ISSUE FLAGS
        issues = []

        if self.sim_speed_t is None:
            issues.append('CRITICAL: /autodrive/roboracer_1/speed has NO PUBLISHER — speed=0.0 stuck, lookahead is minimum only')
        elif self.sim_speed is not None and self.sim_speed == 0.0 and self.estimated_speed is not None and self.estimated_speed > 0.3:
            issues.append(f'CRITICAL: speed=0.0 from sim but pos-diff shows {self.estimated_speed:.2f} m/s — speed topic broken')

        if self.pose_t is None:
            issues.append('CRITICAL: /tracked_pose not receiving — Cartographer not running or wrong domain ID')
        elif (time.monotonic() - self.pose_t) > 1.0:
            issues.append('CRITICAL: /tracked_pose STALE — Cartographer may have crashed')

        if self.cmd_steer is not None and abs(self.cmd_steer) > 0.75:
            issues.append(f'WARNING: steering at limit ({self.cmd_steer:.3f}) — heading error or frame mismatch')

        if len(self._pose_history) >= 3:
            yaw_range = max(self._pose_history) - min(self._pose_history)
            if yaw_range > 30:
                issues.append(f'WARNING: yaw jitter {yaw_range:.1f}° — Cartographer heading unstable at current speed')

        if self.scan_t is None:
            issues.append('WARNING: /scan not receiving — adapter_node may not be running')

        if self.odom_t is None:
            issues.append('WARNING: /odom not receiving — adapter_node may not be running')

        if self.cmd_throttle_t is None:
            issues.append('WARNING: throttle_command not publishing — controller not running')

        if issues:
            print('  ISSUES:')
            for i in issues:
                print(f'    ⚠  {i}')
            print(sep)


def main(args=None):
    rclpy.init(args=args)
    node = CartographerPPMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()