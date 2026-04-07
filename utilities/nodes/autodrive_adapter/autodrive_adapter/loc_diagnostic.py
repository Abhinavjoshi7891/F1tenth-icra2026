#!/usr/bin/env python3
"""
Localization diagnostic node.

Compares IPS ground truth to Cartographer's localization estimate in real-time.
Run this AFTER base_system is launched but BEFORE time_trials.

Usage:
  ros2 run autodrive_adapter loc_diagnostic
  (or just: python3 loc_diagnostic.py)

Output: Periodic log showing:
  - IPS ground truth (x, y)
  - Cartographer estimate (x, y, yaw)
  - Odom dead-reckoning (x, y)
  - Early fusion odom (x, y)
  - Position error (IPS vs Cartographer)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class LocDiagnostic(Node):
    def __init__(self):
        super().__init__('loc_diagnostic')

        # State
        self._ips_x = None
        self._ips_y = None
        self._ips_z = None

        self._carto_x = None
        self._carto_y = None
        self._carto_yaw = None

        self._odom_x = None
        self._odom_y = None
        self._odom_yaw = None
        self._odom_vx = None
        self._odom_wz = None

        self._ef_x = None
        self._ef_y = None
        self._ef_vx = None

        self._scan_count = 0

        # Subscribers
        self.create_subscription(
            Point, '/autodrive/roboracer_1/ips',
            self._ips_cb, RELIABLE_QOS)
        self.create_subscription(
            PoseStamped, '/tracked_pose',
            self._carto_cb, 10)
        self.create_subscription(
            Odometry, '/odom',
            self._odom_cb, 10)
        self.create_subscription(
            Odometry, '/early_fusion/odom',
            self._ef_cb, 10)

        # Log timer at 2 Hz
        self.create_timer(0.5, self._log_cb)

        # One-shot flag for initial report
        self._initial_reported = False

        self.get_logger().info('=== LOC DIAGNOSTIC STARTED ===')
        self.get_logger().info('Waiting for IPS + Cartographer data...')

    def _ips_cb(self, msg: Point):
        self._ips_x = msg.x
        self._ips_y = msg.y
        self._ips_z = msg.z

    def _carto_cb(self, msg: PoseStamped):
        self._carto_x = msg.pose.position.x
        self._carto_y = msg.pose.position.y
        self._carto_yaw = yaw_from_quat(msg.pose.orientation)

    def _odom_cb(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._odom_yaw = yaw_from_quat(msg.pose.pose.orientation)
        self._odom_vx = msg.twist.twist.linear.x
        self._odom_wz = msg.twist.twist.angular.z

    def _ef_cb(self, msg: Odometry):
        self._ef_x = msg.pose.pose.position.x
        self._ef_y = msg.pose.pose.position.y
        self._ef_vx = msg.twist.twist.linear.x

    def _log_cb(self):
        # Wait until we have at least IPS data
        if self._ips_x is None:
            self.get_logger().warn('No IPS data yet')
            return

        # Initial report (once)
        if not self._initial_reported:
            self.get_logger().info(
                f'\n'
                f'╔══════════════════════════════════════════════╗\n'
                f'║         INITIAL SPAWN POSITION               ║\n'
                f'║  IPS (ground truth): x={self._ips_x:+8.3f}  y={self._ips_y:+8.3f}  ║\n'
                f'╚══════════════════════════════════════════════╝')
            self._initial_reported = True

        # Build log line
        lines = []
        lines.append('--- LOC DIAG ---')

        # IPS ground truth
        lines.append(f'  IPS  (truth) : x={self._ips_x:+8.3f}  y={self._ips_y:+8.3f}')

        # Cartographer
        if self._carto_x is not None:
            lines.append(
                f'  CARTO (est)  : x={self._carto_x:+8.3f}  y={self._carto_y:+8.3f}  '
                f'yaw={math.degrees(self._carto_yaw):+7.1f}°')

            # Error
            dx = self._ips_x - self._carto_x
            dy = self._ips_y - self._carto_y
            err = math.sqrt(dx*dx + dy*dy)
            lines.append(f'  ERROR        : {err:8.3f} m  (dx={dx:+.3f} dy={dy:+.3f})')

            if err > 1.0:
                lines.append(f'  ⚠️  LOCALIZATION ERROR > 1m — CARTO IS LOST')
        else:
            lines.append('  CARTO        : NO DATA (not publishing /tracked_pose)')

        # Raw odom
        if self._odom_x is not None:
            lines.append(
                f'  ODOM (raw)   : x={self._odom_x:+8.3f}  y={self._odom_y:+8.3f}  '
                f'vx={self._odom_vx:+6.2f}  wz={self._odom_wz:+6.2f}')

        # Early fusion
        if self._ef_x is not None:
            lines.append(
                f'  EF ODOM      : x={self._ef_x:+8.3f}  y={self._ef_y:+8.3f}  '
                f'vx={self._ef_vx:+6.2f}')

        self.get_logger().info('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = LocDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()