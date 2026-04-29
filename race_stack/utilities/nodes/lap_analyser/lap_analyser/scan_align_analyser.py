#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rclpy.time
import rclpy.duration
import numpy as np
import tf2_ros
from tf_transformations import quaternion_matrix
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan
from f110_msgs.msg import WpntArray
from std_msgs.msg import Float32


class ScanAlignAnalyser(Node):
    """
    Checks if laser scan points overlap with the track boundaries and
    publishes an alignment score [%].
    """

    def __init__(self):
        super().__init__('scan_align_analyser')

        self.declare_parameter('debug', False)
        self.declare_parameter('downsample_factor', 54)
        self.declare_parameter('threshold', 0.1)
        self.declare_parameter('frequency', 5)

        self.debug_flag = self.get_parameter('debug').value
        self.downsample_factor = self.get_parameter('downsample_factor').value
        self.threshold = self.get_parameter('threshold').value
        frequency = self.get_parameter('frequency').value

        self.map = None
        self.trackbounds = None
        self.glb_wpnts = None
        self.scan = None
        self.first_visualization = True
        self.x_viz = None
        self.y_viz = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._laser_cb, 10)
        self.create_subscription(MarkerArray, '/trackbounds/markers', self._trackbounds_cb, 10)
        self.create_subscription(WpntArray, '/global_waypoints', self._glb_wpnts_cb, 10)

        self.debug_lidar_pub = self.create_publisher(MarkerArray, '/scanalign/debug_lidar', 1)
        self.alignment_score_pub = self.create_publisher(Float32, '/scanalign/alignment_score', 1)
        self.alignment_textmrk_pub = self.create_publisher(Marker, '/scanalign/alignment_textmrk', 1)

        self.create_timer(1.0 / frequency, self._loop)
        self.get_logger().info('ScanAlignAnalyser: waiting for scan and trackbounds...')

    def _map_cb(self, data):
        self.map = data

    def _laser_cb(self, data):
        self.scan = data

    def _trackbounds_cb(self, data):
        self.trackbounds = data

    def _glb_wpnts_cb(self, data):
        self.glb_wpnts = data.wpnts[:-1]

    def _loop(self):
        if self.scan is None or self.trackbounds is None:
            return

        try:
            xyz_mf = self._laser2map(self.scan)
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return

        if self.debug_flag:
            self._pub_map_lidarpoints(xyz_mf, self.scan.header.stamp)

        score = self._compute_alignment(xyz_mf, self.trackbounds)
        msg = Float32()
        msg.data = float(score)
        self.alignment_score_pub.publish(msg)
        self._pub_alignscore_txt(score)

    def _laser2map(self, laser: LaserScan):
        tf_msg = self.tf_buffer.lookup_transform(
            'map', 'laser',
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0),
        )
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        T = np.array([t.x, t.y, t.z])
        R = quaternion_matrix([q.x, q.y, q.z, q.w])

        valid = np.array(laser.ranges) > 0.1
        ranges = np.array(laser.ranges)[valid]
        angles = np.linspace(laser.angle_min, laser.angle_max, len(laser.ranges))[valid]

        x_lf = ranges * np.cos(angles)
        y_lf = ranges * np.sin(angles)
        z_lf = -T[2] * np.ones(len(ranges))
        xyz_lf = np.vstack((x_lf, y_lf, z_lf, np.ones(len(ranges))))

        H = R.copy()
        H[0:3, 3] = T
        xyz_mf = H @ xyz_lf

        center = len(xyz_mf[0]) // 2
        n = int(1080 / self.downsample_factor)
        indices = np.random.normal(center, int(0.68 * n), n).astype(int)
        indices = np.clip(indices, 0, len(xyz_mf[0]) - 1)
        return xyz_mf[:, indices]

    def _compute_alignment(self, scan_in_map: np.ndarray, trackbounds: MarkerArray) -> float:
        boundary_pos = np.array([
            [m.pose.position.x, m.pose.position.y, m.pose.position.z]
            for m in trackbounds.markers
        ])
        n_pts = scan_in_map.shape[1]
        overlaps = np.zeros(n_pts, dtype=bool)
        for i in range(n_pts):
            dist = np.linalg.norm(boundary_pos - scan_in_map[:3, i], axis=1)
            overlaps[i] = np.any(dist < self.threshold)
        return (np.sum(overlaps) / (1080 / self.downsample_factor)) * 100.0

    def _pub_map_lidarpoints(self, points: np.ndarray, stamp):
        mrks = MarkerArray()
        for i in range(points.shape[1]):
            mrk = Marker()
            mrk.header.frame_id = 'map'
            mrk.header.stamp = stamp
            mrk.id = i
            mrk.type = Marker.SPHERE
            mrk.action = Marker.ADD
            mrk.pose.position.x = float(points[0, i])
            mrk.pose.position.y = float(points[1, i])
            mrk.pose.position.z = float(points[2, i])
            mrk.pose.orientation.w = 1.0
            mrk.scale.x = mrk.scale.y = mrk.scale.z = 0.1
            mrk.color.a = 1.0
            mrk.color.g = 1.0
            mrk.color.b = 0.6
            mrks.markers.append(mrk)
        self.debug_lidar_pub.publish(mrks)

    def _pub_alignscore_txt(self, score: float):
        if self.first_visualization and self.glb_wpnts is not None:
            self.first_visualization = False
            x0, y0 = self.glb_wpnts[0].x_m, self.glb_wpnts[0].y_m
            x1, y1 = self.glb_wpnts[1].x_m, self.glb_wpnts[1].y_m
            norm = np.array([-(y1 - y0), x0 - x1])
            norm = norm / np.linalg.norm(norm) * 1.5 * self.glb_wpnts[0].d_left
            self.x_viz = x0 + norm[0]
            self.y_viz = y0 + norm[1]

        if self.x_viz is None:
            return

        mrk = Marker()
        mrk.header.frame_id = 'map'
        mrk.header.stamp = self.get_clock().now().to_msg()
        mrk.id = 0
        mrk.type = Marker.TEXT_VIEW_FACING
        mrk.action = Marker.ADD
        mrk.pose.position.x = self.x_viz
        mrk.pose.position.y = self.y_viz
        mrk.pose.position.z = 1.0
        mrk.pose.orientation.w = 1.0
        mrk.scale.x = mrk.scale.y = mrk.scale.z = 0.5
        mrk.color.a = 1.0
        mrk.color.r = 1.0
        mrk.text = f'Scan Alignment Score: {score:.2f} [%]'
        self.alignment_textmrk_pub.publish(mrk)


def main(args=None):
    rclpy.init(args=args)
    node = ScanAlignAnalyser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
