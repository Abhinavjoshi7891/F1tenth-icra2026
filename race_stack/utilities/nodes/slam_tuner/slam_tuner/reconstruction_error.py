import rclpy
from rclpy.node import Node
import rclpy.time
import rclpy.duration
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion
import tf2_ros
import numpy as np
from scipy.interpolate import CubicSpline
from datetime import datetime
import matplotlib.pyplot as plt
import csv
from typing import List, Tuple

from errormetrics.csv_io import save_csv


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ReconstructionError(Node):
    '''Provides error metrics and plots for localization methods.'''

    def __init__(self):
        super().__init__('rec_err_node')

        self.declare_parameter('loc_algo', 'Unknown')
        self.declare_parameter('timeout', 15.0)
        self.declare_parameter('pose_topic', '/car_state/pose')
        self.declare_parameter('n_pose_msgs', 20)
        self.declare_parameter('out_graph', '')

        self.LOC_ALGO: str = self.get_parameter('loc_algo').value
        self.TIMEOUT: float = self.get_parameter('timeout').value
        self.POSE_TOPIC: str = self.get_parameter('pose_topic').value
        self.NUM_POSE_MSGS: int = self.get_parameter('n_pose_msgs').value

        out = self.get_parameter('out_graph').value
        self.FILENAME = out if out else f'{self.LOC_ALGO}_err_metrics'
        self.FILENAME += '_' + datetime.now().strftime('%d%m%y_%H%M%S')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Empty, '/create_plot', self.plot_data, 10)
        self.create_subscription(PoseStamped, self.POSE_TOPIC, self.pose_cb, 10)
        self.create_subscription(PoseStamped, '/tracked_pose', self.slam_pose_cb, 10)

        self._car_state_pose_cnt: int = 0
        self._x_acc: List[float] = []
        self._y_acc: List[float] = []
        self._sint_acc: List[float] = []
        self._cost_acc: List[float] = []

        self.prev_position: List[float] = None
        self.curr_position: List[float] = None

        self.was_stationary = False
        self.time_since_stationary: float = 0

        self.odom_msgs = []
        self.map_bl_list = []
        self.map_odom_list = []
        self.odom_bl_list = []

        self.tf_x: List[float] = []
        self.tf_y: List[float] = []
        self.tf_sint: List[float] = []
        self.tf_cost: List[float] = []
        self.tf_t: List[float] = []

        self.tracked_x: List[float] = []
        self.tracked_y: List[float] = []
        self.tracked_theta: List[float] = []
        self.tracked_time: List[float] = []
        self.tracked_pose: List[Tuple] = []

    def slam_pose_cb(self, data: PoseStamped) -> None:
        x = data.pose.position.x
        y = data.pose.position.y
        theta = euler_from_quaternion([
            data.pose.orientation.x, data.pose.orientation.y,
            data.pose.orientation.z, data.pose.orientation.w,
        ])[2]
        t = _stamp_to_sec(data.header.stamp)

        self.tracked_x.append(x)
        self.tracked_y.append(y)
        self.tracked_theta.append(theta)
        self.tracked_time.append(t)
        self.tracked_pose.append((x, y, theta, t))

    def pose_cb(self, data: PoseStamped):
        x = data.pose.position.x
        y = data.pose.position.y
        theta = euler_from_quaternion([
            data.pose.orientation.x, data.pose.orientation.y,
            data.pose.orientation.z, data.pose.orientation.w,
        ])[2]

        self.update_tf_lists()

        if len(self.tf_t) == 0 or self.map_bl_list[-1][-1] > self.tf_t[-1]:
            self.tf_x.append(self.map_bl_list[-1][0])
            self.tf_y.append(self.map_bl_list[-1][1])
            self.tf_sint.append(np.sin(self.map_bl_list[-1][2]))
            self.tf_cost.append(np.cos(self.map_bl_list[-1][2]))
            self.tf_t.append(self.map_bl_list[-1][3])

        self._x_acc.append(x)
        self._y_acc.append(y)
        self._sint_acc.append(np.sin(theta))
        self._cost_acc.append(np.cos(theta))
        if len(self._x_acc) > self.NUM_POSE_MSGS:
            self._x_acc = self._x_acc[1:]
            self._y_acc = self._y_acc[1:]
            self._sint_acc = self._sint_acc[1:]
            self._cost_acc = self._cost_acc[1:]

        x = np.mean(self._x_acc)
        y = np.mean(self._y_acc)
        theta = np.arctan2(np.mean(self._sint_acc), np.mean(self._cost_acc))

        self.odom_msgs.append((x, y, theta, _stamp_to_sec(data.header.stamp)))
        self._car_state_pose_cnt = 0

        if self.curr_position is None:
            self.prev_position = [x, y, theta]
        self.curr_position = [x, y, theta]

        dx = self.prev_position[0] - x
        dy = self.prev_position[1] - y
        dtheta = self.prev_position[2] - theta

        if np.sqrt(dx ** 2 + dy ** 2) < 0.25 and dtheta < 0.2:
            if not self.was_stationary:
                self.was_stationary = True
                self.time_since_stationary = _stamp_to_sec(data.header.stamp)
            else:
                elapsed = _stamp_to_sec(data.header.stamp) - self.time_since_stationary
                if elapsed > self.TIMEOUT:
                    self.get_logger().info(
                        f'[ReconstructionError]: {self.TIMEOUT:.2f}s stationary. Plotting...'
                    )
                    self.was_stationary = False
                    self.time_since_stationary = _stamp_to_sec(data.header.stamp)
                    self.plot_data(None)
        else:
            self.was_stationary = False

    def update_tf_lists(self):
        def grab_tf(frame1, frame2, lst):
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    frame1, frame2,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0),
                )
            except Exception:
                return
            t = _stamp_to_sec(tf_msg.header.stamp)
            if lst and t == lst[-1][-1]:
                return
            tf_x = tf_msg.transform.translation.x
            tf_y = tf_msg.transform.translation.y
            tf_theta = euler_from_quaternion([
                tf_msg.transform.rotation.x, tf_msg.transform.rotation.y,
                tf_msg.transform.rotation.z, tf_msg.transform.rotation.w,
            ])[2]
            lst.append((tf_x, tf_y, tf_theta, t))

        grab_tf('map', 'base_link', self.map_bl_list)
        grab_tf('map', 'odom', self.map_odom_list)
        grab_tf('odom', 'base_link', self.odom_bl_list)

    def plot_data(self, _):
        self.get_logger().warn(f'[ReconstructionError]: Saving to {self.FILENAME}')

        tf_x_cs = CubicSpline(self.tf_t, self.tf_x)
        tf_y_cs = CubicSpline(self.tf_t, self.tf_y)
        tf_sint_cs = CubicSpline(self.tf_t, self.tf_sint)
        tf_cost_cs = CubicSpline(self.tf_t, self.tf_cost)

        pos_err, ang_err, t_vec, data_out = [], [], [], []

        for odom_pose in self.odom_msgs:
            t = odom_pose[3]
            ix = float(tf_x_cs(t))
            iy = float(tf_y_cs(t))
            itheta = np.arctan2(float(tf_sint_cs(t)), float(tf_cost_cs(t)))

            pos_err.append(np.sqrt((ix - odom_pose[0]) ** 2 + (iy - odom_pose[1]) ** 2))
            ang_err.append(abs(np.arctan2(
                np.sin(odom_pose[2] - itheta), np.cos(odom_pose[2] - itheta)
            )))
            t_vec.append(t)
            data_out.append((t, odom_pose[0], odom_pose[1], odom_pose[2], ix, iy, itheta))

        save_csv(data_out, self.FILENAME + '.csv',
                 header=['Time', 'odom_x', 'odom_y', 'odom_theta', 'tf_x', 'tf_y', 'tf_theta'])

        t_vec = [t - t_vec[0] for t in t_vec]
        avg_pos_err = np.average(pos_err)

        fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1)
        plt.title(f'Error metrics for "{self.LOC_ALGO}", mean pos err: {avg_pos_err:.2f}m')
        ax1.grid(); ax1.set_xlabel('Time (s)'); ax1.set_ylabel('Position error (m)')
        ax1.plot(t_vec, pos_err, color='tab:red')
        ax2.grid(); ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Angular error (rad)')
        ax2.plot(t_vec, ang_err, color='tab:blue')
        fig.tight_layout()
        plt.savefig(self.FILENAME + '.png')
        plt.close()

        fig, axs = plt.subplots(nrows=1, ncols=3)
        plt.title('Trajectory history')
        axs[0].set_title('map→base_link'); axs[0].grid()
        axs[0].plot([t[0] for t in self.map_bl_list], [t[1] for t in self.map_bl_list],
                    'x-', color='tab:red', label='map→base_link TF')
        axs[0].plot([t[0] for t in self.tracked_pose], [t[1] for t in self.tracked_pose],
                    'x-', label='tracked_pose')
        axs[0].legend()
        axs[1].set_title('odom→base_link'); axs[1].grid()
        axs[1].plot([t[0] for t in self.odom_bl_list], [t[1] for t in self.odom_bl_list],
                    'x-', color='tab:blue')
        axs[2].set_title('map→odom'); axs[2].grid()
        axs[2].plot([t[0] for t in self.map_odom_list], [t[1] for t in self.map_odom_list],
                    'x-', color='tab:green')
        fig.tight_layout()
        plt.savefig(self.FILENAME + '_traj_hist.png')
        plt.close()

        save_csv(self.map_bl_list, self.FILENAME + '_tf_mbl.csv', header=['x', 'y', 'theta', 't'])
        save_csv(self.map_odom_list, self.FILENAME + '_tf_mo.csv', header=['x', 'y', 'theta', 't'])
        save_csv(self.odom_bl_list, self.FILENAME + '_tf_obl.csv', header=['x', 'y', 'theta', 't'])
        save_csv(self.tracked_pose, self.FILENAME + '_tracked_pose.csv', header=['x', 'y', 'theta', 't'])


def main(args=None):
    rclpy.init(args=args)
    node = ReconstructionError()

    # Wait for map→base_link transform before spinning (bags need time to start)
    node.get_logger().info('Waiting for map→base_link transform...')
    while not node.tf_buffer.can_transform(
        'map', 'base_link', rclpy.time.Time(),
        timeout=rclpy.duration.Duration(seconds=0.0),
    ):
        node.get_logger().warn('map→base_link not yet available')
        rclpy.spin_once(node, timeout_sec=1.0)
    node.get_logger().info('map→base_link OK')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
