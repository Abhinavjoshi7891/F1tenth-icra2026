#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from f110_msgs.msg import WpntArray
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped


class VelAnalyser(Node):
    def __init__(self):
        super().__init__('vel_analyser')

        self.declare_parameter('wheelbase', 0.321)
        self.wheelbase = self.get_parameter('wheelbase').value

        self.gb_wpnt_sc = None
        self.steer_input = 0.0
        self.pub_cur_vel_flag = True

        self.create_subscription(WpntArray, '/global_waypoints_scaled', self.gb_wpnt_sc_cb, 10)
        self.create_subscription(Odometry, '/car_state/frenet/odom', self.frenet_odom_cb, 10)
        self.create_subscription(
            AckermannDriveStamped,
            '/vesc/high_level/ackermann_cmd_mux/input/nav_1',
            self.ctrl_cb, 10,
        )

        self.vel_traj_pub = self.create_publisher(Point, 'velocity/trajectory', 1000)
        self.steer_traj_pub = self.create_publisher(Point, 'steering/trajectory', 1000)
        self.vel_cur_pub = self.create_publisher(Point, 'velocity/current', 10)
        self.steer_cur_pub = self.create_publisher(Point, 'steering_input', 10)

    def gb_wpnt_sc_cb(self, data: WpntArray):
        self.gb_wpnt_sc = data.wpnts
        # republish full trajectory whenever waypoints update (replaces dyn_reconfig trigger)
        for wpnt in self.gb_wpnt_sc:
            self.vel_traj_pub.publish(Point(x=wpnt.s_m, y=wpnt.vx_mps, z=0.0))
            self.steer_traj_pub.publish(
                Point(x=wpnt.s_m, y=np.arctan(self.wheelbase * wpnt.kappa_radpm), z=0.0)
            )

    def frenet_odom_cb(self, data: Odometry):
        cur_s = data.pose.pose.position.x
        cur_v = data.twist.twist.linear.x
        if np.round(cur_s / 0.05, 1) % 1 == 0.0:
            if self.pub_cur_vel_flag:
                self.vel_cur_pub.publish(Point(x=cur_s, y=cur_v, z=0.0))
                self.steer_cur_pub.publish(Point(x=cur_s, y=self.steer_input, z=0.0))
                self.pub_cur_vel_flag = False
        else:
            self.pub_cur_vel_flag = True

    def ctrl_cb(self, data: AckermannDriveStamped):
        self.steer_input = data.drive.steering_angle


def main(args=None):
    rclpy.init(args=args)
    node = VelAnalyser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
