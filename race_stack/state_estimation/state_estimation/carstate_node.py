import rclpy
from rclpy.node import Node
import tf2_ros

import time
import numpy as np

from sensor_msgs.msg import Imu
from f110_msgs.msg import WpntArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from frenet_conversion.frenet_converter import FrenetConverter
from tf_transformations import euler_from_quaternion

# Carstate node is relevant for SE1 only (basic state estimaiton pipeline).
# The carstate node publishes the "final" state estimation of the car, in this case the velocities from the (early fusion) EKF and the pose from localization (by default cartographer SLAM).

class Carstate(Node):
    def __init__(self):
        super().__init__('carstate',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.get_logger().info("Carstate node started")

        # ros params
        # ros params - these are automatically declared from overrides
        self.odom_in_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.odom_out_topic = self.get_parameter('odom_out_topic').get_parameter_value().string_value
        self.pose_out_topic = self.get_parameter('pose_out_topic').get_parameter_value().string_value
        self.frenet_bool = self.get_parameter('frenet_bool').get_parameter_value().bool_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value

        # Wait until the requried tf transforms exist
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # data containers
        self.ekf_odom = None
        self.frenet_converter = None
        self.gb_wpnts = None
        self.car_state_odom = None
        self._cartesian_first_log = False
        self._frenet_first_log = False

        # subscribers
        self.ekf_odom_sub = self.create_subscription(Odometry, self.odom_in_topic, self.ekf_odom_cb, 10)
        self.gb_wpnts_sub = self.create_subscription(WpntArray, "/global_waypoints", self.gb_wpnts_cb, 10)

        # publishers
        self.state_odom_pub = self.create_publisher(Odometry, self.odom_out_topic, 10)
        self.state_pose_pub = self.create_publisher(PoseStamped, self.pose_out_topic, 10)
        self.frenet_state_odom_pub = self.create_publisher(Odometry, "/car_state/frenet/odom", 10)
        self.frenet_state_pose_pub = self.create_publisher(PoseStamped, "/car_state/frenet/pose", 10)

        # Block until relevant data is here
        self.wait_for_messages(frenet_bool=self.frenet_bool)

        # Publish at 80 Hz
        self.create_timer(1/80, self.cartesian_state_loop)
        if self.frenet_bool:
            self.create_timer(1/80, self.frenet_state_loop)

    def ekf_odom_cb(self, data):
        self.ekf_odom = data

    def gb_wpnts_cb(self, data):
        self.gb_wpnts = data

    def get_slam_tf(self) -> TransformStamped:
        trans = self.tf_buffer.lookup_transform("map", self.base_frame_id, rclpy.time.Time(), rclpy.duration.Duration(seconds=6.9))
        return trans


    def wait_for_messages(self, frenet_bool: bool = False):
        self.get_logger().info('Carstate Node waiting for Odometry messages...')
        ekf_print = False
        frenet_print = False
        while self.ekf_odom is None or (frenet_bool and self.gb_wpnts is None):
            rclpy.spin_once(self)
            if self.ekf_odom is not None and not ekf_print:
                self.get_logger().info('Received Odometry message.')
                ekf_print = True
            if frenet_bool and self.gb_wpnts is not None and not frenet_print:
                waypoint_array = self.gb_wpnts.wpnts
                waypoints_x = [waypoint.x_m for waypoint in waypoint_array]
                waypoints_y = [waypoint.y_m for waypoint in waypoint_array]
                waypoints_psi = [waypoint.psi_rad for waypoint in waypoint_array]
                self.frenet_converter = FrenetConverter(np.array(waypoints_x), np.array(waypoints_y), np.array(waypoints_psi))
                self.get_logger().info('Received Global Waypoints message and frenet converter initialized!')
                frenet_print = True
        self.get_logger().info('All required messages received. Continuing...')


    def cartesian_state_loop(self):
        carstate_pose_msg = PoseStamped()
        carstate_odom_msg = Odometry()

        try:
            trans = self.tf_buffer.lookup_transform(
                "map", self.base_frame_id,
                rclpy.time.Time(), rclpy.duration.Duration(seconds=6.9))
        except Exception as e:
            self.get_logger().warn(
                f"cartesian_state_loop TF lookup failed: {e}",
                throttle_duration_sec=2.0)
            return

        x = trans.transform.translation.x
        y = trans.transform.translation.y
        if not (np.isfinite(x) and np.isfinite(y)):
            self.get_logger().warn(
                f"cartesian_state_loop got NaN/Inf TF: x={x}, y={y}",
                throttle_duration_sec=2.0)
            return

        carstate_pose_msg.header = trans.header
        carstate_pose_msg.pose.position.x = x
        carstate_pose_msg.pose.position.y = y
        carstate_pose_msg.pose.position.z = trans.transform.translation.z
        carstate_pose_msg.pose.orientation = trans.transform.rotation

        carstate_odom_msg.header = carstate_pose_msg.header
        carstate_odom_msg.pose.pose = carstate_pose_msg.pose
        carstate_odom_msg.twist.twist = self.ekf_odom.twist.twist
        self.car_state_odom = carstate_odom_msg

        if not self._cartesian_first_log:
            self.get_logger().info(
                f"cartesian_state_loop publishing (first sample x={x:.2f}, y={y:.2f})")
            self._cartesian_first_log = True

        self.state_odom_pub.publish(carstate_odom_msg)
        self.state_pose_pub.publish(carstate_pose_msg)


    def frenet_state_loop(self):
        if self.car_state_odom is None:
            self.get_logger().warn(
                "frenet_state_loop: car_state_odom is None — cartesian_state_loop hasn't produced data yet",
                throttle_duration_sec=2.0)
            return

        try:
            odom_cart = self.car_state_odom
            x_cart = odom_cart.pose.pose.position.x
            y_cart = odom_cart.pose.pose.position.y
            vx = odom_cart.twist.twist.linear.x
            vy = odom_cart.twist.twist.linear.y
            q_cart = odom_cart.pose.pose.orientation
            theta = euler_from_quaternion([q_cart.x, q_cart.y, q_cart.z, q_cart.w])[2]

            frenet_pos = self.frenet_converter.get_frenet([x_cart], [y_cart])
            frenet_vel = self.frenet_converter.get_frenet_velocities(vx, vy, theta)

            s = float(frenet_pos[0, 0])
            d = float(frenet_pos[1, 0])
            vs = float(frenet_vel[0][0])
            vd = float(frenet_vel[1][0])

            if not all(np.isfinite([s, d, vs, vd])):
                self.get_logger().warn(
                    f"frenet_state_loop produced non-finite values: s={s}, d={d}, vs={vs}, vd={vd} "
                    f"from x_cart={x_cart}, y_cart={y_cart}",
                    throttle_duration_sec=2.0)
                return

            frenet_pose_msg = PoseStamped()
            frenet_pose_msg.header.stamp = odom_cart.header.stamp
            frenet_pose_msg.header.frame_id = "frenet"
            frenet_pose_msg.pose.position.x = s
            frenet_pose_msg.pose.position.y = d

            frenet_odom_msg = Odometry()
            frenet_odom_msg.header = frenet_pose_msg.header
            frenet_odom_msg.pose.pose = frenet_pose_msg.pose
            frenet_odom_msg.twist.twist.linear.x = vs
            frenet_odom_msg.twist.twist.linear.y = vd

            if not self._frenet_first_log:
                self.get_logger().info(
                    f"frenet_state_loop publishing (first sample s={s:.2f}, d={d:.2f})")
                self._frenet_first_log = True

            self.frenet_state_odom_pub.publish(frenet_odom_msg)
            self.frenet_state_pose_pub.publish(frenet_pose_msg)
        except Exception as e:
            self.get_logger().error(
                f"frenet_state_loop crashed: {type(e).__name__}: {e}",
                throttle_duration_sec=2.0)

def main():
    rclpy.init()
    node = Carstate()
    rclpy.spin(node)
    rclpy.shutdown()
