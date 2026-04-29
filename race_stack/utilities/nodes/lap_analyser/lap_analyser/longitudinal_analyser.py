#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from f110_msgs.msg import LapData, PidData
from nav_msgs.msg import Odometry


class LongAnalyser(Node):
    def __init__(self):
        super().__init__('longitudinal_analyser')

        self.create_subscription(Odometry, '/car_state/frenet/odom', self.frenet_odom_cb, 10)
        self.create_subscription(PidData, '/velocity_pid', self.velocity_cb, 10)
        self.create_subscription(PidData, '/trailing/gap_data', self.trailing_cb, 10)

        self.vel_data_pub = self.create_publisher(LapData, '/lap_data/vel_ctr', 10)
        self.trailing_data_pub = self.create_publisher(LapData, '/tlap_data/trailing', 10)

        self.lap_start_time = self.get_clock().now()
        self.last_s = 0
        self.lap_count = -1

        self.accumulated_vel_error = 0.0
        self.max_vel_error = 0.0
        self.vel_datapoints = 0

        self.accumulated_trailing_error = 0.0
        self.max_trailing_error = 0.0
        self.trailing_datapoints = 0

    def frenet_odom_cb(self, msg: Odometry):
        current_s = msg.pose.pose.position.x
        if self._check_lap(current_s):
            if self.lap_count == -1:
                self.lap_start_time = self.get_clock().now()
                self.get_logger().info('LongAnalyser: started first lap')
                self.lap_count = 0
            else:
                self.lap_count += 1
                self._publish_lap()
                self.lap_start_time = self.get_clock().now()
        self.last_s = current_s

    def velocity_cb(self, data: PidData):
        err = abs(data.error)
        self.max_vel_error = max(self.max_vel_error, err)
        self.vel_datapoints += 1
        self.accumulated_vel_error += err

    def trailing_cb(self, data: PidData):
        err = abs(data.error)
        self.max_trailing_error = max(self.max_trailing_error, err)
        self.trailing_datapoints += 1
        self.accumulated_trailing_error += err

    def _check_lap(self, current_s) -> bool:
        return (self.last_s - current_s) > 1.0

    def _publish_lap(self):
        lap_time = (self.get_clock().now() - self.lap_start_time).nanoseconds / 1e9
        self.get_logger().info(f'LongAnalyser: lap #{self.lap_count} in {lap_time:.3f}s')

        if self.vel_datapoints > 0:
            avg = self.accumulated_vel_error / self.vel_datapoints
            msg = LapData()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.lap_count = self.lap_count
            msg.lap_time = lap_time
            msg.average_lateral_error_to_global_waypoints = avg
            msg.max_lateral_error_to_global_waypoints = self.max_vel_error
            self.vel_data_pub.publish(msg)
            self.accumulated_vel_error = 0.0
            self.max_vel_error = 0.0
            self.vel_datapoints = 0

        if self.trailing_datapoints > 0:
            avg = self.accumulated_trailing_error / self.trailing_datapoints
            msg = LapData()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.lap_count = self.lap_count
            msg.lap_time = lap_time
            msg.average_lateral_error_to_global_waypoints = avg
            msg.max_lateral_error_to_global_waypoints = self.max_trailing_error
            self.trailing_data_pub.publish(msg)
            self.accumulated_trailing_error = 0.0
            self.max_trailing_error = 0.0
            self.trailing_datapoints = 0


def main(args=None):
    rclpy.init(args=args)
    node = LongAnalyser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
