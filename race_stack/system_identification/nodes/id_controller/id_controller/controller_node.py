#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


class IDController(Node):

    def __init__(self):
        super().__init__('id_controller')

        self.cmd_pub = self.create_publisher(AckermannDriveStamped, 'drive_topic', 10)
        self.erpm_pub = self.create_publisher(Float64, '/commands/motor/speed', 10)
        self.servo_pub = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.current_pub = self.create_publisher(Float64, '/commands/motor/current', 10)
        self.brake_pub = self.create_publisher(Float64, '/commands/motor/brake', 10)

        self.declare_parameter('experiment', 1)
        self.declare_parameter('const_erpm', 3000)
        self.declare_parameter('const_servo', 0.415)
        self.declare_parameter('const_curr', 40.0)
        self.declare_parameter('const_brake', 50.0)
        self.declare_parameter('accel_time', 1.5)
        self.declare_parameter('decel_time', 1.5)
        self.declare_parameter('const_accel', 5.0)
        self.declare_parameter('const_decel', -5.0)
        self.declare_parameter('angle_time', 20.0)
        self.declare_parameter('start_pos', -0.1)
        self.declare_parameter('end_pos', 0.4)
        self.declare_parameter('start_angle', 0.1)
        self.declare_parameter('end_angle', 0.4)
        self.declare_parameter('const_speed', 3.0)
        self.declare_parameter('acc_profile', 3)
        self.declare_parameter('profile3_speed1', 3.0)
        self.declare_parameter('profile3_speed2', 5.0)
        self.declare_parameter('profile3_duration', 6.0)
        self.declare_parameter('period', 3.0)
        self.declare_parameter('repetitions', 5)
        self.declare_parameter('bangbang_steer', 0.1)

        self.experiment = self.get_parameter('experiment').value
        self.const_erpm = float(self.get_parameter('const_erpm').value)
        self.const_servo = float(self.get_parameter('const_servo').value)
        self.const_curr = float(self.get_parameter('const_curr').value)
        self.const_brake = float(self.get_parameter('const_brake').value)
        self.accel_time = float(self.get_parameter('accel_time').value)
        self.decel_time = float(self.get_parameter('decel_time').value)
        self.const_accel = float(self.get_parameter('const_accel').value)
        self.const_decel = float(self.get_parameter('const_decel').value)
        self.angle_time = float(self.get_parameter('angle_time').value)
        self.start_pos = float(self.get_parameter('start_pos').value)
        self.end_pos = float(self.get_parameter('end_pos').value)
        self.start_angle = float(self.get_parameter('start_angle').value)
        self.end_angle = float(self.get_parameter('end_angle').value)
        self.const_speed = float(self.get_parameter('const_speed').value)
        self.acc_profile = self.get_parameter('acc_profile').value
        self.profile3_speed1 = float(self.get_parameter('profile3_speed1').value)
        self.profile3_speed2 = float(self.get_parameter('profile3_speed2').value)
        self.profile3_duration = float(self.get_parameter('profile3_duration').value)
        self.period = float(self.get_parameter('period').value)
        self.repetitions = self.get_parameter('repetitions').value
        self.bangbang_steer = float(self.get_parameter('bangbang_steer').value)

        self._logged_once = set()
        self.get_logger().warn(f'Starting experiment #{self.experiment}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _log_once(self, msg: str):
        if msg not in self._logged_once:
            self.get_logger().info(msg)
            self._logged_once.add(msg)

    # ── Experiment 1: constant VESC erpm + servo ──────────────────────────────
    def send_const_vesc_cmd(self, _elapsed):
        self.erpm_pub.publish(Float64(data=self.const_erpm))
        self.servo_pub.publish(Float64(data=self.const_servo))

    # ── Experiment 2: accelerate then brake via current commands ──────────────
    def accel_decel(self, elapsed):
        self.servo_pub.publish(Float64(data=self.const_servo))
        if elapsed < self.accel_time:
            self.current_pub.publish(Float64(data=self.const_curr))
            self.get_logger().info('accelerating')
        elif elapsed < self.accel_time + self.decel_time:
            self.brake_pub.publish(Float64(data=self.const_brake))
            self.get_logger().info('decelerating')

    # ── Experiment 3: accel/decel via Ackermann acceleration field ───────────
    def drive_accel_decel(self, elapsed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = 0.0
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.speed = 0.0
        msg.drive.jerk = 512.0  # flag: ackermann controller uses acceleration field
        if elapsed < 3.0:
            self._log_once('experiment starting, waiting 3s')
        elif elapsed < self.accel_time + 3.0:
            msg.drive.acceleration = self.const_accel
            self.get_logger().info('accelerating')
            self.cmd_pub.publish(msg)
        elif elapsed < self.accel_time + self.decel_time + 3.0:
            msg.drive.acceleration = self.const_decel
            self.cmd_pub.publish(msg)
            self.get_logger().info('decelerating')
        else:
            self._log_once('experiment over')

    # ── Experiment 4: sweep servo position at const erpm ─────────────────────
    def increase_servo_position(self, elapsed):
        time_frac = elapsed / self.angle_time
        if time_frac <= 1.0:
            angle = self.end_pos * time_frac + self.start_pos * (1.0 - time_frac)
            self.erpm_pub.publish(Float64(data=self.const_erpm))
            self.servo_pub.publish(Float64(data=angle))
        else:
            self._log_once('experiment over')

    # ── Experiment 5: sweep steering angle at const speed ────────────────────
    def increase_steering_angle(self, elapsed):
        time_frac = elapsed / self.angle_time
        if time_frac <= 1.0:
            angle = self.end_angle * time_frac + self.start_angle * (1.0 - time_frac)
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.steering_angle = angle
            msg.drive.steering_angle_velocity = 0.0
            msg.drive.speed = self.const_speed
            msg.drive.acceleration = 0.0
            msg.drive.jerk = 0.0
            self.cmd_pub.publish(msg)
            if time_frac >= 0.9:
                self.get_logger().warn('Ending soon, stop the bag recording')
        else:
            self._log_once('experiment over')

    # ── Experiment 6: follow acceleration profile ─────────────────────────────
    def acc_profile_recorder(self, elapsed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = 0.0
        msg.drive.steering_angle_velocity = 0.0

        if self.acc_profile == 1:
            # Step to 3 m/s for 4s
            speed, duration = 3.0, 4.0
            if elapsed < 2.0:
                msg.drive.speed = 0.0; msg.drive.jerk = 0.0; msg.drive.acceleration = 0.0
                self.cmd_pub.publish(msg)
                self._log_once('experiment starting')
            elif elapsed > duration + 2.0:
                msg.drive.speed = 0.0; msg.drive.jerk = 0.0; msg.drive.acceleration = 0.0
                self.cmd_pub.publish(msg)
                self._log_once('experiment over')
            else:
                msg.drive.speed = speed; msg.drive.jerk = 0.0; msg.drive.acceleration = 0.0
                self.cmd_pub.publish(msg)
                self.get_logger().info('accelerating')

        elif self.acc_profile == 2:
            # Max accel then max brake
            msg.drive.speed = 0.0; msg.drive.jerk = 512.0
            if elapsed < self.accel_time:
                msg.drive.acceleration = 100.0
                self.get_logger().info('accelerating')
                self.cmd_pub.publish(msg)
            elif elapsed < self.accel_time + self.decel_time:
                msg.drive.acceleration = -100.0
                self.cmd_pub.publish(msg)
                self.get_logger().info('decelerating')
            else:
                self._log_once('experiment over')

        elif self.acc_profile == 3:
            # Speed steps: 0 → speed1 → speed2 → speed2 → 0
            speed1, speed2, duration = self.profile3_speed1, self.profile3_speed2, self.profile3_duration
            if elapsed < 2.0 or elapsed > 2.0 + duration:
                msg.drive.speed = 0.0; msg.drive.jerk = 0.0; msg.drive.acceleration = 0.0
                self.cmd_pub.publish(msg)
                self._log_once('standing still')
            elif elapsed > duration / 3.0 + 2.0 and elapsed < duration / 3.0 * 2.0 + 2.0:
                msg.drive.speed = speed2; msg.drive.jerk = 0.0; msg.drive.acceleration = 0.0
                self.cmd_pub.publish(msg)
                self.get_logger().info('accelerating further')
            elif elapsed > duration / 3.0 * 2.0 + 2.0 and elapsed < duration + 2.0:
                msg.drive.speed = speed2; msg.drive.jerk = 0.0; msg.drive.acceleration = 0.0
                self.cmd_pub.publish(msg)
                self.get_logger().info('decelerating')
            elif elapsed > duration + 2.0:
                msg.drive.speed = 0.0; msg.drive.jerk = 0.0; msg.drive.acceleration = 0.0
                self.cmd_pub.publish(msg)
                self._log_once('experiment over')
            else:
                msg.drive.speed = speed1; msg.drive.jerk = 0.0; msg.drive.acceleration = 0.0
                self.cmd_pub.publish(msg)
                self.get_logger().info('accelerating')
        else:
            self.get_logger().warn(f'Unknown acc_profile {self.acc_profile}')

    # ── Experiment 7: bang-bang servo at const speed ──────────────────────────
    def bang_bang_servo(self, elapsed):
        time_frac = elapsed / self.period
        steer_sign = 1 if (time_frac % 1.0) < 0.5 else -1
        if time_frac <= float(self.repetitions):
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.steering_angle = float(steer_sign) * self.bangbang_steer
            msg.drive.steering_angle_velocity = 0.0
            msg.drive.speed = self.const_speed
            msg.drive.acceleration = 0.0
            msg.drive.jerk = 0.0
            self.cmd_pub.publish(msg)
        else:
            self._log_once('experiment over')

    def run(self):
        dispatch = {
            1: self.send_const_vesc_cmd,
            2: self.accel_decel,
            3: self.drive_accel_decel,
            4: self.increase_servo_position,
            5: self.increase_steering_angle,
            6: self.acc_profile_recorder,
            7: self.bang_bang_servo,
        }
        fn = dispatch.get(self.experiment)
        if fn is None:
            self.get_logger().error(f'Unknown experiment #{self.experiment}. Valid: 1-7.')
            return

        start_time = self._now()
        while rclpy.ok():
            fn(self._now() - start_time)
            rclpy.spin_once(self, timeout_sec=1.0 / 30.0)


def main(args=None):
    rclpy.init(args=args)
    node = IDController()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
