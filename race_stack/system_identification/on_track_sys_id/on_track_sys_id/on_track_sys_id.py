import rclpy
from rclpy.node import Node
import numpy as np
import os
import yaml
import csv
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from datetime import datetime
from tqdm import tqdm

from .helpers.train_model import nn_train
from .helpers.path_resolver import get_package_path

class OnTrackSysIdNode(Node):
    def __init__(self):
        super().__init__('on_track_sys_id')

        # Declare parameters with default values
        self.declare_parameter('racecar_version', 'SIM')
        self.declare_parameter('save_LUT_name', 'NUCx_on_track_pacejka')
        self.declare_parameter('plot_model', True)
        self.declare_parameter('odom_topic', '/car_state/odom')
        self.declare_parameter('ackermann_cmd_topic', '/vesc/high_level/ackermann_cmd_mux/input/nav_1')

        # Get parameter values
        self.racecar_version = self.get_parameter('racecar_version').get_parameter_value().string_value
        self.save_LUT_name = self.get_parameter('save_LUT_name').get_parameter_value().string_value
        self.plot_model = self.get_parameter('plot_model').get_parameter_value().bool_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        ackermann_topic = self.get_parameter('ackermann_cmd_topic').get_parameter_value().string_value

        self.get_logger().info(f"Racecar version: {self.racecar_version}")
        self.get_logger().info(f"Save LUT name: {self.save_LUT_name}")
        self.get_logger().info(f"Plot model: {self.plot_model}")
        self.get_logger().info(f"Odom topic: {odom_topic}")
        self.get_logger().info(f"Ackermann topic: {ackermann_topic}")

        # Resolve path
        self.package_path = get_package_path()

        self.rate = 50.0 # Hz
        self.load_parameters()
        self.setup_data_storage()

        # Subscribe to the topics
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.ackermann_sub = self.create_subscription(AckermannDriveStamped, ackermann_topic, self.ackermann_cb, 10)

        # Timer for data collection loop
        self.timer_period = 1.0 / self.rate
        self.timer = self.create_timer(self.timer_period, self.loop_callback)

        self.pbar = tqdm(total=self.timesteps, desc='Collecting data', ascii=True)

    def setup_data_storage(self):
        self.data_duration = self.nn_params['data_collection_duration']
        self.timesteps = int(self.data_duration * self.rate)
        self.data = np.zeros((self.timesteps, 4))
        self.counter = 0
        self.current_state = np.zeros(4)

    def load_parameters(self):
        yaml_file = os.path.join(self.package_path, 'params', 'nn_params.yaml')
        with open(yaml_file, 'r') as file:
            self.nn_params = yaml.safe_load(file)

    def export_data_as_csv(self):
        user_input = input("\033[33m[WARN] Press 'Y' and then ENTER to export data as CSV, or press ENTER to continue without dumping: \033[0m")
        if user_input.lower() == 'y':
            data_dir = os.path.join(self.package_path, 'data', self.racecar_version)
            if not os.path.exists(data_dir):
                os.makedirs(data_dir)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_file = os.path.join(data_dir, f'{self.racecar_version}_sys_id_data_{timestamp}.csv')
            
            with open(csv_file, mode='w') as file:
                writer = csv.writer(file)
                writer.writerow(['v_x', 'v_y', 'omega', 'delta'])
                for row in self.data:
                    writer.writerow(row)
            self.get_logger().info(f"DATA HAS BEEN EXPORTED TO: {csv_file}")

    def odom_cb(self, msg):
        self.current_state[0] = msg.twist.twist.linear.x
        self.current_state[1] = msg.twist.twist.linear.y
        self.current_state[2] = msg.twist.twist.angular.z
        
    def ackermann_cb(self, msg):
        self.current_state[3] = msg.drive.steering_angle
        
    def collect_data(self):
        if self.current_state[0] > 1.0: # Only collect data when the car is moving
            self.data = np.roll(self.data, -1, axis=0)
            self.data[-1] = self.current_state
            self.counter += 1
            self.pbar.update(1)
            
    def run_nn_train(self):
        self.get_logger().info("Training neural network...")
        nn_train(self.data, self.racecar_version, self.save_LUT_name, self.plot_model)
        
    def loop_callback(self):
        if self.counter < self.timesteps:
            self.collect_data()
            if self.counter >= self.timesteps:
                # We have finished collecting data!
                self.pbar.close()
                self.get_logger().info("Data collection completed.")
                self.run_nn_train()
                self.export_data_as_csv()
                self.get_logger().info("Training completed. Shutting down...")
                raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = OnTrackSysIdNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
