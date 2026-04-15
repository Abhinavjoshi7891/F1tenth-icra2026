#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
import os
import yaml
from ament_index_python.packages import get_package_share_directory

class MPCOnlineTuningServer(Node):
    def __init__(self):
        super().__init__('mpc_param_tuner')

        # Declare version parameters
        self.declare_parameter('mpc_version', 'STMPC')
        self.declare_parameter('racecar_version', 'SIM')

        self.mpc_version = self.get_parameter('mpc_version').value
        self.racecar_version = self.get_parameter('racecar_version').value

        self.get_logger().info(f"Initializing {self.mpc_version} Tuner Server for {self.racecar_version}")

        # Load initial values from stack_master config
        self.params_dict = self.load_initial_params()

        # Define dynamic ranges (from ROS1 reference)
        self.ranges = {
            'qadv': [0.0, 100.0],
            'qv': [0.0, 100.0],
            'qn': [0.0, 100.0],
            'qalpha': [0.0, 100.0],
            'qjerk': [0.0, 1.0],
            'qddelta': [0.0, 100.0],
            'alat_max': [0.0, 20.0],
            'a_min': [-20.0, 0.0],
            'a_max': [0.0, 20.0],
            'track_safety_margin': [0.0, 1.0],
            'qac': [0.0, 1.0], # KMPC specific
            'v_min': [-2.0, 4.0], # KMPC specific
            'v_max': [-1.0, 30.0], # KMPC specific
        }

        # Declare parameters with descriptors
        for key, value in self.params_dict.items():
            if key in self.ranges:
                f_range = FloatingPointRange(
                    from_value=float(self.ranges[key][0]),
                    to_value=float(self.ranges[key][1]),
                    step=0.01
                )
                descriptor = ParameterDescriptor(floating_point_range=[f_range])
                self.declare_parameter(key, float(value), descriptor)
            else:
                # Default declaration for non-ranged parameters
                self.declare_parameter(key, value)

        self.get_logger().info("MPC Online Tuning Server Ready")

    def load_initial_params(self):
        stack_master_path = get_package_share_directory('stack_master')
        filename = 'stmpc_params.yaml' if self.mpc_version == 'STMPC' else 'kmpc_params.yaml'
        config_path = os.path.join(stack_master_path, 'config', self.racecar_version, filename)

        if not os.path.exists(config_path):
            self.get_logger().error(f"Config path not found: {config_path}")
            return {}

        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
            # Both STMPC and KMPC yaml files in this stack have 'controller' -> 'ros__parameters'
            return data['controller']['ros__parameters']

def main(args=None):
    rclpy.init(args=args)
    node = MPCOnlineTuningServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
