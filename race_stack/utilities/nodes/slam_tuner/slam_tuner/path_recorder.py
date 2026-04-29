import csv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion


class PathRecorder(Node):
    '''
    Subscribes to an Odometry, PoseStamped, or PoseWithCovarianceStamped topic
    and saves (t, x, y, theta) rows to a CSV file.
    '''

    def __init__(self):
        super().__init__('path_recorder_node')

        self.declare_parameter('sub_topic', '/car_state/pose')
        self.declare_parameter('topic_type', 'posestamped')
        self.declare_parameter('out_filename', '')

        self.SUB_TOPIC = self.get_parameter('sub_topic').value
        self.TOPIC_TYPE = self.get_parameter('topic_type').value.lower()

        out = self.get_parameter('out_filename').value
        self.OUT_FILENAME = out if out else f'{self.TOPIC_TYPE}_traj.csv'

        if self.TOPIC_TYPE == 'odometry':
            self.create_subscription(Odometry, self.SUB_TOPIC, self.callback, 10)
        elif self.TOPIC_TYPE == 'posestamped':
            self.create_subscription(PoseStamped, self.SUB_TOPIC, self.callback, 10)
        elif self.TOPIC_TYPE == 'posewithcovariancestamped':
            self.create_subscription(PoseWithCovarianceStamped, self.SUB_TOPIC, self.callback, 10)
        else:
            raise ValueError(
                f'topic_type "{self.TOPIC_TYPE}" not supported. '
                "Choose: 'odometry', 'posestamped', 'posewithcovariancestamped'."
            )

        with open(self.OUT_FILENAME, 'w') as f:
            csv.writer(f).writerow(['t', 'x', 'y', 'theta'])

        self.get_logger().info(
            f'PathRecorder ready — topic: {self.SUB_TOPIC}, writing to: {self.OUT_FILENAME}'
        )

    def callback(self, msg):
        if self.TOPIC_TYPE == 'odometry':
            pose: Pose = msg.pose.pose
        elif self.TOPIC_TYPE == 'posestamped':
            pose: Pose = msg.pose
        else:
            pose: Pose = msg.pose.pose

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = pose.position.x
        y = pose.position.y
        theta = euler_from_quaternion([
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        ])[2]

        with open(self.OUT_FILENAME, 'a') as f:
            csv.writer(f).writerow([t, x, y, theta])


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
