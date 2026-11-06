import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class FeatureRecorder(Node):
    def __init__(self):
        super().__init__('feature_recorder')

        self.declare_parameter('duration_sec', 40.0)
        self.declare_parameter('threshold', 321.55)
        self.declare_parameter('output_file', 'feature_mean_data.txt')

        self.duration_sec = self.get_parameter('duration_sec').value
        self.threshold = self.get_parameter('threshold').value
        self.output_file = self.get_parameter('output_file').value

        self.count = 0
        self.open_count = 0
        self.closed_count = 0

        self.start_time = time.monotonic()
        self.file = open(self.output_file, 'w')

        self.create_subscription(
            Float64, '/feature_mean', self.callback, 10
        )
        self.timer = self.create_timer(0.1, self.check_time)

    def callback(self, msg):
        if time.monotonic() - self.start_time >= self.duration_sec:
            return

        self.file.write(f'{msg.data:.4f}\n')
        if msg.data <= self.threshold:
            self.open_count += 1
        else:
            self.closed_count += 1

        self.count += 1

    def check_time(self):
        if time.monotonic() - self.start_time < self.duration_sec:
            return

        self.timer.cancel()
        self.file.close()

        total_samples = (
            self.open_count + self.closed_count
        )
        self.get_logger().info(
            f'Recording complete. Data saved to {self.output_file}'
        )

        self.get_logger().info(
            f'threshold={self.threshold:.4f}; '
            f'z=open (value <= threshold): '
            f'{self.open_count}; '
            f'z=closed (value > threshold): '
            f'{self.closed_count}; total={total_samples}'
        )

        rclpy.shutdown()


def main():
    rclpy.init()
    node = FeatureRecorder()
    try:
        rclpy.spin(node)
    finally:
        if not node.file.closed:
            node.file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()