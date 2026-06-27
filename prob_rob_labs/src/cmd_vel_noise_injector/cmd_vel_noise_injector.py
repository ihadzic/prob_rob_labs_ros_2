import random

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

class CmdVelNoiseInjector(Node):
    def __init__(self):
        super().__init__('cmd_vel_noise_injector')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_noisy')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('actuation_noise_linear_std', 0.006)
        self.declare_parameter('actuation_noise_angular_std', 0.006)
        self.declare_parameter('timeout', 1.0)

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self.act_lin_std = self.get_parameter('actuation_noise_linear_std').get_parameter_value().double_value
        self.act_ang_std = self.get_parameter('actuation_noise_angular_std').get_parameter_value().double_value

        self.latest_cmd = TwistStamped()
        self.latest_cmd_time = None

        timeout_s = self.get_parameter('timeout').get_parameter_value().double_value
        self.timeout = Duration(seconds=timeout_s)

        self.sub = self.create_subscription(
            TwistStamped,
            input_topic,
            self.handle_cmd,
            10,
        )
        self.pub = self.create_publisher(TwistStamped, output_topic, 10)

        period_s = 1.0 / max(publish_rate_hz, 1.0)
        self.timer = self.create_timer(period_s, self.heartbeat)

    def heartbeat(self) -> None:
        now = self.get_clock().now()
        if self.latest_cmd_time is not None and \
           now - self.latest_cmd_time > self.timeout:
            # if input cmd_vel times out stop the robot
            msg = TwistStamped()
            msg.header.frame_id = self.latest_cmd.header.frame_id or 'base_footprint'
            msg.header.stamp = now.to_msg()
            self.publish_noisy_cmd(msg)

    def handle_cmd(self, msg: TwistStamped) -> None:
        # as long as input cmd_vel is coming, just repoublish with noise
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now()
        self.publish_noisy_cmd(msg)

    def publish_noisy_cmd(self, msg: TwistStamped) -> None:
        cmd = TwistStamped()
        cmd.header = msg.header

        v = min(msg.twist.linear.x, 0.5)
        w = min(msg.twist.angular.z, 3.0)

        cmd.twist.linear.x = v + random.gauss(0.0, self.act_lin_std)
        cmd.twist.linear.y = msg.twist.linear.y
        cmd.twist.linear.z = msg.twist.linear.z

        cmd.twist.angular.x = msg.twist.angular.x
        cmd.twist.angular.y = msg.twist.angular.y
        cmd.twist.angular.z = w + random.gauss(0.0, self.act_ang_std)

        self.pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = CmdVelNoiseInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
