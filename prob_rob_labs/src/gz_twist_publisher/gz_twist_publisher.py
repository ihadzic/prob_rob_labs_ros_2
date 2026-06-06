import rclpy
from rclpy.node import Node


heartbeat_period = 0.1

class GzTwistPublisher(Node):

    def __init__(self):
        super().__init__('gz_twist_publisher')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

    def heartbeat(self):
        self.log.info('heartbeat')

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    gz_twist_publisher = GzTwistPublisher()
    gz_twist_publisher.spin()
    gz_twist_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
