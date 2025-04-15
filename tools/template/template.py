import rclpy
from rclpy.node import Node


class CLASS(Node):

    def __init__(self):
        super().__init__(config.node_name)
        self.log = self.get_logger()
        self.timer = self.create_timer(config.heartbeat_period,
                                       self.heartbeat)

    def heartbeat(self):
        self.log.info('heartbeat')

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    NODE = CLASS()
    NODE.spin()
    NODE.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
