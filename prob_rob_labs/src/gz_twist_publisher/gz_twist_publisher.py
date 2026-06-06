import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

class GzTwistPublisher(Node):

    def __init__(self):
        super().__init__('gz_twist_publisher')
        self.log = self.get_logger()
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.handle_odom, 1
        )
        self.pub_twist = self.create_publisher(
            TwistStamped, '/tb3/ground_truth/twist', 1)

    def handle_odom(self, odom):
        twist = TwistStamped()
        twist.header = odom.header
        twist.header.frame_id = odom.child_frame_id
        twist.twist = odom.twist.twist
        self.pub_twist.publish(twist) 

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
