import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64

heartbeat_period = 0.1

class RobotOpenDoor(Node):

    def __init__(self):
        super().__init__('robot_open_door')
        self.log = self.get_logger()

        self.feature_mean = None

        self.declare_parameter('feature_mean_open_threshold', 250.11764705882354)               # signal seems to hover around 250.11764705882354 when the door is fully open
        
        self.feature_mean_open_threshold = (
            self.get_parameter('feature_mean_open_threshold').value
        )

        self.feature_mean_sub = self.create_subscription(
            Float64,
            '/feature_mean',                                                                    # topic to listen to
            self.feature_mean_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(                                               # publisher for robot velocity command topic
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.door_torque_pub = self.create_publisher(                                           # publisher for door-control topic      
            Float64,
            '/hinged_glass_door/torque',
            10
        )

        self.declare_parameter('forward_velocity', 1.0)

        self.log.info(
            f"Forward velocity parameter: "
            f"{self.get_parameter('forward_velocity').value}"
        )

        self.state = 'OPEN_DOOR'
        
        self.state_start_time = time.monotonic()

        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

    def heartbeat(self):
        self.log.info('heartbeat')

        elapsed = time.monotonic() - self.state_start_time

        if self.state == 'OPEN_DOOR':
            self.open_door()

            if (self.feature_mean is not None and self.feature_mean <= self.feature_mean_open_threshold):
                self.get_logger().info(f'Door open detected: feature_mean={self.feature_mean}')
                self.change_state('DRIVE_THROUGH')

        elif self.state == 'DRIVE_THROUGH':
            self.drive_forward()

            if elapsed >= 20.0:
                self.change_state('STOP')

        elif self.state == 'STOP':
            self.stop_robot()

            if elapsed >= 5.0:
                self.change_state('CLOSE_DOOR')

        elif self.state == 'CLOSE_DOOR':
            self.close_door()

    def spin(self):
        rclpy.spin(self)

    def open_door(self):
        msg = Float64()
        msg.data = 5.0
        self.door_torque_pub.publish(msg)

    def close_door(self):
        msg = Float64()
        msg.data = -5.0
        self.door_torque_pub.publish(msg)

    def drive_forward(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self.get_parameter('forward_velocity').value
        msg.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def change_state(self, new_state):
        self.state = new_state
        self.state_start_time = time.monotonic()
        self.log.info(f'State: {new_state}')

    def feature_mean_callback(self, msg):
        self.feature_mean = msg.data


def main():
    rclpy.init()
    robot_open_door = RobotOpenDoor()
    robot_open_door.spin()
    robot_open_door.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
