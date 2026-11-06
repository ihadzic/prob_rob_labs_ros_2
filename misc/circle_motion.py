
import rclpy                                                            # Python client library for ROS2
from rclpy.node import Node                         
from geometry_msgs.msg import TwistStamped                              # ROS2 package containing standard message definitions for position, velocity, and direction


class CircleMotion(Node):                                               # Defining a class called CircleMotion that inherits from Node

    def __init__(self):
        super().__init__('circle_motion')                               # Python class name: CircleMotion and ROS2 node name: circle_motion

        self.publisher_ = self.create_publisher(                        # Create a publisher (an object that sends messages to a topic) - by storing in self.publisher_, it can be accessed later from other methods within the class
            TwistStamped,                                               # The message type being sent
            '/cmd_vel',                                                 # The topic name
            10                                                          # The QoS history depth, which limits how many messages are queued under the default QoS settings
        )

        self.timer = self.create_timer(0.1, self.publish_velocity)      # Creates a timer which tells ROS2 to run a function repeatedly - args(timer period in seconds, the function to call every time the timer fires)

    def publish_velocity(self):                                         # Creates and sends the robot's velocity command
        msg = TwistStamped()                                            # Create a new, empty TwistStamped message object with two parts: msg.header which contains metadata about the message such as timestamp and coordinate frame and msg. twist which contains the actual velocity information (linear and angular velocity)

        msg.header.stamp = self.get_clock().now().to_msg()              # Add the timestamp
        msg.header.frame_id = 'base_link'                               # Set the coordinate frame

        # v = rw
        # w = v/r = 1/2 = 0.5 rad/s
        msg.twist.linear.x = 1.0                                        # Set the velocity
        msg.twist.angular.z = 0.5

        self.publisher_.publish(msg)                                    # Publish the message and send to /cmd_vel


def main(args=None):
    rclpy.init(args=args)                                               # Initializes ROS2 - need to do before creating nodes

    node = CircleMotion()                                               # Create an instance of the node, which also sets up the publisher and timer

    try:
        rclpy.spin(node)                                                # spin() keeps the node running and processes its callbacks
    except KeyboardInterrupt:
        pass
    finally:                                                            # Runs when the program exits, including when it is interrupted
        node.destroy_node()                                             # Clean up the node and shut down the ROS2 client library
        rclpy.shutdown()


if __name__ == '__main__':
    main()