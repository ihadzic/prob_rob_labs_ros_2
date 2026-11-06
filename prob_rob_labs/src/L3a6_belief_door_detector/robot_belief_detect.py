import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
import numpy as np

heartbeat_period = 0.1

measurement_model = np.array([
    [0.9417, 0.0631],
    [0.0583, 0.9369]
])

transition_model = np.array([
    [0.999999, 0.000001],
    [0.000001, 0.999999]
])

class belief_door_detector(Node):

    def __init__(self):
        super().__init__('doorDetector')
        self.log = self.get_logger()

        self.feature_mean = None

        self.bel = np.array([[0.5], [0.5]])
        self.belief_threshold = 0.9999

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

        self.state = 'STOP'
        
        self.state_start_time = time.monotonic()

        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

    def heartbeat(self):
        elapsed = time.monotonic() - self.state_start_time

        if self.state == 'DRIVE_THROUGH':
            self.drive_forward()

            if elapsed >= 20.0:
                self.change_state('STOP')

        elif self.state == 'STOP':
            self.stop_robot()

            if self.bel[0, 0] >= self.belief_threshold:
                self.change_state('DRIVE_THROUGH')

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

    def feature_mean_callback(self, msg):
        self.feature_mean = msg.data

        self.bel = transition_model @ self.bel 
        z = int(self.feature_mean > 321.55)
        posterior = measurement_model[z] * self.bel.flatten()
        posterior = posterior / posterior.sum()
        self.bel = posterior.reshape(2, 1)

        print(f"P(open): {self.bel[0, 0]:.6f}, State: {self.state}", flush=True)


def main():
    rclpy.init()
    doorDetector = belief_door_detector()
    doorDetector.spin()
    doorDetector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()