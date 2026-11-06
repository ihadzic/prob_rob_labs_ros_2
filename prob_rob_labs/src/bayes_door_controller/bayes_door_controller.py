import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, Float64
from geometry_msgs.msg import TwistStamped

class BayesDoorController(Node):
    def __init__(self):
        super().__init__('bayes_door_controller')

        self.declare_parameter('threshold', 321.4572)

        '''
            measurement model:
                P(measured z = open | state x = open) = 0.9363          P(measured z = open | state x = closed) = 0.0750
                P(measured z = closed | state x = open) = 0.0637        P(measured z = closed | state x = closed) = 0.9250
        '''

        self.declare_parameter('p_success', 0.15714285714)                                                          # probability that the door will open when the robot requests it to open with flaky_door_opener
        self.declare_parameter('p_z_open_given_open', 0.9363)
        self.declare_parameter('p_z_open_given_closed', 0.0750)
        self.declare_parameter('p_z_closed_given_open', 0.0637)
        self.declare_parameter('p_z_closed_given_closed', 0.9250)

        '''
            Prior belief:
                P(state x = open) = 0.5
                P(state x = closed) = 0.5
        '''

        self.declare_parameter('initial_belief_open', 0.50)                                                         # robot does not yet know if the door is open or closed, so it starts with a 50% belief that the door is open and a 50% belief that the door is closed
        self.declare_parameter('initial_belief_closed', 0.50)          
        
        self.declare_parameter('belief_threshold', 0.99)                                                            # robot should move if belief is 0.99 or higher
        self.declare_parameter('command_wait_sec', 4.0)
        self.declare_parameter('forward_velocity', 1.0)

        self.threshold = self.get_parameter('threshold').value
        self.p_success = self.get_parameter('p_success').value
        self.p_z_open_given_open = self.get_parameter('p_z_open_given_open').value
        self.p_z_open_given_closed = self.get_parameter('p_z_open_given_closed').value
        self.p_z_closed_given_open = self.get_parameter('p_z_closed_given_open').value
        self.p_z_closed_given_closed = self.get_parameter('p_z_closed_given_closed').value
        self.belief_threshold = self.get_parameter('belief_threshold').value
        self.bel_open = self.get_parameter('initial_belief_open').value
        self.bel_closed = self.get_parameter('initial_belief_closed').value
        self.command_wait_sec = self.get_parameter('command_wait_sec').value
        self.forward_velocity = self.get_parameter('forward_velocity').value

        self.door_pub = self.create_publisher(                                                                      # publish to the flaky_door_opener node to request the door to open
            Empty, '/door_open', 10
        )

        self.cmd_vel_pub = self.create_publisher(                                                                   # publish to the robot's cmd_vel topic to move the robot forward once the door is open
            TwistStamped, '/cmd_vel', 10
        )

        self.feature_sub = self.create_subscription(                                                                # subscribe to the feature_mean topic published by to receive the latest feature mean value for the door state
            Float64,
            '/feature_mean',
            self.feature_mean_callback,
            10
        )

        self.state = "OPEN_DOOR"

        self.feature_mean = None
        self.command_count = 0                                                                                      # will iterate every time a open door request is sent
        self.feature_sample_count = 0                                                                               # will iterate every time a new /feature_mean message comes in

        self.move_start_time = None                                                                                 # will store the time when the robot starts moving forward      
        self.feature_count_at_request = 0                                                                           # will store sample count when open door request is sent so can compare against current count
        
        self.last_command_time = self.get_clock().now()                                                             # stores when robot sends open door request                                          

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            'Bayes door controller started. '
            f'Initial P(open)={self.bel_open:.4f}, '
            f'movement threshold={self.belief_threshold:.2f}'
        )

    def feature_mean_callback(self, msg):
        self.feature_mean = msg.data
        self.feature_sample_count += 1

    def predict(self):
        """
        Prediction after one eligible door-open command.

        Assumptions:
        - An already-open door remains open.
        - A closed door opens with probability p_success.
        """

        p_open_given_open = 1.0                                                                                     # door remains open if it is already open
        p_open_given_closed = self.p_success                                                                        # door opens with probability p_success if it was originally closed

        self.bel_open = self.bel_open * p_open_given_open + self.bel_closed * p_open_given_closed                   # prediction bel(open) = bel(open) * P(xt = open | xt-1 = open, ut-1 = push) + bel(closed) * P(xt = open | xt-1 = closed, ut-1 = push)               
        self.bel_closed = 1.0 - self.bel_open

        self.get_logger().info(
            f'Prediction: P(open)={self.bel_open:.4f}, '
            f'P(closed)={self.bel_closed:.4f}'
        )

    def update(self, feature_mean):
        """Bayes measurement update using one feature-mean sample."""

        z_open = feature_mean <= self.threshold                                                                     # determine if the feature mean indicates an open door

        if z_open:
            likelihood_if_open = self.p_z_open_given_open
            likelihood_if_closed = self.p_z_open_given_closed
        else:
            likelihood_if_open = self.p_z_closed_given_open
            likelihood_if_closed = self.p_z_closed_given_closed
            
        bel_plus_open = likelihood_if_open * self.bel_open                                                          # P(xt = open) = P(zt-1 = open | xt-1 = open) * bel(xt-1 = open) or P(xt = open) = P(zt-1 = closed | xt-1 = open)
        bel_plus_closed = likelihood_if_closed * self.bel_closed                                                    # P(xt = closed) = P(zt-1 = open | xt-1 = closed) * bel(xt-1 = closed) or P(xt = closed) = P(zt-1 = closed | xt-1 = closed)

        denominator = bel_plus_open + bel_plus_closed                                                               # normalization factor

        self.bel_open = bel_plus_open / denominator
        self.bel_closed = bel_plus_closed / denominator

        observation = 'OPEN-likelihood' if z_open else 'CLOSED-likelihood'

        self.get_logger().info(
            f'feature_mean={feature_mean:.2f}, '
            f'observation={observation}, '
            f'P(open)={self.bel_open:.4f}, '
            f'P(closed)={self.bel_closed:.4f}'
        )

    def request_open(self):
        """Send one request and predict the state transition once."""

        self.door_pub.publish(Empty())                                                                              # publish an empty message to the /door_open topic to request the door to open
        self.command_count += 1
        self.state = 'WAIT_FOR_MEASUREMENT'

        self.last_command_time = self.get_clock().now()
        self.predict()                                                                                              # predict door state after one request

        self.get_logger().info(
            f'Published /door_open request #{self.command_count}; '
            f'waiting {self.command_wait_sec:.1f}s before updating.'
        )

    def start_moving(self):
        """Move the robot forward once the belief reaches the threshold."""

        self.state = "DRIVE_THROUGH"
        self.move_start_time = self.get_clock().now()

        self.get_logger().info(
            f'P(open)={self.bel_open:.4f} >= '
            f'{self.belief_threshold:.2f}. '
            'Starting robot movement.'
        )

    def publish_velocity(self, speed):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = speed
        msg.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def control_loop(self):
        if self.state == "FINISHED":                                                                                # robot has already finished moving through the door
            return

        if self.state == "DRIVE_THROUGH":                                                                           # robot continues moving for the configured duration, then stop
            elapsed = (
                self.get_clock().now() - self.move_start_time
            ).nanoseconds * 1e-9

            if elapsed < 20.0:
                self.publish_velocity(self.forward_velocity)
            else:
                self.publish_velocity(0.0)
                self.state = "FINISHED"
                self.get_logger().info(
                    'Movement complete. Controller finished.'
                )
            return

        if self.bel_open >= self.belief_threshold:                                                                  # stop attempting to open door once confidence is high enough
            self.start_moving()
            return

        if self.state == "WAIT_FOR_MEASUREMENT":                                                                    # wait between door command and measurement update.
            elapsed = (
                self.get_clock().now() - self.last_command_time
            ).nanoseconds * 1e-9

            if elapsed < self.command_wait_sec:
                return

            if (
                self.feature_mean is None
                or self.feature_sample_count <= self.feature_count_at_request
            ):
                self.get_logger().warning(
                    'No fresh /feature_mean received after the door request; '
                    'will retry with another door request.'
                )
                return

            self.update(self.feature_mean)
            self.state = "OPEN_DOOR"

            self.get_logger().info(
                f'Updated belief: P(open)={self.bel_open:.4f}, '
                f'P(closed)={self.bel_closed:.4f}'
            )

            if self.bel_open >= self.belief_threshold:
                self.start_moving()

            return

        self.request_open()                                                                                         # send next request when the previous cycle is complete                                                                                    


def main(args=None):
    rclpy.init(args=args)
    node = BayesDoorController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_velocity(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()