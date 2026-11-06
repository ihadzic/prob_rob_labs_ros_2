
import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class StepResponse(Node):

    def __init__(self):
        super().__init__('step_response')

        # Parameters
        self.declare_parameter('target_linear', 1.0)
        self.declare_parameter('target_angular', 1.0)
        self.declare_parameter('baseline_sec', 2.0)
        self.declare_parameter('hold_sec', 5.0)
        self.declare_parameter('rest_sec', 3.0)

        self.target_linear = float(
            self.get_parameter('target_linear').value
        )
        self.target_angular = float(
            self.get_parameter('target_angular').value
        )
        self.baseline_sec = float(
            self.get_parameter('baseline_sec').value
        )
        self.hold_sec = float(
            self.get_parameter('hold_sec').value
        )
        self.rest_sec = float(
            self.get_parameter('rest_sec').value
        )

        # Command publisher
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        # Ground-truth subscriber
        self.gt_sub = self.create_subscription(
            TwistStamped,
            '/tb3/ground_truth/twist',
            self.ground_truth_callback,
            100
        )

        # State machine
        self.state = 'LINEAR_BASELINE'
        self.state_start = time.monotonic()
        self.step_start = None

        self.baseline = []
        self.measurements = []
        self.results = {}

        self.finished = False

        # 50 Hz command publishing
        self.timer = self.create_timer(
            0.02,
            self.heartbeat
        )

        self.get_logger().info(
            'Step Response Test Started'
        )

    # ------------------------------------------
    # Publish velocity command
    # ------------------------------------------

    def publish_cmd(self, v=0.0, w=0.0):
        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'base_link'

        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)

        self.cmd_pub.publish(msg)

    # ------------------------------------------
    # Receive ground-truth velocity
    # ------------------------------------------

    def ground_truth_callback(self, msg):

        if self.finished:
            return

        now = time.monotonic()

        if self.state.startswith('LINEAR'):
            value = msg.twist.linear.x

        elif self.state.startswith('ANGULAR'):
            value = msg.twist.angular.z

        else:
            return

        if self.state.endswith('BASELINE'):
            self.baseline.append(float(value))

        elif self.state.endswith('STEP'):
            elapsed = now - self.step_start

            self.measurements.append(
                (elapsed, float(value))
            )

    # ------------------------------------------
    # Start a new experiment
    # ------------------------------------------

    def start_step(self, kind):

        self.measurements = []
        self.step_start = time.monotonic()
        self.state_start = self.step_start

        if kind == 'LINEAR':
            self.state = 'LINEAR_STEP'
            self.publish_cmd(self.target_linear, 0.0)

        else:
            self.state = 'ANGULAR_STEP'
            self.publish_cmd(0.0, self.target_angular)

        self.get_logger().info(
            f'{kind} velocity step started'
        )

    # ------------------------------------------
    # Calculate dynamic model parameters
    # ------------------------------------------

    def calculate_parameters(self, kind):

        if len(self.baseline) < 5:
            self.get_logger().error(
                f'{kind}: insufficient baseline samples'
            )
            return

        if len(self.measurements) < 10:
            self.get_logger().error(
                f'{kind}: insufficient response samples'
            )
            return

        data = np.asarray(self.measurements)

        times = data[:, 0]
        velocities = data[:, 1]

        # Initial velocity
        v0 = float(np.mean(self.baseline))

        # Steady-state estimate: final 20%
        tail_index = int(0.8 * len(velocities))

        vss = float(np.mean(
            velocities[tail_index:]
        ))

        if kind == 'LINEAR':
            u0 = self.target_linear
            unit = 'm/s'
        else:
            u0 = self.target_angular
            unit = 'rad/s'

        # Input gain
        G = (vss - v0) / u0

        # 90% of total velocity change
        threshold = v0 + 0.9 * (vss - v0)

        # Find crossing time with linear interpolation
        direction = np.sign(vss - v0)
        tau = None

        for i in range(1, len(times)):

            t1 = times[i - 1]
            t2 = times[i]

            y1 = velocities[i - 1]
            y2 = velocities[i]

            before = direction * (y1 - threshold)
            after = direction * (y2 - threshold)

            if before < 0 <= after and y2 != y1:

                fraction = (
                    (threshold - y1) / (y2 - y1)
                )

                tau = t1 + fraction * (t2 - t1)
                break

        if tau is None or tau <= 0:
            self.get_logger().error(
                f'{kind}: 90% rise time not found'
            )
            return

        # Forgetting factor example for dt=0.1 s
        a_01 = 0.1 ** (0.1 / tau)

        self.results[kind] = {
            'G': G,
            'tau': tau
        }

        self.get_logger().info(
            f'\n'
            f'========== {kind} RESULTS ==========\n'
            f'Command:          {u0:.6f} {unit}\n'
            f'Initial velocity: {v0:.6f} {unit}\n'
            f'Steady velocity:  {vss:.6f} {unit}\n'
            f'90% threshold:    {threshold:.6f} {unit}\n'
            f'Gain G:           {G:.6f}\n'
            f'Rise time tau:    {tau:.6f} s\n'
            f'a(dt):            0.1^(dt/{tau:.6f})\n'
            f'a(0.1 s):         {a_01:.6f}\n'
            f'Number of samples: {len(times)}\n'
            f'==================================='
        )

    # ------------------------------------------
    # Automatic experiment sequence
    # ------------------------------------------

    def heartbeat(self):

        if self.finished:
            return

        elapsed = time.monotonic() - self.state_start

        # 1. Initial linear baseline
        if self.state == 'LINEAR_BASELINE':

            self.publish_cmd(0.0, 0.0)

            if elapsed >= self.baseline_sec:
                self.start_step('LINEAR')

        # 2. Linear velocity step
        elif self.state == 'LINEAR_STEP':

            self.publish_cmd(self.target_linear, 0.0)

            if elapsed >= self.hold_sec:

                self.calculate_parameters('LINEAR')

                self.publish_cmd(0.0, 0.0)

                self.state = 'REST'
                self.state_start = time.monotonic()

                self.get_logger().info(
                    'Robot stopping before angular test'
                )

        # 3. Rest between experiments
        elif self.state == 'REST':

            self.publish_cmd(0.0, 0.0)

            if elapsed >= self.rest_sec:

                self.state = 'ANGULAR_BASELINE'
                self.state_start = time.monotonic()
                self.baseline = []

        # 4. Initial angular baseline
        elif self.state == 'ANGULAR_BASELINE':

            self.publish_cmd(0.0, 0.0)

            if elapsed >= self.baseline_sec:
                self.start_step('ANGULAR')

        # 5. Angular velocity step
        elif self.state == 'ANGULAR_STEP':

            self.publish_cmd(0.0, self.target_angular)

            if elapsed >= self.hold_sec:

                self.calculate_parameters('ANGULAR')

                self.publish_cmd(0.0, 0.0)

                self.finished = True
                self.timer.cancel()

                self.print_summary()

    # ------------------------------------------
    # Print final results
    # ------------------------------------------

    def print_summary(self):

        if (
            'LINEAR' not in self.results
            or 'ANGULAR' not in self.results
        ):
            self.get_logger().warning(
                'Some parameters could not be calculated'
            )
            return

        linear = self.results['LINEAR']
        angular = self.results['ANGULAR']

        self.get_logger().info(
            f'\n'
            f'========== FINAL SUMMARY ==========\n'
            f'LINEAR:\n'
            f'G_v = {linear["G"]:.6f}\n'
            f'tau_v = {linear["tau"]:.6f} s\n'
            f'a_v(dt) = 0.1^(dt/{linear["tau"]:.6f})\n'
            f'\n'
            f'ANGULAR:\n'
            f'G_w = {angular["G"]:.6f}\n'
            f'tau_w = {angular["tau"]:.6f} s\n'
            f'a_w(dt) = 0.1^(dt/{angular["tau"]:.6f})\n'
            f'==================================='
        )


def main(args=None):

    rclpy.init(args=args)

    node = StepResponse()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
