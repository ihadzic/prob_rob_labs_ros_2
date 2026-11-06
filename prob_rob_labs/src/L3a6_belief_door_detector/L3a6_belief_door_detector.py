import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

measurement_model = np.array([
    [0.9417, 0.0631],
    [0.0583, 0.9369]
])

transition_model = np.array([
    [0.999999, 0.000001],
    [0.000001, 0.999999]
])

bel = np.array([[0.5], [0.5]])

class belief_door_detector(Node):
    def __init__(self):
        super().__init__('belief_door_detector')
        self.create_subscription(
            Float64, '/feature_mean', self.callback, 10
        )

    def callback(self, msg):
        global bel

        measurement = msg.data
        bel = transition_model @ bel

        unnormalized_posterior = (
            measurement_model *
            np.repeat(bel, 2, axis=1).transpose()
        )[int(measurement > 321.55)]

        posterior = unnormalized_posterior / sum(unnormalized_posterior)
        bel = np.array([posterior]).transpose()

        state = 0 if bel[0, 0] >= 0.9999 else 1

        print(f"State: {state}, P(open): {bel[0, 0]:.5f}")

def main():
    rclpy.init()
    node = belief_door_detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()