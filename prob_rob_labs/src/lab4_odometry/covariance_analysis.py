import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class CovarianceAnalysis(Node):
    def __init__(self):
        super().__init__('covariance_analysis')

        self.declare_parameter('duration_sec', 120.0)
        self.duration = float(self.get_parameter('duration_sec').value)

        self.start_time = None
        self.times = []
        self.data = []
        self.finished = False

        self.create_subscription(Odometry, '/ekf_odom', self.callback, 10)
        self.get_logger().info('Covariance analysis started')

    def callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.start_time is None:
            self.start_time = t

        elapsed = t - self.start_time
        if elapsed < 0:
            return

        # Record approximately once per second
        if self.times and elapsed - self.times[-1] < 1.0:
            return

        p = msg.pose.covariance
        q = msg.twist.covariance

        self.times.append(elapsed)
        self.data.append([p[0], p[7], p[35], q[0], q[35]])

        if elapsed >= self.duration:
            self.finished = True

    def analyze(self):
        if len(self.data) < 10:
            print('Not enough data collected.')
            return

        t = np.array(self.times)
        data = np.array(self.data)
        names = ['Var(x)', 'Var(y)', 'Var(theta)', 'Var(v)', 'Var(omega)']

        # Compare the third quarter with the final quarter
        n = len(data)
        previous = np.mean(data[n//2:3*n//4], axis=0)
        recent = np.mean(data[3*n//4:], axis=0)

        print('\n========== COVARIANCE ANALYSIS ==========')

        for i, name in enumerate(names):
            change = (recent[i] - previous[i]) / max(abs(previous[i]), 1e-12)

            if change > 0.15:
                trend = 'Growing'
            elif change < -0.15:
                trend = 'Decreasing'
            else:
                trend = 'Approximately stable'

            print(f'{name:12s} Initial: {data[0,i]:.6g}  '
                  f'Final: {data[-1,i]:.6g}  Trend: {trend}')

        print('=========================================')

        fig, axes = plt.subplots(5, 1, figsize=(9, 10), sharex=True)
        labels = [
            'Var(x) (m²)', 'Var(y) (m²)', 'Var(theta) (rad²)',
            'Var(v) (m²/s²)', 'Var(omega) (rad²/s²)'
        ]

        for i, ax in enumerate(axes):
            ax.plot(t, data[:, i])
            ax.set_ylabel(labels[i])
            ax.grid(True)

        axes[-1].set_xlabel('Simulation time (s)')
        fig.suptitle('EKF State Covariance Over Time')
        plt.tight_layout()

        output = Path.home() / 'ros2_ws' / 'covariance_analysis_twist.png'
        plt.savefig(output, dpi=150)
        plt.close()
        print(f'\nPlot saved to: {output}')


def main(args=None):
    rclpy.init(args=args)
    node = CovarianceAnalysis()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.analyze()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()