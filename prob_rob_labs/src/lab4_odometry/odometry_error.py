import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from message_filters import Subscriber, ApproximateTimeSynchronizer


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def get_pose(pose):
    # Extract x, y, yaw from ROS Pose
    q = pose.orientation
    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    return pose.position.x, pose.position.y, yaw


class OdometryError(Node):
    def __init__(self):
        super().__init__('odometry_error')

        # subscribe and synchronize Ground Truth and EKF
        self.gt_sub = Subscriber(self, PoseStamped, '/tb3/ground_truth/pose',
                                 qos_profile=qos_profile_sensor_data)
        self.ekf_sub = Subscriber(self, Odometry, '/ekf_odom',
                                  qos_profile=qos_profile_sensor_data)

        self.sync = ApproximateTimeSynchronizer([self.gt_sub, self.ekf_sub], 50, 0.03)
        self.sync.registerCallback(self.callback)

        # publish errors to rqt_plot
        self.pos_pub = self.create_publisher(Float64, '/odom_error/position', 10)
        self.yaw_pub = self.create_publisher(Float64, '/odom_error/yaw', 10)
        self.pos_rmse_pub = self.create_publisher(Float64, '/odom_error/rmse_position', 10)
        self.yaw_rmse_pub = self.create_publisher(Float64, '/odom_error/rmse_yaw', 10)

        self.initial = None
        self.n = 0
        self.sum_pos2 = 0.0
        self.sum_yaw2 = 0.0
        

        self.get_logger().info('Odometry Error Analyzer Started')

    def callback(self, gt_msg, ekf_msg):
        xg, yg, tg = get_pose(gt_msg.pose)
        xe, ye, te = get_pose(ekf_msg.pose.pose)

        # Align both trajectories at their initial pose.
        # This allows comparison between world and odom coordinates.
        if self.initial is None:
            self.initial = (xg, yg, tg, xe, ye, te)

        xg0, yg0, tg0, xe0, ye0, te0 = self.initial
        dtheta = wrap(tg0 - te0)
        dx, dy = xe - xe0, ye - ye0

        xa = xg0 + math.cos(dtheta)*dx - math.sin(dtheta)*dy
        ya = yg0 + math.sin(dtheta)*dx + math.cos(dtheta)*dy
        ta = wrap(te + dtheta)

        # position and heading errors
        ex, ey = xa - xg, ya - yg
        epos = math.hypot(ex, ey)
        eyaw = wrap(ta - tg)

        # RMSE
        self.n += 1
        self.sum_pos2 += ex**2 + ey**2
        self.sum_yaw2 += eyaw**2

        rmse_pos = math.sqrt(self.sum_pos2 / self.n)
        rmse_yaw = math.sqrt(self.sum_yaw2 / self.n)

        # publish results
        self.pos_pub.publish(Float64(data=epos))
        self.yaw_pub.publish(Float64(data=eyaw))
        self.pos_rmse_pub.publish(Float64(data=rmse_pos))
        self.yaw_rmse_pub.publish(Float64(data=rmse_yaw))

        # print every 200 samples
        if self.n % 200 == 0:
            self.get_logger().info(
                f'Samples: {self.n} | Position error: {epos:.4f} m | '
                f'Yaw error: {math.degrees(eyaw):.2f} deg | '
                f'Position RMSE: {rmse_pos:.4f} m | '
                f'Yaw RMSE: {math.degrees(rmse_yaw):.2f} deg'
            )


def main(args=None):
    rclpy.init(args=args)
    node = OdometryError()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.n > 0:
            node.get_logger().info(
                f'FINAL RESULTS | Samples: {node.n} | '
                f'Position RMSE: {math.sqrt(node.sum_pos2/node.n):.6f} m | '
                f'Yaw RMSE: {math.degrees(math.sqrt(node.sum_yaw2/node.n)):.4f} deg'
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()