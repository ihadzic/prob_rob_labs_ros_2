from dbm import error

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from lab4_odometry.EKF_core import EKFCore

from lab4_odometry.EKF_sensors import (
    EKFSensorSync,
    extract_measurement,
    stamp_to_seconds
)

from lab4_odometry.EKF_messages import create_odometry_message


class EKFOdometry(Node):

    def __init__(self):
        super().__init__('ekf_odometry')

        self.declare_parameter('command_timeout', 1.0)
        self.declare_parameter('sync_slop', 0.05)
        self.declare_parameter('sync_queue_size', 50)

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('left_wheel_joint', 'wheel_left_joint')
        self.declare_parameter('right_wheel_joint', 'wheel_right_joint')

        self.declare_parameter('left_wheel_sign', 1.0)
        self.declare_parameter('right_wheel_sign', 1.0)

        self.command_timeout = float(self.get_parameter('command_timeout').value)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.left_joint = str(self.get_parameter('left_wheel_joint').value)
        self.right_joint = str(self.get_parameter('right_wheel_joint').value)

        self.left_sign = float(self.get_parameter('left_wheel_sign').value)
        self.right_sign = float(self.get_parameter('right_wheel_sign').value)

        # EKF mathematical model
        self.ekf = EKFCore()

        # u = [linear_command, angular_command]
        self.command = np.zeros(2)
        # Time at which the most recent command arrived
        self.command_time = None
        # Most recent synchronized sensor timestamp
        self.previous_sensor_time = None


        # command subscriber
        self.cmd_sub = self.create_subscription(TwistStamped, '/cmd_vel', self.command_callback, 10)


        # sensor synchronization
        self.sensors = EKFSensorSync(
            node=self,
            callback=self.sensor_callback,
            queue_size=int(
                self.get_parameter('sync_queue_size').value
            ),
            slop=float(
                self.get_parameter('sync_slop').value
            )
        )

        # odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/ekf_odom', 10)
        self.get_logger().info('EKF Odometry Started')

    # =======================================
    # receive velocity commands
    def command_callback(self, msg):

        u = np.array(
            [msg.twist.linear.x, msg.twist.angular.z],
            dtype=float
        )

        if not np.all(np.isfinite(u)):
            self.get_logger().warning(
                'Ignoring invalid velocity command.'
            )
            return

        self.command = u

        self.command_time = self.get_clock().now().nanoseconds * 1e-9

    # =======================================
    # check command timeout
    def get_current_command(self, sensor_time):

        if self.command_time is None:
            return np.zeros(2)

        age = sensor_time - self.command_time

        # commands older than one second are ignored.
        if 0.0 <= age <= self.command_timeout:
            return self.command.copy()

        return np.zeros(2)

    # =======================================
    # process synchronized sensor messages
    def sensor_callback(self, imu_msg, joint_msg):
        z, gyro_variance = extract_measurement(
            imu_msg,
            joint_msg,
            left_joint=self.left_joint,
            right_joint=self.right_joint,
            left_sign=self.left_sign,
            right_sign=self.right_sign
        )

        # determine sensor timestamp
        imu_time = stamp_to_seconds(imu_msg.header.stamp)
        joint_time = stamp_to_seconds(joint_msg.header.stamp)

        # use newer message to synchronize
        if imu_time >= joint_time:
            current_time = imu_time
            output_stamp = imu_msg.header.stamp
        else:
            current_time = joint_time
            output_stamp = joint_msg.header.stamp

        # calculate dt and predict
        if self.previous_sensor_time is not None:
            dt = current_time - self.previous_sensor_time

            # A large backward jump usually means
            # simulation time has restarted.
            if dt < -0.5:
                self.get_logger().warning(
                    'Simulation time reset. Resetting EKF.'
                )
                self.ekf.reset()
                self.command = np.zeros(2)
                self.command_time = None
                self.previous_sensor_time = None

            elif dt <= 0:
                # Duplicate or out-of-order sample
                return

            else:
                u = self.get_current_command(current_time)
                # EKF prediction
                self.ekf.predict(u, dt)

        # save current timestamp for the next prediction.
        self.previous_sensor_time = current_time
        # update IMU measurement variance
        if gyro_variance is not None:
            self.ekf.Rm[2, 2] = gyro_variance
        self.ekf.update(z)
        self.publish_odometry(output_stamp)

    # =======================================
    # publish EKF result
    def publish_odometry(self, stamp):
        state = self.ekf.get_state()
        covariance = self.ekf.get_covariance()

        msg = create_odometry_message(
            state=state,
            covariance=covariance,
            stamp=stamp,
            odom_frame=self.odom_frame,
            base_frame=self.base_frame
        )
        self.odom_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    node = EKFOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()