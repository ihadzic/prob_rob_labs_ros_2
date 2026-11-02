import numpy as np
from nav_msgs.msg import Odometry

def create_odometry_message(
    state,
    covariance,
    stamp,
    odom_frame='odom',
    base_frame='base_link'
):
    """
    Convert EKF state and covariance to ROS Odometry.
    EKF state: [theta, x, y, v, omega]
    EKF covariance: 5x5 matrix
    ROS pose order: [x, y, z, roll, pitch, yaw]
    ROS twist order: [vx, vy, vz, wx, wy, wz]
    """

    theta, x, y, v, omega = state
    msg = Odometry()

    # Header and reference frames
    msg.header.stamp = stamp
    msg.header.frame_id = odom_frame
    msg.child_frame_id = base_frame

    # Position
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = 0.0

    # Yaw -> Quaternion
    # q = [0, 0, sin(theta/2), cos(theta/2)]
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = float(np.sin(theta / 2.0))
    msg.pose.pose.orientation.w = float(np.cos(theta / 2.0))

    # Linear and angular velocity
    msg.twist.twist.linear.x = float(v)
    msg.twist.twist.linear.y = 0.0
    msg.twist.twist.linear.z = 0.0

    msg.twist.twist.angular.x = 0.0
    msg.twist.twist.angular.y = 0.0
    msg.twist.twist.angular.z = float(omega)

    # Pose covariance
    pose_cov = np.zeros((6, 6))

    # EKF state indices:
    # theta = 0, x = 1, y = 2
    # ROS pose indices:
    # x = 0, y = 1, yaw = 5
    pose_cov[np.ix_([0, 1, 5], [0, 1, 5])] = (
        covariance[np.ix_([1, 2, 0], [1, 2, 0])]
    )

    # Twist covariance
    twist_cov = np.zeros((6, 6))

    # EKF state indices:
    # v = 3, omega = 4
    # ROS twist indices:
    # vx = 0, wz = 5
    twist_cov[np.ix_([0, 5], [0, 5])] = (
        covariance[np.ix_([3, 4], [3, 4])]
    )

    # ROS requires a row-major list of 36 elements.
    msg.pose.covariance = pose_cov.flatten().tolist()
    msg.twist.covariance = twist_cov.flatten().tolist()

    return msg