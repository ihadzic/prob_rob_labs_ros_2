import numpy as np

from sensor_msgs.msg import Imu, JointState
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class EKFSensorSync:
    """
    Synchronize IMU and wheel joint-state messages.
    """
    def __init__(self, node, callback, queue_size=50, slop=0.05):

        # BEST_EFFORT subscriptions are compatible with
        # both BEST_EFFORT and RELIABLE publishers.
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # IMU subscriber
        self.imu_sub = Subscriber(node, Imu, '/imu', qos_profile=sensor_qos)

        # Wheel joint-state subscriber
        self.joint_sub = Subscriber(node, JointState, '/joint_states', qos_profile=sensor_qos)

        # Synchronize messages using their header timestamps.
        self.sync = ApproximateTimeSynchronizer(
            [self.imu_sub, self.joint_sub],
            queue_size=queue_size,
            slop=slop
        )

        # callback(imu_msg, joint_msg) belongs to ekf_odometry.py
        self.sync.registerCallback(callback)


def extract_measurement(
    imu_msg,
    joint_msg,
    left_joint='wheel_left_joint',
    right_joint='wheel_right_joint',
    left_sign=1.0,
    right_sign=1.0
):
    """
    Convert synchronized ROS sensor messages into:

        z = [right_wheel, left_wheel, imu_angular_z]

    Returns:
        z: numpy array, shape (3,)
        gyro_variance: IMU angular z variance or None
    """

    # Match wheel velocities by joint name.
    if len(joint_msg.velocity) != len(joint_msg.name):
        raise ValueError(
            'JointState name and velocity lengths do not match.'
        )

    joint_velocities = dict(zip(joint_msg.name, joint_msg.velocity))

    if left_joint not in joint_velocities:
        raise ValueError(f'Missing joint: {left_joint}')
    if right_joint not in joint_velocities:
        raise ValueError(f'Missing joint: {right_joint}')

    omega_left = left_sign * float(joint_velocities[left_joint])
    omega_right = right_sign * float(joint_velocities[right_joint])
    omega_imu = float(imu_msg.angular_velocity.z)

    z = np.array(
        [omega_right, omega_left, omega_imu],
        dtype=float
    )

    if not np.all(np.isfinite(z)):
        raise ValueError('Sensor measurement contains NaN or Inf.')

    # IMU angular velocity covariance a 3x3 matrix
    # Row-major index 8 corresponds to zz.
    gyro_variance = float(imu_msg.angular_velocity_covariance[8])

    # Use the IMU covariance only when valid.
    if not np.isfinite(gyro_variance) or gyro_variance <= 0:
        gyro_variance = None

    return z, gyro_variance


def stamp_to_seconds(stamp):
    """
    Convert ROS header timestamp into seconds.
    """
    return (float(stamp.sec) + float(stamp.nanosec) * 1e-9)