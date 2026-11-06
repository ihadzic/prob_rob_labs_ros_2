import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    forward_velocity = LaunchConfiguration('forward_velocity', default='1.0')
    

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Set to true for simulation',
    )

    declare_forward_velocity_arg = DeclareLaunchArgument(
        'forward_velocity',
        default_value='1.0',
        description='Forward velocity of the robot'
    )

    robot_open_door_cmd = Node(
        package='prob_rob_labs',
        executable='robot_open_door',
        name='robot_open_door',
        parameters=[{
            'use_sim_time': use_sim_time,
            'forward_velocity': forward_velocity
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_forward_velocity_arg)
    ld.add_action(robot_open_door_cmd)


    return ld