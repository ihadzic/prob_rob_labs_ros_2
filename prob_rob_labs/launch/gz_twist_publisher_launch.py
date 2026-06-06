import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        Node(
            package='prob_rob_labs',
            executable='gz_twist_publisher',
            name='gz_twist_publisher',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])
