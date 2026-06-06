#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frame_name = LaunchConfiguration('frame_name')
    gz_topic = LaunchConfiguration('gz_topic')
    output_topic = LaunchConfiguration('output_topic')
    reference_frame = LaunchConfiguration('reference_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'frame_name',
            default_value='waffle_pi',
            description='Name of the frame to filter from pose stream',
        ),
        DeclareLaunchArgument(
            'gz_topic',
            default_value='/world/default/pose/info',
            description='Gazebo transport topic carrying Pose_V data',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/tb3/ground_truth/pose',
            description='Output PoseStamped topic',
        ),
        DeclareLaunchArgument(
            'reference_frame',
            default_value='world',
            description='Header frame_id to use for published PoseStamped',
        ),
        Node(
            package='prob_rob_labs',
            executable='gz_pose_publisher',
            name='gz_pose_publisher',
            output='screen',
            parameters=[{
                'frame_name': frame_name,
                'gz_topic': gz_topic,
                'output_topic': output_topic,
                'reference_frame': reference_frame,
            }],
        ),
    ])
